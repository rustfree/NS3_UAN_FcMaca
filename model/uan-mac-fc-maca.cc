/* -
by LCH
2018/06/08

 */

/*
    IDLE
    |  /\
pkt |  |ack or time
   \/  |
   WAITACK        BACKOFF(no used so far)

*/


#include "uan-mac-fc-maca.h"
#include "uan-tx-mode.h"
//#include "uan-address.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-fc.h"
#include "ns3/queue.h"
#include "ns3/uan-trailer.h"
#include "ns3/random-variable-stream.h"
#include <algorithm>

#include <iostream>
#define MAX_TIMEOUT_COUNT 1//最多重发次数
#define RANDOM_BOUND 8
#define SEND_FINISH_INTERVAL 4
#define MAX_TIME  6//超时时间
#define WAIT_CTS_TIME  7
/*
Mac类型（type)：
0:数据包，无CRC
1:ACK包
2:数据包，含CRC

*/


namespace ns3
{
int UanMacFcMaca::m_maxNodeNum = 10;
vector<unordered_map<uint16_t,Time>> UanMacFcMaca::m_enQueTime = vector<unordered_map<uint16_t, Time>>(m_maxNodeNum,unordered_map<uint16_t,Time>());


NS_LOG_COMPONENT_DEFINE ("UanMacFcMaca");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacFcMaca);

UanMacFcMaca::UanMacFcMaca ()
  : UanMac (),
    m_state(IDLE),
    m_cleared (false)
       //state initializing
{
   ObjectFactory m_queueFactory;
   m_queueFactory.SetTypeId ("ns3::DropTailQueue<Packet>");
   m_queue = m_queueFactory.Create<Queue<Packet> > ();
   m_nextUid = 0;
   m_erv = CreateObject<UniformRandomVariable>();
  
}

UanMacFcMaca::~UanMacFcMaca ()
{
   NS_LOG_UNCOND ("-UanMacFcMaca--enQueNum:"<< enQueNum << "--txDataNum:"<<txDataNum<<"--rxDataNumGood:" <<rxDataNumGood<< "--rxDataNumUnique:"<<rxDataNumUnique);
   NS_LOG_UNCOND ("                          --txCTSNum:"<<txCTSNum <<"--txRTSNum:"<<txRTSNum << "--dropDataNum:"<<DropPktNum);
   if(rxDataNumGood >0){
     NS_LOG_UNCOND ("                          --meanDelay:"<<totalDelay/rxDataNumGood);
   }
}

void
UanMacFcMaca::Clear ()
{
  if (m_cleared)
    {
      return;
    }
  m_cleared = true;
  if (m_phy)
    {
      m_phy->Clear ();
      m_phy = 0;
    }
}

void
UanMacFcMaca::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacFcMaca::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacFcMaca")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacFcMaca> ()
  ;
  return tid;
}

bool
UanMacFcMaca::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber,const Address &dest)
{
  NS_LOG_FUNCTION (this);
 

  Mac8Address src = Mac8Address::ConvertFrom (GetAddress ());
  Mac8Address udest = Mac8Address::ConvertFrom (dest);

  //添加头部和尾部,data包
  //head
  UanHeaderFc header;
  header.SetSrc (src);
  header.SetDest (udest);
  header.SetType (DATA_TYPE);
  header.SetProtocolNumber (protocolNumber);
  header.SetQueNum(0);
  header.SetUid(m_nextUid);
  m_nextUid ++;
  packet->AddHeader (header);

  uint8_t srcAddr;
  src.CopyTo(&srcAddr);//获取源地址
  m_enQueTime[srcAddr][header.GetUid()] = Simulator::Now(); // 获取入队时刻

  //trailer
  UanTrailer trailer;
  if (Node::ChecksumEnabled ())
  {
      trailer.EnableFcs (true);
      trailer.CalcFcs (packet);// CRC check sum
      packet->AddTrailer (trailer);
  }  
 
  
  NS_LOG_DEBUG(" Packet Enqueue!");
  enQueNum ++; //入队列的包数量统计
  m_queue -> Enqueue(packet);//入队列   
  
  
  if(!m_phy->IsStateTx () && !m_queue->IsEmpty() && m_state == IDLE)//物理层空闲&&发送队列不为空&&为可发送状态
  {
      
      uint32_t curQueSize = m_queue -> GetCurrentSize().GetValue();
      NS_LOG_DEBUG("Mac queue current size:"<< curQueSize << " m_curFcw:" << m_curFcw << " node id:"<< (uint32_t)srcAddr);
      if(curQueSize >= m_curFcw){ //只有队列包数量大于限定值，才允许发送
          NS_LOG_DEBUG(" schedule send RTS!");
          if(m_rtsToSendEvent.IsExpired()	){
              m_rtsToSendEvent = Simulator::Schedule (Seconds (0), &UanMacFcMaca::SendRTSPacket, this);
          }
      }
      
  }else{
      if(m_phy->IsStateTx ()){
          NS_LOG_DEBUG("phy is tx state");
      }
      if(m_state != IDLE){
          NS_LOG_DEBUG("mac is not IDLE");
      }
  }
   return true;
}

void
UanMacFcMaca::SetForwardUpCb (Callback<void, Ptr<Packet>,uint16_t, const Mac8Address& > cb)
{
  m_forUpCb = cb;
}

void
UanMacFcMaca::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacFcMaca::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacFcMaca::RxPacketError, this));

}

void
UanMacFcMaca::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
    NS_LOG_FUNCTION (this);
    NS_LOG_DEBUG(" RxPacketGood!");
    //int NodeId = GetNode()->GetId();
    Ptr<Packet> originalPacket = pkt->Copy ();
    UanHeaderFc header;
    pkt->RemoveHeader (header);
    if(header.GetDest () != GetAddress ()){ //dest非自己,则要进行退避
        if(header.GetType() == CTS_TYPE || header.GetType() == RTS_TYPE){ //此时如果是WAITCTS状态，应该强制改为BACKOFF状态。因为BACKOFF需要等待的时间更长
            //if(m_state != WAITCTS && m_state != BACKOFF){//进入退避状态
                NS_LOG_DEBUG(" Enter BACKOFF state!");
                m_state = BACKOFF;
                NS_LOG_DEBUG (" "<< Simulator::Now ().GetSeconds () << " Backoff state! ");
                double backoff_Time = m_erv->GetValue(0,RANDOM_BOUND) + 2.1*(m_curFcw+2);
                Simulator::Cancel(m_rtsToSendEvent);
                Simulator::Cancel(m_HandleBackoffTimeoutEvent);
                m_HandleBackoffTimeoutEvent = Simulator::Schedule (Seconds (backoff_Time), &UanMacFcMaca::HandleBackOffTimeout, this);//
            //}
        }
        if(header.GetType() == CTS_TYPE){
            m_curFcw = header.GetQueNum();//更新窗口限制
        }

    }else if (header.GetDest () == GetAddress () || header.GetDest () == Mac8Address::GetBroadcast ()){
    
        NS_LOG_DEBUG (" "<< Simulator::Now ().GetSeconds () << " Receiving packet from " << header.GetSrc () << " For " << header.GetDest ());
        
        UanTrailer trailer;
        originalPacket->RemoveTrailer (trailer);
        if (Node::ChecksumEnabled ())
        {
            trailer.EnableFcs (true);
        }
        bool crcGood = trailer.CheckFcs (originalPacket);
        if (!crcGood)//CRC error
        {
            NS_LOG_INFO ("CRC error on Packet " <<originalPacket);
            rxDataNumError ++;
            return;
        }else{ //CRC correct

            NS_LOG_DEBUG ("CRC correct!");
            if(header.GetType() == CTS_TYPE)//recv CTS
            {
                Simulator::Cancel(m_HandleCtsTimeoutEvent);
                NS_LOG_DEBUG ("recv CTS pkt, data pkt dequeue!");
                Ptr<Packet> pkt;
                m_state = TOSEND;
                m_curFcw = header.GetQueNum();
                m_tosendNum = m_curFcw;

                NS_LOG_DEBUG (" m_tosenNum: "<< m_tosendNum) ;

                if(!m_queue->IsEmpty() && !m_phy->IsStateTx ()){
                    pkt = m_queue->Dequeue();
                    txDataNum ++;
                    m_tosendNum -- ;
                    
                    //m_phy->SendPacket (pkt, GetTxModeIndex ());
                    Simulator::Schedule (Seconds (0.3), &UanMacFcMaca::sendPacketToPhy, this, pkt,header.GetProtocolNumber ());
                    if(m_tosendNum == 0){
                        m_state = IDLE;
                    }
                }else
                {
                    NS_LOG_DEBUG ("send data pkt when phy is busy or queue empty!");
                }
                
            }else if(header.GetType() == DATA_TYPE){//real data packet
                NS_LOG_DEBUG ("recv data pkt!");
                rxDataNumGood ++;
                /*统计不重复的包*/
                uint8_t srcAddr;
                header.GetSrc().CopyTo(&srcAddr);
                if(m_srcUidSetVec[srcAddr].count(header.GetUid()) == 0){
                    rxDataNumUnique ++;
                    m_srcUidSetVec[srcAddr].insert(header.GetUid());
                    
                    Time realDelay = Simulator::Now() - m_enQueTime[srcAddr][header.GetUid()];//计算该包的延迟
                    NS_LOG_DEBUG ("recv Pkt delay:"<<realDelay.GetSeconds());
                    totalDelay += realDelay.GetSeconds();
                }
                m_forUpCb (pkt,header.GetProtocolNumber (), header.GetSrc ());//up to

            }else if(header.GetType() == RTS_TYPE){   //RTS packet
                NS_LOG_DEBUG ("recv RTS pkt!");
                if(m_state == IDLE ){
                    Ptr<Packet> ctsPkt = Create<Packet> (ctsLen);//多长合适？105

                    /* sink端更新src端的mac queNum数据 */
                    uint8_t srcAddr;
                    header.GetSrc().CopyTo(&srcAddr);
                    uint16_t oldNum = m_srcQueNum[srcAddr];
                    uint16_t queNum = header.GetQueNum();
                    m_srcQueNum[srcAddr] = queNum;
                    m_srcQueNumSum +=  queNum - oldNum;
                    UpdateFcw();//更新m_curFcw;
          
                    UanHeaderFc ctsHeader;

                    ctsHeader.SetSrc (header.GetDest ());//header是rts包的包头
                    ctsHeader.SetDest (header.GetSrc ());
                    ctsHeader.SetType (CTS_TYPE);
                    ctsHeader.SetProtocolNumber (header.GetProtocolNumber ());
                    ctsHeader.SetUid(UINT16_MAX);
                    ctsHeader.SetQueNum(m_curFcw);//设置限制窗口

                    ctsPkt->AddHeader (ctsHeader);
                    txCTSNum ++;
                    Simulator::ScheduleNow(&UanMacFcMaca::sendPacketToPhy, this, ctsPkt, GetTxModeIndex ());
                }
                
            }
        
        }
             
    }
}

void 
UanMacFcMaca::UpdateFcw(void){
  UpdateFcwLinear();
}

void 
UanMacFcMaca::UpdateFcwPiecewise(void){
    uint16_t oldFcw = m_curFcw;
    
    if(m_curFcw == 1){
        if(m_srcQueNumSum > 100){
            m_curFcw = 5;
        }
    }else if(m_curFcw == 5){
        if(m_srcQueNumSum < 50){
            m_curFcw = 1;
        }else if(m_srcQueNumSum > 300){
            m_curFcw = 10;
        }
    }else if(m_curFcw == 10){
        if(m_srcQueNumSum < 200){
            m_curFcw = 5;
        }
    }
    //m_curFcw = std::max( int(m_srcQueNumSum/20), 1);
    NS_LOG_DEBUG ("m_srcQueNumSum = " << m_srcQueNumSum);
    if(m_curFcw != oldFcw)
        NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " Mac Update new Fcw,curFcw = "<<m_curFcw);
    
}

void 
UanMacFcMaca::UpdateFcwLinear(void){
  uint16_t oldFcw = m_curFcw;
       
  m_curFcw = std::max( int(m_srcQueNumSum/10), 1);
  NS_LOG_DEBUG ("m_srcQueNumSum = " << m_srcQueNumSum);
  if(m_curFcw != oldFcw)
      NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " Mac Update new Fcw,curFcw = "<<m_curFcw);
}



void
UanMacFcMaca::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
  rxDataNumError ++;
}



int64_t
UanMacFcMaca::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

void
UanMacFcMaca::HandleCTSTimeout(Ptr<Packet> pkt,uint16_t protocolNumber)//待改，适应queque
{

  //此时如果backoff状态？如何处理？应该backoff优先
    NS_LOG_FUNCTION (this);
    m_timeoutCnt ++;

    if(m_timeoutCnt > MAX_TIMEOUT_COUNT){
        m_timeoutCnt = 0;
        if(!m_queue->IsEmpty())
          m_queue->Remove();//drop the packet item
        DropPktNum ++;
        if(m_state == WAITCTS)//
           m_state = IDLE;
        
    }
    if(m_state == BACKOFF)//
        return;

    //Simulator::Cancel(m_HandleTimeoutEvent);
    //这个时候还是WAITCTS状态？？
    if(m_state == WAITCTS){  
        double delay = m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME;
        if(m_rtsToSendEvent.IsExpired()	){
            m_state = IDLE;
            m_rtsToSendEvent = Simulator::Schedule (Seconds (delay), &UanMacFcMaca::SendRTSPacket, this);//??待调整,需要判断这个事件是否为空
        }
    }
    
    return;
}

void 
UanMacFcMaca::HandleBackOffTimeout(void){
    NS_LOG_FUNCTION (this);
    //NS_ASSERT_MSG(m_state == BACKOFF, "Handle backoff timeout but it is not backoff state");
    m_state = IDLE;
    if(!m_queue->IsEmpty() && m_rtsToSendEvent.IsExpired() ){
        m_rtsToSendEvent = Simulator::Schedule (Seconds (0), &UanMacFcMaca::SendRTSPacket, this);//
    }

}

void
UanMacFcMaca::sendPacketToPhy(Ptr<Packet> pkt, uint32_t modeNum)//send rts pkt
{
    NS_LOG_FUNCTION (this);  
    if(m_phy->IsStateTx ()) 
    {
        NS_LOG_UNCOND ("send rts pkt when phy busy!");
        //NS_ASSERT (false);
    }
    NS_LOG_DEBUG ("send RTS pkt");
    m_phy->SendPacket (pkt, modeNum);//send data packet to phy
    //立即发ack,此时节点一直处于非TX状态，一定可以直接发送。
    
    //NotifyTxEnd();//待考证！！
}

void UanMacFcMaca::NotifyTxEnd(void)//可以让物理层确定发送延迟后，再告知MAC层.让mac层可以继续发送下一个包。
{
   NS_LOG_FUNCTION (this);
   if(m_queue->IsEmpty()){
      return;
   }
   if(m_state == IDLE)
   {
      if(m_rtsToSendEvent.IsExpired()	){
          m_rtsToSendEvent = Simulator::Schedule (Seconds (m_erv->GetValue(0,4) + MAX_TIME + SEND_FINISH_INTERVAL), &UanMacFcMaca::SendRTSPacket, this);//
      }
   }else if(m_state == TOSEND){
      Ptr<Packet> pkt;
      if( !m_phy->IsStateTx ()){
          pkt = m_queue->Dequeue();
          txDataNum ++;
          //m_phy->SendPacket (pkt, GetTxModeIndex ());
          Simulator::ScheduleNow(&UanMacFcMaca::sendPacketToPhy, this, pkt, GetTxModeIndex ());
          m_tosendNum -- ;
          if(m_tosendNum == 0){
              m_state = IDLE;
          }
          
      }else
          NS_LOG_DEBUG ("send data pkt when phy is busy or queue empty!");
   }
  NS_LOG_DEBUG("NotifyTxEnd test ");
} 




void UanMacFcMaca::SendRTSPacket(void){
    if(m_state != IDLE){
        return;
    }
    if(m_queue ->IsEmpty() || m_queue->GetCurrentSize().GetValue() < m_curFcw){
        return;
    }
    Ptr<const ns3::Packet> pkt = m_queue->Peek();
    UanHeaderFc header;

    pkt->PeekHeader (header);
    //uint16_t protocolNumber = header.GetProtocolNumber ();

    Ptr<Packet> rtsPkt = Create<Packet> (rtsLen);//多长合适？105
    UanHeaderFc rtsHeader;

    rtsHeader.SetSrc (header.GetSrc ());
    rtsHeader.SetDest (header.GetDest ());
    rtsHeader.SetType (RTS_TYPE);
    rtsHeader.SetProtocolNumber (header.GetProtocolNumber ());
    rtsHeader.SetUid(UINT16_MAX);
    rtsHeader.SetQueNum(m_queue->GetCurrentSize().GetValue());

    rtsPkt->AddHeader (rtsHeader);

    txRTSNum ++;    
    Simulator::ScheduleNow(&UanMacFcMaca::sendPacketToPhy, this, rtsPkt, GetTxModeIndex ());
    m_state = WAITCTS;//

    m_HandleCtsTimeoutEvent = Simulator::Schedule (Seconds (WAIT_CTS_TIME), &UanMacFcMaca::HandleCTSTimeout, 
                                                this, rtsPkt, GetTxModeIndex ());
    
}

}
