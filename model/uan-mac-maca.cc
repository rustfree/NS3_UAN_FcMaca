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


#include "uan-mac-maca.h"
#include "uan-tx-mode.h"
//#include "uan-address.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-new.h"
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
int UanMacMaca::m_maxNodeNum = 10;
vector<unordered_map<uint16_t,Time>> UanMacMaca::m_enQueTime = vector<unordered_map<uint16_t, Time>>(m_maxNodeNum,unordered_map<uint16_t,Time>());


NS_LOG_COMPONENT_DEFINE ("UanMacMaca");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacMaca);

UanMacMaca::UanMacMaca ()
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

UanMacMaca::~UanMacMaca ()
{
   NS_LOG_UNCOND ("-UanMacMaca--enQueNum:"<< enQueNum << "--txDataNum:"<<txDataNum<<"--rxDataNumGood:" <<rxDataNumGood<< "--rxDataNumUnique:"<<rxDataNumUnique);
   NS_LOG_UNCOND ("                          --txCTSNum:"<<txCTSNum <<"--txRTSNum:"<<txRTSNum << "--dropDataNum:"<<DropPktNum);
   if(rxDataNumGood >0){
     NS_LOG_UNCOND ("                          --meanDelay:"<<totalDelay/rxDataNumGood);
   }
}

void
UanMacMaca::Clear ()
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
UanMacMaca::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacMaca::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacMaca")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacMaca> ()
  ;
  return tid;
}

bool
UanMacMaca::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber,const Address &dest)
{
  NS_LOG_FUNCTION (this);
 

  Mac8Address src = Mac8Address::ConvertFrom (GetAddress ());
  Mac8Address udest = Mac8Address::ConvertFrom (dest);

  //添加头部和尾部,data包
  //head
  UanHeaderNew header;
  header.SetSrc (src);
  header.SetDest (udest);
  header.SetType (DATA_TYPE);
  header.SetProtocolNumber (protocolNumber);

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
      

      NS_LOG_DEBUG(" schedule send RTS!");
      if(m_rtsToSendEvent.IsExpired()	){
          m_rtsToSendEvent = Simulator::Schedule (Seconds (0), &UanMacMaca::SendRTSPacket, this);
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
UanMacMaca::SetForwardUpCb (Callback<void, Ptr<Packet>,uint16_t, const Mac8Address& > cb)
{
  m_forUpCb = cb;
}

void
UanMacMaca::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacMaca::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacMaca::RxPacketError, this));

}

void
UanMacMaca::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
    NS_LOG_FUNCTION (this);
    NS_LOG_DEBUG(" RxPacketGood!");
    //int NodeId = GetNode()->GetId();
    Ptr<Packet> originalPacket = pkt->Copy ();
    UanHeaderNew header;
    pkt->RemoveHeader (header);
    if(header.GetDest () != GetAddress ()){ //dest非自己,则要进行退避
        if(header.GetType() == CTS_TYPE || header.GetType() == RTS_TYPE){ //此时如果是WAITCTS状态，应该强制改为BACKOFF状态。因为BACKOFF需要等待的时间更长
            //if(m_state != WAITCTS && m_state != BACKOFF){//进入退避状态
                NS_LOG_DEBUG(" Enter BACKOFF state!");
                m_state = BACKOFF;
                NS_LOG_DEBUG (" "<< Simulator::Now ().GetSeconds () << " Backoff state! ");
                double backoff_Time = m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME;
                Simulator::Cancel(m_rtsToSendEvent);
                Simulator::Cancel(m_HandleBackoffTimeoutEvent);
                m_HandleBackoffTimeoutEvent =  Simulator::Schedule (Seconds (backoff_Time), &UanMacMaca::HandleBackOffTimeout, this);//
            //}
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

                if(!m_queue->IsEmpty() && !m_phy->IsStateTx ()){
                    pkt = m_queue->Dequeue();
                    txDataNum ++;
                    Simulator::Schedule (Seconds (0.3), &UanMacMaca::sendPacketToPhy, this, pkt,header.GetProtocolNumber ());
                    //m_phy->SendPacket (pkt, GetTxModeIndex ());
                    m_state = IDLE;
                    
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

                    UanHeaderNew ctsHeader;

                    ctsHeader.SetSrc (header.GetDest ());//header是rts包的包头
                    ctsHeader.SetDest (header.GetSrc ());
                    ctsHeader.SetType (CTS_TYPE);
                    ctsHeader.SetProtocolNumber (header.GetProtocolNumber ());
                    ctsHeader.SetUid(UINT16_MAX);


                    ctsPkt->AddHeader (ctsHeader);
                    txCTSNum ++;
                    Simulator::ScheduleNow(&UanMacMaca::sendPacketToPhy, this, ctsPkt, GetTxModeIndex ());
                }
                
            }
        
        }
             
    }
}




void
UanMacMaca::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds() << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
  rxDataNumError ++;
}



int64_t
UanMacMaca::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

void
UanMacMaca::HandleCTSTimeout(Ptr<Packet> pkt,uint16_t protocolNumber)//待改，适应queque
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
        double delay = m_erv->GetValue(0,RANDOM_BOUND)+ MAX_TIME;
        if(m_rtsToSendEvent.IsExpired()	){
            m_state = IDLE;
            m_rtsToSendEvent = Simulator::Schedule (Seconds (delay), &UanMacMaca::SendRTSPacket, this);//??待调整,需要判断这个事件是否为空
        }
    }
    
    return;
}

void 
UanMacMaca::HandleBackOffTimeout(void){
    NS_LOG_FUNCTION (this);
    //NS_ASSERT_MSG(m_state == BACKOFF, "Handle backoff timeout but it is not backoff state");
    m_state = IDLE;
    if(!m_queue->IsEmpty() && m_rtsToSendEvent.IsExpired() ){
        m_rtsToSendEvent = Simulator::Schedule (Seconds (0), &UanMacMaca::SendRTSPacket, this);//
    }

}

void
UanMacMaca::sendPacketToPhy(Ptr<Packet> pkt, uint32_t modeNum)//send rts pkt
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

void UanMacMaca::NotifyTxEnd(void)//可以让物理层确定发送延迟后，再告知MAC层.让mac层可以继续发送下一个包。
{
   NS_LOG_FUNCTION (this);
   if(m_queue->IsEmpty()){
      return;
   }
   if(m_queue->IsEmpty() == false && m_state == IDLE)
   {
      if(m_rtsToSendEvent.IsExpired()	){
          m_rtsToSendEvent = Simulator::Schedule (Seconds (m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME+ SEND_FINISH_INTERVAL), &UanMacMaca::SendRTSPacket, this);//
      }
   }
   
   NS_LOG_DEBUG("NotifyTxEnd test ");
} 




void UanMacMaca::SendRTSPacket(void){
    if(m_state != IDLE){
        return;
    }
    if(m_queue ->IsEmpty() ){
        return;
    }
    Ptr<const ns3::Packet> pkt = m_queue->Peek();
    UanHeaderNew header;

    pkt->PeekHeader (header);
    //uint16_t protocolNumber = header.GetProtocolNumber ();

    Ptr<Packet> rtsPkt = Create<Packet> (rtsLen);//多长合适？105
    UanHeaderNew rtsHeader;

    rtsHeader.SetSrc (header.GetSrc ());
    rtsHeader.SetDest (header.GetDest ());
    rtsHeader.SetType (RTS_TYPE);
    rtsHeader.SetProtocolNumber (header.GetProtocolNumber ());
    rtsHeader.SetUid(UINT16_MAX);


    rtsPkt->AddHeader (rtsHeader);

    txRTSNum ++;    
    Simulator::ScheduleNow(&UanMacMaca::sendPacketToPhy, this, rtsPkt, GetTxModeIndex ());
    m_state = WAITCTS;//

    m_HandleCtsTimeoutEvent = Simulator::Schedule (Seconds (WAIT_CTS_TIME), &UanMacMaca::HandleCTSTimeout, 
                                                this, rtsPkt, GetTxModeIndex ());
    
}

}
