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

完善版本aloha
*/


#include "uan-mac-new-aloha.h"
#include "uan-tx-mode.h"
//#include "uan-address.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-new.h"
#include "uan-header-common.h"
#include "ns3/queue.h"
#include "ns3/uan-trailer.h"
#include "ns3/random-variable-stream.h"

#include <iostream>

#define MAX_TIMEOUT_COUNT 2//最多重发次数
#define RANDOM_BOUND 8
#define MAX_TIME  10//超时时间？//仿真情况下，该值小于10会有比较好的效果

#define PACKET_TYPE_ACK 1
#define PACKET_TYPE_CRC 2//待CRC尾部校验和的包类型
/*
Mac类型（type)：
0:数据包，无CRC
1:ACK包
2:数据包，含CRC

*/


namespace ns3
{
int UanMacNewAloha::m_maxNodeNum = 10;
vector<unordered_map<uint16_t,Time>> UanMacNewAloha::m_enQueTime = vector<unordered_map<uint16_t, Time>>(m_maxNodeNum,unordered_map<uint16_t,Time>());

NS_LOG_COMPONENT_DEFINE ("UanMacNewAloha");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacNewAloha);

UanMacNewAloha::UanMacNewAloha ()
  : UanMac (),
    m_state(IDLE),
    m_cleared (false)
       //state initializing
{
   ObjectFactory m_queueFactory;
   m_queueFactory.SetTypeId ("ns3::DropTailQueue<Packet>");
   m_queue = m_queueFactory.Create<Queue<Packet> > ();

   m_erv = CreateObject<UniformRandomVariable>();
  
}

UanMacNewAloha::~UanMacNewAloha ()
{
   NS_LOG_UNCOND ("-UanMacNewAloha--enQueNum:"<< enQueNum << "--txDataNum:"<<txDataNum<<"--rxDataNumGood:" <<rxDataNumGood<< "--rxDataNumUnique:"<<rxDataNumUnique<<"-rxDataNumError:"<<rxDataNumError);
   NS_LOG_UNCOND ("                          --rxAckNum:"<<rxAckNum<<"--txAckNum:"<<txAckNum <<"--resendNum:"<<resendNum<<"--DropPktNum:"<<DropPktNum);
   if(rxDataNumGood >0){
     NS_LOG_UNCOND ("                          --meanDelay:"<<totalDelay/rxDataNumUnique);
   }
}

void
UanMacNewAloha::Clear ()
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
UanMacNewAloha::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacNewAloha::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacNewAloha")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacNewAloha> ()
  ;
  return tid;
}

bool
UanMacNewAloha::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber,const Address &dest)
{
  NS_LOG_FUNCTION (this);

  Mac8Address src = Mac8Address::ConvertFrom (GetAddress ());
  Mac8Address udest = Mac8Address::ConvertFrom (dest);
  
  //添加头部和尾部
  //head
  
  UanHeaderNew header;
  header.SetSrc (src);
  header.SetDest (udest);
  header.SetType (PACKET_TYPE_CRC);
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
  enQueNum ++;
  if(AckTxWait == true || m_state == WAITACK || m_phy->IsStateTx () ) //准备发送ack||正在等待ack回复||phy正在发送
  {
      NS_LOG_DEBUG(" Packet Enqueue!");
      m_queue -> Enqueue(packet);   
  }else if( m_queue -> IsEmpty() ) {  //物理层空闲&&发送队列为空
  
      NS_LOG_DEBUG(" send real data!");
      NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Queueing packet for " << Mac8Address::ConvertFrom (dest));
      
      m_state = WAITACK; 
      double delay = m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME;
      //NS_LOG_UNCOND ("resend delay:"<<delay);
      m_HandleTimeoutEvent = Simulator::Schedule (Seconds (delay), &UanMacNewAloha::HandleTimeout, this,packet,protocolNumber);
      
      m_phy->SendPacket (packet, GetTxModeIndex ());//send packet to phy
      txDataNum ++;
      //handle ack timeout,先不进行退避
     
  }else {   //物理层空闲&&发送队列不为空

      NS_LOG_DEBUG(" Packet Enqueue!");
      m_queue -> Enqueue(packet);
      Simulator::Schedule (Seconds (0), &UanMacNewAloha::SendQueuePacket, this);
  }
  return true;
}

void
UanMacNewAloha::SetForwardUpCb (Callback<void, Ptr<Packet>,uint16_t, const Mac8Address& > cb)
{
  m_forUpCb = cb;
}
void
UanMacNewAloha::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacNewAloha::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacNewAloha::RxPacketError, this));

}
void
UanMacNewAloha::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
  NS_LOG_FUNCTION (this);

  Ptr<Packet> originalPacket = pkt->Copy ();
  UanHeaderNew header;
  int headLen = pkt->RemoveHeader (header);
  NS_LOG_DEBUG (" headLen:" << headLen);
  
  if (header.GetDest () == GetAddress () || header.GetDest () == Mac8Address::GetBroadcast ())
    {
       NS_LOG_DEBUG (" "<< Simulator::Now ().GetSeconds () << " Receiving packet from " << header.GetSrc () << " For " << header.GetDest ());
       if(header.GetType() == PACKET_TYPE_CRC)//CRC trailer
       {

           UanTrailer trailer;
           
           if (Node::ChecksumEnabled ())
           {
              originalPacket->RemoveTrailer (trailer);
              trailer.EnableFcs (true);
           }
           bool crcGood = trailer.CheckFcs (originalPacket);
           if (!crcGood)//CRC error
           {
             NS_LOG_INFO ("CRC error on Packet " <<originalPacket);
             rxDataNumError ++;
             return;
           }
           else//CRC correct
           {
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
              
              NS_LOG_DEBUG ("CRC correct!");
              
              AckTxWait = true; //需要准备发送ACk
              
              m_forUpCb (pkt,header.GetProtocolNumber (), header.GetSrc ());

              
              Ptr<Packet> ackPkt = Create<Packet> (ackPktSize);//多长合适？105
              
              UanHeaderNew ackheader;

              ackheader.SetSrc (header.GetDest ());
                    ackheader.SetDest (header.GetSrc ());
                    ackheader.SetType (PACKET_TYPE_ACK);
                    ackheader.SetProtocolNumber (header.GetProtocolNumber ());
                    ackheader.SetUid(0);

              ackPkt->AddHeader (ackheader);
                
              Simulator::ScheduleNow(&UanMacNewAloha::sendpacket, this,ackPkt,GetTxModeIndex ());//send ack是否应该进入队列？
                          //Simulator::Schedule(Seconds(0),&UanMacNewAloha::sendpacket, this,ackPkt,GetTxModeIndex ());//send ack是否应该进入队列？
                          //考虑到通信机切换到RX速度较慢
                          
              txAckNum ++; 
           }
        }
        else if(header.GetType() == PACKET_TYPE_ACK)//ACK
        {
            m_state = IDLE;
            rxAckNum ++;
            Simulator::Cancel (m_HandleTimeoutEvent);

	          timeout_cnt = 0;
	          NS_LOG_DEBUG ("UanMacNewAloha:: recv ack");
            NotifyTxEnd();
        }
    }
}



void
UanMacNewAloha::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
  rxDataNumError ++;
}



int64_t
UanMacNewAloha::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

void
UanMacNewAloha::HandleTimeout(Ptr<Packet> pkt,uint16_t protocolNumber)//待改，适应queque
{
    NS_LOG_FUNCTION (this);
    Simulator::Cancel(m_HandleTimeoutEvent);
    if(m_state == IDLE )
    {
        m_state = IDLE;
        timeout_cnt = 0;
        return ;
    }
    if(m_state == WAITACK) 
    {
         
         if (timeout_cnt == MAX_TIMEOUT_COUNT)
        {
            timeout_cnt = 0;
            DropPktNum ++;
            NS_LOG_DEBUG ("" << Simulator::Now () << " :Time out "<<MAX_TIMEOUT_COUNT<<"times,turn IDLE!");
            m_state = IDLE;
            NotifyTxEnd();//待考证
            return;
        }
         timeout_cnt ++;
         NS_LOG_DEBUG ("" << Simulator::Now () << " :Time out for "<< timeout_cnt << "times");
         m_phy->SendPacket (pkt, GetTxModeIndex ());//立即重发,此时节点一直处于WaitAck状态，一定可以直接发送。
         resendNum ++;
         NS_LOG_INFO ("cancel event");
         double delay = m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME + timeout_cnt;
         // NS_LOG_UNCOND ("resend delay:"<<delay);
	       m_HandleTimeoutEvent = Simulator::Schedule (Seconds (delay), &UanMacNewAloha::HandleTimeout, this,pkt,protocolNumber);
         
    }
    return;
}

void
UanMacNewAloha::sendpacket(Ptr<Packet> pkt, uint32_t modeNum)//只有发ack包才被调用，此刻节点刚接收完，ack包有优先权，Mac层一定不处于发送状态
{
    NS_LOG_FUNCTION (this);  
    if(m_phy->IsStateTx ()) 
      {
        NS_LOG_UNCOND ("send ack when busy!");
        //NS_ASSERT (false);
      }
    NS_LOG_DEBUG ("send ack");
    m_phy->SendPacket (pkt, modeNum);//send ack packet to phy
    //立即发ack,此时节点一直处于非TX状态，一定可以直接发送。
    AckTxWait = false;
    //NotifyTxEnd();//待考证！！
}

void UanMacNewAloha::NotifyTxEnd(void)//可以让物理层才确定发送延迟后，再告知MAC层
{
   NS_LOG_FUNCTION (this);
   if(m_queue->IsEmpty() == false && m_state != WAITACK)
   {
      Simulator::Schedule (Seconds (0), &UanMacNewAloha::SendQueuePacket, this);//??待调整
   }
  NS_LOG_DEBUG("NotifyTxEnd test ");
} 

void UanMacNewAloha::SendQueuePacket(void)
{
   Ptr<Packet> pkt = m_queue->Dequeue();
   UanHeaderNew header;
   pkt->RemoveHeader (header);
   uint16_t protocolNumber = header.GetProtocolNumber ();
   pkt->AddHeader (header);
   if(protocolNumber != 2054)//如果是ARP就不进行ack机制回应
      {  
          m_state = WAITACK; 
          double delay = m_erv->GetValue(0,RANDOM_BOUND) + MAX_TIME;
         // NS_LOG_UNCOND ("resend delay:"<<delay);
          m_HandleTimeoutEvent = Simulator::Schedule (Seconds (delay), &UanMacNewAloha::HandleTimeout, this,pkt,protocolNumber);
      }
    NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Queueing packet for " << Mac8Address::ConvertFrom (header.GetDest()));
   m_phy->SendPacket (pkt, GetTxModeIndex ());//send packet to phy
   
   txDataNum ++;
}


}
