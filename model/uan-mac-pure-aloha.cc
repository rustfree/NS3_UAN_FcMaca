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

非完善版本aloha
*/


#include "uan-mac-pure-aloha.h"
#include "uan-tx-mode.h"
//#include "uan-address.h"
#include "ns3/log.h"
#include "uan-phy.h"
#include "uan-header-common.h"
#include "ns3/queue.h"
#include "ns3/uan-trailer.h"
#include "ns3/random-variable-stream.h"

#include <iostream>
#define MAX_TIMEOUT_COUNT 1//最多重发次数
#define RANDOM_BOUND 8
#define MAX_TIME  8//超时时间？//仿真情况下，该值小于10会有比较好的效果

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

NS_LOG_COMPONENT_DEFINE ("UanMacPureAloha");
  
NS_OBJECT_ENSURE_REGISTERED (UanMacPureAloha);

UanMacPureAloha::UanMacPureAloha ()
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

UanMacPureAloha::~UanMacPureAloha ()
{
   NS_LOG_UNCOND ("-UanMacPureAloha--txDataNum:"<<txDataNum<<"-rxDataNumGood:" <<rxDataNumGood<<"-rxDataNumError:"<<rxDataNumError<<"-rxAckNum:"<<rxAckNum<<"-txAckNum:"<<txAckNum );
   NS_LOG_UNCOND ("                 -resendNum:"<<resendNum<<"-DropPktNum:"<<DropPktNum);
}

void
UanMacPureAloha::Clear ()
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
UanMacPureAloha::DoDispose ()
{
  Clear ();
  UanMac::DoDispose ();
}

TypeId
UanMacPureAloha::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanMacPureAloha")
    .SetParent<UanMac> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanMacPureAloha> ()
  ;
  return tid;
}

bool
UanMacPureAloha::Enqueue (Ptr<Packet> packet, uint16_t protocolNumber,const Address &dest)
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("" << Simulator::Now ().GetSeconds () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Queueing packet for " << Mac8Address::ConvertFrom (dest));

  Mac8Address src = Mac8Address::ConvertFrom (GetAddress ());
  Mac8Address udest = Mac8Address::ConvertFrom (dest);

  //添加头部和尾部
  //head
  UanHeaderCommon header;
  header.SetSrc (src);
  header.SetDest (udest);
  header.SetType (PACKET_TYPE_CRC);
  header.SetProtocolNumber (protocolNumber);
  packet->AddHeader (header);
  //trailer
  UanTrailer trailer;
  if (Node::ChecksumEnabled ())
  {
      trailer.EnableFcs (true);
  }  
  trailer.CalcFcs (packet);// CRC check sum
  packet->AddTrailer (trailer);
  if(AckTxWait == true  || m_phy->IsStateTx () ) //准备发送ack||phy正在发送||正在等待ack回复
  {
      NS_LOG_DEBUG(" Packet Enqueue!");
      m_queue -> Enqueue(packet);   
  }
  else if( m_queue -> IsEmpty() )//物理层空闲&&发送队列为空
  {
      NS_LOG_DEBUG(" send real data!");
 
      m_phy->SendPacket (packet, GetTxModeIndex ());//send packet to phy
      txDataNum ++;
     
    }
  else//物理层空闲&&发送队列不为空
  {
     NS_LOG_DEBUG(" Packet Enqueue!");
     m_queue -> Enqueue(packet);
     Simulator::Schedule (Seconds (0), &UanMacPureAloha::SendQueuePacket, this);
  }
   return true;
}

void
UanMacPureAloha::SetForwardUpCb (Callback<void, Ptr<Packet>,uint16_t, const Mac8Address& > cb)
{
  m_forUpCb = cb;
}
void
UanMacPureAloha::AttachPhy (Ptr<UanPhy> phy)
{
  m_phy = phy;
  m_phy->SetReceiveOkCallback (MakeCallback (&UanMacPureAloha::RxPacketGood, this));
  m_phy->SetReceiveErrorCallback (MakeCallback (&UanMacPureAloha::RxPacketError, this));

}
void
UanMacPureAloha::RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode)
{
  NS_LOG_FUNCTION (this);

  Ptr<Packet> originalPacket = pkt->Copy ();
  UanHeaderCommon header;
  pkt->RemoveHeader (header);
  NS_LOG_DEBUG (" "<< Simulator::Now ().GetSeconds () << " Receiving packet from " << header.GetSrc () << " For " << header.GetDest ());

  if (header.GetDest () == GetAddress () || header.GetDest () == Mac8Address::GetBroadcast ())
    {

       if(header.GetType() == PACKET_TYPE_CRC)//CRC trailer
       {
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
           }
           else//CRC correct
           {
             rxDataNumGood ++;
             NS_LOG_DEBUG ("CRC correct!");
     
             m_forUpCb (pkt,header.GetProtocolNumber (), header.GetSrc ());
	     NotifyTxEnd();
            
           }
        }
  
    }
}



void
UanMacPureAloha::RxPacketError (Ptr<Packet> pkt, double sinr)
{
  NS_LOG_DEBUG ("" << Simulator::Now () << " MAC " << Mac8Address::ConvertFrom (GetAddress ()) << " Received packet in error with sinr " << sinr);
  rxDataNumError ++;
}



int64_t
UanMacPureAloha::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

void
UanMacPureAloha::HandleTimeout(Ptr<Packet> pkt,uint16_t protocolNumber)
{
    NS_LOG_FUNCTION (this);

    return;
}

void
UanMacPureAloha::sendpacket(Ptr<Packet> pkt, uint32_t modeNum)//只有发ack包才被调用，此刻节点刚接收完，ack包有优先权，Mac层一定不处于发送状态
{
    NS_LOG_FUNCTION (this);  
}

void UanMacPureAloha::NotifyTxEnd(void)//可以让物理层才确定发送延迟后，再告知MAC层
{
   NS_LOG_FUNCTION (this);
   if(m_queue->IsEmpty() == false && m_state != WAITACK)
   {
      Simulator::Schedule (Seconds (0), &UanMacPureAloha::SendQueuePacket, this);//??待调整
   }
}

void UanMacPureAloha::SendQueuePacket(void)
{
   Ptr<Packet> pkt = m_queue->Dequeue();
 
   m_phy->SendPacket (pkt, GetTxModeIndex ());//send packet to phy
   
   txDataNum ++;
}


}
