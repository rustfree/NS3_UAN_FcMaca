/*   ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
 **    uan-phy-real-cloud.cc                     *
 **    Created on 2018/10/13               *
 **    Author:lch                          *
 **    For:connecting with real channel    *
 **  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
*/


#include "uan-phy-real-cloud.h"
#include "uan-transducer.h"
#include "uan-channel.h"
#include "uan-net-device.h"
#include "ns3/simulator.h"
#include "ns3/traced-callback.h"
#include "ns3/ptr.h"
#include "ns3/trace-source-accessor.h"
#include "ns3/double.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/uan-tx-mode.h"
#include "ns3/node.h"
#include "ns3/uinteger.h"
#include "ns3/object-vector.h"
#include "ns3/energy-source-container.h"
#include "ns3/acoustic-modem-energy-model.h"
#include "ns3/uan-header-common.h"//by LCH

 #include <unistd.h>

#include <pthread.h>
#include "ns3/system-thread.h"

#include <sys/ioctl.h>
#include <sys/types.h>

#include <netdb.h>
#include <arpa/inet.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <netinet/ip.h>
#include <linux/if_packet.h>
#include <linux/if.h>
#include <linux/sockios.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include "uan-emulation-manager.h"
#include "uan-emulation-packet.h"

namespace ns3 {

//extern std::vector<int> fd_list;

NS_LOG_COMPONENT_DEFINE ("UanPhyRealCloud");

NS_OBJECT_ENSURE_REGISTERED (UanPhyRealCloud);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealCloudPerGenDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealCloudCalcSinrDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealCloudCalcSinrFhFsk);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealCloudPerUmodem);


/*************** UanPhyRealCloudCalcSinrDefault definition *****************/
UanPhyRealCloudCalcSinrDefault::UanPhyRealCloudCalcSinrDefault ()
{

}
UanPhyRealCloudCalcSinrDefault::~UanPhyRealCloudCalcSinrDefault ()
{

}

TypeId
UanPhyRealCloudCalcSinrDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealCloudCalcSinrDefault")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealCloudCalcSinrDefault> ()
  ;
  return tid;
}

double
UanPhyRealCloudCalcSinrDefault::CalcSinrDb (Ptr<Packet> pkt,
                                   Time arrTime,
                                   double rxPowerDb,
                                   double ambNoiseDb,
                                   UanTxMode mode,
                                   UanPdp pdp,
                                   const UanTransducer::ArrivalList &arrivalList) const
{
  if (mode.GetModType () == UanTxMode::OTHER)
    {
      NS_LOG_WARN ("Calculating SINR for unsupported modulation type");
    }

  double intKp = -DbToKp (rxPowerDb); // This packet is in the arrivalList
  UanTransducer::ArrivalList::const_iterator it = arrivalList.begin ();
  for (; it != arrivalList.end (); it++)
    {
      intKp += DbToKp (it->GetRxPowerDb ());
    }

  double totalIntDb = KpToDb (intKp + DbToKp (ambNoiseDb));

  NS_LOG_DEBUG ("Calculating SINR:  RxPower = " << rxPowerDb << " dB.  Number of interferers = " << arrivalList.size () << "  Interference + noise power = " << totalIntDb << " dB.  SINR = " << rxPowerDb - totalIntDb << " dB.");
  return rxPowerDb - totalIntDb;
}

/*************** UanPhyRealCloudCalcSinrFhFsk definition *****************/
UanPhyRealCloudCalcSinrFhFsk::UanPhyRealCloudCalcSinrFhFsk ()
{

}
UanPhyRealCloudCalcSinrFhFsk::~UanPhyRealCloudCalcSinrFhFsk ()
{

}

TypeId
UanPhyRealCloudCalcSinrFhFsk::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealCloudCalcSinrFhFsk")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealCloudCalcSinrFhFsk> ()
    .AddAttribute ("NumberOfHops",
                   "Number of frequencies in hopping pattern.",
                   UintegerValue (13),
                   MakeUintegerAccessor (&UanPhyRealCloudCalcSinrFhFsk::m_hops),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}
double
UanPhyRealCloudCalcSinrFhFsk::CalcSinrDb (Ptr<Packet> pkt,
                                 Time arrTime,
                                 double rxPowerDb,
                                 double ambNoiseDb,
                                 UanTxMode mode,
                                 UanPdp pdp,
                                 const UanTransducer::ArrivalList &arrivalList) const
{
  if (mode.GetModType () != UanTxMode::FSK)
    {
      NS_LOG_WARN ("Calculating SINR for unsupported mode type");
    }



  double ts = 1.0 / mode.GetPhyRateSps ();
  double clearingTime = (m_hops - 1.0) * ts;
  double csp = pdp.SumTapsFromMaxNc (Seconds (0), Seconds (ts));

  // Get maximum arrival offset
  double maxAmp = -1;
  double maxTapDelay = 0.0;
  UanPdp::Iterator pit = pdp.GetBegin ();
  for (; pit != pdp.GetEnd (); pit++)
    {
      if (std::abs (pit->GetAmp ()) > maxAmp)
        {
          maxAmp = std::abs (pit->GetAmp ());
          maxTapDelay = pit->GetDelay ().GetSeconds ();
        }
    }


  double effRxPowerDb = rxPowerDb + KpToDb (csp);

  double isiUpa = rxPowerDb * pdp.SumTapsFromMaxNc (Seconds (ts + clearingTime), Seconds (ts));
  UanTransducer::ArrivalList::const_iterator it = arrivalList.begin ();
  double intKp = -DbToKp (effRxPowerDb);
  for (; it != arrivalList.end (); it++)
    {
      UanPdp intPdp = it->GetPdp ();
      double tDelta = std::abs (arrTime.GetSeconds () + maxTapDelay - it->GetArrivalTime ().GetSeconds ());
      // We want tDelta in terms of a single symbol (i.e. if tDelta = 7.3 symbol+clearing
      // times, the offset in terms of the arriving symbol power is
      // 0.3 symbol+clearing times.

      int32_t syms = (uint32_t)( (double) tDelta / (ts + clearingTime));
      tDelta = tDelta - syms * (ts + clearingTime);

      // Align to pktRx
      if (arrTime + Seconds (maxTapDelay)  > it->GetArrivalTime ())
        {
          tDelta = ts + clearingTime - tDelta;
        }

      double intPower = 0.0;
      if (tDelta < ts)
        {
          intPower += intPdp.SumTapsNc (Seconds (0), Seconds (ts - tDelta));
          intPower += intPdp.SumTapsNc (Seconds (ts - tDelta + clearingTime),
                                        Seconds (2 * ts - tDelta + clearingTime));
        }
      else
        {
          Time start = Seconds (ts + clearingTime - tDelta);
          Time end = start + Seconds (ts);
          intPower += intPdp.SumTapsNc (start, end);

          start = start + Seconds (ts + clearingTime);
          end = start + Seconds (ts);
          intPower += intPdp.SumTapsNc (start, end);
        }
      intKp += DbToKp (it->GetRxPowerDb ()) * intPower;
    }

  double totalIntDb = KpToDb (isiUpa + intKp + DbToKp (ambNoiseDb));

  NS_LOG_DEBUG ("Calculating SINR:  RxPower = " << rxPowerDb << " dB.  Effective Rx power " << effRxPowerDb << " dB.  Number of interferers = " << arrivalList.size () << "  Interference + noise power = " << totalIntDb << " dB.  SINR = " << effRxPowerDb - totalIntDb << " dB.");
  return effRxPowerDb - totalIntDb;
}

/*************** UanPhyRealCloudPerGenDefault definition *****************/
UanPhyRealCloudPerGenDefault::UanPhyRealCloudPerGenDefault ()
{

}

UanPhyRealCloudPerGenDefault::~UanPhyRealCloudPerGenDefault ()
{

}
TypeId
UanPhyRealCloudPerGenDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealCloudPerGenDefault")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealCloudPerGenDefault> ()
    .AddAttribute ("Threshold", "SINR cutoff for good packet reception.",
                   DoubleValue (8),
                   MakeDoubleAccessor (&UanPhyRealCloudPerGenDefault::m_thresh),
                   MakeDoubleChecker<double> ());
  return tid;
}


// Default PER calculation simply compares SINR to a threshold which is configurable
// via an attribute.
double
UanPhyRealCloudPerGenDefault::CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode)
{
  if (sinrDb >= m_thresh)
    {
      return 0;
    }
  else
    {
      return 1;
    }
}

/*************** UanPhyRealCloudPerUmodem definition *****************/
UanPhyRealCloudPerUmodem::UanPhyRealCloudPerUmodem ()
{

}
UanPhyRealCloudPerUmodem::~UanPhyRealCloudPerUmodem ()
{

}

TypeId UanPhyRealCloudPerUmodem::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealCloudPerUmodem")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealCloudPerUmodem> ()
  ;
  return tid;
}

double
UanPhyRealCloudPerUmodem::NChooseK (uint32_t n, uint32_t k)
{
  double result;

  result = 1.0;

  for (uint32_t i = std::max (k,n - k) + 1; i <= n; ++i)
    {
      result *= i;
    }

  for (uint32_t i = 2; i <= std::min (k,n - k); ++i)
    {
      result /= i;
    }

  return result;
}

double
UanPhyRealCloudPerUmodem::CalcPer (Ptr<Packet> pkt, double sinr, UanTxMode mode)
{
  uint32_t d[] =
  { 12, 14, 16, 18, 20, 22, 24, 26, 28 };
  double Bd[] =
  {
    33, 281, 2179, 15035LLU, 105166LLU, 692330LLU, 4580007LLU, 29692894LLU,
    190453145LLU
  };

  // double Rc = 1.0 / 2.0;
  double ebno = std::pow (10.0, sinr / 10.0);
  double perror = 1.0 / (2.0 + ebno);
  double P[9];

  if (sinr >= 10)
    {
      return 0;
    }
  if (sinr <= 6)
    {
      return 1;
    }

  for (uint32_t r = 0; r < 9; r++)
    {
      double sumd = 0;
      for (uint32_t k = 0; k < d[r]; k++)
        {
          sumd = sumd + NChooseK (d[r] - 1 + k, k) * std::pow (1 - perror, (double) k);
        }
      P[r] = std::pow (perror, (double) d[r]) * sumd;

    }

  double Pb = 0;
  for (uint32_t r = 0; r < 8; r++)
    {
      Pb = Pb + Bd[r] * P[r];
    }

  // cout << "Pb = " << Pb << endl;
  uint32_t bits = pkt->GetSize () * 8;

  double Ppacket = 1;
  double temp = NChooseK (bits, 0);
  temp *= std::pow ( (1 - Pb), (double) bits);
  Ppacket -= temp;
  temp = NChooseK (288, 1) * Pb * std::pow ( (1 - Pb), bits - 1.0);
  Ppacket -= temp;

  if (Ppacket > 1)
    {
      return 1;
    }
  else
    {
      return Ppacket;
    }
}

/*************** UanPhyRealCloud definition *****************/
UanPhyRealCloud::UanPhyRealCloud ()
  : UanPhy (),
    m_state (IDLE),
    m_channel (0),
    m_transducer (0),
    m_device (0),
    m_mac (0),
    m_rxGainDb (0),
    m_txPwrDb (0),
    m_rxThreshDb (0),
    m_ccaThreshDb (0),
    m_pktRx (0),
    m_pktTx (0),
    m_cleared (false)
{
  m_pg = CreateObject<UniformRandomVariable> ();

  m_energyCallback.Nullify ();
  NS_LOG_DEBUG ("UanPhyRealCloud:::~~~");

//初始化函数，参数计算，连接Modem
  Simulator::Schedule (Seconds (0.5), &UanPhyRealCloud::Init, this);
}

UanPhyRealCloud::~UanPhyRealCloud ()
{

}

void
UanPhyRealCloud::Clear ()
{
  NS_LOG_FUNCTION(this);
  NS_LOG_UNCOND("NodeId:"<<m_nodeId<<"--PhyRealNew-- txPktNum:"<<txPktNum<<"--rxPktNum:"<<rxPktNum);
  if (m_cleared)
    {
      return;
    }
  m_cleared = true;
  m_listeners.clear ();
  if (m_channel)
    {
      m_channel->Clear ();
      m_channel = 0;
    }
  if (m_transducer)
    {
      m_transducer->Clear ();
      m_transducer = 0;
    }
  if (m_device)
    {
      m_device->Clear ();
      m_device = 0;
    }
  if (m_mac)
    {
      m_mac->Clear ();
      m_mac = 0;
    }
  if (m_per)
    {
      m_per->Clear ();
      m_per = 0;
    }
  if (m_sinr)
    {
      m_sinr->Clear ();
      m_sinr = 0;
    }
  m_pktRx = 0;
  m_pktTx = 0;
  r_state = DISABLED;
  //stop ns3 Task
  UanEmulationManager* uanEmuManagerPtr = UanEmulationManager::getInstance();
  uanEmuManagerPtr->stopNs3Task();//tell uanserver to stop ns3 Task

    
  close(m_uanServerFd);//关闭连接

  
}

void
UanPhyRealCloud::DoDispose ()
{
  Clear ();
  m_energyCallback.Nullify ();
  UanPhy::DoDispose ();
}

UanModesList
UanPhyRealCloud::GetDefaultModes (void)
{
  UanModesList l;
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::FSK,80,80,22000,4000,13,"FSK"));
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::PSK,200, 200, 22000, 4000, 4, "QPSK"));
  return l;
}
TypeId
UanPhyRealCloud::GetTypeId (void)
{

  static TypeId tid = TypeId ("ns3::UanPhyRealCloud")
    .SetParent<UanPhy> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealCloud> ()
    .AddAttribute ("CcaThreshold",
                   "Aggregate energy of incoming signals to move to CCA Busy state dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealCloud::m_ccaThreshDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxThreshold",
                   "Required SNR for signal acquisition in dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealCloud::m_rxThreshDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute  ("PacketSize",                                
		   "The size of Packet in Phy",
		   UintegerValue(1024),//
		   MakeUintegerAccessor (&UanPhyRealCloud::packetSize),
	           MakeUintegerChecker< uint16_t> ())

    .AddAttribute ("TxPower",
                   "Transmission output power in dB.",
                   DoubleValue (190),
                   MakeDoubleAccessor (&UanPhyRealCloud::m_txPwrDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Gain added to incoming signal at receiver.",
                   DoubleValue (0),
                   MakeDoubleAccessor (&UanPhyRealCloud::m_rxGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SupportedModes",
                   "List of modes supported by this PHY.",
                   UanModesListValue (UanPhyRealCloud::GetDefaultModes ()),
                   MakeUanModesListAccessor (&UanPhyRealCloud::m_modes),
                   MakeUanModesListChecker () )
    .AddAttribute ("PerModel",
                   "Functor to calculate PER based on SINR and TxMode.",
                   StringValue ("ns3::UanPhyRealCloudPerGenDefault"),
                   MakePointerAccessor (&UanPhyRealCloud::m_per),
                   MakePointerChecker<UanPhyPer> ())
    .AddAttribute ("SinrModel",
                   "Functor to calculate SINR based on pkt arrivals and modes.",
                   StringValue ("ns3::UanPhyRealCloudCalcSinrDefault"),
                   MakePointerAccessor (&UanPhyRealCloud::m_sinr),
                   MakePointerChecker<UanPhyCalcSinr> ())

    .AddTraceSource ("RxOk",
                     "A packet was received successfully.",
                     MakeTraceSourceAccessor (&UanPhyRealCloud::m_rxOkLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("RxError",
                     "A packet was received unsuccessfully.",
                     MakeTraceSourceAccessor (&UanPhyRealCloud::m_rxErrLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("Tx",
                     "Packet transmission beginning.",
                     MakeTraceSourceAccessor (&UanPhyRealCloud::m_txLogger),
                     "ns3::UanPhy::TracedCallback")
  ;
  return tid;

}

void
UanPhyRealCloud::SetEnergyModelCallback (DeviceEnergyModel::ChangeStateCallback cb)
{
  NS_LOG_FUNCTION (this);
  m_energyCallback = cb;
}

void
UanPhyRealCloud::UpdatePowerConsumption (const State state)
{
  NS_LOG_FUNCTION (this);

  if (!m_energyCallback.IsNull ())
    {
      m_energyCallback (state);
    }
}

void
UanPhyRealCloud::EnergyDepletionHandler ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("Energy depleted at node " << m_device->GetNode ()->GetId () <<
                ", stopping rx/tx activities");
  
  m_state = DISABLED;
  if(m_txEndEvent.IsRunning ())
    {
      Simulator::Cancel (m_txEndEvent);
      NotifyTxDrop (m_pktTx);
      m_pktTx = 0;
    }
  if(m_rxEndEvent.IsRunning ())
    {
      Simulator::Cancel (m_rxEndEvent);
      NotifyRxDrop (m_pktRx);
      m_pktRx = 0;
    }
}

void
UanPhyRealCloud::EnergyRechargeHandler ()
{
  NS_LOG_FUNCTION (this);
  NS_LOG_DEBUG ("Energy recharged at node " << m_device->GetNode ()->GetId () <<
                ", restoring rx/tx activities");

  m_state = IDLE;
}







 /*
  *  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
  *     about send packet and receive ,to/from modem   * * * * * * * * * *
  *  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
  */
void
UanPhyRealCloud::Init()
{
    NS_LOG_FUNCTION(this);
    Ptr< Node > m_node = m_device->GetNode();
    m_nodeId = m_node->GetId();
    ConnectUanServer();//初始化时就连接Uan server
    r_state = IDLE;
    Simulator::Schedule (Seconds (0), &UanPhyRealCloud::StartRX, this);//0.5s后开启接收状态
}

void
UanPhyRealCloud::StartRxPacket (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode, UanPdp pdp)//by LCH
{
    NS_LOG_FUNCTION (this);
    double txdelay = 0;
    uint32_t dstNodeId = (m_device->GetNode() )->GetId();
    
   Simulator::ScheduleWithContext (dstNodeId ,Seconds (txdelay), &UanPhyRealCloud::RxEndEvent, this, pkt, rxPowerDb, txMode);
   
}

void
UanPhyRealCloud::RxEndEvent (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode)//by LCH
{
  NS_LOG_FUNCTION (this);
  UanHeaderCommon header;
   pkt->RemoveHeader (header);
   pkt->AddHeader (header);//为了有包头 信息，先剥除，再添加
  m_rxOkLogger (pkt, m_minRxSinrDb, txMode);//tracing
  m_recOkCb (pkt, 0, txMode);
  rxPktNum ++;

}

void
UanPhyRealCloud::SendPacket (Ptr<Packet> pkt, uint32_t modeNum)//by LCH
{
  NS_LOG_DEBUG ("PHY " << m_mac->GetAddress () << ": Transmitting packet");
  NS_LOG_DEBUG ("send:packet size:" << pkt->GetSize());

  if(r_state == TX || r_state == RX)//
  {
    NS_LOG_DEBUG("Channel is busy!Can not transmit!");
    return; //
  }
  

  //double txdelay = 0;
  //m_callEndEvent = Simulator::Schedule (Seconds ( txdelay), &UanPhyRealCloud::StartTX, this, pkt);
  /*终止接收*/
    //UanEmuRecvSend::sendUanOnlyConstHeader(m_uanServerFd, kns3RecvThreadStop,m_nodeId);
    //recvthr->Join();//等待接收线程结束

  //开启发送线程
  m_pktTx = pkt;
  r_state = TX;
  /*
   sendthr = Create<SystemThread> (
      MakeCallback (&UanPhyRealCloud::sendDataThread, this));
  sendthr ->Start ();*/
  
  
  UanPhyRealCloud::sendDataThread();
  //m_txLogger (pkt, m_txPwrDb, m_pktRxMode);//trcaing
  txPktNum ++;
}

void
UanPhyRealCloud::sendDataThread(void)
{
    NS_LOG_DEBUG("send thread start!");
    
    /*终止接收*/
    /*
    UanEmuRecvSend::sendUanOnlyConstHeader(m_uanServerFd, kns3RecvThreadStop,m_nodeId);
    recvthr->Join();//等待接收线程结束
    */
   
    /*传输数据*/
    int len_packet = m_pktTx->GetSize ();//包的实际长度          
  
    /* 多截断，少补零，到1024*/

    char * newPkt = (char *)malloc(1024);
    m_pktTx->CopyData((uint8_t *)(newPkt),len_packet);

    std::string newNs3Pkt = std::string(newPkt,newPkt + len_packet);
    UanEmuRecvSend::sendUanServerNs3Pkt(m_uanServerFd, newNs3Pkt,m_nodeId);
    
    /*
    while(true){
        UanEmuPacket recvpkt = UanEmuRecvSend::recvUanServerPkt(m_uanServerFd);
        if(recvpkt.m_conHeader._dataType == kns3SendFinished){
            NS_LOG_DEBUG("Modem send finish!");
            break;
        }else{
            NS_LOG_DEBUG("Go on: Wait for the Modem send finish!");
        }
    }*/
    
    //r_state = IDLE;
    //Simulator::Schedule (Seconds (0), &UanPhyRealCloud::StartRX, this);
    //m_mac -> NotifyTxEnd();//By LCH
    NS_LOG_DEBUG("send thread finish!");
    free(newPkt);
}



void 
UanPhyRealCloud::StartRX()
{
  NS_LOG_FUNCTION(this);
	NS_LOG_DEBUG("Modem ready to receive!");
  recvthr = Create<SystemThread> (MakeCallback (&UanPhyRealCloud::recvDataThread, this));
  recvthr ->Start ();
  
}

//class UanPhyRealCloud
void 
UanPhyRealCloud::recvDataThread(void )//该函数必须由新的线程执行，new thread
{
    NS_LOG_DEBUG("recvDataThread!nFD:"<< m_uanServerFd);
    while(true)//
    {
        UanEmuPacket recvpkt = UanEmuRecvSend::recvUanServerPkt(m_uanServerFd);
        if(recvpkt.m_conHeader._dataType == kns3Packet){
            NS_LOG_DEBUG("recvDataThread: phy recv new ns3 pkt!");
            int contentLen = recvpkt.m_conHeader._sendContentLen;
            m_pktRx = Create<Packet> ((uint8_t *)(recvpkt.m_content.c_str()), contentLen);//
            StartRxPacket (m_pktRx, 0, m_pktRxMode,m_pktRxPdp);//上传packet到mac 
        }else if(recvpkt.m_conHeader._dataType == kns3TaskStop)
        {
            NS_LOG_DEBUG("NS3 task stop!");
            break;
        }else if(recvpkt.m_conHeader._dataType == kns3RecvThreadStop){

            //NS_LOG_DEBUG("kns3RecvThreadStop: recv thread stop!");
            //break;
        }else if(recvpkt.m_conHeader._dataType == kns3SendFinished){
            NS_LOG_DEBUG("Get kns3SendFinished:Modem send finish!, set state Idle");
            r_state = IDLE;
            m_mac -> NotifyTxEnd();//By LCH
        }else{
            NS_LOG_DEBUG("recvDataThread: recv data exception!");
        }
    }//while  
    return ;
}

bool UanPhyRealCloud::IsrStateRX(void)
{
       return  (r_state ==RX);
}
void UanPhyRealCloud::setrStateIDLE(void)
{
    r_state = IDLE;
}

int UanPhyRealCloud::readnFd(void)
{
       return m_uanServerFd;
}

void UanPhyRealCloud::ConnectUanServer(void)
{
    NS_LOG_FUNCTION(this);

  /*
    Ptr< Node > m_node = m_device->GetNode();
    m_nodeId = m_node->GetId();
    //NS_LOG_DEBUG("Node ID:"<<nodeId);
    nFd = fd_list[nodeId];*/

    struct sockaddr_in serverAddr;
    m_uanServerFd = socket(AF_INET, SOCK_STREAM, 0);
    if(m_uanServerFd < 0){
        NS_ASSERT_MSG(0,"fail to create socket");
    }

    bzero(&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port   = htons(m_uanServerPort);
    serverAddr.sin_addr.s_addr = inet_addr(m_uanServerIp.c_str());

    int ret = connect(m_uanServerFd, (struct sockaddr*)(&serverAddr), sizeof(serverAddr));
    if(ret < 0){
        NS_ASSERT_MSG(0,"fail to connect to the uanserver");
    }	else{
        NS_LOG_DEBUG("connect to the uanserver ok!");
    }

    UanEmuRecvSend::sendUanOnlyConstHeader(m_uanServerFd,kns3NodeInfo,m_nodeId);
    NS_LOG_DEBUG("Send to Uan server:kns3NodeInfo!");

}

/**********************************/

void
UanPhyRealCloud::TxEndEvent ()
{
  if (m_state == SLEEP || m_state == DISABLED)
    {
      NS_LOG_DEBUG ("Transmission ended but node sleeping or dead");
      return;
    }

  NS_ASSERT (m_state == TX);
  if (GetInterferenceDb ( (Ptr<Packet>) 0) > m_ccaThreshDb)
    {
      m_state = CCABUSY;
      NotifyListenersCcaStart ();
    }
  else
    {
      m_state = IDLE;
    }
  UpdatePowerConsumption (IDLE);
}

void
UanPhyRealCloud::RegisterListener (UanPhyListener *listener)
{
  m_listeners.push_back (listener);
}




void
UanPhyRealCloud::SetReceiveOkCallback (RxOkCallback cb)
{
  m_recOkCb = cb;
}

void
UanPhyRealCloud::SetReceiveErrorCallback (RxErrCallback cb)
{
  m_recErrCb = cb;
}
bool
UanPhyRealCloud::IsStateSleep (void)
{
  return m_state == SLEEP;
}
bool
UanPhyRealCloud::IsStateIdle (void)
{
  return m_state == IDLE;
}
bool
UanPhyRealCloud::IsStateBusy (void)
{
  return !IsStateIdle () && !IsStateSleep ();
}
bool
UanPhyRealCloud::IsStateRx (void)
{
  return m_state == RX;
}
bool
UanPhyRealCloud::IsStateTx (void)
{
  return m_state == TX;
}

bool
UanPhyRealCloud::IsStateCcaBusy (void)
{
  return m_state == CCABUSY;
}


void
UanPhyRealCloud::SetRxGainDb (double gain)
{
  m_rxGainDb = gain;

}
void
UanPhyRealCloud::SetTxPowerDb (double txpwr)
{
  m_txPwrDb = txpwr;
}
void
UanPhyRealCloud::SetRxThresholdDb (double thresh)
{
  m_rxThreshDb = thresh;
}
void
UanPhyRealCloud::SetCcaThresholdDb (double thresh)
{
  m_ccaThreshDb = thresh;
}
double
UanPhyRealCloud::GetRxGainDb (void)
{
  return m_rxGainDb;
}
double
UanPhyRealCloud::GetTxPowerDb (void)
{
  return m_txPwrDb;

}
double
UanPhyRealCloud::GetRxThresholdDb (void)
{
  return m_rxThreshDb;
}
double
UanPhyRealCloud::GetCcaThresholdDb (void)
{
  return m_ccaThreshDb;
}

Ptr<UanChannel>
UanPhyRealCloud::GetChannel (void) const
{
  return m_channel;
}

Ptr<UanNetDevice>
UanPhyRealCloud::GetDevice (void) const
{
  return m_device;
}

Ptr<UanTransducer>
UanPhyRealCloud::GetTransducer (void)
{
  return m_transducer;
}
void
UanPhyRealCloud::SetChannel (Ptr<UanChannel> channel)
{
  m_channel = channel;
}

void
UanPhyRealCloud::SetDevice (Ptr<UanNetDevice> device)
{
  m_device = device;
}

void
UanPhyRealCloud::SetMac (Ptr<UanMac> mac)
{
  m_mac = mac;
}

void
UanPhyRealCloud::SetTransducer (Ptr<UanTransducer> trans)
{
  m_transducer = trans;
  m_transducer->AddPhy (this);
}

void
UanPhyRealCloud::SetSleepMode (bool sleep)
{
  if (sleep)
    {
      m_state = SLEEP;
      if (!m_energyCallback.IsNull ())
        {
          m_energyCallback (SLEEP);
        }
    }
  else if (m_state == SLEEP)
    {
      if (GetInterferenceDb ((Ptr<Packet>) 0) > m_ccaThreshDb)
        {
          m_state = CCABUSY;
          NotifyListenersCcaStart ();
        }
      else
        {
          m_state = IDLE;
        }

      if (!m_energyCallback.IsNull ())
        {
          m_energyCallback (IDLE);
        }
    }
}

int64_t
UanPhyRealCloud::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_pg->SetStream (stream);
  return 1;
}

void
UanPhyRealCloud::NotifyTransStartTx (Ptr<Packet> packet, double txPowerDb, UanTxMode txMode)
{
  if (m_pktRx)
    {
      m_minRxSinrDb = -1e30;
    }
}

void
UanPhyRealCloud::NotifyIntChange (void)
{
  if (m_state == CCABUSY && GetInterferenceDb (Ptr<Packet> ()) < m_ccaThreshDb)
    {
      m_state = IDLE;
      NotifyListenersCcaEnd ();
    }
}

double
UanPhyRealCloud::CalculateSinrDb (Ptr<Packet> pkt, Time arrTime, double rxPowerDb, UanTxMode mode, UanPdp pdp)
{
  double noiseDb = m_channel->GetNoiseDbHz ( (double) mode.GetCenterFreqHz () / 1000.0) + 10 * std::log10 (mode.GetBandwidthHz ());
  return m_sinr->CalcSinrDb (pkt, arrTime, rxPowerDb, noiseDb, mode, pdp, m_transducer->GetArrivalList ());
}

double
UanPhyRealCloud::GetInterferenceDb (Ptr<Packet> pkt)
{

  const UanTransducer::ArrivalList &arrivalList = m_transducer->GetArrivalList ();

  UanTransducer::ArrivalList::const_iterator it = arrivalList.begin ();

  double interfPower = 0;

  for (; it != arrivalList.end (); it++)
    {
      if (pkt != it->GetPacket ())
        {
          interfPower += DbToKp (it->GetRxPowerDb ());
        }
    }

  return KpToDb (interfPower);

}

double
UanPhyRealCloud::DbToKp (double db)
{
  return std::pow (10, db / 10.0);
}
double
UanPhyRealCloud::KpToDb (double kp)
{
  return 10 * std::log10 (kp);
}

void
UanPhyRealCloud::NotifyListenersRxStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxStart ();
    }

}
void
UanPhyRealCloud::NotifyListenersRxGood (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndOk ();
    }
}
void
UanPhyRealCloud::NotifyListenersRxBad (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndError ();
    }
}
void
UanPhyRealCloud::NotifyListenersCcaStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaStart ();
    }
}
void
UanPhyRealCloud::NotifyListenersCcaEnd (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaEnd ();
    }
}

void
UanPhyRealCloud::NotifyListenersTxStart (Time duration)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyTxStart (duration);
    }
}

uint32_t
UanPhyRealCloud::GetNModes (void)
{
  return m_modes.GetNModes ();
}

UanTxMode
UanPhyRealCloud::GetMode (uint32_t n)
{
 
  //NS_ASSERT (n < m_modes.GetNModes ());
  //n=1;
  return m_modes[n];
}

Ptr<Packet>
UanPhyRealCloud::GetPacketRx (void) const
{
  return m_pktRx;
}


} // namespace ns3
