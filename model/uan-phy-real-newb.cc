/*   ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
 **    uan-phy-real-newb.cc                     *
 **    Created on 2018/7/18               *
 **    Author:lch                          *
 **    For:connecting with real channel    *
 **  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
*/
//尝试物理层每次只发一帧，只收一帧

#include "uan-phy-real-newb.h"
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

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("UanPhyRealNewb");

NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewb);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewbPerGenDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewbCalcSinrDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewbCalcSinrFhFsk);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewbPerUmodem);


/*************** UanPhyRealNewbCalcSinrDefault definition *****************/
UanPhyRealNewbCalcSinrDefault::UanPhyRealNewbCalcSinrDefault ()
{

}
UanPhyRealNewbCalcSinrDefault::~UanPhyRealNewbCalcSinrDefault ()
{

}

TypeId
UanPhyRealNewbCalcSinrDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewbCalcSinrDefault")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewbCalcSinrDefault> ()
  ;
  return tid;
}

double
UanPhyRealNewbCalcSinrDefault::CalcSinrDb (Ptr<Packet> pkt,
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

/*************** UanPhyRealNewbCalcSinrFhFsk definition *****************/
UanPhyRealNewbCalcSinrFhFsk::UanPhyRealNewbCalcSinrFhFsk ()
{

}
UanPhyRealNewbCalcSinrFhFsk::~UanPhyRealNewbCalcSinrFhFsk ()
{

}

TypeId
UanPhyRealNewbCalcSinrFhFsk::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewbCalcSinrFhFsk")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewbCalcSinrFhFsk> ()
    .AddAttribute ("NumberOfHops",
                   "Number of frequencies in hopping pattern.",
                   UintegerValue (13),
                   MakeUintegerAccessor (&UanPhyRealNewbCalcSinrFhFsk::m_hops),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}
double
UanPhyRealNewbCalcSinrFhFsk::CalcSinrDb (Ptr<Packet> pkt,
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

/*************** UanPhyRealNewbPerGenDefault definition *****************/
UanPhyRealNewbPerGenDefault::UanPhyRealNewbPerGenDefault ()
{

}

UanPhyRealNewbPerGenDefault::~UanPhyRealNewbPerGenDefault ()
{

}
TypeId
UanPhyRealNewbPerGenDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewbPerGenDefault")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewbPerGenDefault> ()
    .AddAttribute ("Threshold", "SINR cutoff for good packet reception.",
                   DoubleValue (8),
                   MakeDoubleAccessor (&UanPhyRealNewbPerGenDefault::m_thresh),
                   MakeDoubleChecker<double> ());
  return tid;
}


// Default PER calculation simply compares SINR to a threshold which is configurable
// via an attribute.
double
UanPhyRealNewbPerGenDefault::CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode)
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

/*************** UanPhyRealNewbPerUmodem definition *****************/
UanPhyRealNewbPerUmodem::UanPhyRealNewbPerUmodem ()
{

}
UanPhyRealNewbPerUmodem::~UanPhyRealNewbPerUmodem ()
{

}

TypeId UanPhyRealNewbPerUmodem::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewbPerUmodem")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewbPerUmodem> ()
  ;
  return tid;
}

double
UanPhyRealNewbPerUmodem::NChooseK (uint32_t n, uint32_t k)
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
UanPhyRealNewbPerUmodem::CalcPer (Ptr<Packet> pkt, double sinr, UanTxMode mode)
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

/*************** UanPhyRealNewb definition *****************/
UanPhyRealNewb::UanPhyRealNewb ()
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
  NS_LOG_DEBUG ("UanPhyRealNewb");
 // Init();//初始化函数，参数计算，连接Modem
  Simulator::Schedule (Seconds (0.5), &UanPhyRealNewb::Init, this);
}

UanPhyRealNewb::~UanPhyRealNewb ()
{

}

void
UanPhyRealNewb::Clear ()
{
  NS_LOG_FUNCTION(this);
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
  free(tcpsendbuffer);
  free(tcprecvbuffer);
  free(recvbuffer);
  close(nFd);//关闭连接
}

void
UanPhyRealNewb::DoDispose ()
{
  Clear ();
  m_energyCallback.Nullify ();
  UanPhy::DoDispose ();
}

UanModesList
UanPhyRealNewb::GetDefaultModes (void)
{
  UanModesList l;
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::FSK,80,80,22000,4000,13,"FSK"));
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::PSK,200, 200, 22000, 4000, 4, "QPSK"));
  return l;
}
TypeId
UanPhyRealNewb::GetTypeId (void)
{

  static TypeId tid = TypeId ("ns3::UanPhyRealNewb")
    .SetParent<UanPhy> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewb> ()
    .AddAttribute ("CcaThreshold",
                   "Aggregate energy of incoming signals to move to CCA Busy state dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealNewb::m_ccaThreshDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxThreshold",
                   "Required SNR for signal acquisition in dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealNewb::m_rxThreshDb),
                   MakeDoubleChecker<double> ())
	.AddAttribute  ("PacketSize",                                  //by LCH
			      "The size of Packet in Phy",
			      UintegerValue(1024),//???
			      MakeUintegerAccessor (&UanPhyRealNewb::packetSize),
			      MakeUintegerChecker< uint16_t> ())
	 .AddAttribute  ("ServerAddr",                                  //by LCH
			 "The Ip of the Modem",
			UintegerValue(0),//???
			MakeUintegerAccessor (&UanPhyRealNewb::serverAddr),
			MakeUintegerChecker< uint32_t> ())
    .AddAttribute ("TxPower",
                   "Transmission output power in dB.",
                   DoubleValue (190),
                   MakeDoubleAccessor (&UanPhyRealNewb::m_txPwrDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Gain added to incoming signal at receiver.",
                   DoubleValue (0),
                   MakeDoubleAccessor (&UanPhyRealNewb::m_rxGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SupportedModes",
                   "List of modes supported by this PHY.",
                   UanModesListValue (UanPhyRealNewb::GetDefaultModes ()),
                   MakeUanModesListAccessor (&UanPhyRealNewb::m_modes),
                   MakeUanModesListChecker () )
    .AddAttribute ("PerModel",
                   "Functor to calculate PER based on SINR and TxMode.",
                   StringValue ("ns3::UanPhyRealNewbPerGenDefault"),
                   MakePointerAccessor (&UanPhyRealNewb::m_per),
                   MakePointerChecker<UanPhyPer> ())
    .AddAttribute ("SinrModel",
                   "Functor to calculate SINR based on pkt arrivals and modes.",
                   StringValue ("ns3::UanPhyRealNewbCalcSinrDefault"),
                   MakePointerAccessor (&UanPhyRealNewb::m_sinr),
                   MakePointerChecker<UanPhyCalcSinr> ())

    .AddTraceSource ("RxOk",
                     "A packet was received successfully.",
                     MakeTraceSourceAccessor (&UanPhyRealNewb::m_rxOkLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("RxError",
                     "A packet was received unsuccessfully.",
                     MakeTraceSourceAccessor (&UanPhyRealNewb::m_rxErrLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("Tx",
                     "Packet transmission beginning.",
                     MakeTraceSourceAccessor (&UanPhyRealNewb::m_txLogger),
                     "ns3::UanPhy::TracedCallback")
  ;
  return tid;

}

void
UanPhyRealNewb::SetEnergyModelCallback (DeviceEnergyModel::ChangeStateCallback cb)
{
  NS_LOG_FUNCTION (this);
  m_energyCallback = cb;
}

void
UanPhyRealNewb::UpdatePowerConsumption (const State state)
{
  NS_LOG_FUNCTION (this);

  if (!m_energyCallback.IsNull ())
    {
      m_energyCallback (state);
    }
}

void
UanPhyRealNewb::EnergyDepletionHandler ()
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
UanPhyRealNewb::EnergyRechargeHandler ()
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
UanPhyRealNewb::Init()
{
    NS_LOG_FUNCTION(this);
    
    sendBytesNum = frameNum * dataPerFrame;
    zeroBytesNum = sendBytesNum - dataSize; // 帧补零数,这里是补零到N帧数据
    sendPackageNum = static_cast<int>(ceil((static_cast<double>(sendBytesNum)) / (static_cast<double>(dataPerBag))));
    
    setModemIP();
    ConnectModem();//初始化时就连接通信机
    r_state = IDLE;
    Simulator::Schedule (Seconds (0.5), &UanPhyRealNewb::StartRX, this);//0.5s后开启接收状态
}

void
UanPhyRealNewb::StartRxPacket (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode, UanPdp pdp)//by LCH
{
    NS_LOG_FUNCTION (this);
    double txdelay = 0;
    uint32_t dstNodeId = (m_device->GetNode() )->GetId();
   Simulator::ScheduleWithContext (dstNodeId ,Seconds (txdelay), &UanPhyRealNewb::RxEndEvent, this, pkt, rxPowerDb, txMode);
   
}

void
UanPhyRealNewb::RxEndEvent (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode)//by LCH
{
  NS_LOG_FUNCTION (this);
  UanHeaderCommon header;
   pkt->RemoveHeader (header);
   pkt->AddHeader (header);//为了有包头 信息，先剥除，再添加
  m_rxOkLogger (pkt, m_minRxSinrDb, txMode);//tracing
  m_recOkCb (pkt, 0, txMode);
  

}

void
UanPhyRealNewb::SendPacket (Ptr<Packet> pkt, uint32_t modeNum)//by LCH
{
  NS_LOG_DEBUG ("PHY " << m_mac->GetAddress () << ": Transmitting packet");
  NS_LOG_DEBUG ("send:packet size:" << pkt->GetSize());

  if(r_state == TX || r_state == RX)//
  {
    NS_LOG_DEBUG("Channel is busy!Can not transmit!");
    return; //
  }
  
  
  //double txdelay = 0;
  //m_callEndEvent = Simulator::Schedule (Seconds ( txdelay), &UanPhyRealNewb::StartTX, this, pkt);
  len_pktTx = pkt ->GetSize();//获取待发packet的实际长度（内容+包头）
  //开启发送线程
  r_state = TX;
 
  sendthr = Create<SystemThread> (
      MakeCallback (&UanPhyRealNewb::sendDataThread_1f, this));

  sendthr ->Start ();
  m_pktTx = pkt;
  
  m_txLogger (pkt, m_txPwrDb, m_pktRxMode);//trcaing
}

void
UanPhyRealNewb::sendDataThread_1f(void)
{
         NS_LOG_DEBUG("send thread start!");
	/*终止接收*/
	memset((void *)tcpsendbuffer,0,tcppacketlen);//清零
	tcpsendbuffer[0] = 0x14;
	tcpsendbuffer[1] = 0x00;
	send(nFd,tcpsendbuffer,tcppacketlen,0);
	memset((void *)tcprecvbuffer,0,tcppacketlen);//清零

	 recvthr->Join();//等待接收线程结束
    /*开启发送*/
    frameNum = 1;
    memset((void *)tcpsendbuffer,0,tcppacketlen);//清零
    tcpsendbuffer[0] = 0x16;
    tcpsendbuffer[1] = 0x00;
    tcpsendbuffer[2] = (char)((uint16_t)transmitAmp & 0x00ff);     //发射幅度
    tcpsendbuffer[3] = (char)((((uint16_t)transmitAmp & 0xff00) >> 8) & 0x00ff);
    tcpsendbuffer[4] = (char)((sampleRate >> 16) & 0x000000ff);
    tcpsendbuffer[5] = (char)((sampleRate >> 24) & 0x000000ff);
    tcpsendbuffer[6] = (char)(sampleRate & 0x000000ff);
    tcpsendbuffer[7] = (char)((sampleRate >> 8) & 0x000000ff);
    tcpsendbuffer[8] = (char)(frameNum & 0x00ff);
    tcpsendbuffer[9] = (char)((frameNum >> 8) & 0x00ff);
    nRet = send(nFd,tcpsendbuffer,tcppacketlen,0);

    
   /*通信机准备好QPSK方式发送*/
    while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0)) > 0 )
    {
        if ( tcprecvbuffer[0] == (char)0x16 && tcprecvbuffer[1] == (char)0x01 )//
        break;
    }
    NS_LOG_DEBUG("Modem turn to Tx state !\n");
	/*传输数据*/
    int len_packet = m_pktTx->GetSize ();//包的实际长度          
    dataPerBag = 239;
    char* newPkt = new char[dataPerBag];//239
    if (m_pktTx->GetSize() <= (uint16_t)dataPerBag) { // 不够239就补零
      memset(newPkt, 0, dataPerBag);
      m_pktTx->CopyData((uint8_t *)(newPkt),len_packet);
    }else{                      // 超过239就截断
      m_pktTx
	  ->CopyData((uint8_t *)(newPkt),dataPerBag);
    }
       
	int sendframeNum = 1;
	memset((void *)tcpsendbuffer, 0, tcppacketlen);
	
	tcpsendbuffer[0] = 0x17;
	tcpsendbuffer[1] = 0x00;
	tcpsendbuffer[2] = (char)(dataPerBag & 0x00ff);
	tcpsendbuffer[3] = (char)((dataPerBag >> 8) & 0x00ff);//长度为239
	tcpsendbuffer[4] = (char)(sendframeNum & 0x00ff);
	tcpsendbuffer[5] = (char)((sendframeNum >> 8) & 0x00ff);


	memcpy(tcpsendbuffer + 8, newPkt, dataPerBag);//
	nRet = send(nFd,tcpsendbuffer,tcppacketlen,0);
	while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0)) > 0 )//阻塞
	 {
	     if ( tcprecvbuffer[0] == (char)0x17 && tcprecvbuffer[1] == (char)0x01 )//
             break;
	 }
	 /*发射数据*/
     memset((void *)tcpsendbuffer,0,tcppacketlen);//清零
     tcpsendbuffer[0] = 0x18;
     tcpsendbuffer[1] = 0x00;
       
	 nRet =  send(nFd,tcpsendbuffer,tcppacketlen,0);
	 if (nRet == -1)
		{
			NS_LOG_DEBUG ("send error !");
		}
    while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0))>0 )//等待发送完毕
	{
			if( tcprecvbuffer[0] == (char)0x18 && tcprecvbuffer[1] == (char)0x01 )  
		{   
			   NS_LOG_DEBUG ("Modem is transmitting packet");
		}
	     else if( tcprecvbuffer[0] == (char)0x19 && tcprecvbuffer[1] == (char)0x01 ) //成功完成发送
		{  
			   NS_LOG_DEBUG ("Modem finishs Tx");
			   break;
		}
    }
      
    r_state = IDLE;
    Simulator::Schedule (Seconds (0), &UanPhyRealNewb::StartRX, this);
    //StartRX();
    NS_LOG_DEBUG("send thread finish!");


}

void
UanPhyRealNewb::sendDataThread_5f(void)
{
	NS_LOG_DEBUG("send thread start!");
	
}



void 
UanPhyRealNewb::StartRX()
{
     NS_LOG_FUNCTION(this);

        frameNum = 1;
	memset((void *)tcpsendbuffer,0,tcppacketlen);//清零
       /*指示通信机进行Qpsk数据接收*/
	tcpsendbuffer[0] = 0x13;
	tcpsendbuffer[1] = 0x00;
	tcpsendbuffer[2] = (char)(gain & 0x00ff);
	tcpsendbuffer[3] = (char)((gain >> 8) & 0x00ff);
	tcpsendbuffer[4] = (char)(correlateThreadShort & 0x00ff);
	tcpsendbuffer[5] = (char)((correlateThreadShort >> 8) & 0x00ff);
	tcpsendbuffer[6] = (char)((sampleRate >> 16) & 0x000000ff);
	tcpsendbuffer[7] = (char)((sampleRate >> 24) & 0x000000ff);
	tcpsendbuffer[8] = (char)(sampleRate & 0x000000ff);
	tcpsendbuffer[9] = (char)((sampleRate >> 8) & 0x000000ff);
	tcpsendbuffer[10] = (char)(frameNum & 0x00ff);
	tcpsendbuffer[11] = (char)((frameNum >> 8) & 0x00ff);
	tcpsendbuffer[12] = (char)(qpskNumChannel & 0x00ff);
	tcpsendbuffer[13] = (char)((qpskNumChannel >> 8) & 0x00ff);
        
	send(nFd,tcpsendbuffer,tcppacketlen,0);
	while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0))>0 )
	{
		  if ( tcprecvbuffer[0] == (char)0x13 && tcprecvbuffer[1] == (char)0x01 )  break;
	}
	NS_LOG_DEBUG("Modem ready to receive!");
     recvthr = Create<SystemThread> (
      MakeCallback (&UanPhyRealNewb::recvDataThread, this));
       recvthr ->Start ();
  
}



void
UanPhyRealNewb::ConnectModem()
{
    NS_LOG_FUNCTION(this);
    struct sockaddr_in server_addr;

    /* socket descripter */
    nFd = socket(AF_INET,SOCK_STREAM,0);
    if (-1 == nFd)
    {
    	NS_ASSERT_MSG(-1 != nFd,"socket error!");
    }
    bzero(&server_addr, sizeof(server_addr));
    server_addr.sin_family = AF_INET;					/*协议族*/
    server_addr.sin_port = htons(serverPort);

    server_addr.sin_addr.s_addr = serverAddr;//IP

    /*connect the modem*/
    nRet=connect(nFd, (struct sockaddr*)&server_addr, sizeof(struct sockaddr));

    NS_ASSERT_MSG(nRet != -1,"Connect Modem Error!");
    //NS_LOG_DEBUG("Connect Modem Success!\n");
}


//class UanPhyRealNewb
void 
UanPhyRealNewb::recvDataThread(void )//该函数必须由新的线程执行，new thread
{
   
     NS_LOG_UNCOND("Enter recvDataThread!");
     
     memset((void *)recvbuffer,0,tcppacketlen);//清零
     int ReadLen;
	 while((ReadLen = recv(nFd,recvbuffer,tcppacketlen,0))>0)//
	 {
            // sleep(1);
	     //nReadLen = recv(nFd,recvbuffer,tcppacketlen,MSG_DONTWAIT);
             NS_LOG_DEBUG("Get frame");
	     if(ReadLen > 0)
		 {
		      if ( recvbuffer[0] == (char)0x15 && recvbuffer[1] == (char)0x01 )
		      {

		           framecnt = (recvbuffer[2] & 0x00ff) | ((recvbuffer[3] << 8) & 0xff00);        //帧号
		           messageLen = (recvbuffer[4] & 0x00ff) | ((recvbuffer[5] << 8) & 0xff00);      //数据长度，正常来说是239,否则异常
		           tuborItertime = (recvbuffer[6] & 0x00ff) | ((recvbuffer[7] << 8) & 0xff00);   //迭代次数	        
			   if(framecnt == 1)
                             { 
                                  //
                                  
                                 
                                  m_pktRx = Create<Packet> ((uint8_t *)(recvbuffer+8), 117);
                                        
                                  StartRxPacket (m_pktRx, 0, m_pktRxMode,m_pktRxPdp);
                                  memset((void *)recvbuffer,0,tcppacketlen);//清零
                                 
                             }
			   NS_LOG_DEBUG("Get frame:framecnt:"<<framecnt<<" messageLen:"<<messageLen<<" tuborItertime:"<<tuborItertime);
			   
		       }
		       else if ( recvbuffer[0] == (char)0x14 && recvbuffer[1] == (char)0x01 )
		       {
			      NS_LOG_DEBUG("break recv thread");
                              break;			   
		       }
		 }		 
	 }//while  

    return ;
}




bool UanPhyRealNewb::IsrStateRX(void)
{
       return  (r_state ==RX);
}
void UanPhyRealNewb::setrStateIDLE(void)
{
    r_state = IDLE;
}

int UanPhyRealNewb::readnFd(void)
{
       return nFd;
}

void UanPhyRealNewb::setModemIP()
{
      NS_LOG_FUNCTION(this);
      if(serverAddr == 0)
      {
	      Ptr< Node > m_node = m_device->GetNode();
	      uint32_t nodeId = m_node->GetId();
	      //NS_LOG_DEBUG("Node ID:"<<nodeId);
	      serverAddr = inet_addr( IpList[nodeId] );	
      }

}





/**********************************/

void
UanPhyRealNewb::TxEndEvent ()
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
UanPhyRealNewb::RegisterListener (UanPhyListener *listener)
{
  m_listeners.push_back (listener);
}




void
UanPhyRealNewb::SetReceiveOkCallback (RxOkCallback cb)
{
  m_recOkCb = cb;
}

void
UanPhyRealNewb::SetReceiveErrorCallback (RxErrCallback cb)
{
  m_recErrCb = cb;
}
bool
UanPhyRealNewb::IsStateSleep (void)
{
  return m_state == SLEEP;
}
bool
UanPhyRealNewb::IsStateIdle (void)
{
  return m_state == IDLE;
}
bool
UanPhyRealNewb::IsStateBusy (void)
{
  return !IsStateIdle () && !IsStateSleep ();
}
bool
UanPhyRealNewb::IsStateRx (void)
{
  return m_state == RX;
}
bool
UanPhyRealNewb::IsStateTx (void)
{
  return m_state == TX;
}

bool
UanPhyRealNewb::IsStateCcaBusy (void)
{
  return m_state == CCABUSY;
}


void
UanPhyRealNewb::SetRxGainDb (double gain)
{
  m_rxGainDb = gain;

}
void
UanPhyRealNewb::SetTxPowerDb (double txpwr)
{
  m_txPwrDb = txpwr;
}
void
UanPhyRealNewb::SetRxThresholdDb (double thresh)
{
  m_rxThreshDb = thresh;
}
void
UanPhyRealNewb::SetCcaThresholdDb (double thresh)
{
  m_ccaThreshDb = thresh;
}
double
UanPhyRealNewb::GetRxGainDb (void)
{
  return m_rxGainDb;
}
double
UanPhyRealNewb::GetTxPowerDb (void)
{
  return m_txPwrDb;

}
double
UanPhyRealNewb::GetRxThresholdDb (void)
{
  return m_rxThreshDb;
}
double
UanPhyRealNewb::GetCcaThresholdDb (void)
{
  return m_ccaThreshDb;
}

Ptr<UanChannel>
UanPhyRealNewb::GetChannel (void) const
{
  return m_channel;
}

Ptr<UanNetDevice>
UanPhyRealNewb::GetDevice (void) const
{
  return m_device;
}

Ptr<UanTransducer>
UanPhyRealNewb::GetTransducer (void)
{
  return m_transducer;
}
void
UanPhyRealNewb::SetChannel (Ptr<UanChannel> channel)
{
  m_channel = channel;
}

void
UanPhyRealNewb::SetDevice (Ptr<UanNetDevice> device)
{
  m_device = device;
}

void
UanPhyRealNewb::SetMac (Ptr<UanMac> mac)
{
  m_mac = mac;
}

void
UanPhyRealNewb::SetTransducer (Ptr<UanTransducer> trans)
{
  m_transducer = trans;
  m_transducer->AddPhy (this);
}

void
UanPhyRealNewb::SetSleepMode (bool sleep)
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
UanPhyRealNewb::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_pg->SetStream (stream);
  return 1;
}

void
UanPhyRealNewb::NotifyTransStartTx (Ptr<Packet> packet, double txPowerDb, UanTxMode txMode)
{
  if (m_pktRx)
    {
      m_minRxSinrDb = -1e30;
    }
}

void
UanPhyRealNewb::NotifyIntChange (void)
{
  if (m_state == CCABUSY && GetInterferenceDb (Ptr<Packet> ()) < m_ccaThreshDb)
    {
      m_state = IDLE;
      NotifyListenersCcaEnd ();
    }
}

double
UanPhyRealNewb::CalculateSinrDb (Ptr<Packet> pkt, Time arrTime, double rxPowerDb, UanTxMode mode, UanPdp pdp)
{
  double noiseDb = m_channel->GetNoiseDbHz ( (double) mode.GetCenterFreqHz () / 1000.0) + 10 * std::log10 (mode.GetBandwidthHz ());
  return m_sinr->CalcSinrDb (pkt, arrTime, rxPowerDb, noiseDb, mode, pdp, m_transducer->GetArrivalList ());
}

double
UanPhyRealNewb::GetInterferenceDb (Ptr<Packet> pkt)
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
UanPhyRealNewb::DbToKp (double db)
{
  return std::pow (10, db / 10.0);
}
double
UanPhyRealNewb::KpToDb (double kp)
{
  return 10 * std::log10 (kp);
}

void
UanPhyRealNewb::NotifyListenersRxStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxStart ();
    }

}
void
UanPhyRealNewb::NotifyListenersRxGood (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndOk ();
    }
}
void
UanPhyRealNewb::NotifyListenersRxBad (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndError ();
    }
}
void
UanPhyRealNewb::NotifyListenersCcaStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaStart ();
    }
}
void
UanPhyRealNewb::NotifyListenersCcaEnd (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaEnd ();
    }
}

void
UanPhyRealNewb::NotifyListenersTxStart (Time duration)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyTxStart (duration);
    }
}

uint32_t
UanPhyRealNewb::GetNModes (void)
{
  return m_modes.GetNModes ();
}

UanTxMode
UanPhyRealNewb::GetMode (uint32_t n)
{
  
  //NS_ASSERT (n < m_modes.GetNModes ());
  //n=1;
  return m_modes[n];
}

Ptr<Packet>
UanPhyRealNewb::GetPacketRx (void) const
{
  return m_pktRx;
}


} // namespace ns3
