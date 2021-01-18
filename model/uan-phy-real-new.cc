/*   ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
 **    uan-phy-real-new.cc                     *
 **    Created on 2018/7/18               *
 **    Author:lch                          *
 **    For:connecting with real channel    *
 **  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
*/


#include "uan-phy-real-new.h"
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

NS_LOG_COMPONENT_DEFINE ("UanPhyRealNew");

NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNew);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewPerGenDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewCalcSinrDefault);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewCalcSinrFhFsk);
NS_OBJECT_ENSURE_REGISTERED (UanPhyRealNewPerUmodem);


/*************** UanPhyRealNewCalcSinrDefault definition *****************/
UanPhyRealNewCalcSinrDefault::UanPhyRealNewCalcSinrDefault ()
{

}
UanPhyRealNewCalcSinrDefault::~UanPhyRealNewCalcSinrDefault ()
{

}

TypeId
UanPhyRealNewCalcSinrDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewCalcSinrDefault")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewCalcSinrDefault> ()
  ;
  return tid;
}

double
UanPhyRealNewCalcSinrDefault::CalcSinrDb (Ptr<Packet> pkt,
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

/*************** UanPhyRealNewCalcSinrFhFsk definition *****************/
UanPhyRealNewCalcSinrFhFsk::UanPhyRealNewCalcSinrFhFsk ()
{

}
UanPhyRealNewCalcSinrFhFsk::~UanPhyRealNewCalcSinrFhFsk ()
{

}

TypeId
UanPhyRealNewCalcSinrFhFsk::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewCalcSinrFhFsk")
    .SetParent<UanPhyCalcSinr> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewCalcSinrFhFsk> ()
    .AddAttribute ("NumberOfHops",
                   "Number of frequencies in hopping pattern.",
                   UintegerValue (13),
                   MakeUintegerAccessor (&UanPhyRealNewCalcSinrFhFsk::m_hops),
                   MakeUintegerChecker<uint32_t> ())
  ;
  return tid;
}
double
UanPhyRealNewCalcSinrFhFsk::CalcSinrDb (Ptr<Packet> pkt,
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

/*************** UanPhyRealNewPerGenDefault definition *****************/
UanPhyRealNewPerGenDefault::UanPhyRealNewPerGenDefault ()
{

}

UanPhyRealNewPerGenDefault::~UanPhyRealNewPerGenDefault ()
{

}
TypeId
UanPhyRealNewPerGenDefault::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewPerGenDefault")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewPerGenDefault> ()
    .AddAttribute ("Threshold", "SINR cutoff for good packet reception.",
                   DoubleValue (8),
                   MakeDoubleAccessor (&UanPhyRealNewPerGenDefault::m_thresh),
                   MakeDoubleChecker<double> ());
  return tid;
}


// Default PER calculation simply compares SINR to a threshold which is configurable
// via an attribute.
double
UanPhyRealNewPerGenDefault::CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode)
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

/*************** UanPhyRealNewPerUmodem definition *****************/
UanPhyRealNewPerUmodem::UanPhyRealNewPerUmodem ()
{

}
UanPhyRealNewPerUmodem::~UanPhyRealNewPerUmodem ()
{

}

TypeId UanPhyRealNewPerUmodem::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::UanPhyRealNewPerUmodem")
    .SetParent<UanPhyPer> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNewPerUmodem> ()
  ;
  return tid;
}

double
UanPhyRealNewPerUmodem::NChooseK (uint32_t n, uint32_t k)
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
UanPhyRealNewPerUmodem::CalcPer (Ptr<Packet> pkt, double sinr, UanTxMode mode)
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

/*************** UanPhyRealNew definition *****************/
UanPhyRealNew::UanPhyRealNew ()
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
  NS_LOG_DEBUG ("UanPhyRealNew");
 // Init();//初始化函数，参数计算，连接Modem
  Simulator::Schedule (Seconds (0.5), &UanPhyRealNew::Init, this);
}

UanPhyRealNew::~UanPhyRealNew ()
{

}

void
UanPhyRealNew::Clear ()
{
  NS_LOG_FUNCTION(this);
  NS_LOG_UNCOND("NodeId:"<<nodeId<<"--PhyRealNew-- txPktNum:"<<txPktNum<<"--rxPktNum:"<<rxPktNum);
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
UanPhyRealNew::DoDispose ()
{
  Clear ();
  m_energyCallback.Nullify ();
  UanPhy::DoDispose ();
}

UanModesList
UanPhyRealNew::GetDefaultModes (void)
{
  UanModesList l;
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::FSK,80,80,22000,4000,13,"FSK"));
  l.AppendMode (UanTxModeFactory::CreateMode (UanTxMode::PSK,200, 200, 22000, 4000, 4, "QPSK"));
  return l;
}
TypeId
UanPhyRealNew::GetTypeId (void)
{

  static TypeId tid = TypeId ("ns3::UanPhyRealNew")
    .SetParent<UanPhy> ()
    .SetGroupName ("Uan")
    .AddConstructor<UanPhyRealNew> ()
    .AddAttribute ("CcaThreshold",
                   "Aggregate energy of incoming signals to move to CCA Busy state dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealNew::m_ccaThreshDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxThreshold",
                   "Required SNR for signal acquisition in dB.",
                   DoubleValue (10),
                   MakeDoubleAccessor (&UanPhyRealNew::m_rxThreshDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute  ("PacketSize",                                
			      "The size of Packet in Phy",
			      UintegerValue(1024),//
			      MakeUintegerAccessor (&UanPhyRealNew::packetSize),
			      MakeUintegerChecker< uint16_t> ())
    .AddAttribute  ("ServerAddr",                                  
			 "The Ip of the Modem",
			UintegerValue(0),//???
			MakeUintegerAccessor (&UanPhyRealNew::serverAddr),
			MakeUintegerChecker< uint32_t> ())
    .AddAttribute ("TxPower",
                   "Transmission output power in dB.",
                   DoubleValue (190),
                   MakeDoubleAccessor (&UanPhyRealNew::m_txPwrDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("RxGain",
                   "Gain added to incoming signal at receiver.",
                   DoubleValue (0),
                   MakeDoubleAccessor (&UanPhyRealNew::m_rxGainDb),
                   MakeDoubleChecker<double> ())
    .AddAttribute ("SupportedModes",
                   "List of modes supported by this PHY.",
                   UanModesListValue (UanPhyRealNew::GetDefaultModes ()),
                   MakeUanModesListAccessor (&UanPhyRealNew::m_modes),
                   MakeUanModesListChecker () )
    .AddAttribute ("PerModel",
                   "Functor to calculate PER based on SINR and TxMode.",
                   StringValue ("ns3::UanPhyRealNewPerGenDefault"),
                   MakePointerAccessor (&UanPhyRealNew::m_per),
                   MakePointerChecker<UanPhyPer> ())
    .AddAttribute ("SinrModel",
                   "Functor to calculate SINR based on pkt arrivals and modes.",
                   StringValue ("ns3::UanPhyRealNewCalcSinrDefault"),
                   MakePointerAccessor (&UanPhyRealNew::m_sinr),
                   MakePointerChecker<UanPhyCalcSinr> ())

    .AddTraceSource ("RxOk",
                     "A packet was received successfully.",
                     MakeTraceSourceAccessor (&UanPhyRealNew::m_rxOkLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("RxError",
                     "A packet was received unsuccessfully.",
                     MakeTraceSourceAccessor (&UanPhyRealNew::m_rxErrLogger),
                     "ns3::UanPhy::TracedCallback")
    .AddTraceSource ("Tx",
                     "Packet transmission beginning.",
                     MakeTraceSourceAccessor (&UanPhyRealNew::m_txLogger),
                     "ns3::UanPhy::TracedCallback")
  ;
  return tid;

}

void
UanPhyRealNew::SetEnergyModelCallback (DeviceEnergyModel::ChangeStateCallback cb)
{
  NS_LOG_FUNCTION (this);
  m_energyCallback = cb;
}

void
UanPhyRealNew::UpdatePowerConsumption (const State state)
{
  NS_LOG_FUNCTION (this);

  if (!m_energyCallback.IsNull ())
    {
      m_energyCallback (state);
    }
}

void
UanPhyRealNew::EnergyDepletionHandler ()
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
UanPhyRealNew::EnergyRechargeHandler ()
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
UanPhyRealNew::Init()
{
    NS_LOG_FUNCTION(this);
    
    sendBytesNum = frameNum * dataPerFrame;
    zeroBytesNum = sendBytesNum - dataSize; // 帧补零数,这里是补零到N帧数据
    sendPackageNum = static_cast<int>(ceil((static_cast<double>(sendBytesNum)) / (static_cast<double>(dataPerBag))));
    
    setModemIP();
    ConnectModem();//初始化时就连接通信机
    r_state = IDLE;
    Simulator::Schedule (Seconds (0.5), &UanPhyRealNew::StartRX, this);//0.5s后开启接收状态
}

void
UanPhyRealNew::StartRxPacket (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode, UanPdp pdp)//by LCH
{
    NS_LOG_FUNCTION (this);
    double txdelay = 0;
    uint32_t dstNodeId = (m_device->GetNode() )->GetId();
    
   Simulator::ScheduleWithContext (dstNodeId ,Seconds (txdelay), &UanPhyRealNew::RxEndEvent, this, pkt, rxPowerDb, txMode);
   
}

void
UanPhyRealNew::RxEndEvent (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode)//by LCH
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
UanPhyRealNew::SendPacket (Ptr<Packet> pkt, uint32_t modeNum)//by LCH
{
  NS_LOG_DEBUG ("PHY " << m_mac->GetAddress () << ": Transmitting packet");
  NS_LOG_DEBUG ("send:packet size:" << pkt->GetSize());

  if(r_state == TX || r_state == RX)//
  {
    NS_LOG_DEBUG("Channel is busy!Can not transmit!");
    return; //
  }
  
  NS_LOG_DEBUG ("test1 " );
  //double txdelay = 0;
  //m_callEndEvent = Simulator::Schedule (Seconds ( txdelay), &UanPhyRealNew::StartTX, this, pkt);
  //开启发送线程
  r_state = TX;
   sendthr = Create<SystemThread> (
      MakeCallback (&UanPhyRealNew::sendDataThread, this));
  sendthr ->Start ();
  
  m_pktTx = pkt;
  NS_LOG_DEBUG ("test2 " );
  //m_txLogger (pkt, m_txPwrDb, m_pktRxMode);//trcaing
  txPktNum ++;
}

void
UanPhyRealNew::sendDataThread(void)
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
 
  /* 多截断，少补零，到1024*/
  char* newPkt = new char[dataSize];//1024
  if (len_packet <= dataSize) { // 不够1024就补零,这里是一次补零，补到1024,下面还有二次补零，补到N帧数据
      memset(newPkt, 0, dataSize);
      m_pktTx->CopyData((uint8_t *)(newPkt),len_packet);
    }else{                      // 超过1024就截断
      m_pktTx->CopyData((uint8_t *)(newPkt),dataSize);
    }
  
  /* 分帧，发送*/
  for (int i = 0; i < sendPackageNum; i++) {//？？？
      if (i == sendPackageNum - 1) {

        memset((void *)tcpsendbuffer, 0, tcppacketlen);
        tcpsendbuffer[0] = 0x17;
        tcpsendbuffer[1] = 0x00;
        int leftDataLen = sendBytesNum - i * dataPerBag;
        tcpsendbuffer[2] = (char)(leftDataLen & 0x00ff);
        tcpsendbuffer[3] = (char)((leftDataLen >> 8) & 0x00ff);
        tcpsendbuffer[4] = (char)(frameNum & 0x00ff);
        tcpsendbuffer[5] = (char)((frameNum >> 8) & 0x00ff);

        memcpy(tcpsendbuffer + 8, newPkt + i * dataPerBag, leftDataLen - zeroBytesNum); // 在最后一个包中，余下的数据段减去二次补零数据段就是剩余的真实数据（真实指的是非二次补零数据的数据段）
        nRet = send(nFd,tcpsendbuffer,tcppacketlen,0);
	 while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0)) > 0 )
	 {
	     if ( tcprecvbuffer[0] == (char)0x17 && tcprecvbuffer[1] == (char)0x01 )//
             break;
	 }
      }else{

        memset((void *)tcpsendbuffer, 0, tcppacketlen);
        tcpsendbuffer[0] = 0x17;
        tcpsendbuffer[1] = 0x00;
        tcpsendbuffer[2] = (char)(dataPerBag & 0x00ff);
        tcpsendbuffer[3] = (char)((dataPerBag >> 8) & 0x00ff);//长度为1024
        tcpsendbuffer[4] = (char)(frameNum & 0x00ff);
        tcpsendbuffer[5] = (char)((frameNum >> 8) & 0x00ff);
        
        memcpy(tcpsendbuffer + 8, newPkt + i * dataPerBag, dataPerBag);//
        nRet = send(nFd,tcpsendbuffer,tcppacketlen,0);
	 while( (nReadLen = recv(nFd,tcprecvbuffer,tcppacketlen,0)) > 0 )//阻塞
	 {
	     if ( tcprecvbuffer[0] == (char)0x17 && tcprecvbuffer[1] == (char)0x01 )//
             break;
	 }
      }
    }//for


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
    Simulator::Schedule (Seconds (0), &UanPhyRealNew::StartRX, this);
    m_mac -> NotifyTxEnd();//By LCH
    //StartRX();
    NS_LOG_DEBUG("send thread finish!");
}



void 
UanPhyRealNew::StartRX()
{
     NS_LOG_FUNCTION(this);


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
      MakeCallback (&UanPhyRealNew::recvDataThread, this));
       recvthr ->Start ();
  
}



void
UanPhyRealNew::ConnectModem()
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


//class UanPhyRealNew
void 
UanPhyRealNew::recvDataThread(void )//该函数必须由新的线程执行，new thread
{
   
     NS_LOG_DEBUG("recvDataThread!nFD:"<< nFd);
     
     memset((void *)recvbuffer,0,tcppacketlen);//清零
     int ReadLen;
	 while((ReadLen = recv(nFd,recvbuffer,tcppacketlen,0))>0)//
	 {
            // sleep(1);
	     //nReadLen = recv(nFd,recvbuffer,tcppacketlen,MSG_DONTWAIT);
	     if(ReadLen > 0)
		 {
		      if ( recvbuffer[0] == (char)0x15 && recvbuffer[1] == (char)0x01 )
		      {

		           framecnt = (recvbuffer[2] & 0x00ff) | ((recvbuffer[3] << 8) & 0xff00);        //帧号
		           messageLen = (recvbuffer[4] & 0x00ff) | ((recvbuffer[5] << 8) & 0xff00);      //数据长度，正常来说是239,否则异常
		           tuborItertime = (recvbuffer[6] & 0x00ff) | ((recvbuffer[7] << 8) & 0xff00);   //迭代次数	        
			   if(framecnt == 1) r_state = RX; //收到第一帧进入RX状态
			   NS_LOG_DEBUG("Get frame:framecnt:"<<framecnt<<" messageLen:"<<messageLen<<" tuborItertime:"<<tuborItertime);
			   if(framecnt == frameNum)//收到最后一帧
			    {       
                                //默认：packetSize>=956,前面的4次:4*239=956。以后加入更多可选范围
			        int leftlen = packetSize-(frameNum-1)*messageLen;
			        //NS_LOG_DEBUG("debug leftlen:"<<leftlen);
			        memcpy(tempbuffer+(framecnt-1)*messageLen, recvbuffer+8,leftlen );

		                //m_pktRx = Create<Packet> ((uint8_t *)(tempbuffer), packetSize);
                                m_pktRx = Create<Packet> ((uint8_t *)(tempbuffer), 477);//

		                StartRxPacket (m_pktRx, 0, m_pktRxMode,m_pktRxPdp);//上传packet到mac 

			        memset((void *)tempbuffer,0,tcppacketlen);//清零
			        framecnt = 0;
			        r_state = IDLE;//收到最后一帧进入IDLE状态
		            }             
   			    else
      			    {
			         memcpy(tempbuffer+(framecnt-1)*messageLen,recvbuffer+8,messageLen);//合并
			     
			    }
		       }
		       else if ( recvbuffer[0] == (char)0x14 && recvbuffer[1] == (char)0x01 )
		       {
			     // NS_LOG_DEBUG("test!!test!!!1401");
                              break;			   
		       }
		 }		 
	 }//while  

    return ;
}




bool UanPhyRealNew::IsrStateRX(void)
{
       return  (r_state ==RX);
}
void UanPhyRealNew::setrStateIDLE(void)
{
    r_state = IDLE;
}

int UanPhyRealNew::readnFd(void)
{
       return nFd;
}

void UanPhyRealNew::setModemIP()
{
      NS_LOG_FUNCTION(this);
      if(serverAddr == 0)
      {
	      Ptr< Node > m_node = m_device->GetNode();
	      nodeId = m_node->GetId();
	      //NS_LOG_DEBUG("Node ID:"<<nodeId);
	      serverAddr = inet_addr( IpList[nodeId] );	
      }

}





/**********************************/

void
UanPhyRealNew::TxEndEvent ()
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
UanPhyRealNew::RegisterListener (UanPhyListener *listener)
{
  m_listeners.push_back (listener);
}




void
UanPhyRealNew::SetReceiveOkCallback (RxOkCallback cb)
{
  m_recOkCb = cb;
}

void
UanPhyRealNew::SetReceiveErrorCallback (RxErrCallback cb)
{
  m_recErrCb = cb;
}
bool
UanPhyRealNew::IsStateSleep (void)
{
  return m_state == SLEEP;
}
bool
UanPhyRealNew::IsStateIdle (void)
{
  return m_state == IDLE;
}
bool
UanPhyRealNew::IsStateBusy (void)
{
  return !IsStateIdle () && !IsStateSleep ();
}
bool
UanPhyRealNew::IsStateRx (void)
{
  return m_state == RX;
}
bool
UanPhyRealNew::IsStateTx (void)
{
  return m_state == TX;
}

bool
UanPhyRealNew::IsStateCcaBusy (void)
{
  return m_state == CCABUSY;
}


void
UanPhyRealNew::SetRxGainDb (double gain)
{
  m_rxGainDb = gain;

}
void
UanPhyRealNew::SetTxPowerDb (double txpwr)
{
  m_txPwrDb = txpwr;
}
void
UanPhyRealNew::SetRxThresholdDb (double thresh)
{
  m_rxThreshDb = thresh;
}
void
UanPhyRealNew::SetCcaThresholdDb (double thresh)
{
  m_ccaThreshDb = thresh;
}
double
UanPhyRealNew::GetRxGainDb (void)
{
  return m_rxGainDb;
}
double
UanPhyRealNew::GetTxPowerDb (void)
{
  return m_txPwrDb;

}
double
UanPhyRealNew::GetRxThresholdDb (void)
{
  return m_rxThreshDb;
}
double
UanPhyRealNew::GetCcaThresholdDb (void)
{
  return m_ccaThreshDb;
}

Ptr<UanChannel>
UanPhyRealNew::GetChannel (void) const
{
  return m_channel;
}

Ptr<UanNetDevice>
UanPhyRealNew::GetDevice (void) const
{
  return m_device;
}

Ptr<UanTransducer>
UanPhyRealNew::GetTransducer (void)
{
  return m_transducer;
}
void
UanPhyRealNew::SetChannel (Ptr<UanChannel> channel)
{
  m_channel = channel;
}

void
UanPhyRealNew::SetDevice (Ptr<UanNetDevice> device)
{
  m_device = device;
}

void
UanPhyRealNew::SetMac (Ptr<UanMac> mac)
{
  m_mac = mac;
}

void
UanPhyRealNew::SetTransducer (Ptr<UanTransducer> trans)
{
  m_transducer = trans;
  m_transducer->AddPhy (this);
}

void
UanPhyRealNew::SetSleepMode (bool sleep)
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
UanPhyRealNew::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_pg->SetStream (stream);
  return 1;
}

void
UanPhyRealNew::NotifyTransStartTx (Ptr<Packet> packet, double txPowerDb, UanTxMode txMode)
{
  if (m_pktRx)
    {
      m_minRxSinrDb = -1e30;
    }
}

void
UanPhyRealNew::NotifyIntChange (void)
{
  if (m_state == CCABUSY && GetInterferenceDb (Ptr<Packet> ()) < m_ccaThreshDb)
    {
      m_state = IDLE;
      NotifyListenersCcaEnd ();
    }
}

double
UanPhyRealNew::CalculateSinrDb (Ptr<Packet> pkt, Time arrTime, double rxPowerDb, UanTxMode mode, UanPdp pdp)
{
  double noiseDb = m_channel->GetNoiseDbHz ( (double) mode.GetCenterFreqHz () / 1000.0) + 10 * std::log10 (mode.GetBandwidthHz ());
  return m_sinr->CalcSinrDb (pkt, arrTime, rxPowerDb, noiseDb, mode, pdp, m_transducer->GetArrivalList ());
}

double
UanPhyRealNew::GetInterferenceDb (Ptr<Packet> pkt)
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
UanPhyRealNew::DbToKp (double db)
{
  return std::pow (10, db / 10.0);
}
double
UanPhyRealNew::KpToDb (double kp)
{
  return 10 * std::log10 (kp);
}

void
UanPhyRealNew::NotifyListenersRxStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxStart ();
    }

}
void
UanPhyRealNew::NotifyListenersRxGood (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndOk ();
    }
}
void
UanPhyRealNew::NotifyListenersRxBad (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyRxEndError ();
    }
}
void
UanPhyRealNew::NotifyListenersCcaStart (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaStart ();
    }
}
void
UanPhyRealNew::NotifyListenersCcaEnd (void)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyCcaEnd ();
    }
}

void
UanPhyRealNew::NotifyListenersTxStart (Time duration)
{
  ListenerList::const_iterator it = m_listeners.begin ();
  for (; it != m_listeners.end (); it++)
    {
      (*it)->NotifyTxStart (duration);
    }
}

uint32_t
UanPhyRealNew::GetNModes (void)
{
  return m_modes.GetNModes ();
}

UanTxMode
UanPhyRealNew::GetMode (uint32_t n)
{
 
  //NS_ASSERT (n < m_modes.GetNModes ());
  //n=1;
  return m_modes[n];
}

Ptr<Packet>
UanPhyRealNew::GetPacketRx (void) const
{
  return m_pktRx;
}


} // namespace ns3
