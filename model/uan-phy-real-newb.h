/*   ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
 **    uan-phy-real-new.h                     *
 **    Created on 2018/7/18                *
 **    Author:lch                          *
 **    For:connecting with real channel    *
 **  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
*/
#ifndef UAN_PHY_REAL_NEW_H
#define UAN_PHY_REAL_NEW_H


#include "uan-phy.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/device-energy-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/event-id.h"
#include "ns3/system-thread.h"
#include <list>

//#include <cstring>

namespace ns3 {

/**
 * \ingroup uan
 */
void * ReceivePacket(void * arg);

class UanPhyRealNewbPerGenDefault : public UanPhyPer
{
public:
  /** Constructor */
  UanPhyRealNewbPerGenDefault ();
  /** Destructor */
  virtual ~UanPhyRealNewbPerGenDefault ();
  static TypeId GetTypeId (void);
  virtual double CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode);
private: 
  double m_thresh;  //!< SINR threshold.

};  // class UanPhyRealNewbPerGenDefault


class UanPhyRealNewbPerUmodem : public UanPhyPer
{
public:
  /** Constructor */
  UanPhyRealNewbPerUmodem ();
  /** Destructor */
  virtual ~UanPhyRealNewbPerUmodem ();
  static TypeId GetTypeId (void); 
  virtual double CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode);

private:

  double NChooseK (uint32_t n, uint32_t k);

};  // class UanPhyRealNewbPerUmodem


/**
 * \ingroup uan
 */
class UanPhyRealNewbCalcSinrDefault : public UanPhyCalcSinr
{

public:
  /** Constructor */
  UanPhyRealNewbCalcSinrDefault ();
  /** Destructor */
  virtual ~UanPhyRealNewbCalcSinrDefault ();
  
  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);
  

  virtual double CalcSinrDb (Ptr<Packet> pkt,
                             Time arrTime,
                             double rxPowerDb,
                             double ambNoiseDb,
                             UanTxMode mode,
                             UanPdp pdp,
                             const UanTransducer::ArrivalList &arrivalList
                             ) const;

};  // class UanPhyRealNewbCalcSinrDefault


/**
 * \ingroup uan
 */
class UanPhyRealNewbCalcSinrFhFsk : public UanPhyCalcSinr
{

public:
  /** Constructor */
  UanPhyRealNewbCalcSinrFhFsk ();
  /** Destructor */
  virtual ~UanPhyRealNewbCalcSinrFhFsk ();

  static TypeId GetTypeId (void);

  virtual double CalcSinrDb (Ptr<Packet> pkt,
                             Time arrTime,
                             double rxPowerDb,
                             double ambNoiseDb,
                             UanTxMode mode,
                             UanPdp pdp,
                             const UanTransducer::ArrivalList &arrivalList
                             ) const;
private:
  uint32_t m_hops;  //!< Number of hops.

};  // class UanPhyRealNewbCalcSinrFhFsk


/**
 * \ingroup uan
 *
 * Generic PHY model.
 *
 * This is a generic PHY class.  SINR and PER information
 * are controlled via attributes.  By adapting the SINR
 * and PER models to a specific situation, this PHY should
 * be able to model a wide variety of networks.
 */
class UanPhyRealNewb : public UanPhy
{
public:
  /** Constructor */
  UanPhyRealNewb ();
  /** Dummy destructor, see DoDispose */
  virtual ~UanPhyRealNewb ();
  /**
   * Get the default transmission modes.
   *
   * \return The default mode list.
   */
  static UanModesList GetDefaultModes (void);

  
  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);

  // Inherited methods
  virtual void SetEnergyModelCallback (DeviceEnergyModel::ChangeStateCallback cb);
  virtual void EnergyDepletionHandler (void);
  virtual void EnergyRechargeHandler (void);
  virtual void SendPacket (Ptr<Packet> pkt, uint32_t modeNum);
  virtual void RegisterListener (UanPhyListener *listener);
  virtual void StartRxPacket (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode, UanPdp pdp);
  virtual void SetReceiveOkCallback (RxOkCallback cb);
  virtual void SetReceiveErrorCallback (RxErrCallback cb);
  virtual bool IsStateSleep (void);
  virtual bool IsStateIdle (void);
  virtual bool IsStateBusy (void);
  virtual bool IsStateRx (void);
  virtual bool IsStateTx (void);
  virtual bool IsStateCcaBusy (void);
  virtual void SetRxGainDb (double gain);
  virtual void SetTxPowerDb (double txpwr);
  virtual void SetRxThresholdDb (double thresh);
  virtual void SetCcaThresholdDb (double thresh);
  virtual double GetRxGainDb (void);
  virtual double GetTxPowerDb (void);
  virtual double GetRxThresholdDb (void);
  virtual double GetCcaThresholdDb (void);
  virtual Ptr<UanChannel> GetChannel (void) const;
  virtual Ptr<UanNetDevice> GetDevice (void) const;
  virtual Ptr<UanTransducer> GetTransducer (void);
  virtual void SetChannel (Ptr<UanChannel> channel);
  virtual void SetDevice (Ptr<UanNetDevice> device);
  virtual void SetMac (Ptr<UanMac> mac);
  virtual void SetTransducer (Ptr<UanTransducer> trans);
  virtual void NotifyTransStartTx (Ptr<Packet> packet, double txPowerDb, UanTxMode txMode);
  virtual void NotifyIntChange (void);
  virtual uint32_t GetNModes (void);
  virtual UanTxMode GetMode (uint32_t n);
  virtual Ptr<Packet> GetPacketRx (void) const;
  virtual void Clear (void);
  virtual void SetSleepMode (bool sleep);
  int64_t AssignStreams (int64_t stream);
  
  /*new added*/
  //void StartTX(Ptr<Packet> pkt);
  //void EndRX(void);
  //void EndTX(void);
  void StartRX(void);
  //void ReceivePacket(void );
 // void TransmitPacket (Ptr<Packet> pkt);
  void ConnectModem(void);
  void Init(void);//数据初始化
  void setModemIP(void);//设置mocdem的IP

  bool IsrStateRX(void);
  void setrStateIDLE(void);
  int readnFd(void);

  /*thread method*/
  void sendDataThread_1f(void);
  void sendDataThread_5f(void);
  void recvDataThread(void);
  
private:
  /** List of Phy Listeners. */
  typedef std::list<UanPhyListener *> ListenerList;

  UanModesList m_modes;             //!< List of modes supported by this PHY.

  State m_state,r_state;                    //!< Phy state.
  ListenerList m_listeners;         //!< List of listeners.
  RxOkCallback m_recOkCb;           //!< Callback for packets received without error.
  RxErrCallback m_recErrCb;         //!< Callback for packets received with errors.
  Ptr<UanChannel> m_channel;        //!< Attached channel.
  Ptr<UanTransducer> m_transducer;  //!< Associated transducer.
  Ptr<UanNetDevice> m_device;       //!< Device hosting this Phy.
  Ptr<UanMac> m_mac;                //!< MAC layer.
  Ptr<UanPhyPer> m_per;             //!< Error model.
  Ptr<UanPhyCalcSinr> m_sinr;       //!< SINR calculator.

  double m_rxGainDb;                //!< Receive gain.
  double m_txPwrDb;                 //!< Transmit power.
  double m_rxThreshDb;              //!< Receive SINR threshold.
  double m_ccaThreshDb;             //!< CCA busy threshold.
  Ptr<Packet> m_pktRx;              //!< Received packet.
  Ptr<Packet> m_pktTx;              //!< Sent packet.
  
  double m_minRxSinrDb;             //!< Minimum receive SINR during packet reception.
  double m_rxRecvPwrDb;             //!< Receiver power.
  Time m_pktRxArrTime;              //!< Packet arrival time.
  UanPdp m_pktRxPdp;                //!< Power delay profile of pakket.
  UanTxMode m_pktRxMode;            //!< Packet transmission mode at receiver.

  bool m_cleared;                   //!< Flag when we've been cleared.
  
  EventId m_txEndEvent;             //!< Tx event
  EventId m_rxEndEvent;             //!< Rx event
  EventId m_callEndEvent; 




  /*
   *   *argument about the real machine *
   *          
   */
/*
  //TX
  float txGain = 0.01;
  float transmitAmp = txGain * 32767;
 // int sampleRate = 96000;
  int frameNum = 5;
  int time = 0;
  //RX
  */
  
  

  /*veriable about the socket*/
  int nFd = 0; 
  int nRet = 0;
  int nReadLen =0;
  int len_pktTx = 0;
  int len_pktRx = 0;
  /*veriable about the modem packet*/ 
 int tcppacketlen = 1032;
  char * tcpsendbuffer = (char *)malloc(tcppacketlen);
  char * tcprecvbuffer = (char *)malloc(tcppacketlen);
  char * tempbuffer = (char *)malloc(tcppacketlen);
  char * recvbuffer = (char *)malloc(tcppacketlen);

     int framecnt;           //帧号
     int messageLen;         //数据长度
     int tuborItertime;      //迭代次数
//rx
     float threshold = 0.1;
      float rxgain = 1; 

      uint16_t gain = (uint16_t)(rxgain/2.5*65535);
      uint16_t correlateThreadShort = (uint16_t)(threshold*65535);;//?
      int sampleRate = 96000;
      uint16_t qpskNumChannel = 4;
      int frameNum = 1;
//tx
        float txGain = 0.01;
        float transmitAmp = txGain * 32767;


  Ptr<SystemThread> recvthr,sendthr;
 
  
  //int MaxSize = 1024; //datasize允许的最大值
  uint16_t dataSize = 1024;
  uint16_t packetSize = 1024;       //max:1024，传到phy层的packet的大小,等于应用层的包大小+3,目前为测试状态
  int dataPerFrame = 239;  //
  int dataPerBag =1024;
  int sendBytesNum;
  int zeroBytesNum;
  int sendPackageNum;
 
 
  short serverPort=80;  //端口号
  uint32_t serverAddr=0; //???

  const char * IpList[3]={"192.168.2.101","192.168.2.102","192.168.2.103"};
  



  /** Provides uniform random variables. */
  Ptr<UniformRandomVariable> m_pg;

  /** Energy model callback. */
  DeviceEnergyModel::ChangeStateCallback m_energyCallback;
  /** A packet destined for this Phy was received without error. */
  ns3::TracedCallback<Ptr<const Packet>, double, UanTxMode > m_rxOkLogger;
  /** A packet destined for this Phy was received with error. */
  ns3::TracedCallback<Ptr<const Packet>, double, UanTxMode > m_rxErrLogger;
  /** A packet was sent from this Phy. */
  ns3::TracedCallback<Ptr<const Packet>, double, UanTxMode > m_txLogger;


  double CalculateSinrDb (Ptr<Packet> pkt, Time arrTime, double rxPowerDb,
                          UanTxMode mode, UanPdp pdp);

  double GetInterferenceDb (Ptr<Packet> pkt);

  double DbToKp (double db);

  double KpToDb (double kp);
 
  void RxEndEvent (Ptr<Packet> pkt, double rxPowerDb, UanTxMode txMode);
  /** Event to process end of packet transmission. */
  void TxEndEvent ();

  void UpdatePowerConsumption (const State state);


  /** Call UanListener::NotifyRxStart on all listeners. */
  void NotifyListenersRxStart (void);  
  /** Call UanListener::NotifyRxEndOk on all listeners. */
  void NotifyListenersRxGood (void);
  /** Call UanListener::NotifyRxEndError on all listeners. */
  void NotifyListenersRxBad (void);
  /** Call UanListener::NotifyCcaStart on all listeners. */
  void NotifyListenersCcaStart (void);
  /** Call UanListener::NotifyCcaEnd on all listeners. */
  void NotifyListenersCcaEnd (void);
 
  void NotifyListenersTxStart (Time duration);

protected:
  virtual void DoDispose ();

};  // class UanPhyRealNewb

} // namespace ns3

#endif /* UAN_PHY_REAL_H */
