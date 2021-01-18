/*   ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
 **    uan-phy-real-new.h                     *
 **    Created on 2018/10/13                *
 **    Author:lch                          *
 **    For:connecting with real channel    *
 **  ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * ~ * 
*/
#ifndef UAN_PHY_REAL_CLOUD_H
#define UAN_PHY_REAL_CLOUD_H


#include "uan-phy.h"
#include "ns3/traced-callback.h"
#include "ns3/nstime.h"
#include "ns3/device-energy-model.h"
#include "ns3/random-variable-stream.h"
#include "ns3/event-id.h"
#include "ns3/system-thread.h"
#include "ns3/net-device.h"
#include <list>
#include <vector>

//#include <cstring>

namespace ns3 {

std::vector<int> fd_list;

/**
 * \ingroup uan
 */
void * ReceivePacket(void * arg);

class UanPhyRealCloudPerGenDefault : public UanPhyPer
{
public:
  /** Constructor */
  UanPhyRealCloudPerGenDefault ();
  /** Destructor */
  virtual ~UanPhyRealCloudPerGenDefault ();
  static TypeId GetTypeId (void);
  virtual double CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode);
private: 
  double m_thresh;  //!< SINR threshold.

};  // class UanPhyRealCloudPerGenDefault


class UanPhyRealCloudPerUmodem : public UanPhyPer
{
public:
  /** Constructor */
  UanPhyRealCloudPerUmodem ();
  /** Destructor */
  virtual ~UanPhyRealCloudPerUmodem ();
  static TypeId GetTypeId (void); 
  virtual double CalcPer (Ptr<Packet> pkt, double sinrDb, UanTxMode mode);

private:

  double NChooseK (uint32_t n, uint32_t k);

};  // class UanPhyRealCloudPerUmodem


/**
 * \ingroup uan
 */
class UanPhyRealCloudCalcSinrDefault : public UanPhyCalcSinr
{

public:
  /** Constructor */
  UanPhyRealCloudCalcSinrDefault ();
  /** Destructor */
  virtual ~UanPhyRealCloudCalcSinrDefault ();
  
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

};  // class UanPhyRealCloudCalcSinrDefault


/**
 * \ingroup uan
 */
class UanPhyRealCloudCalcSinrFhFsk : public UanPhyCalcSinr
{

public:
  /** Constructor */
  UanPhyRealCloudCalcSinrFhFsk ();
  /** Destructor */
  virtual ~UanPhyRealCloudCalcSinrFhFsk ();

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

};  // class UanPhyRealCloudCalcSinrFhFsk


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
class UanPhyRealCloud : public UanPhy
{
public:
  /** Constructor */
  UanPhyRealCloud ();
  /** Dummy destructor, see DoDispose */
  virtual ~UanPhyRealCloud ();
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
  

  void StartRX(void);

  void Init(void);//数据初始化


  bool IsrStateRX(void);
  void setrStateIDLE(void);
  int readnFd(void);
  void ConnectUanServer(void);

  /*thread method*/
  void sendDataThread(void);
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
  
  double m_minRxSinrDb=0;             //!< Minimum receive SINR during packet reception.
  double m_rxRecvPwrDb;             //!< Receiver power.
  Time m_pktRxArrTime;              //!< Packet arrival time.
  UanPdp m_pktRxPdp;                //!< Power delay profile of pakket.
  UanTxMode m_pktRxMode;            //!< Packet transmission mode at receiver.

  bool m_cleared;                   //!< Flag when we've been cleared.
  
  EventId m_txEndEvent;             //!< Tx event
  EventId m_rxEndEvent;             //!< Rx event
  EventId m_callEndEvent; 

    
  uint32_t m_nodeId = 0;
  uint16_t m_vid; // uan ns3 node id;
  /*veriable about the socket*/

  int nRet = 0;
  int nReadLen =0;
  int packetSize = 0;

 

  //统计
  int txPktNum = 0;
  int rxPktNum = 0;

  Ptr<SystemThread> recvthr,sendthr;

 
 
  int m_uanServerPort = 8888;  //端口号
  uint32_t uanServerAddr=0; //???
  int m_uanServerFd = 0;
  std::string m_uanServerIp = "127.0.0.1";

  //const char * IpList[3]={"192.168.2.101","192.168.2.103","192.168.2.104"};//send,recv,router
  



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

};  // class UanPhyRealCloud

} // namespace ns3

#endif /* UAN_PHY_REAL_H */
