
/*
 2018/6/19
 * Author: LCH
 */

#ifndef UAN_MAC_NEW_ALOHA_H
#define UAN_MAC_NEW_ALOHA_H

#include "uan-mac.h"
#include "ns3/mac8-address.h"
#include "ns3/event-id.h"
#include "ns3/random-variable-stream.h"
//#include "ns3/ptr.h"
//#include <cstring>
#include <vector>
#include <unordered_set>
#include <unordered_map>
using std::vector;
using std::unordered_set;
using std::unordered_map;

namespace ns3
{

class UanPhy;
class UanTxMode;
template <typename Item> class Queue;

class UanMacFcMaca : public UanMac
{
public:
  /** Default constructor */
  UanMacFcMaca ();
  /** Dummy destructor, see DoDispose. */
  virtual ~UanMacFcMaca ();
  /**
   * Register this type.
   * \return The TypeId.
   */
  static TypeId GetTypeId (void);


  // Inherited methods
  virtual bool Enqueue (Ptr<Packet> pkt, uint16_t protocolNumber, const Address &dest);
  virtual void SetForwardUpCb (Callback<void, Ptr<Packet>, uint16_t, const Mac8Address&> cb);
  virtual void AttachPhy (Ptr<UanPhy> phy);
  virtual void Clear (void);
  virtual void NotifyTxEnd(void);

  int64_t AssignStreams (int64_t stream);
  void HandleCTSTimeout(Ptr<Packet> pkt,uint16_t protocolNumber);
  void sendPacketToPhy(Ptr<Packet> pkt, uint32_t modeNum);
  //void SendQueuePacket(void);
  void SendRTSPacket(void);
  void HandleBackOffTimeout(void);
  void UpdateFcw(void);
  void UpdateFcwLinear(void);
  void UpdateFcwPiecewise(void);
private:
  enum State
  {
    IDLE,     //!< Idle state.
    WAITCTS,  //等待CTS
    BACKOFF,   //退避(让其它节点发送)
    TOSEND     //待发送
  };
 enum PacketType
  {
    RTS_TYPE,
    CTS_TYPE,
    DATA_TYPE
  };

  uint16_t m_timeoutCnt = 0;
  Ptr<UniformRandomVariable> m_erv;
  bool AckTxWait = false;
  EventId m_HandleCtsTimeoutEvent;
  EventId m_rtsToSendEvent;
  EventId m_HandleBackoffTimeoutEvent;
  vector<EventId> handevent;
  State m_state;

  int rtsLen = 250;
  int ctsLen = rtsLen;
  int txDataNum = 0, enQueNum = 0, rxDataNumUnique = 0, rxDataNumGood = 0,rxDataNumError = 0,txCTSNum = 0,txRTSNum = 0;
  int DropPktNum = 0;


  static int m_maxNodeNum;
  vector<unordered_set<uint16_t>> m_srcUidSetVec = vector<unordered_set<uint16_t>>(m_maxNodeNum, unordered_set<uint16_t>());

  static vector<unordered_map<uint16_t,Time>> m_enQueTime;
  double totalDelay = 0;

  vector<uint16_t> m_srcQueNum = vector<uint16_t>(m_maxNodeNum, 0);
  uint16_t m_srcQueNumSum = 0;
  uint16_t m_curFcw = 1;
  uint16_t m_tosendNum = 0;
  


  uint16_t m_nextUid = 0;
  Ptr<Queue<Packet>> m_queue;
  //Flag m_flag;
  int16_t timeout_cnt = 0;
  /** The MAC address. */
  Mac8Address m_address;
  /** PHY layer attached to this MAC. */
  Ptr<UanPhy> m_phy;
  /** Forwarding up callback. */
  Callback<void, Ptr<Packet>, uint16_t, const Mac8Address& > m_forUpCb;
  /** Flag when we've been cleared. */
  bool m_cleared;

  void RxPacketGood (Ptr<Packet> pkt, double sinr, UanTxMode txMode);
  void RxPacketError (Ptr<Packet> pkt, double sinr);
protected:
  virtual void DoDispose ();

};  // class UanMacFcMaca

} // namespace ns3

#endif /* UAN_MAC_ALOHA_H */
