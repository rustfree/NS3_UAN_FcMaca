
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
using std::unordered_set;
using std::vector;
using std::unordered_map;

namespace ns3
{

class UanPhy;
class UanTxMode;
template <typename Item> class Queue;

class UanMacNewAloha : public UanMac
{
public:
  /** Default constructor */
  UanMacNewAloha ();
  /** Dummy destructor, see DoDispose. */
  virtual ~UanMacNewAloha ();
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
  void HandleTimeout(Ptr<Packet> pkt,uint16_t protocolNumber);
  void sendpacket(Ptr<Packet> pkt, uint32_t modeNum);
  void SendQueuePacket(void);
private:
  enum State
  {
    IDLE,     //!< Idle state.
    WAITACK,  //等待ack
    BACKOFF   //退避
  };
  enum Flag
  {
    RECVACK,  //收到ack但handletime未处理
    DONEACK
  };
  Ptr<UniformRandomVariable> m_erv;
  bool AckTxWait = false;
  EventId m_HandleTimeoutEvent;
  vector<EventId> handevent;
  State m_state;

  int ackPktSize = 250;

  int enQueNum = 0, txDataNum = 0,rxDataNumGood = 0,rxDataNumUnique = 0, rxDataNumError = 0,rxAckNum = 0,txAckNum = 0;
  int txArp = 0,rxArp = 0,DropPktNum = 0;
  int resendNum = 0;//还有重发次数
  uint16_t m_nextUid = 0;//赋值给mac包头uid，数据包的唯一标识

  static int m_maxNodeNum;
  vector<unordered_set<uint16_t>> m_srcUidSetVec = vector<unordered_set<uint16_t>>(m_maxNodeNum, unordered_set<uint16_t>());

  static vector<unordered_map<uint16_t,Time>> m_enQueTime;
  double totalDelay = 0;

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

};  // class UanMacNewAloha

} // namespace ns3

#endif /* UAN_MAC_ALOHA_H */
