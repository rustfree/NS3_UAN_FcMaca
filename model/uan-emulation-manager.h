#ifndef UAN_EMULATION_MANAGER_H
#define UAN_EMULATION_MANAGER_H
#include <unordered_map>
#include <vector>
#include <string>
#include <mutex>
#include "ns3/object.h"

namespace ns3{

//ns3 common
const uint16_t NS3_VID_CHECK= 60000;
const uint16_t NS3_VID_MIN = 60001;
const uint16_t NS3_VID_MAX = 65534;


const char kPacketRequest = 0x01;
const char kPacketReply   = 0x02;

//ns3  to uan server
const char kns3NodeInfo = 0x30;
const char kns3TaskStop = 0x31;
const char kns3Packet = 0x32;
const char kns3NodeRuqestCheck = 0x33;
const char kns3RecvThreadStop = 0x34;

//from uan server
const char kns3AllowAndStart = 0x44;
const char kns3ModemStart = 0x45;
const char kns3NotAllow = 0x46;
const char kns3SendFinished = 0x47;


class UanEmulationManager//:public Object
{
private:
    static std::unordered_map<uint16_t,uint16_t>    m_ns3ToModemMap;//ns3 node id,modem node id
    static std::unordered_map<uint16_t,uint16_t>    m_ns3ToUanVid;
    static bool                                     m_isMapValid;
    static const std::string                        m_serverIp;
    static const int                                m_serverPort;
    static int                                      m_serverFd;
    static std::mutex                               m_mutex;
    static UanEmulationManager*                     m_ptr;

private:
    UanEmulationManager(const UanEmulationManager&) = delete;
    UanEmulationManager& operator=(const UanEmulationManager&) = delete;

    UanEmulationManager(std::vector<uint16_t> modemList);
    UanEmulationManager(std::unordered_map<uint16_t,uint16_t> nodeMap);
    UanEmulationManager(){}

public:
    static UanEmulationManager* getInstance();
    static UanEmulationManager* getInstance(std::vector<uint16_t> ModemList);
    static UanEmulationManager* getInstance(std::unordered_map<uint16_t,uint16_t> nodeMap);

    bool checkMapIsValid(std::vector<uint16_t> modemList);
    void setNodeMap(std::vector<uint16_t> ModemList);
    void stopNs3Task();
    uint16_t getVidFromNs3Id(uint16_t);

};



}
#endif