#include "uan-emulation-manager.h"
#include "uan-emulation-packet.h"
#include "ns3/network-module.h"
#include "ns3/stats-module.h"
#include "ns3/core-module.h"
#include "uan_tool.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h> 

//特别说明，ns3 node id来源于ns3仿真器生成，而uanVid则是符合uan server服务规则的ns3 客户端id，两者不相等。
//默认ns3 node id 从0开始累加，而uanVid则是从60001开始累加，两者一一对应。

namespace ns3{

NS_LOG_COMPONENT_DEFINE ("UanEmulationManager");

//class UanEmulationManager
 std::unordered_map<uint16_t,uint16_t>  UanEmulationManager::m_ns3ToModemMap;//ns3 node id,modem node id
 std::unordered_map<uint16_t,uint16_t>  UanEmulationManager::m_ns3ToUanVid;
 bool                                   UanEmulationManager::m_isMapValid = false;
 const std::string                      UanEmulationManager::m_serverIp = "127.0.0.1";
 const int                              UanEmulationManager::m_serverPort = 8888;
 int                                    UanEmulationManager::m_serverFd = 0;
 std::mutex                             UanEmulationManager::m_mutex;
 UanEmulationManager*                   UanEmulationManager::m_ptr = NULL;

UanEmulationManager* UanEmulationManager::getInstance(){
    if(m_ptr == NULL){
        m_mutex.lock();
        if(m_ptr == NULL){
            m_ptr = new UanEmulationManager();
        }
        m_mutex.unlock();
    }
    return m_ptr;
}

UanEmulationManager* UanEmulationManager::getInstance(std::vector<uint16_t> modemList){
    if(m_ptr == NULL){
        m_mutex.lock();
        if(m_ptr == NULL){
            m_ptr = new UanEmulationManager(modemList);
        }
        m_mutex.unlock();
    }
    return m_ptr;
}
UanEmulationManager* UanEmulationManager::getInstance(std::unordered_map<uint16_t,uint16_t> nodeMap){
    if(m_ptr == NULL){
        m_mutex.lock();
        if(m_ptr == NULL){
            m_ptr = new UanEmulationManager(nodeMap);
        }
        m_mutex.unlock();
    }
    return m_ptr;
}

void UanEmulationManager::setNodeMap(std::vector<uint16_t> modemList){
    m_mutex.lock();
    m_ns3ToModemMap.clear();
    for(unsigned int i = 0; i < modemList.size(); ++i){
        m_ns3ToModemMap[i] = modemList[i];
    }
    m_mutex.unlock();
    return;
}

bool UanEmulationManager::checkMapIsValid(std::vector<uint16_t> modemList){
     m_mutex.lock();
     //to finish
     NodeContainer allNode = NodeContainer::GetGlobal();
     if(allNode.GetN() != modemList.size()){
         NS_ASSERT_MSG(0,"NS3 Node number is not equal to Modemlist!");
     }


    std::vector<uint16_t> vidList;//uan ns3 node id
    uint16_t vidAllocator = NS3_VID_MIN;
    for(unsigned int i = 0; i < allNode.GetN(); ++i){
        m_ns3ToModemMap[i] = modemList[i];
        m_ns3ToUanVid[i] = vidAllocator;
        vidList.push_back(vidAllocator);
        ++ vidAllocator;
    }
    std::vector<uint16_t> vidAndModemList;
    vidAndModemList.insert(vidAndModemList.end(), vidList.begin(), vidList.end());
    vidAndModemList.insert(vidAndModemList.end(), modemList.begin(), modemList.end());



    struct sockaddr_in serverAddr;
    m_serverFd = socket(AF_INET, SOCK_STREAM, 0);
    if(m_serverFd < 0){
        NS_ASSERT_MSG(0,"fail to create socket");
    }

    bzero(&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port   = htons(m_serverPort);
    serverAddr.sin_addr.s_addr = inet_addr(m_serverIp.c_str());

    int ret = connect(m_serverFd, (struct sockaddr*)(&serverAddr), sizeof(serverAddr));
    if(ret < 0){
        NS_ASSERT_MSG(0,"fail to connect server");
    }

    
    VarHeader checkVarHeader;
    checkVarHeader._recvNodeList = vidAndModemList;
    
    ConstHeader checkConHeader;
    checkConHeader._sendId = NS3_VID_CHECK;
    checkConHeader._dataType = kns3NodeRuqestCheck;
    checkConHeader._methodType = kPacketRequest;
    checkConHeader._recvNodesNum = vidAndModemList.size();
    checkConHeader._sendContentLen = 0;//0
    
    UanEmuRecvSend::sendUanServerPkt(m_serverFd, checkConHeader, 
                                checkVarHeader, std::string());

    UanEmuPacket recvPkt = UanEmuRecvSend::recvUanServerPkt(m_serverFd);
    //close(m_serverFd);
    

    if(recvPkt.m_conHeader._dataType == kns3AllowAndStart){
        m_isMapValid = true;
        m_mutex.unlock();
        return true;
    }
    else if(recvPkt.m_conHeader._dataType == kns3NotAllow){
        m_mutex.unlock();
        return false;
    }
    else{
        //NS_LOG_INFO("Get Uan server DataType exception");
        m_mutex.unlock();
        return false;
    }

 }

UanEmulationManager::UanEmulationManager(std::vector<uint16_t> modemList){
    for(unsigned int i = 0; i < modemList.size(); ++i){
        m_ns3ToModemMap[i] = modemList[i];
    }
}
UanEmulationManager::UanEmulationManager(std::unordered_map<uint16_t,uint16_t> nodeMap){
    //need check!
    m_ns3ToModemMap = nodeMap;
}

void UanEmulationManager::stopNs3Task(){
    m_mutex.lock();
    if(m_isMapValid == true){
        m_isMapValid = false;
        NS_LOG_DEBUG("UanEmulationManager:tell uan server to Stop Ns3 Task!m_serverFd:"<<m_serverFd);
        UanEmuRecvSend::sendUanOnlyConstHeader(m_serverFd,kns3TaskStop,NS3_VID_CHECK);
        close(m_serverFd);
    }
    m_mutex.unlock();
}

uint16_t UanEmulationManager::getVidFromNs3Id(uint16_t ns3NodeId){//小于60000的id需要进行映射，映射到0+60001
    if(ns3NodeId >= NS3_VID_CHECK)
        return ns3NodeId;
    return m_ns3ToUanVid[ns3NodeId];
}

}//end namespace