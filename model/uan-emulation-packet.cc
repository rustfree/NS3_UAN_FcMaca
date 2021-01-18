#include"uan-emulation-packet.h"
#include "ns3/assert.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "uan_tool.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <arpa/inet.h> 

namespace ns3{

NS_LOG_COMPONENT_DEFINE ("UanEmulationPacket");
//NS_OBJECT_ENSURE_REGISTERED (UanEmuPacket);


UanEmuPacket UanEmuRecvSend::recvUanServerPkt2(int serverFd){//not stable because of the recv
    NS_LOG_DEBUG("UanEmuRecvSend::recvUanServerPkt()");
    UanEmuPacket recvPkt;
    char * tcprecvbuffer = (char *)malloc(1024);
    int ReadLen;

    //1.get constHeader
    ReadLen = recv(serverFd,tcprecvbuffer,kConHeadLen,0);//not stable here,can't get complete data sometime
    if(ReadLen != kConHeadLen){
        std::cout<<"Exception! readLen:"<<ReadLen<<std::endl;
        NS_LOG_DEBUG("Exception! readLen:");
        //NS_ASSERT_MSG(0,"recv Const Header Exception!");
    }
    
    UAN_Tool::strToCheader(std::string(tcprecvbuffer,tcprecvbuffer+ReadLen),
                                                    recvPkt.m_conHeader);
    //2.get varheader
    int varLen = recvPkt.m_conHeader._recvNodesNum*2 + recvPkt.m_conHeader._sendFileNameLen;
    if(varLen > 0){
        ReadLen = recv(serverFd,tcprecvbuffer,varLen,0);
        if(ReadLen != varLen){
            NS_ASSERT_MSG(0,"recv Var Header Exception!");
        }
        UAN_Tool::strToVheader(std::string(tcprecvbuffer,tcprecvbuffer+ReadLen),
                                recvPkt.m_conHeader._recvNodesNum,
                                recvPkt.m_conHeader._sendFileNameLen,
                                recvPkt.m_varHeader);
    }

    //3.get Content

    int contenLen = recvPkt.m_conHeader._sendContentLen;
    //int countLen = 0;
    if(contenLen > 0){
        ReadLen = recv(serverFd,tcprecvbuffer,contenLen,0);
        if(ReadLen != varLen){
            NS_LOG_DEBUG("recv ContentLen:"<<contenLen<<",recv ReadLen:"<<ReadLen);
            NS_ASSERT_MSG(0,"recv Content Exception!");
        }
        recvPkt.m_content = std::string(tcprecvbuffer, tcprecvbuffer+ReadLen);
    }

    NS_LOG_DEBUG("UanEmuRecvSend::recv new pkt from Uan server!");
    free(tcprecvbuffer);
    return recvPkt;
}

UanEmuPacket UanEmuRecvSend::recvUanServerPkt(int serverFd){
    NS_LOG_DEBUG("UanEmuRecvSend::recvUanServerPkt()");
    UanEmuPacket recvPkt;
    char * tcprecvbuffer = (char *)malloc(1024);
    int readLen, leftLen, cntLen;

    //0.get check header
    leftLen = kCheckHeadLen;
    cntLen = 0;
    do{
        readLen = recv(serverFd,tcprecvbuffer+cntLen,leftLen,0);
        leftLen = leftLen - readLen;
        cntLen += readLen;
    }while (leftLen > 0);
    for(int i = 0; i < kCheckHeadLen; ++i){
        if(tcprecvbuffer[i] != kUanHeaderArray[i]){
            NS_LOG_DEBUG("Get Check Head:"<<tcprecvbuffer[i]);
            NS_ASSERT_MSG(0,"Check Head error!");
        }
    }

    //1.get constHeader
    leftLen = kConHeadLen;
    cntLen = 0;
    do{
        readLen = recv(serverFd,tcprecvbuffer+cntLen,leftLen,0);
        leftLen = leftLen - readLen;
        cntLen += readLen;

    }while (leftLen > 0);

    UAN_Tool::strToCheader(std::string(tcprecvbuffer,tcprecvbuffer+cntLen), recvPkt.m_conHeader);

    //2.get varheader
    int varLen = recvPkt.m_conHeader._recvNodesNum*2 + recvPkt.m_conHeader._sendFileNameLen;
    leftLen = varLen;
    cntLen = 0;

    if(leftLen > 0){
        do{
            readLen = recv(serverFd,tcprecvbuffer+cntLen,leftLen,0);
            leftLen = leftLen - readLen;
            cntLen += readLen;
        }while (leftLen > 0);
    }

    UAN_Tool::strToVheader(std::string(tcprecvbuffer,tcprecvbuffer+cntLen),
                            recvPkt.m_conHeader._recvNodesNum,
                            recvPkt.m_conHeader._sendFileNameLen,
                            recvPkt.m_varHeader);

    //3.get Content
    int contenLen = recvPkt.m_conHeader._sendContentLen;
    leftLen = contenLen;
    cntLen = 0;
    if(contenLen > 1024){
        NS_ASSERT_MSG(0,"Content is too long to receive");
    }

    if(contenLen > 0){
        do{
            readLen = recv(serverFd,tcprecvbuffer+cntLen,leftLen,0);
            leftLen = leftLen - readLen;
            cntLen += readLen;
        }while (leftLen > 0);
    }
    recvPkt.m_content = std::string(tcprecvbuffer, tcprecvbuffer+cntLen);

    NS_LOG_DEBUG("UanEmuRecvSend::recv new pkt from Uan server!");
    free(tcprecvbuffer);
    return recvPkt;

}

void UanEmuRecvSend::sendUanServerPkt(int serverFd, ConstHeader cheader, 
                                        VarHeader vheader, const std::string& pktcontent){

    // check
    if((cheader._recvNodesNum != int(vheader._recvNodeList.size())) ||
       (cheader._sendFileNameLen != int(vheader._fileName.size())) ||
       (cheader._sendContentLen != int(pktcontent.size()))
    ){
        NS_ASSERT_MSG(0,"ConstHeader , VarHeader or cmdStr error! please check it");
    }
    //combine the packet
    std::string totalPakcet = kUanCheckHead;
    std::string tempString;

    UAN_Tool::cheaderToStr(cheader,tempString);
    totalPakcet += tempString;
    tempString.clear();

    UAN_Tool::vheaderToStr(vheader,tempString);
    totalPakcet += tempString;
    tempString.clear();

    totalPakcet += pktcontent;

    //send to the uan server
    int ret = send(serverFd,totalPakcet.c_str(),totalPakcet.size(),0);
    if(ret > 0){
        NS_LOG_DEBUG("UanEmuRecvSend::send new pkt to Uan server!SendId:"<<cheader._sendId);
    }
    return;
}

void UanEmuRecvSend::sendUanServerNs3Pkt(int serverFd, const std::string& pkt, uint32_t ns3NodeId){

    UanEmulationManager * ptr = UanEmulationManager::getInstance();
    uint16_t sendVid = ptr->getVidFromNs3Id(ns3NodeId);
    ConstHeader cheader;  /// 1 
    VarHeader vheader;    // 2
    cheader._sendId = sendVid;
    cheader._methodType = kPacketRequest;
    cheader._dataType = kns3Packet;
    cheader._sendContentLen = pkt.size();

    sendUanServerPkt(serverFd, cheader,vheader,pkt);

    return;
}

void UanEmuRecvSend::sendUanOnlyConstHeader(int serverFd, int8_t dataType, uint32_t ns3NodeId){

    UanEmulationManager * ptr = UanEmulationManager::getInstance();
    uint16_t sendVid = ptr->getVidFromNs3Id(ns3NodeId);
    ConstHeader cheader;  /// 1 
    VarHeader vheader;    // 2
    cheader._sendId = sendVid;
    cheader._methodType = kPacketRequest;
    cheader._dataType = dataType;

    sendUanServerPkt(serverFd, cheader,vheader,std::string());
}

}//end namespace