#ifndef UAN_EMULATION_PACKET_H
#define UAN_EMULATION_PACKET_H

#include "ns3/object.h"
#include <vector>
#include <string>
#include "uan-emulation-manager.h"

namespace ns3{

const int kConHeadLen = 10;
const int kCheckHeadLen = 4; 

const uint8_t kUanHeaderArray[4] = {0x35, 0x71, 0x49, 0x68};//"5qIh"
const std::string kUanCheckHead = "5qIh";


struct ConstHeader{
    int8_t    _methodType;      // 1. request or reply
    int8_t    _dataType;        // 2. 
    uint16_t  _sendId;          // 3. send node's id
    int8_t    _recvNodesNum;    // 4. only web server can use it, and it is useless for uan server and uan nodes. 
    int8_t    _sendFileNameLen; // 5. present the file name's length, <=0, present a commond, >0 present send a file
    int32_t   _sendContentLen;  // 6. send content length.

    ConstHeader():_methodType(kPacketRequest),
                  _dataType(kns3Packet),
                  _sendId(NS3_VID_CHECK),
                  _recvNodesNum(0),
                  _sendFileNameLen(0),
                  _sendContentLen(0){}
};

struct VarHeader{
    std::vector<uint16_t>  _recvNodeList;
    std::string            _fileName;
}; 

class UanEmuPacket{
public:
    
    ConstHeader     m_conHeader;
    VarHeader       m_varHeader;
    std::string     m_content;

};

class UanEmuRecvSend//:public Object
{

public:
    static void sendUanServerNs3Pkt(int, const std::string&, uint32_t);
    static UanEmuPacket recvUanServerPkt(int);
    static UanEmuPacket recvUanServerPkt2(int);//not used

    static void sendUanServerPkt(int , ConstHeader, VarHeader, const std::string&);
    static void sendUanOnlyConstHeader(int, int8_t, uint32_t);

};




}//end namespace
#endif