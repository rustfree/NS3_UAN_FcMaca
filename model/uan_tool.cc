#include "uan_tool.h"
#include "uan_byteOrder.h"
//#include "uan/uan_common.h"
#include <algorithm>
#include <sys/stat.h> // _stat
#include <ctime>
#include <iostream>
#include <sstream>

namespace ns3{

template<class T> // T is integer
void UAN_Tool::itoStr(const T& number, std::string& str){
    str.clear();
    int len = sizeof(T);
    char* pBuff = (char*)(&number);

    if(ByteOrder::getInstance()->isLitter()){ // 
        for(int loop(0); loop < len; ++loop){
           str.insert(str.begin(), *(pBuff + loop));
        }
    }else{
        for(int loop(0); loop < len; ++loop){
            str.push_back(*(pBuff + loop));
        }
    }    
}
    
int UAN_Tool::strToI(const std::string& str, const int& size, int& nums){
    nums = 0;
    if (int(str.size()) >= size) {
		char* pBuff = (char*)(&nums);
		if (ByteOrder::getInstance()->isLitter()) {
			for (int loop(0); loop < size; ++loop) {
				*(pBuff + loop) = str[size - 1 - loop];
			}
		}
		else {
			for (int loop(0); loop < size; ++loop) {
				*(pBuff + loop) = str[loop];
			}
		}
	}
	else {
		return -1;
	}
    return 0;
}
int UAN_Tool::strToI(const char* p, const int& len, int& nums){
    nums = 0;
    if(p == NULL){
        return -1;
    }
    char* pBuff = (char*)(&nums);
    if(ByteOrder::getInstance()->isLitter()){
        for(int loop(0); loop < len; ++loop){
            *(pBuff + loop) = *(p + len - 1 - loop);
        }
    }else{
        for(int loop(0); loop < len; ++loop){
            *(pBuff + loop) = *(p + loop);
        }
    }

    return 0;
}

int UAN_Tool::getFileSize(const std::string& path){
    int fileSize = -1;
    struct stat statBuff;
    if(stat(path.c_str(), &statBuff) >= 0){
        fileSize = statBuff.st_size;
    }
    return fileSize;
}

int UAN_Tool::cheaderToStr(const ConstHeader& cheader, std::vector<char>& strVect)
{
    strVect.clear();
    strVect.push_back(cheader._methodType);
    strVect.push_back(cheader._dataType);
    std::string buff("");
    UAN_Tool::itoStr<uint16_t>(cheader._sendId, buff);
    for(auto ite : buff){
        strVect.push_back(ite);
    }
    strVect.push_back(cheader._recvNodesNum);
    strVect.push_back(cheader._sendFileNameLen);
    UAN_Tool::itoStr<int32_t>(cheader._sendContentLen, buff);
    for(auto ite : buff){
        strVect.push_back(ite);
    }
    return 0;
}

int UAN_Tool::cheaderToStr(const ConstHeader& cheader, std::list<char>& strList)
{
    strList.clear();
    strList.push_back(cheader._methodType);
    strList.push_back(cheader._dataType);
    std::string buff("");
    UAN_Tool::itoStr<uint16_t>(cheader._sendId, buff);
    for(auto ite : buff){
        strList.push_back(ite);
    }
    strList.push_back(cheader._recvNodesNum);
    strList.push_back(cheader._sendFileNameLen);
    UAN_Tool::itoStr<int32_t>(cheader._sendContentLen, buff);
    for(auto ite : buff){
        strList.push_back(ite);
    }
    return 0;
}
int UAN_Tool::cheaderToStr(const ConstHeader& cheader, std::string& str){
    str.clear();
    str.push_back(cheader._methodType);
    str.push_back(cheader._dataType);
    std::string buff("");
    UAN_Tool::itoStr<uint16_t>(cheader._sendId, buff);
    str += buff;

    str.push_back(cheader._recvNodesNum);
    str.push_back(cheader._sendFileNameLen);
    UAN_Tool::itoStr<int32_t>(cheader._sendContentLen, buff);
    str += buff;
    
    return 0;
}
    
int UAN_Tool::strToCheader(const std::vector<char>& strVect, ConstHeader& cheader)
{
    if(strVect.size() < kConHeadLen){
        return -1;
    }
    cheader._methodType = strVect[0];
    cheader._dataType   = strVect[1];
    int numBuff(0);
    UAN_Tool::strToI(&strVect[2], 2, numBuff);
    cheader._sendId = numBuff;
    cheader._recvNodesNum = strVect[4];
    cheader._sendFileNameLen = strVect[5];
    UAN_Tool::strToI(&strVect[6], 4, numBuff);
    cheader._sendContentLen = numBuff;
    return 0;

}

int UAN_Tool::strToCheader(const std::list<char>& strList,   
                           ConstHeader&      cheader)
{
    if(strList.size() < kConHeadLen){
        return -1;
    }
    std::list<char>::const_iterator ite = strList.begin();
    for(int loop(0); loop <= kConHeadLen; ++loop){
        ++ite;
    }
    std::vector<char> vectBuff(strList.begin(),ite);
    strToCheader(vectBuff, cheader);
    return 0;
}

int UAN_Tool::strToCheader(const std::string& str,           
                           ConstHeader&  cheader)
{
    if(str.size() < kConHeadLen){
        return -1;
    }
    std::vector<char> vectBuff(str.begin(), str.begin() + kConHeadLen);
    strToCheader(vectBuff, cheader);
    return 0;
}

//  2.
int UAN_Tool::vheaderToStr(const VarHeader& vheader, std::vector<char>& strVect){
    std::string str("");
    vheaderToStr(vheader, str);
    strVect.resize(str.size());
    std::copy(str.begin(), str.end(), strVect.begin());
    return 0;
}
int UAN_Tool::vheaderToStr(const VarHeader& vheader, std::list<char>& strList){
    std::string str("");
    vheaderToStr(vheader, str);
    strList.resize(str.size());
    std::copy(str.begin(), str.end(), strList.begin());
    return 0;
}

int UAN_Tool::vheaderToStr(const VarHeader& vheader, std::string& str)
{
    str.clear();
    std::string buff("");
    for(auto ite : vheader._recvNodeList){
        UAN_Tool::itoStr<uint16_t>(ite, buff);
        str += buff;
    }
    str += vheader._fileName;
    return 0;
}

int UAN_Tool::strToVheader(const std::string& str, 
                           const int&         nodeNums, 
                           const int&         fileNameLen, 
                           VarHeader&    vheader)
{
    int totalLen = nodeNums*2 + fileNameLen;
    if(int(str.size()) < totalLen){
        return -1;
    }
    int iBuff(0);
    const char* p = str.c_str();
    vheader._recvNodeList.resize(nodeNums);
    for(int loop(0); loop < nodeNums; ++loop){
        UAN_Tool::strToI(p + 2*loop, 2, iBuff);
        vheader._recvNodeList[loop] = iBuff;
    }
    vheader._fileName = str.substr(nodeNums*2, fileNameLen);
    return 0;
}

/**********************************************
 * function: std::string UAN_Log::getCurrentTime()
 * 
***********************************************/
std::string UAN_Tool::getCurrentTime(){
    std::string res = "";
    std::time_t t = std::time(nullptr);
    struct tm* pt = std::gmtime(&t);
    res += std::to_string(pt->tm_year + 1900) + "-" + std::to_string(pt->tm_mon + 1 ) + "-" + std::to_string(pt->tm_mday);
	res += " " + std::to_string(pt->tm_hour +8 ) + ":" + std::to_string(pt->tm_min) + ":" + std::to_string(pt->tm_sec);
	return res;
}

std::string UAN_Tool::getCurrentDate(){
    std::string res = "";
    std::time_t t = std::time(nullptr);
    struct tm* pt = std::gmtime(&t);
    res += std::to_string(pt->tm_year + 1900) + "-" + std::to_string(pt->tm_mon + 1 ) + "-" + std::to_string(pt->tm_mday);
    return res;
}

//by lch 
std::vector<std::string> UAN_Tool::split(const std::string& strIn,
                                          const char& splitSign, 
                                          const bool& rmSpace){
    std::vector<std::string> result;
    std::string temp;
    std::stringstream iss(strIn);
    while(getline(iss,temp,splitSign)){
        result.push_back(temp);
    }
    if(rmSpace == true){
        //strIn.clear();
    }
    return result;
    }
} // end namespace