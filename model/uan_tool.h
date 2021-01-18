#ifndef __UAN_TOOL_H__
#define __UAN_TOOL_H__

#include <string>
#include <vector>
#include <list>
#include "uan-emulation-packet.h"


namespace ns3{

class UAN_Tool{
private:
    // remove the space of the begin or end
    static std::string rmSpace(std::string& strIn);

    // 1.
    template<class T> // T is integer
    static void itoStr(const T& number, std::string& str);
    
    static int strToI(const std::string& str, const int& size, int& nums);
    static int strToI(const char* p, const int& len, int& nums);

public:
 
    // string
    static std::vector<std::string> split(const std::string& strIn,
                                          const char&        splitSign = ',', 
                                          const bool&        rmSpace = true); 

    // get file size, file is less than 2G
    // if path is wrong , return -1.
    static int getFileSize(const std::string& path); 


    // protocol
    // 1. constheader
    static int cheaderToStr(const ConstHeader& cheader, std::vector<char>& strVect);
    static int cheaderToStr(const ConstHeader& cheader, std::list<char>& strList);
    static int cheaderToStr(const ConstHeader& cheader, std::string& str);
    
    static int strToCheader(const std::vector<char>& strVect, ConstHeader& cheader);
    static int strToCheader(const std::list<char>& strList,   ConstHeader& cheader);
    static int strToCheader(const std::string& str,           ConstHeader& cheader);

    // 2. varheader
    static int vheaderToStr(const VarHeader& vheader, std::vector<char>& strVect);
    static int vheaderToStr(const VarHeader& vheader, std::list<char>& strList);
    static int vheaderToStr(const VarHeader& vheader, std::string& str);

    static int strToVheader(const std::string& str, const int& nodeNums, const int& fileNameLen, VarHeader& vheader);

    // 3
    static std::string getCurrentTime(); 
    static std::string getCurrentDate();
    
};

} // end namespace

#endif