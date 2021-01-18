#ifndef __UAN_BYTEORDER_H__
#define __UAN_BYTEORDER_H__

#include <iostream>
#include <mutex>

namespace ns3{

class ByteOrder{
private:
    static ByteOrder* m_ptr;
    bool              m_isLitter;
private:
    ByteOrder(){
        init();
    }
    ByteOrder(const ByteOrder& b) = delete;
    ByteOrder& operator=(const ByteOrder& b);
    void init(){
        int iBuff = 0x12345678;
	char cBuf = *(char*)(&iBuff);
	if(cBuf == 0x12){
	    m_isLitter = false;
	}else{
	    m_isLitter = true;
	}
    }
public:
    static ByteOrder* getInstance(){
        if(m_ptr == NULL){
	    std::mutex mt;
	    mt.lock();
	    if(m_ptr == NULL){
	        m_ptr = new ByteOrder();
	    }
	    mt.unlock();
	}
	return m_ptr;
    }
    bool isLitter(){
        return m_isLitter;
    }
};
ByteOrder* ByteOrder::m_ptr = NULL;
} // end namespace;

#endif
