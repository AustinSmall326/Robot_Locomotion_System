/** @file    TCPHeartbeatWrapper.h
 *  @brief   header file to TCPHeartbeatWrapper class, which wraps the TCPStream class, and implements
 *           a heartbeat for connection verification.
 *  @author  Austin Small.
 */

#include <vector>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <sys/time.h>
#include <unistd.h>

#include "../TCPLibrary/TCPStream.h"
#include "../TCPLibrary/TCPAcceptor.h"
#include "../TCPLibrary/TCPConnector.h"

#ifndef TCP_HEARTBEAT_WRAPPER_H
#define TCP_HEARTBEAT_WRAPPER_H

class TCPHeartbeatWrapper
{
    public:
        // Methods.
        TCPHeartbeatWrapper(int argPort, int argMaxLength);
        ~TCPHeartbeatWrapper(void);
    
        int connect(std::string IP);
        int communicate(char* inputMsg, char* buffer);
    
        bool isActive(void);
    
    private:
        // Methods.
        void mainThreadFunc(void);
        void waitForResponseFunc(void);
    
        // Fields.
        TCPConnector* connector;
        TCPStream* stream;
    
        bool mainThreadActive;
        boost::thread* mainThread;
    
        std::vector<std::string> inbuffer;
        std::vector<std::string> outbuffer;
        bool connectionActive;
        bool applicationClosing;
        bool waitingForResponse;
    
        bool transmissionInProgress;
        std::string receiveBufferString;
    
        const int PORT;
        const int MAX_PACKET_LENGTH;
        static const int CONNECT_TIMEOUT = 15;
        static const int SEND_TIMEOUT    = 1;
        static const int RECV_TIMEOUT    = 45;
        static const int HEARTBEAT_TIME  = 2;
};

#endif