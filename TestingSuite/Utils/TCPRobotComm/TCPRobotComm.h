/** @file    TCPRobotComm.h
 *  @brief   header file to TCPRobotComm class, which enables TCP communication between robot
 *           and computer.
 *  @author  Austin Small.
 */

#ifndef TCP_ROBOT_COMM_H
#define TCP_ROBOT_COMM_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <sys/time.h>
#include <unistd.h>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include "../TCPLibrary/TCPStream.h"
#include "../TCPLibrary/TCPAcceptor.h"
#include "../TCPLibrary/TCPConnector.h"

#define PORT 12500
#define MAX_PACKET_LENGTH 1400 // Max length of an individual packet.

namespace AL
{
    class ALBroker;
}

/** @brief		TCPRobotComm class allows for communicating data and commands from the Nao
 *              to a computer.
 */
class TCPRobotComm : public AL::ALModule
{
    public:
        // Methods.
        TCPRobotComm(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~TCPRobotComm();
        void exit(void);
    
    private:
        // Methods.
        void communicationThreadFunc(void);
        void handleTCPError(void);
        std::string delegate(std::string);
        std::string managePacketSize(std::string);
    
        // Fields.
        TCPAcceptor* acceptor;
        TCPStream* stream;
        boost::thread communicationThread;
        bool init;
        boost::shared_ptr<AL::ALBroker> broker;
        AL::ALProxy* proxy;
        std::string sendBufferString;
    
        static const int ACCEPT_TIMEOUT  = 3;
        static const int SEND_TIMEOUT    = 1;
        static const int RECV_TIMEOUT    = 1;
        static const int HEARTBEAT_TIME  = 45;
};

#endif