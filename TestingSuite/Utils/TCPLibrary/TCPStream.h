/** @file    TCPStream.h
 *  @brief   header file to TCPStream class, which allows for communicating data between
 *           robot and a computer.
 *  @author  Austin Small.
 */

#ifndef TCP_STREAM_H
#define TCP_STREAM_H

#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string>
#include <arpa/inet.h>
#include <errno.h>

using namespace std;

class TCPStream
{
    public:
        friend class TCPAcceptor;
        friend class TCPConnector;
    
        // Methods.
        ~TCPStream();
    
        ssize_t send(const char* buffer, size_t len, int timeoutSec);
        ssize_t receive(char* buffer, size_t len, int timeoutSec);
    
        string getPeerIP();
        int getPeerPort();
    
    private:
        // Methods.
        // Constructor is private, and therefore only accessible to friend classes.
        TCPStream(int sd, struct sockaddr_in* address);
    
        // Fields.
        int     sd;
        string  peerIP;
        int     peerPort;
        int     timeoutSec;
};

#endif