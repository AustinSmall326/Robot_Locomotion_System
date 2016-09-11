/** @file    TCPAcceptor.h
 *  @brief   header file to TCPAcceptor class, which encapsulates the socket mechanisms to
 *           passively accept connections from a client.
 *  @author  Austin Small.
 */

#ifndef TCP_ACCEPTOR_H
#define TCP_ACCEPTOR_H

#include <netinet/in.h>
#include <string>
#include <string.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <fcntl.h>

#include "TCPStream.h"

using namespace std;

class TCPAcceptor
{
    public:
        // Methods.
        TCPAcceptor(int port, const char* address);
        ~TCPAcceptor();
    
        int start();
        TCPStream* accept(int timeoutSec);
    
    private:
        // Fields.
        int     sd;
        string  clientIP;
        int     port;
        bool    listening;
};

#endif