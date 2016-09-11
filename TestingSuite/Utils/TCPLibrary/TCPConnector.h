/** @file    TCPConnector.h
 *  @brief   header file to TCPConnector class, which encapsulates the socket mechanisms to actively
 *           connect to a server.
 *  @author  Austin Small.
 */

#ifndef TCP_CONNECTOR_H
#define TCP_CONNECTOR_H

#include <netinet/in.h>
#include <string.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <fcntl.h>

#include "TCPStream.h"

class TCPConnector
{
    public:
        // Methods.
        TCPStream* connect(const char* server, int port, int timeoutSec);
    
    private:
};

#endif