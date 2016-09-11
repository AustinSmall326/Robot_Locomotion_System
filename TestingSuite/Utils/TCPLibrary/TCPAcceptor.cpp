/** @file    TCPAcceptor.cpp
 *  @brief   cpp file to TCPAcceptor class, which encapsulates the socket mechanisms to
 *           passively accept connections from a client.
 *  @author  Austin Small.
 */

#include "TCPAcceptor.h"
#include <iostream>
#include <errno.h>

/** @brief   TCPAcceptor class constructor.
 *
 *  @param   port           Server listening port number.
 *  @param   address        Client IP address.
 */

TCPAcceptor::TCPAcceptor(int port, const char* clientIP) :
    sd(0),
    port(port),
    clientIP(clientIP),
    listening(false)
{ }

/** @brief   TCPAcceptor class destructor.
 *
 */

TCPAcceptor::~TCPAcceptor()
{
    if (close(sd) < 0)
    {
        std::cout << "failed to close socked descriptor" << std::endl;
    }
}

/** @brief   Start listening for connections.
 *
 */

int TCPAcceptor::start()
{
    // No need to call this method if already listening.
    if (listening == true)
    {
        return 0;
    }
    
    // Create a socket.  The socket signature is as follows: socket(int domain, int type, int protocol)
    sd = socket(PF_INET, SOCK_STREAM, 0);
    
    int optval = 1;
    
    if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval) == -1)
    {
        std::cout << "failed to set socket option" << std::endl;
    }
    
    // Configure address struct.
    struct sockaddr_in address;
    
    memset(&address, 0, sizeof(address));
    address.sin_family = PF_INET;
    address.sin_port = htons(port); // Convert from host to TCP network byte order.
    
    // The client IP address is optional, and if it is not specified, all client IPs are accepted.
    if (clientIP.size() > 0)
    {
        inet_pton(PF_INET, clientIP.c_str(), &(address.sin_addr));
    }
    else
    {
        address.sin_addr.s_addr = INADDR_ANY;
    }
    
    // Bind the listening socket address to the socket descriptor.
    int result = bind(sd, (struct sockaddr*)&address, sizeof(address));
    
    
    if (result < 0)
    {
        //char buffer[ 256 ];
        //char * errorMessage = strerror_r( errno, buffer, 256 ); // get string message from errn
        //std::string msg (errorMessage);
        //std::cout << msg << std::endl;
        std::cout << "failed on bind" << std::endl;
        return result;
    }
    
    // Begin listening for a client that is attempting to connect.
    result = listen(sd, 5);
    
    if (result < 0)
    {
        std::cout << "failed on listen" << std::endl;
        return result;
    }
    
    listening = true;
    
    return result;
}

/** @brief   Accept connections from clients.
 *
 *  @param  timeoutSec  Timeout period for accept command.
 *
 */

TCPStream* TCPAcceptor::accept(int timeoutSec)
{
    // Don't accept a connection if the socket is not in a listening state.
    if (listening == false)
    {
        return NULL;
    }
    
    // Create empty address struct.
    struct sockaddr_in address;
    
    socklen_t len = sizeof(address);
    memset(&address, 0, sizeof(address));
    
    // Attempt to accept connection with client with timeout on attempt.
    fd_set set;
    FD_ZERO(&set);    // Clear the set.
    FD_SET(sd, &set); // Add our file descriptor to the set.
    
    struct timeval timeout;
    timeout.tv_sec  = timeoutSec;
    timeout.tv_usec = 0;
    
    int ret;
    ret = select(sd + 1, &set, NULL, NULL, &timeout);
    
    // First check if an error or timeout occurred.  Otherwise, call accept method.
    if (ret == -1 || ret == 0)
    {
        return NULL;
    }
    else
    {
        // Address is filled in with the address of the peer socket, as known to the
        // communication layer.
        int tempsd = ::accept(sd, (struct sockaddr*)&address, &len);
        
        if (sd < 0)
        {
            return NULL;
        }
        
        return new TCPStream(tempsd, &address);
    }
}