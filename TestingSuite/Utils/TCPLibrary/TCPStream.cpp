/** @file    TCPStream.cpp
 *  @brief   cpp file to TCPStream class, which provides methods to send and receive 
 *           data over a TCP/IP connection.
 *  @author  Austin Small.
 */

#include "TCPStream.h"
#include <iostream>

/** @brief   TCPStream class constructor.
 *
 *  @param   argsd      Socket descriptor.
 *  @param   address    sockaddr_in struct.
 */

TCPStream::TCPStream(int argsd, struct sockaddr_in* address) :
    sd(argsd)
{
    char ip[50];
    
    // Convert a numeric address into a text string.
    //      struct sockaddr_in
    //      {
    //          short sin_family;
    //          unsigned short sin_port;
    //          struct in_addr sin_addr;
    //          char sin_zero[8];
    //      };
    //
    inet_ntop(PF_INET, (struct in_addr*)&(address->sin_addr.s_addr), ip, sizeof(ip));
    
    peerIP = ip;
    
    // Convert from network byte order to host byte order.
    peerPort = ntohs(address->sin_port);
}

/** @brief   TCPComputerComm class destructor.
 *
 */

TCPStream::~TCPStream()
{
    std::cout << "closing fd" << std::endl;
    
    if (close(sd) < 0)
    {
        std::cout << "file descriptor not closed successfully" << std::endl;
    }
}

/** @brief   Wrapper function to send data.
 *
 *  @param   buffer     Pointer to first character of string.
 *  @param   len        Size of input string.
 *  @param   timeoutSec Timeout period for write command.
 *
 *  @return  Number of bytes written, -1 if a non-timeout error occured, and -2 if a timeout occurred.
 */

ssize_t TCPStream::send(const char* buffer, size_t len, int timeoutSec)
{
    // Attempt to send data with a timeout on write.
    fd_set set;
    FD_ZERO(&set);    // Clear the set.
    FD_SET(sd, &set); // Add our file descriptor to the set.
    
    struct timeval timeout;
    timeout.tv_sec  = timeoutSec;
    timeout.tv_usec = 0;
    
    int ret;
    ret = select(sd + 1, NULL, &set, NULL, &timeout);
    
    // First check if an error or timeout occurred.  Otherwise, call accept method.
    if (ret == -1)
    {
        return -1;
    }
    else if (ret == 0)
    {
        return -2;
    }
    else
    {
        return write(sd, buffer, len);
    }
}

/** @brief   Wrapper function to receive data.
 *
 *  @param   buffer     Pointer to first character of buffer to store received string.
 *  @param   len        Max number of bytes to read from file descriptor.
 *  @param   timeoutSec Timeout period for read command.
 *
 *  @return  Number of bytes read or -1 if a non-timeout error occurred and -2 if a timeout occurred.
 */

ssize_t TCPStream::receive(char* buffer, size_t len, int timeoutSec)
{
    // Attempt to send data with a timeout on write.
    fd_set set;
    FD_ZERO(&set);    // Clear the set.
    FD_SET(sd, &set); // Add our file descriptor to the set.
    
    struct timeval timeout;
    timeout.tv_sec  = timeoutSec;
    timeout.tv_usec = 0;
    
    int ret;
    ret = select(sd + 1, &set, NULL, NULL, &timeout);
    
    // First check if an error or timeout occurred.  Otherwise, call read method.
    if (ret == -1)
    {
        return -1;
    }
    else if (ret == 0)
    {
        return -2;
    }
    else
    {
        //std::cout << "attempting to read" << std::endl;
        return read(sd, buffer, len);
    }
}

/** @brief   Get peerIP address.
 *
 *  @return  peerIP address.
 */

string TCPStream::getPeerIP(void)
{
    return peerIP;
}

/** @brief   Get peer port.
 *
 *  @return  Peer port.
 */

int TCPStream::getPeerPort(void)
{
    return peerPort;
}