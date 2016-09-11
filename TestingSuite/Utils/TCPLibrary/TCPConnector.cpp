/** @file    TCPConnector.cpp
 *  @brief   cpp file to TCPConnector class, which encapsulates the socket mechanisms to actively
 *           connect to a server.
 *  @author  Austin Small.
 */

#include "TCPConnector.h"
#include <iostream>
#include <errno.h>


/** @brief   This method establishes a connection with the server (robot).
 *
 *  @param   server     Server IP address.
 *  @param   port       Server port number.
 *  @param   timeoutSec Number of seconds before timout of connect method.
 */

TCPStream* TCPConnector::connect(const char* serverIP, int port, int timeoutSec)
{
    std::cout << "connect was called" << std::endl;
    
    struct sockaddr_in address;
    
    // Store all zeros for address struct.
    memset(&address, 0, sizeof(address));
    
    // Configure address struct.
    address.sin_family = AF_INET;
    address.sin_port = htons(port);                    // Convert from host to TCP network byte order.
    inet_pton(PF_INET, serverIP, &(address.sin_addr)); // Convert IP address to network byte order.
    
    // Create a socket.  The socket signature is as follows: socket(int domain, int type, int protocol)
    int sd = socket(AF_INET, SOCK_STREAM, 0);

    //int optval = 1;
    
    //if (setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof optval) == -1)
    //{
     //   std::cout << "failed to set socket option" << std::endl;
    //}
    
    // Set socket to be non-blocking.
    int arg;
    arg  = fcntl(sd, F_GETFL, NULL);
    arg |= O_NONBLOCK;
    fcntl(sd, F_SETFL, arg);
    
    // Connect with time limit.
    fd_set set;
    FD_ZERO(&set);    // Clear the set.
    FD_SET(sd, &set); // Add our file descriptor to the set.
    
    struct timeval timeout;
    timeout.tv_sec  = timeoutSec;
    timeout.tv_usec = 0;
    
    // If the connect call returns 0, then the connection was established.  Otherwise,
    // check if the three-way handshake is underway.
    if (::connect(sd, (struct sockaddr *)&address, sizeof(address)) < 0)
    {
        // If the handshake is underway.
        if (errno == EINPROGRESS)
        {
            std::cout << "handshake in progress" << std::endl;

            
            // Designate timeout period.
            int ret = select(sd + 1, NULL, &set, NULL, &timeout);
            
            std::cout << "return value from select : " << ret << std::endl;
            
            // Check if timeout or an error occurred.
            if (ret <= 0)
            {
                std::cout << "return less than 0" << std::endl;
                std::cout << "closing socket descriptor" << std::endl;

                
                
                
                if (close(sd) < 0)
                {

                    
                    char * newerrorMessage = strerror( errno); // get string message from errn
                    std::string newmsg (newerrorMessage);
                    std::cout << newmsg << std::endl;

                    
                    std::cout << "failed to close socket descriptor" << std::cout;
                }
                

                
                return NULL;
            }
            else
            {
                // Check if select returned 1 due to an error.
                int valopt;
                socklen_t len = sizeof(int);
                
                getsockopt(sd, SOL_SOCKET, SO_ERROR, (void*)(&valopt), &len);
                
                if (valopt)
                {
                    char * errorMessage = strerror( errno); // get string message from errn
                    std::string msg (errorMessage);
                    std::cout << msg << std::endl;
                    
                    std::cout << "closing socket descriptor" << std::endl;
                    
                    if (close(sd) < 0)
                    {
                        
                        char * newerrorMessage = strerror( errno); // get string message from errn
                        std::string newmsg (newerrorMessage);
                        std::cout << newmsg << std::endl;

                        
                        std::cout << "failed to close socket descriptor" << std::cout;
                    }
                    
                    
                    return NULL;
                    
                }
            }
            
        }
        else
        {
            std::cout << "error but not EINPROGRESS" << std::endl;

            char * errorMessage = strerror( errno); // get string message from errn
            std::string msg (errorMessage);
            std::cout << msg << std::endl;

            
            
            
            
            
            if (close(sd) < 0)
            {

                
                char * newerrorMessage = strerror( errno); // get string message from errn
                std::string newmsg (newerrorMessage);
                std::cout << newmsg << std::endl;
                
                
                std::cout << "failed to close socket descriptor" << std::cout;
            }
            

            
            return NULL;
        }
    }
    
    // Return socket to blocking mode.
    arg = fcntl(sd, F_GETFL, NULL);
    arg &= (~O_NONBLOCK);
    fcntl(sd, F_SETFL, arg);
    
    // Create stream object.
    return new TCPStream(sd, &address);
}