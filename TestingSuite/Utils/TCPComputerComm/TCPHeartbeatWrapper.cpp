/** @file    TCPHeartbeatWrapper.cpp
 *  @brief   cpp file to TCPHeartbeatWrapper class, which wraps the TCPStream class, and implements
 *           a heartbeat for connection verification.
 *  @author  Austin Small.
 */

#include "TCPHeartbeatWrapper.h"
#include <unistd.h>

/** @brief   TCPHeartbeatWrapper class constructor.
 *
 *  @param   argPort        Server (robot) port number.
 *  @param   argMaxLength   Max length of messages to send and receive.
 */

TCPHeartbeatWrapper::TCPHeartbeatWrapper(int argPort, int argMaxLength) :
    connector(NULL), stream(NULL),
    mainThreadActive(false), mainThread(NULL),
    connectionActive(false), applicationClosing(false), waitingForResponse(false),
    PORT(argPort), MAX_PACKET_LENGTH(argMaxLength),
    receiveBufferString(""), transmissionInProgress(false)
{ }

/** @brief   TCPHeartbeatWrapper class destructor.
 *
 */

TCPHeartbeatWrapper::~TCPHeartbeatWrapper(void)
{
    std::cout << "deconstructing wrapper" << std::endl;
    
    applicationClosing = true;
    
    // Check if mainThread pointer is non-null.
    if (mainThread)
    {
        // Check if mainThread is still running.
        if (mainThreadActive)
        {
            mainThread->interrupt();
            mainThread->join();
        }
        
        delete mainThread;
        mainThread = NULL;
    }
    
    // Make sure that the communicate method is not currently in a waiting state.
    while (waitingForResponse)
    {
        // Pause application for 100 us.
        usleep(100);
    }
    
    delete stream;
    stream = NULL;
    
    delete connector;
    connector = NULL;
}

/** @brief   Attempt to connect to server.
 *
 *  @param   argIP  IP address of server to connect to.
 *
 *  @return  1 if connection is successful and -1 if the connection failed.
 *
 */

int TCPHeartbeatWrapper::connect(std::string IP)
{
    std::cout << "attempting to connect" << std::endl;
    
    // Make sure connector and stream are NULL.
    delete stream;
    delete connector;
    
    stream = NULL;
    connector = NULL;
    
    // Instantiate stream and connector.
    connector = new TCPConnector();
    stream = connector->connect(IP.c_str(), PORT, CONNECT_TIMEOUT);
    
    if (stream)
    {
        std::cout << " did connect" << std::endl;
        
        connectionActive = true;
        
        // Make sure mainThread is NULL.
        delete mainThread;
        mainThread = NULL;
        
        // Start main thread.
        mainThread = new boost::thread(&TCPHeartbeatWrapper::mainThreadFunc, this);
        
        return 1;
    }
    else
    {
        std::cout << "stream was null" << std::endl;
        
        delete stream;
        stream = NULL;
        delete connector;
        connector = NULL;
        
        return -1;
    }
}

/** @brief   Attempt to send message and return response.
 *
 *  @param   inputMsg   Message to communicate to server (robot).
 *  @param   buffer     Location to store output message.
 *
 *  @return  1 if successful, -1 if an error occurred and -2 if the inbuffer is already occupied.
 *
 */

int TCPHeartbeatWrapper::communicate(char* inputMsg, char* buffer)
{
    // Check if buffer is non-empty.  Return -2 in this case, since another message is already
    // in the queue.
    if (inbuffer.size() > 0)
    {
        return -2;
    }
    
    // Add message to buffer.
    inbuffer.push_back(std::string (inputMsg));
    
    // Wait for a response.
    while (true)
    {
        waitingForResponse = true;
        
        // Pause thread for 100 us to avoid CPU overusage.
        usleep(100);
        
        // If the connection ever becomes inactive, return -1.
        if (!connectionActive)
        {
            waitingForResponse = false;
            return -1;
        }
        
        // Check if the application is closing.
        if (applicationClosing)
        {
            waitingForResponse = false;
            return -1;
        }
        
        // Check if a return message is available.
        if (outbuffer.size() > 0)
        {
            std::string returnMessage(outbuffer.front());
            outbuffer.clear();
            waitingForResponse = false;
            strcpy(buffer, returnMessage.c_str());
            return 1;
        }
    }
}

/** @brief   Primary thread function that coordinates communications as well as heartbeat.
 *
 */

void TCPHeartbeatWrapper::mainThreadFunc(void)
{
    mainThreadActive = true;
    
    struct timeval tv;
    
    gettimeofday(&tv, NULL);
    double start = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000; // sec.
    
    while (true)
    {
        try
        {
            // The thread will check for interrupts at this point through every iteration.
            boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            
            // Check if it is time to send a heartbeat.
            gettimeofday(&tv, NULL);
            double currentTime = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000; // sec.
            
            if ((currentTime - start) > (double) HEARTBEAT_TIME)
            {
                start = currentTime;

                std::string heartbeatMsg("heartbeat_");
                char heartbeatResponse[MAX_PACKET_LENGTH + 200];
                
                int sendRet = stream->send(heartbeatMsg.c_str(), heartbeatMsg.size(), SEND_TIMEOUT);
                
                //std::cout << "send ret val" << sendRet << std::endl;
                
                
                // Attempt to receive heartbeat response.
                int recvRet = stream->receive(heartbeatResponse, sizeof(heartbeatResponse), RECV_TIMEOUT);
                
                
                //std::cout << "recv ret val" << recvRet << std::endl;
                
                
                // Check if an error or timeout occurred, or if heartbeat failed.
                if (sendRet < 0 || recvRet < 0 || heartbeatMsg.compare(std::string(heartbeatResponse, recvRet)) != 0)
                {
                    delete stream;
                    stream = NULL;
                    delete connector;
                    connector = NULL;
                    
                    mainThreadActive = false;

                    inbuffer.clear();
                    outbuffer.clear();

                    connectionActive = false;

                    break;
                }
            }
            
            char data[MAX_PACKET_LENGTH + 200]; // +200 because there may be extra data added by the TCP layer,
                                                // beyond the MAX_PACKET_LENGTH.
            std::string message("");

            // Check if there is a transmission in progress.
            if (transmissionInProgress)
            {
                message.append("ok_");
            }
            else
            {
                // If there is a message in buffer, attempt to send.
                if (inbuffer.size() < 1)
                {
                    continue;
                }
                
                message.append("ok_");
                message.append(inbuffer.front());
            }
            
            int sendRet = stream->send(message.c_str(), message.size(), SEND_TIMEOUT);
            
            // Poll for response.
            int recvRet = stream->receive(data, sizeof(data), RECV_TIMEOUT);
            
            // Check if an error or timeout occurred.
            if (sendRet < 0 || recvRet < 0)
            {
                delete stream;
                stream = NULL;
                delete connector;
                connector = NULL;
                
                mainThreadActive = false;
                
                inbuffer.clear();
                outbuffer.clear();
                
                connectionActive = false;
                
                break;
            }
            
            std::string dataString(std::string(data, recvRet));
            
            int endLocus = dataString.find("_");
            
            // Note: possible commands include:
            //       - transmissionInProgress
            //       - ok
            std::string command(dataString.substr(0, endLocus));
            message = dataString.substr(endLocus + 1, dataString.length() - (endLocus + 1));
            
            if (command.compare("transmissionInProgress") == 0)
            {
                transmissionInProgress = true;
                receiveBufferString.append(message);
            }
            else
            {
                transmissionInProgress = false;
                receiveBufferString.append(message);
                outbuffer.clear();
                outbuffer.push_back(receiveBufferString);
                receiveBufferString = "";
                inbuffer.pop_back(); // Wait for response before clearing inbuffer.
            }
        }
        catch (boost::thread_interrupted& interruption)
        {
            delete stream;
            stream = NULL;
            delete connector;
            connector = NULL;
            
            mainThreadActive = false;
            
            inbuffer.clear();
            outbuffer.clear();
            
            connectionActive = false;
            
            break;
        }
        catch (std::exception& e)
        {
            delete stream;
            stream = NULL;
            delete connector;
            connector = NULL;
            
            mainThreadActive = false;
            
            inbuffer.clear();
            outbuffer.clear();
            
            connectionActive = false;
            
            break;
        }
    }
}

/** @brief   Checks if the connection is active.
 *
 *  @return  True if the connection is active, and false otherwise.
 *
 */
bool TCPHeartbeatWrapper::isActive(void)
{
    return connectionActive;
}