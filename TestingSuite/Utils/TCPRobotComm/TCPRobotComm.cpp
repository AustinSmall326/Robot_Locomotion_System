#include "TCPRobotComm.h"

/** @brief   TCPRobotComm class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

TCPRobotComm::TCPRobotComm(boost::shared_ptr<AL::ALBroker> argBroker, const std::string &name) :
    AL::ALModule(argBroker, name),
    broker(argBroker),
    init(false),
    acceptor(NULL), stream(NULL), proxy(NULL),
    sendBufferString("")
{
    setModuleDescription("Module to communicate data between robot and computer using TCP.");
    
    // Open robot for connections to potential clients.
    acceptor = new TCPAcceptor(PORT, "");
    int ret = acceptor->start();
    
    if (ret < 0)
    {
        throw ALERROR(getName(), "Initialization", "Failed to begin listening.");
    }
    
    communicationThread = boost::thread(&TCPRobotComm::communicationThreadFunc, this);
}

/** @brief   Destructor for TCPRobotComm class.
 *
 */

TCPRobotComm::~TCPRobotComm()
{
    communicationThread.interrupt();
    communicationThread.join();      // Make sure thread is gone before destroying class data.
}

/** @brief   Exit method before unsubscribing from RobotComm module.
 *
 */

void TCPRobotComm::exit(void)
{ }

/** @brief   Method to run within thread that communicates data between robot and computer.
 *
 */

void TCPRobotComm::communicationThreadFunc(void)
{
    struct timeval tv;
    double lastBeat;
    
    std::string heartbeatMsg("heartbeat");
    
    while (true)
    {
        try
        {
            // The thread will check for interrupts at this point through every iteration.
            boost::this_thread::interruption_point();
            
            std::cout << "break 0" << std::endl;
            
            
            if (init == false)
            {
                stream = acceptor->accept(ACCEPT_TIMEOUT);
                
                // Check if an error occurred.
                if (!stream)
                {
                    std::cout<< "stream not initialized" << std::endl;
                    continue;
                }
                
                init = true;
                
                gettimeofday(&tv, NULL);
                lastBeat = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000; // sec.
            }
            
            // Make sure we've received a heartbeat in an acceptable amount of time.
            gettimeofday(&tv, NULL);
            double currentTime = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000; // sec.

            if ((currentTime - lastBeat) > (double) HEARTBEAT_TIME)
            {
                std::cout << "heartbeat error" << std::endl;
                init = false;
                handleTCPError();
                continue;
            }
            
            int ret;
            static char data[MAX_PACKET_LENGTH + 200];
            
            std::cout << "break 1" << std::endl;
            
            ret = stream->receive(data, sizeof(data), RECV_TIMEOUT);
            
            std::cout << "ret" << ret << std::endl;
            
            std::cout << "break 2" << std::endl;
            
            
            // Check if an error or timeout occurred in receiving message.
            // If return is zero, consider that a timeout.  In that case, the heartbeat will
            // handle a scenario where the computer application has been killed.
            if (ret == -1)
            {
                std::cout << "error" << std::endl;
                //handleTCPError();
                init = false;
                handleTCPError();
                
                continue;
            }
            else if (ret == -2 || ret == 0)
            {
                std::cout << "timeout" << std::endl;
                continue;
            }
            
            std::string inMsg((const char*) data, ret);
            
            int endLocus = inMsg.find("_");
            
            // Note: possible commands include:
            //       - heartbeat
            //       - ok
            std::string command(inMsg.substr(0, endLocus));
            std::string message(inMsg.substr(endLocus + 1, inMsg.length() - (endLocus + 1)));
            
            // Check if the input message was a heartbeat.
            std::string returnMessage("");
            
            if (command.compare(heartbeatMsg) == 0)
            {
                returnMessage.append(heartbeatMsg);
                returnMessage.append("_");
                
                gettimeofday(&tv, NULL);
                lastBeat = (double)(tv.tv_sec) + (double)(tv.tv_usec) / 1000000; // sec.
            }
            else if (sendBufferString.length() > 0)
            {
                std::string tempMessage(managePacketSize(sendBufferString));
                
                if (sendBufferString.length() > 0)
                {
                    returnMessage.append("transmissionInProgress_");
                    returnMessage.append(tempMessage);
                }
                else
                {
                    returnMessage.append("ok_");
                    returnMessage.append(tempMessage);
                }
            }
            else
            {
                returnMessage.append(delegate(message));
                std::string tempMessage(managePacketSize(returnMessage));
                returnMessage = "";
                
                if (sendBufferString.length() > 0)
                {
                    returnMessage.append("transmissionInProgress_");
                    returnMessage.append(tempMessage);
                }
                else
                {
                    returnMessage.append("ok_");
                    returnMessage.append(tempMessage);
                }
            }
            
            ret = stream->send(returnMessage.c_str(), returnMessage.size(), SEND_TIMEOUT);

            std::cout << "break 3" << std::endl;
            
            std::cout << "ret" << ret << std::endl;
            

            // Check if an error or timeout occurred in sending message.
            if (ret < 0)
            {
                init = false;
                handleTCPError();
                continue;
            }
        }
        catch (boost::thread_interrupted& interruption)
        {
            delete acceptor;
            delete stream;
            
            acceptor = NULL;
            stream = NULL;
            
            break;
        }
        catch (std::exception& e)
        {
            delete acceptor;
            delete stream;
            
            acceptor = NULL;
            stream = NULL;
            
            break;
        }
    }
}

/** @brief   Method to handle TCP communication errors.  In particular, when an error occurs,
 *           this method closes and then reopens the existing socket.
 *
 */

void TCPRobotComm::handleTCPError(void)
{
    delete stream;
    delete acceptor;
    
    stream = NULL;
    acceptor = NULL;
    
    acceptor = new TCPAcceptor(PORT, "");
    int ret = acceptor->start();
    
    if (ret < 0)
    {
        throw ALERROR(getName(), "Initialization", "Failed to begin listening.");
    }
}

/** @brief   Method to communicate string data between computer and NaoQi modules.
 *
 */

std::string TCPRobotComm::delegate(std::string inMsg)
{
    int endLocus = inMsg.find("_");
    
    std::string moduleName(inMsg.substr(0, endLocus));
    std::string message(inMsg.substr(endLocus + 1, inMsg.length() - (endLocus + 1)));
    
    // Create a proxy to module.
    proxy = new AL::ALProxy(broker, moduleName, 0, 0);

    return proxy->call<std::string>("communicate", message);
}

/** @brief   Method to check if the message to be sent exceeds maximum packet size.
 *           if so, it returns a properly size packet and stores the rest of the message
 *           in the global string sendBufferString.
 *
 *  @param   The entire message to be sent.
 *
 *  @return  A properly sized data packet.
 *
 */

std::string TCPRobotComm::managePacketSize(std::string entireMessage)
{
    if (entireMessage.length() > MAX_PACKET_LENGTH)
    {
        std::string output(entireMessage.substr(0, MAX_PACKET_LENGTH));
        sendBufferString = entireMessage.substr(MAX_PACKET_LENGTH, entireMessage.length() - MAX_PACKET_LENGTH);
        
        return output;
    }
    else
    {
        sendBufferString = "";
        return entireMessage;
    }
}