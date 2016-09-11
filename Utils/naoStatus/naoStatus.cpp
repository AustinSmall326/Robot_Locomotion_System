/** @file    naoStatus.cpp
 *  @brief   cpp file to naoStatus class, which allows for communicating sensor values to computer.
 *  @author  Austin Small.
 */

#include "naoStatus.h"

/** @brief   naoStatus class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

naoStatus::naoStatus(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name)
{
    setModuleDescription("Module to communicate sensor values to computer.");
    
    // Load communicate into NAOQI, such that it can be called from other modules.
    functionName("communicate", getName() , "Communicate messages between computer and naoStatus module.");
    addParam("message", "Message to communicate.");
    BIND_METHOD(naoStatus::communicate);
    
    proxy = new AL::ALProxy(broker, "shm", 0, 0);
}

/** @brief   Destructor for naoStatus class.
 *
 */

naoStatus::~naoStatus()
{ }

/** @brief   Communication method, which is made public to NaoQi.
 *
 */

std::string naoStatus::communicate(std::string inputMsg)
{
    int endLocus = inputMsg.find("_");
    
    std::string command(inputMsg.substr(0, endLocus));
    std::string message(inputMsg.substr(endLocus + 1, inputMsg.length() - (endLocus + 1)));

    if (command.compare("GETJOINTACTUALPOSITION") == 0)
    {
        // Extract which joint to return position of.
        endLocus = message.find("_");
        std::string jointIDString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));

        int jointID = (int) (::atof(joingIDString.c_str()));
        
        std::string outMsg("");
        
        outMsg.append(boost::lexical_cast<std::string>(proxy->call<float>("getActualJointPosition", i)));
        outMsg.append("_");
        
        return outMsg;
    }
    
    return "Failed";
}