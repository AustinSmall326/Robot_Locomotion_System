/** @file    tunePIDParam.cpp
 *  @brief   cpp file to tunePIDParam class, which helps with tuning PID parameters.
 *  @author  Austin Small.
 */

#include "tunePIDParam.h"


/** @brief   tunePIDParam class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

tunePIDParam::tunePIDParam(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name)
{
    indexToStringMap[0]  = "L_ARM_SHOULDER_PITCH";  // Left arm.
    indexToStringMap[1]  = "L_ARM_SHOULDER_ROLL";
    indexToStringMap[2]  = "L_ARM_ELBOW_YAW";
    indexToStringMap[3]  = "L_ARM_ELBOW_ROLL";
    indexToStringMap[4]  = "L_ARM_WRIST_YAW";
    indexToStringMap[5]  = "R_ARM_SHOULDER_PITCH";  // Right arm.
    indexToStringMap[6]  = "R_ARM_SHOULDER_ROLL";
    indexToStringMap[7]  = "R_ARM_ELBOW_YAW";
    indexToStringMap[8]  = "R_ARM_ELBOW_ROLL";
    indexToStringMap[9]  = "R_ARM_WRIST_YAW";
    indexToStringMap[10] = "L_LEG_HIP_YAW_PITCH";   // Left leg.
    indexToStringMap[11] = "L_LEG_HIP_ROLL";
    indexToStringMap[12] = "L_LEG_HIP_PITCH";
    indexToStringMap[13] = "L_LEG_KNEE_PITCH";
    indexToStringMap[14] = "L_LEG_ANKLE_PITCH";
    indexToStringMap[15] = "L_LEG_ANKLE_ROLL";
    indexToStringMap[16] = "R_LEG_HIP_YAW_PITCH";   // Right leg.
    indexToStringMap[17] = "R_LEG_HIP_ROLL";
    indexToStringMap[18] = "R_LEG_HIP_PITCH";
    indexToStringMap[19] = "R_LEG_KNEE_PITCH";
    indexToStringMap[20] = "R_LEG_ANKLE_PITCH";
    indexToStringMap[21] = "R_LEG_ANKLE_ROLL";
    indexToStringMap[22] = "HEAD_YAW";              // Head.
    indexToStringMap[23] = "HEAD_PITCH";

    setModuleDescription("Module to tune PID parameters.");
    
    // Load communicate into NAOQI, such that it can be called from other modules.
    functionName("communicate", getName() , "Communicate messages between computer and tunePIDParam module.");
    addParam("message", "Message to communicate.");
    BIND_METHOD(tunePIDParam::communicate);
    
    proxy = new AL::ALProxy(broker, "shm", 0, 0);
}

/** @brief   Destructor for tunePIDParam class.
 *
 */

tunePIDParam::~tunePIDParam()
{ }

/** @brief   Communication method, which is made public to NaoQi.
 *
 */

std::string tunePIDParam::communicate(std::string inputMsg)
{
    int endLocus = inputMsg.find("_");
    
    std::string command(inputMsg.substr(0, endLocus));
    std::string message(inputMsg.substr(endLocus + 1, inputMsg.length() - (endLocus + 1)));

    if (command.compare("TESTPARAM") == 0)
    {
        // Check if there is data in the data vectors, in which case the robot it in the process
        // of transmitting.  Do not proceed.
        if (timeStampVect.size() > 0)
        {
            return "Failure";
        }
        
        // Extract which joint to move.
        endLocus = message.find("_");
        std::string jointIDString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));

        int jointID = (int) (::atof(jointIDString.c_str()));
        
        // Extract new position for joint.
        endLocus = message.find("_");
        std::string nextJointPosString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        double nextJointPos = ::atof(nextJointPosString.c_str());
        
        // Extract movement duration.
        endLocus = message.find("_");
        std::string durationString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        double duration = ::atof(durationString.c_str());
        
        // Extract PID state.
        endLocus = message.find("_");
        std::string PIDStateString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        int PIDState = (int) (::atof(PIDStateString.c_str()));
        
        // Extract proportional gain.
        endLocus = message.find("_");
        std::string kpString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        double Kp = ::atof(kpString.c_str());
        
        // Extract integral gain.
        endLocus = message.find("_");
        std::string kiString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        double Ki = ::atof(kiString.c_str());
        
        // Extract differential gain.
        endLocus = message.find("_");
        std::string kdString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        double Kd = ::atof(kdString.c_str());
        
        double currentPosition  = (double) proxy->call<float>("getActualJointPosition", jointID);
        double currentStiffness = (double) proxy->call<float>("getCommandedJointStiffness", jointID);
        
        // Determine slope (position vs. time) for each actuator.
        double slopePosition = (nextJointPos - currentPosition) / duration;
        
        /** Update actuator positions and stiffness in a linear fashion. **/
        // Instantiate PID controller.
        PID pid(PIDState, Kp, Ki, Kd);
        
        // Indicate to shared memory that an actuator is in motion.
        // Note:  DO NOT PROCEED UNTIL ALL MOTION HAS STOPPED.
        while (proxy->call<bool>("getInMotion", jointID))
        {
            usleep(2000); // 2 ms.
        }
        
        proxy->call<void>("setInMotion", jointID, true);
        
        double sampleInterval = 0.040;
        
        // Command joint angles with current stiffness.
        proxy->call<void>("setJointPosition", jointID, nextJointPos);

        struct timeval tv;
        
        gettimeofday(&tv, NULL);
        unsigned long long startTime      = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000; // ms.
        unsigned long long lastTimePID    = startTime - PIDdt; // ms.  Set 40 ms in past so PID controller immediately assumes control.
        unsigned long long lastTimeSample = startTime - sampleInterval;
        unsigned long long currentTime    = startTime;         // ms.
        
        while (currentTime <= (startTime + 1000 * duration))
        {
            double timeDiffMSecPID   = currentTime - lastTimePID;
            double timeDiffSecPID    = timeDiffMSecPID / 1000;
            double timeDiffSecSample = (currentTime - lastTimeSample) / 1000;
            
            // Check if shared memory is ready for update that we have exceeded the PID time - which is related to the fastest rate at which
            // measured joint angles are read in the NAO.
            if (proxy->call<int>("getFlag") == 1 && timeDiffMSecPID >= PIDdt)
            {
                // Update stiffness for all actuators.
                double expectedAngle = currentPosition + slopePosition * (currentTime - startTime) / 1000;
                double actualAngle   = proxy->call<float>("getActualJointPosition", jointID);
                
                // Store data to return to computer.
                if (timeDiffSecSample >= sampleInterval)
                {
                    std::cout << "samle recorded" << std::endl;
                    
                    actualJointPosVect.push_back(actualAngle);
                    expectedJointPosVect.push_back(expectedAngle);
                    timeStampVect.push_back(currentTime);
                    
                    lastTimeSample = currentTime;
                }
                
                currentStiffness = (double) proxy->call<float>("getCommandedJointStiffness", jointID);
                double temp = currentStiffness + pid.calculate(expectedAngle, actualAngle, slopePosition, timeDiffMSecPID);
                
                
                
                std::cout << "modulated value : " << temp << std::endl;
                
                
                if (temp < 0)
                {
                    temp = 0;
                }
                else if (temp > 1)
                {
                    temp = 1;
                }
                
                proxy->call<void>("setJointStiffness", jointID, temp);
                
                proxy->call<void>("setFlag", 0);
                
                // Update last time of update to actuators.
                gettimeofday(&tv, NULL);
                lastTimePID = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
            }
            else
            {
                // Sleep for a period to prevent CPU overusage.
                usleep(2000); // 2 ms.
            }
            
            // Update the current time.
            gettimeofday(&tv, NULL);
            currentTime = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
        }
        
        // Indicate to shared memory that actuators are no longer in motion.
        proxy->call<void>("setInMotion", jointID, false);

        /** Prepare output message string. **/
        std::string tempOutputMsg("");
        
        int firstPacketSize = 0;
        
        for (; firstPacketSize < maxPacketSize; firstPacketSize++)
        {
            if (timeStampVect.size() < 1)
            {
                break;
            }
            
            tempOutputMsg.append(boost::lexical_cast<std::string>(actualJointPosVect.front()));
            tempOutputMsg.append("_");
            tempOutputMsg.append(boost::lexical_cast<std::string>(expectedJointPosVect.front()));
            tempOutputMsg.append("_");
            tempOutputMsg.append(boost::lexical_cast<std::string>(timeStampVect.front()));
            tempOutputMsg.append("_");
            
            actualJointPosVect.erase(actualJointPosVect.begin());
            expectedJointPosVect.erase(expectedJointPosVect.begin());
            timeStampVect.erase(timeStampVect.begin());
        }
        
        // Number of data packets.
        int numPackets = ceil(((double) timeStampVect.size()) / maxPacketSize);
        
        std::string outputMsg("");
        
        outputMsg.append(boost::lexical_cast<std::string>(numPackets));
        outputMsg.append("_");
        
        // Number of data points in first packet.
        outputMsg.append(boost::lexical_cast<std::string>(firstPacketSize));
        outputMsg.append("_");
        
        outputMsg.append(tempOutputMsg);
        
        std::cout << "output message : " << outputMsg << std::endl;
        
        return outputMsg;
    }
    else if (command.compare("REQUESTTESTPARAMDATA") == 0)
    {
        /** Prepare output message string. **/
        std::string tempOutputMsg("");
        
        int firstPacketSize = 0;
        
        for (; firstPacketSize < maxPacketSize; firstPacketSize++)
        {
            if (timeStampVect.size() < 1)
            {
                break;
            }
            
            tempOutputMsg.append(boost::lexical_cast<std::string>(actualJointPosVect.front()));
            tempOutputMsg.append("_");
            tempOutputMsg.append(boost::lexical_cast<std::string>(expectedJointPosVect.front()));
            tempOutputMsg.append("_");
            tempOutputMsg.append(boost::lexical_cast<std::string>(timeStampVect.front()));
            tempOutputMsg.append("_");
            
            actualJointPosVect.erase(actualJointPosVect.begin());
            expectedJointPosVect.erase(expectedJointPosVect.begin());
            timeStampVect.erase(timeStampVect.begin());
        }
        
        std::string outputMsg("");
        
        // Number of data points in first packet.
        outputMsg.append(boost::lexical_cast<std::string>(firstPacketSize));
        outputMsg.append("_");
        
        outputMsg.append(tempOutputMsg);
        
        return outputMsg;
    }
    
    return "Failed";
}