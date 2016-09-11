/** @file    keyframeGen.cpp
 *  @brief   cpp file to keyframeGen class, which allows for easy creation of keyframes.
 *  @author  Austin Small.
 */

#include "keyframeGen.h"

/** @brief   keyframeGen class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

keyframeGen::keyframeGen(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    threadActive(false),
    interpolationThread(NULL)
{
    setModuleDescription("Module to handle keyframe generation.");
    
    // Load communicate into NAOQI, such that it can be called from other modules.
    functionName("communicate", getName() , "Communicate messages between computer and keyframeGen module.");
    addParam("message", "Message to communicate.");
    BIND_METHOD(keyframeGen::communicate);
    
    proxy = new AL::ALProxy(broker, "shm", 0, 0);
}

/** @brief   Destructor for keyframeGen class.
 *
 */

keyframeGen::~keyframeGen()
{
    // Make sure that the interpolation thread has been properly disposed of.
    if (threadActive)
    {
        interpolationThread->interrupt();
        interpolationThread->join();
    }
    
    delete interpolationThread;
    interpolationThread = NULL;
}

/** @brief   Communication method, which is made public to NaoQi.
 *
 */

std::string keyframeGen::communicate(std::string inputMsg)
{
    int endLocus = inputMsg.find("_");
    
    std::string command(inputMsg.substr(0, endLocus));
    std::string message(inputMsg.substr(endLocus + 1, inputMsg.length() - (endLocus + 1)));
    
    if (command.compare("GETVALUES") == 0)
    {        
        std::vector<float> jointPositionVect(GLOBAL_NUM_ACTUATORS);
        std::vector<float> jointStiffnessVect(GLOBAL_NUM_ACTUATORS);
        
        proxy->call<void>("getActualJointPosition", jointPositionVect);
        proxy->call<void>("getCommandedJointStiffness", jointStiffnessVect);
        
        std::string outMsg("");
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            outMsg.append(boost::lexical_cast<std::string>(jointPositionVect[i]));
            outMsg.append("_");
            outMsg.append(boost::lexical_cast<std::string>(jointStiffnessVect[i]));
            outMsg.append("_");
        }
        
        return outMsg;
    }
    else if (command.compare("SETVALUES") == 0)
    {
        std::vector<float> jointPositionVect(GLOBAL_NUM_ACTUATORS);
        std::vector<float> jointStiffnessVect(GLOBAL_NUM_ACTUATORS);

        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            // Ignore wrist yaw.
            if (i == 4 || i == 9)
            {
                continue;
            }
            
            // Extract position from string.
            endLocus = message.find("_");
            std::string position(message.substr(0, endLocus));
            
            // Extract stiffness from string.
            message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
            endLocus = message.find("_");
            std::string stiffness(message.substr(0, endLocus));
            message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));

            jointPositionVect[i]  = (float)(::atof(position.c_str()));
            jointStiffnessVect[i] = (float)(::atof(stiffness.c_str()));
        }
        
        // Note:  DO NOT PROCEED UNTIL ALL MOTION HAS STOPPED.
        while (true)
        {
            usleep(2000); // 2 ms sleep.
            
            std::vector <bool> tempVect(GLOBAL_NUM_ACTUATORS);
            proxy->call<void>("getInMotion", tempVect);
            
            for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
            {
                if (tempVect[i])
                {
                    continue;
                }
            }
            
            break;
        }
        
        std::vector <bool> inMotionVect(GLOBAL_NUM_ACTUATORS);
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            inMotionVect[i] = true;
        }
        
        proxy->call<void>("setInMotion", inMotionVect);
        
        // Send position and stiffness command.
        proxy->call<void>("setJointPosition", jointPositionVect);
        proxy->call<void>("setJointStiffness", jointStiffnessVect);
        
        // Indicate that servos are no longer in motion.
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            inMotionVect[i] = false;
        }
        
        proxy->call<void>("setInMotion", inMotionVect);
        
        return "Success";
    }
    else if (command.compare("ITERPVALUES") == 0)
    {
        // Extract number of keyframes.
        endLocus = message.find("_");
        std::string keyframes(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        int numKeyframes = (int) (::atof(keyframes.c_str()));
        
        // Extract which keyframe is sent, of the total number.
        endLocus = message.find("_");
        std::string currentKeyframeString(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        int currentKeyframe = (int) (::atof(currentKeyframeString.c_str()));
        
        if (currentKeyframe == 1)
        {
            positionQueue.clear();
            stiffnessQueue.clear();
            duration.clear();
            interpolationType.clear();
        }
        
        positionQueue.push_back(std::vector <float> (GLOBAL_NUM_ACTUATORS));
        stiffnessQueue.push_back(std::vector <float> (GLOBAL_NUM_ACTUATORS));
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            // Handle wrist yaw separately.  Store current position and stiffness values.
            if (i == 4 || i == 9)
            {
                std::vector<float> jointPositionVect(GLOBAL_NUM_ACTUATORS);
                std::vector<float> jointStiffnessVect(GLOBAL_NUM_ACTUATORS);
                
                proxy->call<void>("getActualJointPosition", jointPositionVect);
                proxy->call<void>("getCommandedJointStiffness", jointStiffnessVect);
                
                positionQueue[currentKeyframe - 1][i]  = jointPositionVect[i];
                stiffnessQueue[currentKeyframe - 1][i] = jointStiffnessVect[i];
                
                continue;
            }
            
            // Extract position from string.
            endLocus = message.find("_");
            std::string position(message.substr(0, endLocus));
            message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
            
            // Extract stiffness from string.
            endLocus = message.find("_");
            std::string stiffness(message.substr(0, endLocus));
            message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
            
            // Store position and stiffness values.
            positionQueue[currentKeyframe - 1][i]  = ::atof(position.c_str());
            stiffnessQueue[currentKeyframe - 1][i] = ::atof(stiffness.c_str());
        }
        
        // Extract duration from string.
        endLocus = message.find("_");
        std::string durationStr(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        duration.push_back(::atof(durationStr.c_str()));
        
        // Extract interpolation type from string.
        endLocus = message.find("_");
        std::string interpType(message.substr(0, endLocus));
        message.assign(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        interpolationType.push_back(interpType);
        
        // Execute motion upon receiving all keyframes.
        if (currentKeyframe == numKeyframes)
        {
            // Spawn a thread to run the interpolation commands without holding up TCP communication progress.
            // First make sure the thread pointer is null.
            if (threadActive)
            {
                interpolationThread->interrupt();
                interpolationThread->join();
            }
            
            delete interpolationThread;
            interpolationThread = NULL;
            
            interpolationThread = new boost::thread(boost::bind(&keyframeGen::interpolationLinearThreadFunc, this));
        }
        
        return "Success";
    }
    
    return "Failed";
}

/** @brief    Method to run within thread that interpolates keyframe motions.
 *
 */

void keyframeGen::interpolationLinearThreadFunc(void)
{
    threadActive = true;
    
    try
    {
        // Interpolation motion.
        Iterp interpolator(proxy);
        
        for (int i = 0; i < positionQueue.size(); i++)
        {
            // The thread will check for interrupts at this point.
            boost::this_thread::sleep(boost::posix_time::milliseconds(10));
                        
            if (interpolationType[i].compare("Linear") == 0)
            {
                interpolator.linearIterp(positionQueue[i], stiffnessQueue[i], duration[i]);
            }
            else
            {
                continue;
            }
        }
        
        threadActive = false;
    }
    catch (boost::thread_interrupted& interruption)
    {
        threadActive = false;
    }
    catch (std::exception& e)
    {
        threadActive = false;
    }
}