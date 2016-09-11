/** @file    fastGetSetDCM.h
 *  @brief   Header file to fastGetSetDCM class, which allows for updating actuator values.
 *  @author  Austin Small / Zheng Tian.
 */

#ifndef FAST_GET_SET_DCM_H
#define FAST_GET_SET_DCM_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <vector>
#include <sys/time.h>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alerror/alerror.h>

#include <alproxies/dcmproxy.h>

// Used to read values of ALMemory directly in RAM.
#include <almemoryfastaccess/almemoryfastaccess.h>

#include "../../LocomotionDefines.h"

namespace AL
{
    class ALBroker;
    class ALMemoryFastAccess;
    class DCMProxy;
}

/** @brief		fastGetSetDCM class allows for updating actuator stiffness and position values.
 */
class fastGetSetDCM : public AL::ALModule
{
    public:
        // Methods.
        fastGetSetDCM(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~fastGetSetDCM();
        void start();
        void stop();
        
    private:
        // Methods.
        void init();                              // Initialize ALMemory/DCM link.
        void initFastAccess();                    // Helper method to initialize ALMemory link.
        void createHardnessActuatorAlias();       // Create DCM hardness actuator alias.
        void createPositionActuatorAlias();       // Create DCM position actuator alias.
    
        void prepareStiffnessActuatorCommand();
        void preparePositionActuatorCommand();
        void setStiffness(std::vector <float> stiffnessVect);  // Update stiffness for all actuators.
        void setPosition(std::vector <float> positionVect);    // Update position for all actuators.
    
        void connectToDCMloop();                  // Connect callback to the DCM post proccess.
        void synchronisedDCMcallback();           // Callback called by the DCM every 10ms.
    
        // Fields.
        ProcessSignalConnection fDCMPostProcessConnection;           // Used for postprocess sync with the DCM.
        std::vector<std::string> fSensorKeys;                        // Sensor keys in ALMemory.
        boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;
    
        boost::shared_ptr<AL::DCMProxy> dcmProxy;
        AL::ALValue stiffnessCommands;                               // Used to store stiffness commands to send.
        AL::ALValue positionCommands;                                // Used to store position commands to send.
    
        AL::ALProxy* proxy;

        std::vector<float>  initialJointStiffness;
        std::vector<float>  initialJointPosition;

        std::string indexToStringMap[GLOBAL_NUM_ACTUATORS];          // Array to store a mapping from array
                                                                     // indices to actuator names.
};

enum SensorType { L_ARM_SHOULDER_PITCH,
                  L_ARM_SHOULDER_ROLL,
                  L_ARM_ELBOW_YAW,
                  L_ARM_ELBOW_ROLL,
                  L_ARM_WRIST_YAW,
                  R_ARM_SHOULDER_PITCH,
                  R_ARM_SHOULDER_ROLL,
                  R_ARM_ELBOW_YAW,
                  R_ARM_ELBOW_ROLL,
                  R_ARM_WRIST_YAW,
                  L_LEG_HIP_YAW_PITCH,
                  L_LEG_HIP_ROLL,
                  L_LEG_HIP_PITCH,
                  L_LEG_KNEE_PITCH,
                  L_LEG_ANKLE_PITCH,
                  L_LEG_ANKLE_ROLL,
                  R_LEG_HIP_ROLL,
                  R_LEG_HIP_PITCH,
                  R_LEG_KNEE_PITCH,
                  R_LEG_ANKLE_PITCH,
                  R_LEG_ANKLE_ROLL,
                  HEAD_YAW,
                  HEAD_PITCH};

#endif