/** @file    shm.h
 *  @brief   Header file to shm class, which allows for transferring joint stiffness and position
 *           between independent applications.
 *  @author  Austin Small.
 */

#ifndef SHM_H
#define SHM_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>
#include <alvalue/alvalue.h>

#include <alerror/alerror.h>

#include <alproxies/almemoryproxy.h>

// Used to read values of ALMemory directly in RAM.
#include <almemoryfastaccess/almemoryfastaccess.h>

#include "../../LocomotionDev/Modules/LocomotionDefines.h"

namespace AL
{
    class ALBroker;
}

/** @brief		shm class allows for transferring joint stiffness and position between 
 *              independent applications.
 */
class shm : public AL::ALModule
{
    public:
        // Methods.
        shm(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~shm();
    
        void setJointStiffness(std::vector<float> stiffness);
        void getCommandedJointStiffness(std::vector<float>& outputVect);
    
        void setJointPosition(std::vector<float> position);
        void getCommandedJointPosition(std::vector<float>& outputVect);
        void getActualJointPosition(std::vector<float>& outputVect);
    
        void setFlag(int newValue);
        int  getFlag(void);
    
        void setInMotion(std::vector<bool> state);
        void getInMotion(std::vector<bool>& outputVect);
    
    private:
        // Methods.
        void initFastAccess(void); // Helper method to initialize ALMemory link.

        // Fields.
        boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccessActualPosition;
    
        int flag;
        std::vector<float> positionVect;
        std::vector<float> stiffnessVect;
        std::vector<bool>  boolInMotionVect;
        std::vector<int>   intInMotionVect;

        std::vector<std::string> fKeysActualPosition;

        // Array to store a mapping from array indices to actuator names.
        std::string indexToStringMap[GLOBAL_NUM_ACTUATORS];
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
                  HEAD_PITCH };

#endif