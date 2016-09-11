/** @file    tunePIDParam.h
 *  @brief   Header file to tunePIDParam class, which helps with tuning PID parameters.
 *  @author  Austin Small.
 */

#ifndef NAO_STATUS_H
#define NAO_STATUS_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <math.h>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include "../../../LocomotionParameters.h"
#include "../PID.h"

namespace AL
{
    class ALBroker;
}

/** @brief		tunePIDParam class allows for tuning PID parameters.
 */
class tunePIDParam : public AL::ALModule
{
    public:
        // Methods.
        tunePIDParam(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~tunePIDParam();
    
        std::string communicate(std::string);
    
    private:
        // Fields.
        AL::ALProxy* proxy;
        
        // Array to store a mapping from array indices to actuator names.
        std::string indexToStringMap[24];
        
        static const int PIDdt = GLOBAL_DCM_READ_TIME_STEP; // ms.
    
        // Vectors to store data to return to computer.
        std::vector <double> actualJointPosVect;
        std::vector <double> expectedJointPosVect;
        std::vector <double> timeStampVect;
    
        static const int maxPacketSize = 15;

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
                  R_LEG_HIP_YAW_PITCH,
                  R_LEG_HIP_ROLL,
                  R_LEG_HIP_PITCH,
                  R_LEG_KNEE_PITCH,
                  R_LEG_ANKLE_PITCH,
                  R_LEG_ANKLE_ROLL,
                  HEAD_YAW,
                  HEAD_PITCH };

#endif