#ifndef TEST_POSE_H
#define TEST_POSE_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alerror/alerror.h>

#include "../../Kinematics/KinematicsWrapper.h"
#include "../../Representations/Transform.h"

#include <vector>

// Output stream libraries included for testing only.
#include <iostream>
#include <fstream>

namespace AL
{
    class ALBroker;
    class ALMemoryFastAccess;
}

class testPose : public AL::ALModule
{
    public:
        // Methods.
        testPose(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~testPose();
    
    private:
        // Fields.
        // Proxies.
        AL::ALProxy* shmProxy;
        //AL::ALProxy* dcmProxy;
    
        // Array to store joint angles for all 24 actuators.
        std::vector<double> actuatorPosition;
    
        // Transforms for head, left arm, right arm, left leg, and right leg.
        Transform headT;
        Transform leftArmT;
        Transform rightArmT;
        Transform leftLegT;
        Transform rightLegT;
    
        // Vectors to store effector angles for head, left arm, right arm, left leg, and right leg.
        std::vector<double> headP;
        std::vector<double> leftArmP;
        std::vector<double> rightArmP;
        std::vector<double> leftLegP;
        std::vector<double> rightLegP;
    
        // Array to store a mapping from array indices to actuator names.
        std::string indexToStringMap[24];
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
                  HEAD_PITCH};

#endif