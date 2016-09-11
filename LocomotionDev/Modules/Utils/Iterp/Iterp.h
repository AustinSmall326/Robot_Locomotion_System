/** @file    Iterp.h
 *  @brief   Header file to iterp class, which interpolates the Nao's motion from its
 *           current stance to a new, user defined stance.
 *  @author  Austin Small.
 */

#ifndef ITERP_H
#define ITERP_H

#include <alcommon/alproxy.h>
#include "../../LocomotionDefines.h"
#include "../../Kinematics/KinematicsWrapper.h"

// Output stream libraries included for testing only.
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sys/time.h>
#include <vector>

/** @brief		iterp class interpolates the Nao's motion from its current stance
 *              to a new, user defined stance.
 */
class Iterp
{
    public:
        // Methods.
        Iterp(AL::ALProxy* proxy);
        virtual ~Iterp();
    
        void linearIterp(Transform headT, Transform leftArmT, Transform rightArmT,
                         Transform leftLegT, Transform rightLegT, double stiffnessVal, double totalT);
        void linearIterp(std::vector<float> nextPositionVect, double stiffnessVal, double totalT);
        void linearIterp(std::vector<float> nextPositionVect, std::vector<float> nextStiffnessVect, double totalT);
    
    private:
        // Fields.
        AL::ALProxy* shmProxy;
    
        std::vector<float> expectedPositionVect;
        std::vector<float> expectedStiffnessVect;
    
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
                  HEAD_PITCH};

#endif