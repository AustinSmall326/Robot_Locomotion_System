#ifndef PRINT_JOINTS_H
#define PRINT_JOINTS_H

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include <alerror/alerror.h>

#include <alproxies/dcmproxy.h>

// Used to read values of ALMemory directly in RAM.
#include <almemoryfastaccess/almemoryfastaccess.h>

// Output stream libraries included for testing only.
#include <iostream>
#include <fstream>

namespace AL
{
    class ALBroker;
    class ALMemoryFastAccess;
}

class printJoints : public AL::ALModule
{
    public:
        // Methods.
        printJoints(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~printJoints();
    
    private:
        // Fields.
        std::vector<std::string> fSensorKeys;                        // Sensor keys in ALMemory.
        boost::shared_ptr<AL::ALMemoryFastAccess> fMemoryFastAccess;
    
        std::vector<float> sensorValues;
    
        static const int numActuators = 24;

        std::string indexToStringMap[numActuators];                  // Array to store a mapping from array indices to actuator names.
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