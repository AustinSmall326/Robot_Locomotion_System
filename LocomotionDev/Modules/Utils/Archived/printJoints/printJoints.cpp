#include "printJoints.h"

printJoints::printJoints(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
    // Initialize index to string map.
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

    // Load module into NAOQI.
    setModuleDescription("Module to get joint values.");
    
    fSensorKeys.clear();
    fSensorKeys.resize(numActuators);
    
    // Joint sensors list.
    fSensorKeys[L_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    fSensorKeys[L_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    fSensorKeys[L_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    fSensorKeys[L_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    fSensorKeys[L_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    fSensorKeys[R_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    fSensorKeys[R_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    fSensorKeys[R_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    fSensorKeys[R_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    fSensorKeys[R_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");
    fSensorKeys[L_LEG_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    fSensorKeys[L_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    fSensorKeys[L_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    fSensorKeys[L_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    fSensorKeys[L_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    fSensorKeys[L_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    fSensorKeys[R_LEG_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/RHipYawPitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    fSensorKeys[R_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    fSensorKeys[HEAD_YAW]             = std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    fSensorKeys[HEAD_PITCH]           = std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    
    // Create the fast memory access.
    fMemoryFastAccess->ConnectToVariables(getParentBroker(), fSensorKeys, false);
    
    fMemoryFastAccess->GetValues(sensorValues);
    
    std::ofstream myfile;
    myfile.open ("/home/nao/lib/jointAngles.txt");
    
    for (int i = 0; i < numActuators; i++)
    {
        myfile << indexToStringMap[i] << " : " << sensorValues[i] << std::endl;
    }
    
    myfile.close();
}

printJoints::~printJoints()
{ }