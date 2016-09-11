#include "shmTestModule.h"

shmTestModule::shmTestModule(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
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

    setModuleDescription("Module developed to test functionality of shm library.");
    
    proxy = new AL::ALProxy(broker, "shm", 0, 0);

    // Stiffness.
    proxy->call<void>("setJointStiffness", 3, 0.2);
    proxy->call<void>("setJointStiffness", 5, 0.402);
    proxy->call<void>("setJointStiffness", 10, 0.204);
    
    // Position.
    proxy->call<void>("setJointPosition", 3, 2.2);
    proxy->call<void>("setJointPosition", 5, 1.03);
    proxy->call<void>("setJointPosition", 10, 0.1);
    
    // Flag.
    proxy->call<void>("setFlag", 1);

    // Print joint stiffness / position and flag.
    std::ofstream outputFile;
    outputFile.open("/home/nao/lib/test.txt");
    
    for (int i = 0; i < 24; i++)
    {
        outputFile << "Joint : " << i << " Stiffness : " << proxy->call<float>("getJointStiffness", i) << std::endl;
    }
    
    for (int i = 0; i < 24; i++)
    {
        outputFile << "Joint : " << i << " Position : " << proxy->call<float>("getJointPosition", i) << std::endl;
    }
    
    outputFile << "Flag : " << proxy->call<int>("getFlag") << std::endl;
    
    outputFile.close();
}

shmTestModule::~shmTestModule()
{}