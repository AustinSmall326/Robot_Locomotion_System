#include "testPose.h"

testPose::testPose(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    actuatorPosition(24), headP(2), leftArmP(4), rightArmP(4), leftLegP(6), rightLegP(6)
{
    std::ofstream myfile;
    myfile.open ("/home/nao/lib/testPoseOutput.txt");
    
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
    
    setModuleDescription("Module developed to test out different stationary poses for the robot.");
    
    // First, initialize shm and dcm proxies.
    shmProxy = new AL::ALProxy(broker, "shm", 0, 0);
    //dcmproxy = new AL::ALProxy(broker, "fastGetSetDCM", 0, 0);
    
    // Store temp joint angles in shm.
    shmProxy->call<void>("setJointPosition", 0,  1.65000f);
    shmProxy->call<void>("setJointPosition", 1,  0.18000f);
    shmProxy->call<void>("setJointPosition", 2, -0.53000f);
    shmProxy->call<void>("setJointPosition", 3, -0.32000f);
    shmProxy->call<void>("setJointPosition", 4,  0.00000f);
    shmProxy->call<void>("setJointPosition", 5,  1.65000f);
    shmProxy->call<void>("setJointPosition", 6, -0.18000f);
    shmProxy->call<void>("setJointPosition", 7,  0.53000f);
    shmProxy->call<void>("setJointPosition", 8,  0.32000f);
    shmProxy->call<void>("setJointPosition", 9,  0.00000f);
    shmProxy->call<void>("setJointPosition", 10,-0.16000f);
    shmProxy->call<void>("setJointPosition", 11, 0.09000f);
    shmProxy->call<void>("setJointPosition", 12,-0.33000f);
    shmProxy->call<void>("setJointPosition", 13, 1.79000f);
    shmProxy->call<void>("setJointPosition", 14,-1.15000f);
    shmProxy->call<void>("setJointPosition", 15,-0.03000f);
    shmProxy->call<void>("setJointPosition", 16, 0.00000f);
    shmProxy->call<void>("setJointPosition", 17, 0.09000f);
    shmProxy->call<void>("setJointPosition", 18,-0.33000f);
    shmProxy->call<void>("setJointPosition", 19, 1.79000f);
    shmProxy->call<void>("setJointPosition", 20,-1.15000f);
    shmProxy->call<void>("setJointPosition", 21, 0.03000f);
    shmProxy->call<void>("setJointPosition", 22, 0.00000f);
    shmProxy->call<void>("setJointPosition", 23, 0.00000f);
    
    for (int i = 0; i < 24; i++)
    {
        actuatorPosition[i] = (double) shmProxy->call<float>("getJointPosition", i);
    }
    
    // Transfer joint angles to effector vectors.
    headP[0] = actuatorPosition[HEAD_YAW];
    headP[1] = actuatorPosition[HEAD_PITCH];
    
    leftArmP[0] = actuatorPosition[L_ARM_SHOULDER_PITCH];
    leftArmP[1] = actuatorPosition[L_ARM_SHOULDER_ROLL];
    leftArmP[2] = actuatorPosition[L_ARM_ELBOW_YAW];
    leftArmP[3] = actuatorPosition[L_ARM_ELBOW_ROLL];
    
    // print left artm effector
    myfile<< " left arm effector angles" << std::endl;
    
    for (int i = 0; i < leftArmP.size(); i++)
    {
        myfile << leftArmP[i] << std::endl;
        
    }

    rightArmP[0] = actuatorPosition[R_ARM_SHOULDER_PITCH];
    rightArmP[1] = actuatorPosition[R_ARM_SHOULDER_ROLL];
    rightArmP[2] = actuatorPosition[R_ARM_ELBOW_YAW];
    rightArmP[3] = actuatorPosition[R_ARM_ELBOW_ROLL];
    
    leftLegP[0] = actuatorPosition[L_LEG_HIP_YAW_PITCH];
    leftLegP[1] = actuatorPosition[L_LEG_HIP_ROLL];
    leftLegP[2] = actuatorPosition[L_LEG_HIP_PITCH];
    leftLegP[3] = actuatorPosition[L_LEG_KNEE_PITCH];
    leftLegP[4] = actuatorPosition[L_LEG_ANKLE_PITCH];
    leftLegP[5] = actuatorPosition[L_LEG_ANKLE_ROLL];
    
    rightLegP[0] = actuatorPosition[R_LEG_HIP_YAW_PITCH];
    rightLegP[1] = actuatorPosition[R_LEG_HIP_ROLL];
    rightLegP[2] = actuatorPosition[R_LEG_HIP_PITCH];
    rightLegP[3] = actuatorPosition[R_LEG_KNEE_PITCH];
    rightLegP[4] = actuatorPosition[R_LEG_ANKLE_PITCH];
    rightLegP[5] = actuatorPosition[R_LEG_ANKLE_ROLL];
    
    // Generate transforms for head, left arm, right arm, left leg, and right leg.
    headT     = ForwardHead(headP);
    leftArmT  = ForwardArmL(leftArmP);
    rightArmT = ForwardArmR(rightArmP);
    leftLegT  = ForwardLegL(leftLegP);
    rightLegT = ForwardLegR(rightLegP);
    
    // Decompose transforms into position and orientation.
    std::vector<double> head6D     = position6D(headT);
    std::vector<double> leftArm6D  = position6D(leftArmT);
    std::vector<double> rightArm6D = position6D(rightArmT);
    std::vector<double> leftLeg6D  = position6D(leftLegT);
    std::vector<double> rightLeg6D = position6D(rightLegT);
    
    
    
    myfile<< " left arm effector decomposed" << std::endl;
    
    for (int i = 0; i < leftArm6D.size(); i++)
    {
        myfile << leftArm6D[i] << std::endl;
        
    }
    
    
    
    // Adjust position and orientation of effectors.
    leftArm6D[2] = leftArm6D[2] + 25.0;
    
    // Regenerate transforms.
    headT     = transform6D(head6D);
    leftArmT  = transform6D(leftArm6D);
    rightArmT = transform6D(rightArm6D);
    leftLegT  = transform6D(leftLeg6D);
    rightLegT = transform6D(rightLeg6D);
    
    // Regenerate joint angles.
    headP     = InvertHead(headT);
    leftArmP  = InvertArmL(leftArmT);
    rightArmP = InvertArmR(rightArmT);
    leftLegP  = InvertLegL(leftLegT);
    rightLegP = InvertLegR(rightLegT);
    
    // print left artm effector
    myfile<< " left arm effector angles updated" << std::endl;
    
    for (int i = 0; i < leftArmP.size(); i++)
    {
        myfile << leftArmP[i] << std::endl;
        
    }

    // Transfer updated joint angles to actuator position array.
    actuatorPosition[HEAD_YAW]   = headP[0];
    actuatorPosition[HEAD_PITCH] = headP[1];
    
    actuatorPosition[L_ARM_SHOULDER_PITCH] = leftArmP[0];
    actuatorPosition[L_ARM_SHOULDER_ROLL]  = leftArmP[1];
    actuatorPosition[L_ARM_ELBOW_YAW]      = leftArmP[2];
    actuatorPosition[L_ARM_ELBOW_ROLL]     = leftArmP[3];
    
    actuatorPosition[R_ARM_SHOULDER_PITCH] = rightArmP[0];
    actuatorPosition[R_ARM_SHOULDER_ROLL]  = rightArmP[1];
    actuatorPosition[R_ARM_ELBOW_YAW]      = rightArmP[2];
    actuatorPosition[R_ARM_ELBOW_ROLL]     = rightArmP[3];
    
    actuatorPosition[L_LEG_HIP_YAW_PITCH] = leftLegP[0];
    actuatorPosition[L_LEG_HIP_ROLL]      = leftLegP[1];
    actuatorPosition[L_LEG_HIP_PITCH]     = leftLegP[2];
    actuatorPosition[L_LEG_KNEE_PITCH]    = leftLegP[3];
    actuatorPosition[L_LEG_ANKLE_PITCH]   = leftLegP[4];
    actuatorPosition[L_LEG_ANKLE_ROLL]    = leftLegP[5];
    
    actuatorPosition[R_LEG_HIP_YAW_PITCH] = rightLegP[0];
    actuatorPosition[R_LEG_HIP_ROLL]      = rightLegP[1];
    actuatorPosition[R_LEG_HIP_PITCH]     = rightLegP[2];
    actuatorPosition[R_LEG_KNEE_PITCH]    = rightLegP[3];
    actuatorPosition[R_LEG_ANKLE_PITCH]   = rightLegP[4];
    actuatorPosition[R_LEG_ANKLE_ROLL]    = rightLegP[5];
    
    // Store these angles in output file.
    for (int i = 0; i < 24; i++)
    {
        myfile << indexToStringMap[i] << " : " << actuatorPosition[i] << std::endl;

    }
    
    myfile.close();

    
    // Update shared memory.
    for (int i = 0; i < 24; i++)
    {
        shmProxy->call<void>("setJointPosition", i, (float) actuatorPosition[i]);
    }
}

testPose::~testPose()
{ }