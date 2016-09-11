/** @file    fastGetSetDCM.cpp
 *  @brief   cpp file to fastGetSetDCM class, which allows for updating actuator values.
 *  @author  Austin Small / Zheng Tian.
 */

#include "fastGetSetDCM.h"

/** @brief   fastGetSetDCM class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

fastGetSetDCM::fastGetSetDCM(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    fMemoryFastAccess(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess()))
{
    // Initialize proxy to shared memory.
    proxy = new AL::ALProxy(broker, "shm", 0, 0);
    
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
    indexToStringMap[16] = "R_LEG_HIP_ROLL";        // Right leg.
    indexToStringMap[17] = "R_LEG_HIP_PITCH";
    indexToStringMap[18] = "R_LEG_KNEE_PITCH";
    indexToStringMap[19] = "R_LEG_ANKLE_PITCH";
    indexToStringMap[20] = "R_LEG_ANKLE_ROLL";
    indexToStringMap[21] = "HEAD_YAW";              // Head.
    indexToStringMap[22] = "HEAD_PITCH";

    // Load module into NAOQI.
    setModuleDescription("Module to get/set joints every 10ms with minimum delays.");
    
    functionName("start", getName() , "start");
    BIND_METHOD(fastGetSetDCM::start);
    
    functionName("stop", getName() , "stop");
    BIND_METHOD(fastGetSetDCM::stop);
    
    start();
}

/** @brief   Destructor for shm class.
 *
 */

fastGetSetDCM::~fastGetSetDCM()
{
    stop();
}

/** @brief   Start the fastGetSetDCM process of updating actuator stiffness/position values.
 *
 */

void fastGetSetDCM::start()
{
    signed long isDCMRunning;
    
    // Initialize dcmProxy.
    try
    {
        dcmProxy = getParentBroker()->getDcmProxy();
    }
    catch (AL::ALError& e)
    {
        throw ALERROR(getName(), "start()", "Failed to create DCM proxy : " + e.toString());
    }
    
    // Make sure that DCM is running.  Otherwise, nothing will work.
    // Note: This code has been deprecated because we do not use ALLauncher on out robots.
    /*
    try
    {
        isDCMRunning = getParentBroker()->getProxy("ALLauncher")->call<bool>("isModulePresent", std::string("DCM"));
    }
    catch (AL::ALError& e)
    {
        throw ALERROR(getName(), "start()", "Error when connecting to DCM : " + e.toString());
    }
    
    if (!isDCMRunning)
    {
        throw ALERROR(getName(), "start()", "DCM is not running.");
    }
    */
    
    init();
    connectToDCMloop();
}

/** @brief   Stop the fastGetSetDCM process.
 *
 */

void fastGetSetDCM::stop()
{
    std::vector <float> stiffnessVect(GLOBAL_NUM_ACTUATORS);
    
    // Turn off all actuators prior to exit.    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        stiffnessVect[i] = 0.0f;
    }
    
    setStiffness(stiffnessVect);
    
    fDCMPostProcessConnection.disconnect();
}

/** @brief   Initializes ALMemory fast access, aliases, DCM commands,
 *           and sets actuator stiffness/position.
 *
 */

void fastGetSetDCM::init()
{
    initFastAccess();
    createHardnessActuatorAlias();
    createPositionActuatorAlias();
    
    prepareStiffnessActuatorCommand();
    preparePositionActuatorCommand();
    
    // Set initial stiffness / position of actuators.
    std::vector<float> stiffnessVect(GLOBAL_NUM_ACTUATORS);
    std::vector<float> positionVect(GLOBAL_NUM_ACTUATORS);
    
    proxy->call<void>("getCommandedJointStiffness", stiffnessVect);
    proxy->call<void>("getCommandedJointPosition", positionVect);
    
    setStiffness(stiffnessVect);
    setPosition(positionVect);
}

/** @brief   Helper method to initialize ALMemory fast access.
 *
 */

void fastGetSetDCM::initFastAccess()
{
    fSensorKeys.clear();
    fSensorKeys.resize(GLOBAL_NUM_ACTUATORS);
    
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
    fSensorKeys[R_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    fSensorKeys[R_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    fSensorKeys[R_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    fSensorKeys[HEAD_YAW]             = std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    fSensorKeys[HEAD_PITCH]           = std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");

    // Create the fast memory access.
    fMemoryFastAccess->ConnectToVariables(getParentBroker(), fSensorKeys, false);
}

/** @brief   Create aliases for actuator stiffness.  Not sure exactly what an alias is, but it seems to tell DCM
 *           which sensors we want access to.
 */

void fastGetSetDCM::createHardnessActuatorAlias()
{
    AL::ALValue jointAliasses;

    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointStiffness");
    jointAliasses[1].arraySetSize(GLOBAL_NUM_ACTUATORS);
    
    // Joint actuators list.
    jointAliasses[1][L_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Hardness/Actuator/Value");
    jointAliasses[1][L_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Hardness/Actuator/Value");
    jointAliasses[1][R_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Hardness/Actuator/Value");
    jointAliasses[1][R_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Hardness/Actuator/Value");
    jointAliasses[1][R_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Hardness/Actuator/Value");
    jointAliasses[1][L_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Hardness/Actuator/Value");
    jointAliasses[1][R_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Hardness/Actuator/Value");
    jointAliasses[1][R_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Hardness/Actuator/Value");
    jointAliasses[1][R_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Hardness/Actuator/Value");
    jointAliasses[1][R_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Hardness/Actuator/Value");
    jointAliasses[1][HEAD_YAW]             = std::string("Device/SubDeviceList/HeadYaw/Hardness/Actuator/Value");
    jointAliasses[1][HEAD_PITCH]           = std::string("Device/SubDeviceList/HeadPitch/Hardness/Actuator/Value");
    
    // Create alias.
    try
    {
        dcmProxy->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "createHardnessActuatorAlias()", "Error when creating alias : " + e.toString());
    }
}

/** @brief   Create aliases for actuator position.  Not sure exactly what an alias is, but it seems to tell DCM
 *           which sensors we want access to.
 */

void fastGetSetDCM::createPositionActuatorAlias()
{
    AL::ALValue jointAliasses;
    
    jointAliasses.arraySetSize(2);
    jointAliasses[0] = std::string("jointActuator");
    jointAliasses[1].arraySetSize(GLOBAL_NUM_ACTUATORS);
    
    // Joint actuators list.
    jointAliasses[1][L_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Actuator/Value");
    jointAliasses[1][L_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Actuator/Value");
    jointAliasses[1][L_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Actuator/Value");
    jointAliasses[1][L_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Actuator/Value");
    jointAliasses[1][L_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Actuator/Value");
    jointAliasses[1][R_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Actuator/Value");
    jointAliasses[1][R_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Actuator/Value");
    jointAliasses[1][R_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Actuator/Value");
    jointAliasses[1][R_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Actuator/Value");
    jointAliasses[1][R_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Actuator/Value");
    jointAliasses[1][L_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Actuator/Value");
    jointAliasses[1][L_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Actuator/Value");
    jointAliasses[1][L_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Actuator/Value");
    jointAliasses[1][L_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Actuator/Value");
    jointAliasses[1][R_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Actuator/Value");
    jointAliasses[1][R_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Actuator/Value");
    jointAliasses[1][R_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Actuator/Value");
    jointAliasses[1][R_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Actuator/Value");
    jointAliasses[1][R_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Actuator/Value");
    jointAliasses[1][HEAD_YAW]             = std::string("Device/SubDeviceList/HeadYaw/Position/Actuator/Value");
    jointAliasses[1][HEAD_PITCH]           = std::string("Device/SubDeviceList/HeadPitch/Position/Actuator/Value");
    
    // Create alias.
    try
    {
        dcmProxy->createAlias(jointAliasses);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "createPositionActuatorAlias()", "Error when creating alias : " + e.toString());
    }
}

/** @brief   Initialize / set up stiffnessCommands array.
 */

void fastGetSetDCM::prepareStiffnessActuatorCommand()
{
    stiffnessCommands.arraySetSize(6);
    stiffnessCommands[0] = std::string("jointStiffness");
    stiffnessCommands[1] = std::string("ClearAll");       // When sent to DCM, clear all previous commands.
    stiffnessCommands[2] = std::string("time-separate");
    stiffnessCommands[3] = 0;
    
    // commands[4][0] represents the desired DCM execution time of the new command.
    stiffnessCommands[4].arraySetSize(1);
    
    stiffnessCommands[5].arraySetSize(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        // commands[5][i][0] is the stiffness value for each actuator.
        stiffnessCommands[5][i].arraySetSize(1);
    }
}

/** @brief   Initialize / set up positionCommands array.
 */

void fastGetSetDCM::preparePositionActuatorCommand()
{
    positionCommands.arraySetSize(6);
    positionCommands[0] = std::string("jointActuator");
    positionCommands[1] = std::string("ClearAll");       // When sent to DCM, clear all previous commands.
    positionCommands[2] = std::string("time-separate");
    positionCommands[3] = 0;
    
    // commands[4][0] represents the desired DCM execution time of the new command.
    positionCommands[4].arraySetSize(1);
    
    positionCommands[5].arraySetSize(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        // commands[5][i][0] is the stiffness value for each actuator.
        positionCommands[5][i].arraySetSize(1);
    }
}

/** @brief   Updates the stiffness of all actuators.
 *  
 *  @param   stiffnessArr   An array containing stiffness values for all actuators.
 *
 */

void fastGetSetDCM::setStiffness(std::vector <float> stiffnessVect)
{
    // Use current DCM time such that new command is executed as soon as it is received by the DCM.
    int DCMtime;
    
    try
    {
        DCMtime = dcmProxy->getTime(0);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setStiffness()", "Error with DCM getTime : " + e.toString());
    }
    
    stiffnessCommands[4][0] = DCMtime;
    
    // Load stiffness values into stiffnessCommands array.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        stiffnessCommands[5][i].arraySetSize(1);
        stiffnessCommands[5][i][0] = stiffnessVect[i];
    }
    
    try
    {
        dcmProxy->set(stiffnessCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setStiffness()", "Error when sending stiffness to DCM : " + e.toString());
    }
}

/** @brief   Updates the position of all actuators.
 *
 *  @param   stiffnessArr   An array containing position values for all actuators.
 *
 */

void fastGetSetDCM::setPosition(std::vector <float> positionVect)
{
    // Use current DCM time such that new command is executed as soon as it is received by the DCM.
    int DCMtime;
    
    try
    {
        DCMtime = dcmProxy->getTime(0);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setAngles()", "Error with DCM getTime : " + e.toString());
    }
    
    positionCommands[4][0] = DCMtime;
    
    // Load position values into positionCommands array.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        positionCommands[5][i].arraySetSize(1);
        positionCommands[5][i][0] = positionVect[i];
    }
    
    try
    {
        dcmProxy->set(positionCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "setAngles()", "Error when sending stiffness to DCM : " + e.toString());
    }
}

/** @brief   Connect fastGetSetDCM to the DCM loop.
 *
 */

void fastGetSetDCM::connectToDCMloop()
{
    // Connect callback method to the DCM post proccess.
    try
    {
        fDCMPostProcessConnection =
          getParentBroker()->getProxy("DCM")->getModule()->atPostProcess(boost::bind(&fastGetSetDCM::synchronisedDCMcallback, this));
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "connectToDCMloop()", "Error when connecting to DCM postProccess: " + e.toString());
    }
}

/** @brief   Callback method that reads stiffness / position values from shared memory and sends them 
 *           to the actuators via DCM.  This method will be called every 10 ms.
 *
 */

void fastGetSetDCM::synchronisedDCMcallback()
{
    // Use current DCM time such that new command is executed as soon as it is received by the DCM.
    int DCMtime;
    
    try
    {
        // Get absolute time 0 ms in the future (now).
        DCMtime = dcmProxy->getTime(0);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "synchronisedDCMcallback()", "Error with DCM getTime : " + e.toString());
    }
    
    stiffnessCommands[4][0] = DCMtime;
    positionCommands[4][0]  = DCMtime;
    
    // Load stiffness / position values from shared memory.
    std::vector <float> commandedStiffnessVect(GLOBAL_NUM_ACTUATORS);
    proxy->call<void>("getCommandedJointStiffness", commandedStiffnessVect);
    
    std::vector <float> commandedPositionVect(GLOBAL_NUM_ACTUATORS);
    proxy->call<void>("getCommandedJointPosition", commandedPositionVect);
    
    // Update flag to indicate that values have been read from shm.
    proxy->call<void>("setFlag", 1);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        stiffnessCommands[5][i][0] = commandedStiffnessVect[i];
        positionCommands[5][i][0]  = commandedPositionVect[i];
    }
    
    try
    {
        dcmProxy->setAlias(stiffnessCommands);
        dcmProxy->setAlias(positionCommands);
    }
    catch (const AL::ALError &e)
    {
        throw ALERROR(getName(), "synchronisedDCMcallback()", "Error when sending command to DCM : " + e.toString());
    }
}