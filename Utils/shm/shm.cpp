/** @file    shm
 *  @brief   cpp file to shm class, which allows for transferring joint angles accross 
 *           independent applications.
 *  @author  Austin Small.
 */

#include "shm.h"

/** @brief   shm class constructor.
 *
 *  @param   broker     Pointer to NAOQI broker.
 *  @param   name       Module name.
 */

shm::shm(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    fMemoryFastAccessActualPosition(boost::shared_ptr<AL::ALMemoryFastAccess>(new AL::ALMemoryFastAccess())),
    positionVect(GLOBAL_NUM_ACTUATORS),
    stiffnessVect(GLOBAL_NUM_ACTUATORS),
    boolInMotionVect(GLOBAL_NUM_ACTUATORS),
    intInMotionVect(GLOBAL_NUM_ACTUATORS)
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
    indexToStringMap[16] = "R_LEG_HIP_ROLL";        // Right leg.
    indexToStringMap[17] = "R_LEG_HIP_PITCH";
    indexToStringMap[18] = "R_LEG_KNEE_PITCH";
    indexToStringMap[19] = "R_LEG_ANKLE_PITCH";
    indexToStringMap[20] = "R_LEG_ANKLE_ROLL";
    indexToStringMap[21] = "HEAD_YAW";              // Head.
    indexToStringMap[22] = "HEAD_PITCH";
    
    initFastAccess();
    
    setModuleDescription("Module to handle communication between two applications, through ALMemory.");
    
    // Load setJointStiffness into NAOQI, such that it can be called from other modules.
    functionName("setJointStiffness", getName() , "Set stiffness for all joints.");
    addParam("Value vector.", "New stiffness values for all joints");
    BIND_METHOD(shm::setJointStiffness);
    
    // Load getCommandedJointStiffness into NAOQI, such that it can be called from other modules.
    functionName("getCommandedJointStiffness", getName() , "Get last commanded joint stiffness for all joints.");
    addParam("Output vector.", "Reference to output vector.");
    BIND_METHOD(shm::getCommandedJointStiffness);
    
    // Load setJointPosition into NAOQI, such that it can be called from other modules.
    functionName("setJointPosition", getName() , "Set position for all joints.");
    addParam("Value vector.", "New position for all joints");
    BIND_METHOD(shm::setJointPosition);
    
    // Load getCommandedJointPosition into NAOQI, such that it can be called from other modules.
    functionName("getCommandedJointPosition", getName() , "Get last commanded joint position for all joints.");
    addParam("Output vector.", "Reference to output vector.");
    BIND_METHOD(shm::getCommandedJointPosition);
    
    // Load getActualJointPosition into NAOQI, such that it can be called from other modules.
    functionName("getActualJointPosition", getName() , "Get current position for all joints.");
    addParam("Output vector.", "Reference to output vector.");
    BIND_METHOD(shm::getActualJointPosition);
    
    // Load setFlag into NAOQI, such that it can be called from other modules.
    functionName("setFlag", getName() , "Set the read/write flag.");
    addParam("newValue", "Updated value for flag");
    BIND_METHOD(shm::setFlag);
    
    // Load getFlag into NAOQI, such that it can be called from other modules.
    functionName("getFlag", getName() , "Get the read/write flag value.");
    BIND_METHOD(shm::getFlag);
    
    // Load setInMotion into NAOQI, such that it can be called from other modules.
    functionName("setInMotion", getName() , "Set a boolean value, indicating whether a joint is in motion.");
    addParam("newBoolArray", "Updated actuator states.");
    BIND_METHOD(shm::setInMotion);
    
    // Load getInMotion into NAOQI, such that it can be called from other modules.
    functionName("getInMotion", getName() , "Get the boolean value, indicating whether a joint is in motion.");
    addParam("Output vector.", "Reference to output vector.");
    BIND_METHOD(shm::getInMotion);

    // Initialize shared memory.
    // Stiffness.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        stiffnessVect[i] = GLOBAL_REST_STIFFNESS;
    }
    
    // Positions.
    fMemoryFastAccessActualPosition->GetValues(positionVect);
    
    // Flag.
    flag = 0;
    
    // Actuators in motion.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        intInMotionVect[i] = 0; // False.
    }
}

/** @brief   Destructor for shm class.
 *
 */

shm::~shm()
{}

/** @brief   Helper method to initialize ALMemory fast access.
 *
 */

void shm::initFastAccess(void)
{
    fKeysActualPosition.clear();
    fKeysActualPosition.resize(GLOBAL_NUM_ACTUATORS);
   
    // Joint sensors list (for actual joint angles).
    fKeysActualPosition[L_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/LShoulderPitch/Position/Sensor/Value");
    fKeysActualPosition[L_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/LShoulderRoll/Position/Sensor/Value");
    fKeysActualPosition[L_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/LElbowYaw/Position/Sensor/Value");
    fKeysActualPosition[L_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/LElbowRoll/Position/Sensor/Value");
    fKeysActualPosition[L_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/LWristYaw/Position/Sensor/Value");
    fKeysActualPosition[R_ARM_SHOULDER_PITCH] = std::string("Device/SubDeviceList/RShoulderPitch/Position/Sensor/Value");
    fKeysActualPosition[R_ARM_SHOULDER_ROLL]  = std::string("Device/SubDeviceList/RShoulderRoll/Position/Sensor/Value");
    fKeysActualPosition[R_ARM_ELBOW_YAW]      = std::string("Device/SubDeviceList/RElbowYaw/Position/Sensor/Value");
    fKeysActualPosition[R_ARM_ELBOW_ROLL]     = std::string("Device/SubDeviceList/RElbowRoll/Position/Sensor/Value");
    fKeysActualPosition[R_ARM_WRIST_YAW]      = std::string("Device/SubDeviceList/RWristYaw/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_HIP_YAW_PITCH]  = std::string("Device/SubDeviceList/LHipYawPitch/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/LHipRoll/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/LHipPitch/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/LKneePitch/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/LAnklePitch/Position/Sensor/Value");
    fKeysActualPosition[L_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/LAnkleRoll/Position/Sensor/Value");
    fKeysActualPosition[R_LEG_HIP_ROLL]       = std::string("Device/SubDeviceList/RHipRoll/Position/Sensor/Value");
    fKeysActualPosition[R_LEG_HIP_PITCH]      = std::string("Device/SubDeviceList/RHipPitch/Position/Sensor/Value");
    fKeysActualPosition[R_LEG_KNEE_PITCH]     = std::string("Device/SubDeviceList/RKneePitch/Position/Sensor/Value");
    fKeysActualPosition[R_LEG_ANKLE_PITCH]    = std::string("Device/SubDeviceList/RAnklePitch/Position/Sensor/Value");
    fKeysActualPosition[R_LEG_ANKLE_ROLL]     = std::string("Device/SubDeviceList/RAnkleRoll/Position/Sensor/Value");
    fKeysActualPosition[HEAD_YAW]             = std::string("Device/SubDeviceList/HeadYaw/Position/Sensor/Value");
    fKeysActualPosition[HEAD_PITCH]           = std::string("Device/SubDeviceList/HeadPitch/Position/Sensor/Value");
    
    // Create the fast memory access.
    fMemoryFastAccessActualPosition->ConnectToVariables(getParentBroker(), fKeysActualPosition, false);
}

/** @brief   Stores the stiffness for all joints in ALMemory.
 *
 *  @param   jointValue New stiffness for specified joint.
 *
 */

void shm::setJointStiffness(std::vector<float> stiffness)
{
    // Copy values to stiffnessVect.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        stiffnessVect[i] = stiffness[i];
    }
}

/** @brief   Returns the last commanded stiffness for all joints in ALMemory.
 *
 *  @return  Joint last commanded stiffness.
 *
 */

void shm::getCommandedJointStiffness(std::vector<float>& outputVect)
{
    // Copy values to outputVect.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        outputVect[i] = stiffnessVect[i];
    }
}

/** @brief   Stores the position for a specified joint in ALMemory.
 *
 *  @param   jointValue New position for all joints.
 *
 */

void shm::setJointPosition(std::vector<float> position)
{    
    // Copy values to positionVect.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        positionVect[i] = position[i];
    }
}

/** @brief   Returns the last commanded position for all joints in ALMemory.
 *
 *  @return  Joint last commanded angle.
 *
 */

void shm::getCommandedJointPosition(std::vector<float>& outputVect)
{
    // Copy values to outputVect.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        outputVect[i] = positionVect[i];
    }
}

/** @brief   Returns the actual position for all joints in ALMemory.
 *
 *  @return  Joint actual angle.
 *
 */

void shm::getActualJointPosition(std::vector<float>& outputVect)
{
    fMemoryFastAccessActualPosition->GetValues(outputVect);
}

/** @brief   Updates the flag value.
 *
 *  @param   newValue    Updated flag value (either 0 or 1 - a boolean value).
 *
 */

void shm::setFlag(int newValue)
{
    // Make sure newValue is either a 0 or 1.
    if (newValue != 0 && newValue != 1)
    {
        return;
    }
    
    flag = newValue;
}

/** @brief   Returns the flag value.
 *
 *  @return   The flag's value.
 *
 */

int shm::getFlag()
{
    return flag;
}

/** @brief   Updates the inMotion state of an actuator.
 *
 *  @param   state    Updated boolean value (true if the actuator is in motion, and false otherwise).
 *
 */

void shm::setInMotion(std::vector<bool> state)
{
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        if (state[i] == true)
        {
            intInMotionVect[i] = 1;
        }
        else
        {
            intInMotionVect[i] = 0;
        }
    }
}

/** @brief   Returns the inMotion state of an actuator.
 *
 */

void shm::getInMotion(std::vector<bool>& outputVect)
{
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        if (intInMotionVect[i] == 1)
        {
            outputVect[i] = true;
        }
        else
        {
            outputVect[i] = false;
        }
    }
}