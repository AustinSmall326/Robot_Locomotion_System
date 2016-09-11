/** @file    Iterp.cpp
 *  @brief   cpp file to Iterp class, which interpolates the Nao's motion from its
 *           current stance to a new, user defined stance.
 *  @author  Austin Small.
 */

#include "Iterp.h"

/** @brief   Iterp class constructor.
 *
 *  @param   proxy      Pointer to shm module.
 */

Iterp::Iterp(AL::ALProxy* proxy) :
    expectedPositionVect(GLOBAL_NUM_ACTUATORS),
    expectedStiffnessVect(GLOBAL_NUM_ACTUATORS)
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
    
    shmProxy = proxy;
}

/** @brief   Destructor for Iterp class.
 *
 */

Iterp::~Iterp()
{}

/** @brief   Linearly interpolates actuator motion from the Nao's current stance to a new,
 *           user defined stance.
 *
 *  @param   Transform headT
 *  @param   Transform leftArmT
 *  @param   Transform rightArmT
 *  @param   Transform leftLegT
 *  @param   Transform rightLegT
 *  @param   totalT     Duration of linear interpolation.
 *  @param   stiffness  Stiffness of actuators during linear interpolation.
 *
 */

void Iterp::linearIterp(Transform headT, Transform leftArmT, Transform rightArmT, Transform leftLegT,
                        Transform rightLegT, double stiffnessVal, double totalT)
{
    // Store the Nao's current joint angles.
    std::vector <float> currentPositionVect(GLOBAL_NUM_ACTUATORS);
    shmProxy->call<void>("getActualJointPosition", currentPositionVect);
    
    // Determine new joint angles for Nao.
    std::vector<double> leftArmP  = InvertArmL(leftArmT);
    std::vector<double> rightArmP = InvertArmR(rightArmT);
    std::vector<double> legsP     = InvertLegs(leftLegT, rightLegT);
    std::vector<double> headP     = InvertHead(headT);
    
    std::vector<float> nextPositionVect(GLOBAL_NUM_ACTUATORS);

    nextPositionVect[L_ARM_SHOULDER_PITCH] = (float) leftArmP[0];
    nextPositionVect[L_ARM_SHOULDER_ROLL]  = (float) leftArmP[1];
    nextPositionVect[L_ARM_ELBOW_YAW]      = (float) leftArmP[2];
    nextPositionVect[L_ARM_ELBOW_ROLL]     = (float) leftArmP[3];
    nextPositionVect[L_ARM_WRIST_YAW]      = (float) currentPositionVect[L_ARM_WRIST_YAW];
    nextPositionVect[R_ARM_SHOULDER_PITCH] = (float) rightArmP[0];
    nextPositionVect[R_ARM_SHOULDER_ROLL]  = (float) rightArmP[1];
    nextPositionVect[R_ARM_ELBOW_YAW]      = (float) rightArmP[2];
    nextPositionVect[R_ARM_ELBOW_ROLL]     = (float) rightArmP[3];
    nextPositionVect[R_ARM_WRIST_YAW]      = (float) currentPositionVect[R_ARM_WRIST_YAW];
    nextPositionVect[L_LEG_HIP_YAW_PITCH]  = (float) legsP[0];
    nextPositionVect[L_LEG_HIP_ROLL]       = (float) legsP[1];
    nextPositionVect[L_LEG_HIP_PITCH]      = (float) legsP[2];
    nextPositionVect[L_LEG_KNEE_PITCH]     = (float) legsP[3];
    nextPositionVect[L_LEG_ANKLE_PITCH]    = (float) legsP[4];
    nextPositionVect[L_LEG_ANKLE_ROLL]     = (float) legsP[5];
    nextPositionVect[R_LEG_HIP_ROLL]       = (float) legsP[6];
    nextPositionVect[R_LEG_HIP_PITCH]      = (float) legsP[7];
    nextPositionVect[R_LEG_KNEE_PITCH]     = (float) legsP[8];
    nextPositionVect[R_LEG_ANKLE_PITCH]    = (float) legsP[9];
    nextPositionVect[R_LEG_ANKLE_ROLL]     = (float) legsP[10];
    nextPositionVect[HEAD_YAW]             = (float) headP[0];
    nextPositionVect[HEAD_PITCH]           = (float) headP[1];
    
    // Determine new stiffness for joints.
    std::vector<float> nextStiffnessVect(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        nextStiffnessVect[i] = (float) stiffnessVal;
    }
    
    linearIterp(nextPositionVect, nextStiffnessVect, totalT);
}

/** @brief   Linearly interpolates actuator motion from the Nao's current stance to a new,
 *           user defined stance.
 *
 *  @param   angles     A vector containing angles for all 24 actuators.
 *  @param   totalT     Duration of linear interpolation.
 *  @param   stiffness  Stiffness of actuators during linear interpolation.
 *
 */

void Iterp::linearIterp(std::vector<float> nextPositionVect, double stiffnessVal, double totalT)
{
    // Determine new stiffness for joints.
    std::vector<float> nextStiffnessVect(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        nextStiffnessVect[i] = (float) stiffnessVal;
    }
    
    linearIterp(nextPositionVect, nextStiffnessVect, totalT);
}

/** @brief   Linearly interpolates actuator motion from the Nao's current stance to a new,
 *           user defined stance.
 *
 *  @param   angles     A vector containing angles for all 23 actuators.
 *  @param   stiffness  A vector containing stiffness values for all 23 actuators.
 *  @param   totalT     Duration of linear interpolation in seconds.
 *
 */

void Iterp::linearIterp(std::vector<float> nextPositionVect, std::vector<float> nextStiffnessVect, double totalT)
{
    // Store the Nao's current joint angles.
    std::vector <float> currentPositionVect(GLOBAL_NUM_ACTUATORS);
    shmProxy->call<void>("getActualJointPosition", currentPositionVect);
    
    // Store the Nao's current joing stiffness.
    std::vector <float> currentStiffnessVect(GLOBAL_NUM_ACTUATORS);
    shmProxy->call<void>("getCommandedJointStiffness", currentStiffnessVect);
    
    std::cout << "current stiffness" << std::endl;
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        std::cout << currentStiffnessVect[i] << std::endl;
    }
    // Determine slope (position vs. time) for each actuator.
    std::vector <float> slopePosition(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        slopePosition[i] = (nextPositionVect[i] - currentPositionVect[i]) / totalT;
    }
    
    // Determine slope (stiffness vs. time) for each actuator.
    std::vector <float> slopeStiffness(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        slopeStiffness[i] = (nextStiffnessVect[i] - currentStiffnessVect[i]) / totalT;
    }
    
    /** Update actuator positions and stiffness in a linear fashion. **/
    
    // Indicate to shared memory that actuators are in motion.
    // Note:  DO NOT PROCEED UNTIL ALL MOTION HAS STOPPED.
    
    while (true)
    {
        usleep(2000); // 2 ms sleep.

        std::vector <bool> tempVect(GLOBAL_NUM_ACTUATORS);
        shmProxy->call<void>("getInMotion", tempVect);
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            if (tempVect[i])
            {
                continue;
            }
        }
        
        break;
    }
    
    std::vector <bool> inMotionVect(GLOBAL_NUM_ACTUATORS);

    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        inMotionVect[i] = true;
    }
        
    shmProxy->call<void>("setInMotion", inMotionVect);

    struct timeval tv;
    
    gettimeofday(&tv, NULL);
    unsigned long long startTime   = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000; // ms.
    unsigned long long lastTime    = startTime - GLOBAL_DCM_TIME_STEP * 1000;                                          // ms.
    unsigned long long currentTime = startTime;                                                                        // ms.
    
    while (currentTime <= (startTime + 1000 * totalT))
    {
        double timeDiffMSec = currentTime - lastTime;
        double timeDiffSec  = timeDiffMSec / 1000;
        
        // Check if shared memory is ready for update and that we have exceeded the DCM time step.
        if (shmProxy->call<int>("getFlag") == 1)
        {
            // Update position and stiffness for all actuators.
            for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
            {
                expectedPositionVect[i]  = currentPositionVect[i] + slopePosition[i] * (currentTime - startTime) / 1000;
                expectedStiffnessVect[i] = currentStiffnessVect[i] + slopeStiffness[i] * (currentTime - startTime) / 1000;
            }
            
            shmProxy->call<void>("setJointPosition",  expectedPositionVect);
            shmProxy->call<void>("setJointStiffness", expectedStiffnessVect);
            
            shmProxy->call<void>("setFlag", 0);
            
            // Update last time of update to actuators.
            gettimeofday(&tv, NULL);
            lastTime = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
        }
        else
        {
            // Sleep for a period to prevent CPU overusage.
            usleep(500); // 0.5 ms sleep.
        }
        
        // Update the current time.
        gettimeofday(&tv, NULL);
        currentTime = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
    }
    
    // Indicate to shared memory that actuators are no longer in motion.
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        inMotionVect[i] = false;
    }
    
    shmProxy->call<void>("setInMotion", inMotionVect);
}