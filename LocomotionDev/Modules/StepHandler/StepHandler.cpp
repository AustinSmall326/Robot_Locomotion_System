/** @file    StepHandler.cpp
 *  @brief   Contains StepHandler class, which interfaces behavior modules with locomotion.
 *  @author  Austin Small.
 */

#include "StepHandler.h"
#include <unistd.h>

/** @brief   StepHandler class constructor.
 */

StepHandler::StepHandler(boost::shared_ptr<AL::ALBroker> broker, const std::string &name) :
    AL::ALModule(broker, name),
    shmProxy(NULL),
    pauseLoop(false),
    stopLoop(false),
    loopRunning(false),
    loopPaused(false),
    continueTestMotion(false),
    testMotionLoopRunning(false),
    testMotionLoopRunOnce(false)
{
    loadLocomotionParameters();
    
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
    
    setModuleDescription("Module runs locomotion system.");

    // Load communicate into NAOQI, such that it can be called from other modules.
    functionName("communicate", getName() , "Communicate messages between computer and StepHandler module.");
    addParam("message", "Message to communicate.");
    BIND_METHOD(StepHandler::communicate);

    shmProxy = new AL::ALProxy(broker, "shm", 0, 0);
}

StepHandler::~StepHandler()
{ }

void StepHandler::exit(void)
{
    stopLoop = true;
    
    while (loopRunning)
    {
        usleep(200);
    }
    
    restStance();
}

std::string StepHandler::communicate(std::string inMessage)
{
    int endLocus = inMessage.find("_");
    std::string command(inMessage.substr(0, endLocus));
    std::string message(inMessage.substr(endLocus + 1, inMessage.length() - (endLocus + 1)));
        
    if (command.compare("RUNTESTS") == 0)
    {
        if (message.compare("COMContainerTests") == 0)
        {            
            return COMContainerTests::runTests();
        }
        else if (message.compare("TransformTests") == 0)
        {
            return TransformTests::runTests();
        }
        else if (message.compare("TrajectoryTests") == 0)
        {
            return TrajectoryTests::runTests();
        }
        else if (message.compare("PointTests") == 0)
        {
            return PointTests::runTests();
        }
        else if (message.compare("KinematicsTests") == 0)
        {
            return TestKinematics::runTests();
        }
        else if (message.compare("StepHandlerTests") == 0)
        {
            return runTests();
        }
        else if (message.compare("StepHandlerVisualTestsPartOne") == 0)
        {
            return runVisualTestsPartOne();
        }
        else if (message.compare("StepHandlerVisualTestsPartTwo") == 0)
        {
            return runVisualTestsPartTwo();
        }
    }
    /*
    else if (command.compare("GETPARAMVALUE") == 0)
    {
        return getParamValue(message);
    }
    else if (command.compare("SETPARAMVALUE") == 0)
    {
        int endLocus = message.find("_");
        std::string paramName(message.substr(0, endLocus));
        std::string paramValue(message.substr(endLocus + 1, message.length() - (endLocus + 1)));
        
        return setParamValue(paramName, paramValue);
    }
    else if (command.compare("STARTTESTMOTION") == 0)
    {
        startTestMotion();
        
        return std::string("Success");
    }
    else if (command.compare("STOPTESTMOTION") == 0)
    {
        stopTestMotion();
        
        return std::string("Success");
    }
*/
    
    return "Failed";
}

void StepHandler::readyStance(void)
{
    // Generate vector for angles at rest pose.
    std::vector<double> anglesRestDouble(GLOBAL_NUM_ACTUATORS);
    
    anglesRestDouble[0]  = GLOBAL_REST_L_ARM_SHOULDER_PITCH;  // Left arm.
    anglesRestDouble[1]  = GLOBAL_REST_L_ARM_SHOULDER_ROLL;
    anglesRestDouble[2]  = GLOBAL_REST_L_ARM_ELBOW_YAW;
    anglesRestDouble[3]  = GLOBAL_REST_L_ARM_ELBOW_ROLL;
    anglesRestDouble[4]  = GLOBAL_REST_L_ARM_WRIST_YAW;
    anglesRestDouble[5]  = GLOBAL_REST_R_ARM_SHOULDER_PITCH;  // Right arm.
    anglesRestDouble[6]  = GLOBAL_REST_R_ARM_SHOULDER_ROLL;
    anglesRestDouble[7]  = GLOBAL_REST_R_ARM_ELBOW_YAW;
    anglesRestDouble[8]  = GLOBAL_REST_R_ARM_ELBOW_ROLL;
    anglesRestDouble[9]  = GLOBAL_REST_R_ARM_WRIST_YAW;
    anglesRestDouble[10] = GLOBAL_REST_L_LEG_HIP_YAW_PITCH;   // Left leg.
    anglesRestDouble[11] = GLOBAL_REST_L_LEG_HIP_ROLL;
    anglesRestDouble[12] = GLOBAL_REST_L_LEG_HIP_PITCH;
    anglesRestDouble[13] = GLOBAL_REST_L_LEG_KNEE_PITCH;
    anglesRestDouble[14] = GLOBAL_REST_L_LEG_ANKLE_PITCH;
    anglesRestDouble[15] = GLOBAL_REST_L_LEG_ANKLE_ROLL;
    anglesRestDouble[16] = GLOBAL_REST_R_LEG_HIP_ROLL;        // Right leg.
    anglesRestDouble[17] = GLOBAL_REST_R_LEG_HIP_PITCH;
    anglesRestDouble[18] = GLOBAL_REST_R_LEG_KNEE_PITCH;
    anglesRestDouble[19] = GLOBAL_REST_R_LEG_ANKLE_PITCH;
    anglesRestDouble[20] = GLOBAL_REST_R_LEG_ANKLE_ROLL;
    anglesRestDouble[21] = GLOBAL_REST_HEAD_YAW;              // Head.
    anglesRestDouble[22] = GLOBAL_REST_HEAD_PITCH;
    
    std::vector<float> anglesRestFloat(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        anglesRestFloat[i] = (float) anglesRestDouble[i];
    }
    
    // Generate vector for angles at midway point in interpolation.
    std::vector<double> anglesMidwayDouble(GLOBAL_NUM_ACTUATORS);
    
    anglesMidwayDouble[0]  = GLOBAL_GETUP_MIDWAY_L_ARM_SHOULDER_PITCH;  // Left arm.
    anglesMidwayDouble[1]  = GLOBAL_GETUP_MIDWAY_L_ARM_SHOULDER_ROLL;
    anglesMidwayDouble[2]  = GLOBAL_GETUP_MIDWAY_L_ARM_ELBOW_YAW;
    anglesMidwayDouble[3]  = GLOBAL_GETUP_MIDWAY_L_ARM_ELBOW_ROLL;
    anglesMidwayDouble[4]  = GLOBAL_GETUP_MIDWAY_L_ARM_WRIST_YAW;
    anglesMidwayDouble[5]  = GLOBAL_GETUP_MIDWAY_R_ARM_SHOULDER_PITCH;  // Right arm.
    anglesMidwayDouble[6]  = GLOBAL_GETUP_MIDWAY_R_ARM_SHOULDER_ROLL;
    anglesMidwayDouble[7]  = GLOBAL_GETUP_MIDWAY_R_ARM_ELBOW_YAW;
    anglesMidwayDouble[8]  = GLOBAL_GETUP_MIDWAY_R_ARM_ELBOW_ROLL;
    anglesMidwayDouble[9]  = GLOBAL_GETUP_MIDWAY_R_ARM_WRIST_YAW;
    anglesMidwayDouble[10] = GLOBAL_GETUP_MIDWAY_L_LEG_HIP_YAW_PITCH;   // Left leg.
    anglesMidwayDouble[11] = GLOBAL_GETUP_MIDWAY_L_LEG_HIP_ROLL;
    anglesMidwayDouble[12] = GLOBAL_GETUP_MIDWAY_L_LEG_HIP_PITCH;
    anglesMidwayDouble[13] = GLOBAL_GETUP_MIDWAY_L_LEG_KNEE_PITCH;
    anglesMidwayDouble[14] = GLOBAL_GETUP_MIDWAY_L_LEG_ANKLE_PITCH;
    anglesMidwayDouble[15] = GLOBAL_GETUP_MIDWAY_L_LEG_ANKLE_ROLL;
    anglesMidwayDouble[16] = GLOBAL_GETUP_MIDWAY_R_LEG_HIP_ROLL;        // Right leg.
    anglesMidwayDouble[17] = GLOBAL_GETUP_MIDWAY_R_LEG_HIP_PITCH;
    anglesMidwayDouble[18] = GLOBAL_GETUP_MIDWAY_R_LEG_KNEE_PITCH;
    anglesMidwayDouble[19] = GLOBAL_GETUP_MIDWAY_R_LEG_ANKLE_PITCH;
    anglesMidwayDouble[20] = GLOBAL_GETUP_MIDWAY_R_LEG_ANKLE_ROLL;
    anglesMidwayDouble[21] = GLOBAL_GETUP_MIDWAY_HEAD_YAW;              // Head.
    anglesMidwayDouble[22] = GLOBAL_GETUP_MIDWAY_HEAD_PITCH;
    
    std::vector<float> anglesMidwayFloat(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        anglesMidwayFloat[i] = (float) anglesMidwayDouble[i];
    }
    
    // Generate transforms for hands and head relative to COM.
    std::vector<double> leftArmP(4);
    std::vector<double> rightArmP(4);
    std::vector<double> headP(2);
    
    Transform leftArmT;
    Transform rightArmT;
    Transform headT;
    
    leftArmP[0] = GLOBAL_L_ARM_SHOULDER_PITCH;
    leftArmP[1] = GLOBAL_L_ARM_SHOULDER_ROLL;
    leftArmP[2] = GLOBAL_L_ARM_ELBOW_YAW;
    leftArmP[3] = GLOBAL_L_ARM_ELBOW_ROLL;
    
    rightArmP[0] = GLOBAL_R_ARM_SHOULDER_PITCH;
    rightArmP[1] = GLOBAL_R_ARM_SHOULDER_ROLL;
    rightArmP[2] = GLOBAL_R_ARM_ELBOW_YAW;
    rightArmP[3] = GLOBAL_R_ARM_ELBOW_ROLL;
    
    headP[0] = GLOBAL_HEAD_YAW;
    headP[1] = GLOBAL_HEAD_PITCH;
    
    leftArmT  = ForwardArmL(leftArmP);
    rightArmT = ForwardArmR(rightArmP);
    headT     = ForwardHead(headP);
    
    // Generate transforms for both feet.  It should be noted here that the IK
    // library uses a right handed coordinate system with x forward and y to the left.
    // The step handler uses a right handed coordinate system with y forward, and x
    // to the right.
    Transform leftLegT;  // IK coordinate system.
    Transform rightLegT;
    
    leftLegT.translateX(-1000 * GLOBAL_COM_OFFSET_Y);
    leftLegT.translateY( 1000 * GLOBAL_STEP_WIDTH / 2 - 1000 * GLOBAL_COM_OFFSET_X);
    leftLegT.translateZ(-1000 * GLOBAL_COM_HEIGHT);
    
    rightLegT.translateX(-1000 * GLOBAL_COM_OFFSET_Y);
    rightLegT.translateY(-1000 * GLOBAL_STEP_WIDTH / 2 - 1000 * GLOBAL_COM_OFFSET_X);
    rightLegT.translateZ(-1000 * GLOBAL_COM_HEIGHT);
    
    Iterp interpolator(shmProxy);
    
    interpolator.linearIterp(anglesRestFloat, GLOBAL_REST_STIFFNESS, 1.0);
    interpolator.linearIterp(anglesMidwayFloat, GLOBAL_GETUP_MIDWAY_STIFFNESS, 1.0);
    interpolator.linearIterp(headT, leftArmT, rightArmT, leftLegT, rightLegT, GLOBAL_READY_STIFFNESS, 1.0);
}

void StepHandler::restStance(void)
{
    // Generate vector for angles at rest pose.
    std::vector<double> anglesRestDouble(GLOBAL_NUM_ACTUATORS);
    
    anglesRestDouble[0]  = GLOBAL_REST_L_ARM_SHOULDER_PITCH;  // Left arm.
    anglesRestDouble[1]  = GLOBAL_REST_L_ARM_SHOULDER_ROLL;
    anglesRestDouble[2]  = GLOBAL_REST_L_ARM_ELBOW_YAW;
    anglesRestDouble[3]  = GLOBAL_REST_L_ARM_ELBOW_ROLL;
    anglesRestDouble[4]  = GLOBAL_REST_L_ARM_WRIST_YAW;
    anglesRestDouble[5]  = GLOBAL_REST_R_ARM_SHOULDER_PITCH;  // Right arm.
    anglesRestDouble[6]  = GLOBAL_REST_R_ARM_SHOULDER_ROLL;
    anglesRestDouble[7]  = GLOBAL_REST_R_ARM_ELBOW_YAW;
    anglesRestDouble[8]  = GLOBAL_REST_R_ARM_ELBOW_ROLL;
    anglesRestDouble[9]  = GLOBAL_REST_R_ARM_WRIST_YAW;
    anglesRestDouble[10] = GLOBAL_REST_L_LEG_HIP_YAW_PITCH;   // Left leg.
    anglesRestDouble[11] = GLOBAL_REST_L_LEG_HIP_ROLL;
    anglesRestDouble[12] = GLOBAL_REST_L_LEG_HIP_PITCH;
    anglesRestDouble[13] = GLOBAL_REST_L_LEG_KNEE_PITCH;
    anglesRestDouble[14] = GLOBAL_REST_L_LEG_ANKLE_PITCH;
    anglesRestDouble[15] = GLOBAL_REST_L_LEG_ANKLE_ROLL;
    anglesRestDouble[16] = GLOBAL_REST_R_LEG_HIP_ROLL;        // Right leg.
    anglesRestDouble[17] = GLOBAL_REST_R_LEG_HIP_PITCH;
    anglesRestDouble[18] = GLOBAL_REST_R_LEG_KNEE_PITCH;
    anglesRestDouble[19] = GLOBAL_REST_R_LEG_ANKLE_PITCH;
    anglesRestDouble[20] = GLOBAL_REST_R_LEG_ANKLE_ROLL;
    anglesRestDouble[21] = GLOBAL_REST_HEAD_YAW;              // Head.
    anglesRestDouble[22] = GLOBAL_REST_HEAD_PITCH;
    
    std::vector<float> anglesRestFloat(GLOBAL_NUM_ACTUATORS);
    
    for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
    {
        anglesRestFloat[i] = (float) anglesRestDouble[i];
    }
    
    Iterp interpolator(shmProxy);
    
    interpolator.linearIterp(anglesRestFloat, GLOBAL_REST_STIFFNESS, 1.5);
}

/** @brief   Creates and adds a step to the step vector for later execution.
 *
 *  @param   argFoot        Enum indicating left or right foot.
 *  @param   argTransform   Transform representing orientation of foot.
 *  @param   argStepType    Enum indicating whether this is the first, mid, or last step in a sequence.
 */

/*
void StepHandler::addStep(Point::Foot argFoot, Transform& argTransform, Point::StepType argStepType)
{
    // Instantiate new point and add to steps vector.
    Point newPoint (argFoot, argStepType);
    newPoint.setFootPos(argTransform);
    stepsBuffer.push_back(newPoint);
}
 */

/** @brief   Initializes motion by setting appropriate fields and beginning loop in
 *           a detached thread.
 */

/*
void StepHandler::initializeMotion(void)
{
    // Start loop thread.
    pauseLoop = false;
    stopLoop = false;
    
    loopThread = boost::thread(&StepHandler::loop, this);
}
*/

/** @brief   StepHandler loop that will run in an independent thread.  This loop processes footsteps.
 */

/*
void StepHandler::loop(void)
{
    loopRunning = true;
    
    while (true)
    {
        try
        {
            if (stopLoop)
            {
                loopRunning = false;
                break;
            }
            
            if (pauseLoop)
            {
                loopPaused = true;
                usleep(200);
                continue;
            }
            
            loopPaused = false;
            
            // If no current steps exist, this is the first step.
            if (currentStepBuffer.size() == 0)
            {
                // The robot starts with both feet stationary and horizontally in-line.  The first step
                // passed to the locomotion system is the first step for the right root, where the foot
                // undergoes a physical displacemnet.  However, an implicit left-step occurs before this
                // step, where the left foot doesn't move, but the robot's COM and swing foot begin moving.
                
                // Generate this implicit step.
                generateImplicitStep();
                
                // Generate COM and swing foot trajectories for implicit step.
                generateImplicitStepTrajectories(currentStepBuffer.front());
            }
            
            // Create and launch two threads, one to handle robot motion, the other to perform the calculations.
            boost::thread computationThread = boost::thread(&StepHandler::computeMotion, this);
            boost::thread executionThread   = boost::thread(&StepHandler::executeMotion, this);
            
            // Join the threads with the main thread.
            computationThread.join();
            executionThread.join();
            
            // After completion of both threads, update the step buffers.
            if (!stepsBuffer.empty())
            {
                currentStepBuffer[0] = stepsBuffer.front();
                stepsBuffer.erase(stepsBuffer.begin());
            }
        }
        catch (std::exception& e)
        {
            loopRunning = false;
            break;
        }
    }
}
*/

/** @brief   Method to handle loading locomotion parameters as variables at runtime.
 */

void StepHandler::loadLocomotionParameters(void)
{
    std::ifstream myfile ("LocomotionParameters.txt");
    std::string line;
    
    for (int i = 0; i < NumLocomotionVariables; i++)
    {
        getline(myfile, line);
                
        int startLocusVariableName  = line.find("$") + 1;
        int endLocusVariableName    = line.find("$$");
        int startLocusVariableValue = line.find("$$$") + 3;
        int endLocusVariableValue   = line.find("$$$$");
        
        std::string variableName(line.substr(startLocusVariableName, endLocusVariableName - startLocusVariableName));
        std::string variableValue(line.substr(startLocusVariableValue, endLocusVariableValue - startLocusVariableValue));
        
        if (i == GLOBAL_STEP_WIDTH_ENUM)
        {
            GLOBAL_STEP_WIDTH = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_STEP_LENGTH_ENUM)
        {
            GLOBAL_STEP_LENGTH = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_STEP_HEIGHT_ENUM)
        {
            GLOBAL_STEP_HEIGHT = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_COM_HEIGHT_ENUM)
        {
            GLOBAL_COM_HEIGHT = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_COM_OFFSET_X_ENUM)
        {
            GLOBAL_COM_OFFSET_X = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_COM_OFFSET_Y_ENUM)
        {
            GLOBAL_COM_OFFSET_Y = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_G_ENUM)
        {
            GLOBAL_G = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_Y0POS_ENUM)
        {
            GLOBAL_Y0POS = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_Y0VEL_ENUM)
        {
            GLOBAL_Y0VEL = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_X0POS_ENUM)
        {
            GLOBAL_X0POS = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_X0VEL_ENUM)
        {
            GLOBAL_X0VEL = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_FIRST_SWING_LIFT_START_PHASE_ENUM)
        {
            GLOBAL_FIRST_SWING_LIFT_START_PHASE = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_FIRST_SWING_LIFT_END_PHASE_ENUM)
        {
            GLOBAL_FIRST_SWING_LIFT_END_PHASE = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_SWING_LIFT_START_PHASE_ENUM)
        {
            GLOBAL_SWING_LIFT_START_PHASE = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_SWING_LIFT_END_PHASE_ENUM)
        {
            GLOBAL_SWING_LIFT_END_PHASE = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_STEP_FREQUENCY_ENUM)
        {
            GLOBAL_STEP_FREQUENCY = boost::lexical_cast<double>(variableValue);
        }
        else if (i == GLOBAL_RWEIGHT_ENUM)
        {
            GLOBAL_RWEIGHT = boost::lexical_cast<double>(variableValue);
        }
    }
    
    myfile.close();
}

/** @brief   Method to handle computation of trajectories for next footstep.
 */

void StepHandler::computeMotion(void)
{
    // If there are not steps for computation, return immediately.
    if (stepsBuffer.size() <= 1)
    {
        pauseLoop = true;
        return;
    }
    
    Point& currentStep = currentStepBuffer[0];
    Point& nextStep = stepsBuffer[0];
    Point& nextNextStep = stepsBuffer[1];

    // Store LIMP parameters.  If nextStep is the first step, then step parameters are
    // the global parameters.  Also check if nextStep is specified as the final step.
    if (nextStep.getStepType() == Point::begin)
    {
        COMContainer stepParameters;
        
        stepParameters.setX0Pos(-1 * GLOBAL_X0POS);
        stepParameters.setX0Vel(GLOBAL_X0VEL);
        stepParameters.setY0Pos(GLOBAL_Y0POS);
        stepParameters.setY0Vel(GLOBAL_Y0VEL);
        stepParameters.setR(0.0);
        
        nextStep.setTrajectoryParameters(stepParameters);
        
        // Compute TEnd for the next step, and TStart for the nextNextStep.
        computeTransitionTime(nextStep, nextNextStep);
    }
    else if (nextStep.getStepType() == Point::end)
    {
        return; // dummy return for testing purposes.
    }
    else
    {
        COMContainer stepParameters;
        
        // Set parameters that we know already.
        if (nextStep.getFoot() == Point::left)
        {
            stepParameters.setX0Pos(GLOBAL_X0POS);
        }
        else if (nextStep.getFoot() == Point::right)
        {
            stepParameters.setX0Pos(-1 * GLOBAL_X0POS);
        }
        
        stepParameters.setX0Vel(GLOBAL_X0VEL);
        stepParameters.setRWeight(GLOBAL_RWEIGHT);
        
        nextStep.setTrajectoryParameters(stepParameters);
        
        // Compute TEnd for the next step, and TStart for the nextNextStep.
        computeTransitionTime(nextStep, nextNextStep);
        
        // y0Pos, y0Vel, and r still need to be set.
        optimizePendulumParameters(currentStep, nextStep);
    }
    
    // Generate COM and swing foot trajectory.
    generateGenericStepTrajectories(currentStep, nextStep, nextNextStep);
}

/** @brief   Method to handle execution of robot motion for one footstep.
 */

/*
void StepHandler::executeMotion(void)
{
    // Iterate through all trajectories of current footstep.
    for (int i = 0; i < currentStepBuffer.front().getNumTrajectories(); i++)
    {
        // Extract trajectories.
        Trajectory stepTrajectory(currentStepBuffer.front().getTrajectory(i));
        
        // Extract transforms for COM and swing foot.
        Transform foot2COMTransform(stepTrajectory.getCOMTransform());
        Transform foot2SwingTransform(stepTrajectory.getSwingFootTransform());
        
        // Change coordinate system, such that position/orientation of left and
        // right feet are represented relative to COM/torso.
        std::vector<double> foot2COMPos(position6D(foot2COMTransform));
        std::vector<double> foot2SwingPos(position6D(foot2SwingTransform));
        
        std::vector<double> COM2FootPos(6);
        std::vector<double> COM2SwingPos(6);
        
        for (int i = 0; i < foot2COMPos.size(); i++)
        {
            COM2FootPos[i]  = -1 * foot2COMPos[i];
        }
        
        for (int i = 0; i < foot2SwingPos.size(); i++)
        {
            COM2SwingPos[i] = COM2FootPos[i] + foot2SwingPos[i];
        }
        
        // Note that the coordinate system used for trajectory computation is different
        // from that used for inverse kinematics.  The IK library uses a right handed
        // coordinate system with x forward and y to the left.  In addition, units are in mm, not m.
        // Disregarded rotation for now, since this walk is currently uni-directional.
        double COM2FootPosX =      COM2FootPos[1];
        double COM2FootPosY = -1 * COM2FootPos[0];

        COM2FootPos[0] = 1000 * COM2FootPosX - 1000 * GLOBAL_COM_OFFSET_Y;
        COM2FootPos[1] = 1000 * COM2FootPosY - 1000 * GLOBAL_COM_OFFSET_X;
        COM2FootPos[2] = 1000 * COM2FootPos[2];
        
        double COM2SwingPosX = COM2SwingPos[1];
        double COM2SwingPosY = -1 * COM2SwingPos[0];
        
        COM2SwingPos[0] = 1000 * COM2SwingPosX - 1000 * GLOBAL_COM_OFFSET_Y;
        COM2SwingPos[1] = 1000 * COM2SwingPosY - 1000 * GLOBAL_COM_OFFSET_X;
        COM2SwingPos[2] = 1000 * COM2SwingPos[2];
        
        Transform COM2Foot(transform6D(COM2FootPos));
        Transform COM2Swing(transform6D(COM2SwingPos));
    
        // Compute leg angles.
        std::vector<double> legAngles;
        
        if (currentStepBuffer.front().getFoot() == Point::left)
        {
            legAngles = InvertLegs(COM2Foot, COM2Swing);
        }
        else
        {
            legAngles = InvertLegs(COM2Swing, COM2Foot);
        }
        
        std::vector<float> anglesDouble(GLOBAL_NUM_ACTUATORS);
        
        anglesDouble[L_ARM_SHOULDER_PITCH] = GLOBAL_L_ARM_SHOULDER_PITCH;
        anglesDouble[L_ARM_SHOULDER_ROLL]  = GLOBAL_L_ARM_SHOULDER_ROLL;
        anglesDouble[L_ARM_ELBOW_YAW]      = GLOBAL_L_ARM_ELBOW_YAW;
        anglesDouble[L_ARM_ELBOW_ROLL]     = GLOBAL_L_ARM_ELBOW_ROLL;
        anglesDouble[L_ARM_WRIST_YAW]      = 0.0;
        anglesDouble[R_ARM_SHOULDER_PITCH] = GLOBAL_R_ARM_SHOULDER_PITCH;
        anglesDouble[R_ARM_SHOULDER_ROLL]  = GLOBAL_R_ARM_SHOULDER_ROLL;
        anglesDouble[R_ARM_ELBOW_YAW]      = GLOBAL_R_ARM_ELBOW_YAW;
        anglesDouble[R_ARM_ELBOW_ROLL]     = GLOBAL_R_ARM_ELBOW_ROLL;
        anglesDouble[R_ARM_WRIST_YAW]      = 0.0;
        anglesDouble[L_LEG_HIP_YAW_PITCH]  = legAngles[0];
        anglesDouble[L_LEG_HIP_ROLL]       = legAngles[1];
        anglesDouble[L_LEG_HIP_PITCH]      = legAngles[2];
        anglesDouble[L_LEG_KNEE_PITCH]     = legAngles[3];
        anglesDouble[L_LEG_ANKLE_PITCH]    = legAngles[4];
        anglesDouble[L_LEG_ANKLE_ROLL]     = legAngles[5];
        anglesDouble[R_LEG_HIP_ROLL]       = legAngles[6];
        anglesDouble[R_LEG_HIP_PITCH]      = legAngles[7];
        anglesDouble[R_LEG_KNEE_PITCH]     = legAngles[8];
        anglesDouble[R_LEG_ANKLE_PITCH]    = legAngles[9];
        anglesDouble[R_LEG_ANKLE_ROLL]     = legAngles[10];
        anglesDouble[HEAD_YAW]             = GLOBAL_HEAD_YAW;
        anglesDouble[HEAD_PITCH]           = GLOBAL_HEAD_PITCH;

        std::vector<float> anglesFloat(GLOBAL_NUM_ACTUATORS);
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            anglesFloat[i] = (float) anglesDouble[i];
        }
        
        std::vector<float> stiffnessFloat(GLOBAL_NUM_ACTUATORS);
        
        for (int i = 0; i < GLOBAL_NUM_ACTUATORS; i++)
        {
            stiffnessFloat[i] = 1.0f;
        }
        
        while (true)
        {
            // Check if shared memory is ready for update and that we have exceeded the DCM time step.
            if (shmProxy->call<int>("getFlag") == 1)
            {
                // Update position and stiffness for all actuators.
                shmProxy->call<void>("setJointPosition",  anglesFloat);
                shmProxy->call<void>("setJointStiffness", stiffnessFloat);
                
                shmProxy->call<void>("setFlag", 0);
                
                break;
            }
            else
            {
                // Sleep for a period to prevent CPU overusage.
                usleep(200); // 0.5 ms sleep.
            }
        }
    }
}
*/

                            /******************************************/
                            /*** Pendulum Trajectory Helper Methods ***/
                            /******************************************/

/** @brief   Generates an implicit first step for the left foot.
 */

void StepHandler::generateImplicitStep(void)
{
    Transform leftFootPos;
    
    leftFootPos.translate(-1.0 * GLOBAL_STEP_WIDTH / 2.0, 0.0, 0.0);
    
    Point leftFootPoint(Point::left, Point::begin);
    leftFootPoint.setFootPos(leftFootPos);
    
    currentStepBuffer.push_back(leftFootPoint);
}

/** @brief   Generates COM and swing foot trajectories for the implicit.
 *
 *  @param   implicitStep   The implicit step to generate com and swing foot trajectories for.
 *                          Note that the position of this first implicit step will be a global position.
 *                          All further step positions are relative to the immediately previous footstep.
 */

void StepHandler::generateImplicitStepTrajectories(Point& implicitStep)
{
    // Compute the start time of the first footstep.
    computeStartTimeFirstStep(stepsBuffer[0]);
    
    // Temporary storage for COM and swing foot trajectories.
    std::vector<Transform> comVector;
    std::vector<Transform> swingVector;
    
    // Relevant pendulum parameters.
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    
    // Generate Bezier Curve
    // The bezier curve is characterized by three points (1, 2, 3), positioned locally relative to the
    // foot origin (0,0).
    
    Point firstStep = stepsBuffer[0];
    std::vector<double> firstStepPos = position6D(firstStep.getFootPos());
    double thetaR = firstStepPos[5] * M_PI / 180;
    
    COMContainer firstStepParams(firstStep.getTrajectoryParameters());
    double phaseStartTime = firstStepParams.getTStart();
    
    double bezX1 = GLOBAL_STEP_WIDTH / 2;
    double bezY1 = 0;
    double bezX2;
    double bezY2 = 0;
    double bezX3 = GLOBAL_STEP_WIDTH / 2;
    
    double firstPointFirstStepCoordX = -1 * GLOBAL_X0POS * cosh(k * phaseStartTime) + (GLOBAL_X0VEL / k) * sinh(k * phaseStartTime);
    double firstPointFirstStepCoordY =      GLOBAL_Y0POS * cosh(k * phaseStartTime) + (GLOBAL_Y0VEL / k) * sinh(k * phaseStartTime);

    double bezY3 = firstStepPos[1] + sin(-1 * thetaR) * firstPointFirstStepCoordX + cos(-1 * thetaR) * firstPointFirstStepCoordY;
    
    // Slope of first pendulum phase trajectory at intersection with bezier curve.
    // Note that trajectory must be converted to coordinate system of implicit step.
    double deltaX = (k * GLOBAL_X0POS * sinh(k * phaseStartTime));
    double deltaY = (k * GLOBAL_Y0POS * sinh(k * phaseStartTime) + GLOBAL_Y0VEL * cosh(k * phaseStartTime));
    
    // Convert deltaX and deltaY to coordinate system of implicit step.
    deltaX = cos(-1 * thetaR) * deltaX - sin(-1 * thetaR) * deltaX;
    deltaY = sin(-1 * thetaR) * deltaY + cos(-1 * thetaR) * deltaY;
    double pendulumSlope  = fabs(deltaY / deltaX);
    
    bezX2 = (bezY2 - bezY3 + pendulumSlope * bezX3) / pendulumSlope;
    
    // Determine the speed of COM at the beginning of the first pendulum phase.
    // Shouldn't need to convert to implicit coordinate system here -> result should be the same.
    double xVel = GLOBAL_X0POS * k * sinh(k * phaseStartTime) + GLOBAL_X0VEL * cosh(k * phaseStartTime);
    double yVel = GLOBAL_Y0POS * k * sinh(k * phaseStartTime) + GLOBAL_Y0VEL * cosh(k * phaseStartTime);
    double speed = sqrt(xVel * xVel + yVel * yVel);
    
    // Determine the total length of the bezier curve.
    std::vector<double> fdxdt(2);
    fdxdt[0] = 2 * (bezX1 - 2 * bezX2 + bezX3);
    fdxdt[1] = 2 * (bezX2 - bezX1);
    
    std::vector<double> fdydt(2);
    fdydt[0] = 2 * (bezY1 - 2 * bezY2 + bezY3);
    fdydt[1] = 2 * (bezY2 - bezY1);
    
    double L = integratePathLength(0.0, 1.0, fdxdt, fdydt);
    
    // Determine the ending time of the bezier curve (real time).
    // bezTime, introduced later, is just a parameter to pass into the Bezier equation - does not signify real time.
    double endTime = 2 * L / speed;
    
    double swingStartTime = GLOBAL_FIRST_SWING_LIFT_START_PHASE * endTime;
    double dxdtSwingStart = GLOBAL_FIRST_SWING_LIFT_END_PHASE * endTime;
    
    // Generate swing foot trajectory
    std::vector<double> coefficientsX = computeSwingCoefficients(GLOBAL_STEP_WIDTH, (firstStepPos[0] + GLOBAL_STEP_WIDTH) / 2, firstStepPos[0],
                                                                 swingStartTime, swingStartTime + (endTime - swingStartTime) / 2, endTime);
    std::vector<double> coefficientsY = computeSwingCoefficients(0, firstStepPos[1] / 2, firstStepPos[1],
                                                                 swingStartTime, swingStartTime + (endTime - swingStartTime) / 2, endTime);
    std::vector<double> coefficientsZ = computeSwingCoefficients(0, GLOBAL_STEP_HEIGHT, 0,
                                                                 swingStartTime, swingStartTime + (endTime - swingStartTime) / 2, endTime);
    
    // Determine incremental rotation of swing foot.
    double totalRotation     = firstStepPos[5];
    double rotationIncrement = totalRotation / (endTime - swingStartTime);
    
    // Generate the swing foot trajectory points as well as the actual points on the bezier curve.
    for (double t = 0; t <= endTime; t += GLOBAL_DCM_TIME_STEP * GLOBAL_STEP_FREQUENCY)
    {
        // Swing foot trajectory.
        double rotateZ;
        double x;
        double y;
        double z;
        
        if (t < swingStartTime)
        {
            rotateZ = 0.0;
            x = GLOBAL_STEP_WIDTH;
            y = 0.0;
            z = 0.0;
        }
        else
        {
            rotateZ = (endTime - swingStartTime) * rotationIncrement;
            
            if (t <= swingStartTime + (endTime - swingStartTime) / 2)
            {
                x = coefficientsX[0] + coefficientsX[1] * t + coefficientsX[2] * pow(t, 2) + coefficientsX[3] * pow(t, 3);
                y = coefficientsY[0] + coefficientsY[1] * t + coefficientsY[2] * pow(t, 2) + coefficientsY[3] * pow(t, 3);
                z = coefficientsZ[0] + coefficientsZ[1] * t + coefficientsZ[2] * pow(t, 2) + coefficientsZ[3] * pow(t, 3);
            }
            else
            {
                x = coefficientsX[4] + coefficientsX[5] * t + coefficientsX[6] * pow(t, 2) + coefficientsX[7] * pow(t, 3);
                y = coefficientsY[4] + coefficientsY[5] * t + coefficientsY[6] * pow(t, 2) + coefficientsY[7] * pow(t, 3);
                z = coefficientsZ[4] + coefficientsZ[5] * t + coefficientsZ[6] * pow(t, 2) + coefficientsZ[7] * pow(t, 3);
            }
        }
        
        Transform footPos;
        footPos.rotateZ(rotateZ);
        footPos.translate(x, y, z);
        
        swingVector.push_back(footPos);
        
        // Bezier curve trajectory.
        // Current arc length.
        double s = (speed / endTime) * (t * t) / 2;
    
        double bezTime = bezierTimeWrapper(s, L, fdxdt, fdydt);
        
        double newXPoint = (bezX3 - 2 * bezX2 + bezX1) * bezTime * bezTime + (2 * bezX2 - 2 * bezX1) * bezTime + bezX1;
        double newYPoint = (bezY3 - 2 * bezY2 + bezY1) * bezTime * bezTime + (2 * bezY2 - 2 * bezY1) * bezTime + bezY1;
        
        Transform comPos;
        comPos.rotateZ(rotateZ / 2);
        comPos.translateX(newXPoint);
        comPos.translateY(newYPoint);
        comPos.translateZ(GLOBAL_COM_HEIGHT);
        comVector.push_back(comPos);
    }
    
    // Generate trajectories from COM and swing foot position vectors.
    // Note that for now, the time stamp is set to zero, since I'm not sure how to incorporate
    // that aspect yet.
    for (int i = 0; i < comVector.size(); i++)
    {
        Trajectory newTrajectory;
        newTrajectory.setTime(0.0);
        newTrajectory.setCOMTransform(comVector[i]);
        newTrajectory.setSwingFootTransform(swingVector[i]);
        
        implicitStep.addTrajectory(newTrajectory);
    }
}

/** @brief   Generates COM and swing foot trajectories for a generic pendulum phase.
 *
 *  @param   currentStep        The step whose motion is being executed.
 *  @param   nextStep           The step whose trajectories we are computing.
 *  @param   nextNextStep       The step after nextStep.
 */

void StepHandler::generateGenericStepTrajectories(Point& currentStep, Point& nextStep, Point& nextNextStep)
{
    // Temporary storage for COM and swing foot trajectories.
    std::vector<Transform> comVector;
    std::vector<Transform> swingVector;

    std::vector<double> nextStepPos     = position6D(nextStep.getFootPos()); // Note that this is currently in the current step's frame.
    std::vector<double> nextNextStepPos = position6D(nextNextStep.getFootPos());
    
    double thetaR             = nextStepPos[5] * M_PI / 180;
    double nextStepNextFrameX = nextStepPos[0] * cos(thetaR) - nextStepPos[1] * sin(thetaR);
    double nextStepNextFrameY = nextStepPos[0] * sin(thetaR) + nextStepPos[1] * cos(thetaR);
    
    COMContainer stepParameters(nextStep.getTrajectoryParameters());
    
    // Generate COM and swing foot trajectory points.
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    
    // Swing foot trajectory prep.
    double liftStart = stepParameters.getTStart() + GLOBAL_SWING_LIFT_START_PHASE * (stepParameters.getTEnd() - stepParameters.getTStart());
    double liftEnd   = stepParameters.getTStart() + GLOBAL_SWING_LIFT_END_PHASE   * (stepParameters.getTEnd() - stepParameters.getTStart());
    
    std::vector<double> coefficientsX = computeSwingCoefficients(-1 * nextStepNextFrameX, (-1 * nextStepNextFrameX + nextNextStepPos[0]) / 2, nextNextStepPos[0],
                                                                 liftStart, liftStart + (liftEnd - liftStart) / 2, liftEnd);
    std::vector<double> coefficientsY = computeSwingCoefficients(-1 * nextStepNextFrameY, -1 * nextStepNextFrameY + (nextNextStepPos[1] + nextStepNextFrameY) / 2, nextNextStepPos[1],
                                                                 liftStart, liftStart + (liftEnd - liftStart) / 2, liftEnd);
    std::vector<double> coefficientsZ = computeSwingCoefficients(0, GLOBAL_STEP_HEIGHT, 0,
                                                                 liftStart, liftStart + (liftEnd - liftStart) / 2, liftEnd);
    
    // Determine incremental rotation of swing foot.
    double totalRotation     = nextStepPos[5] + nextNextStepPos[5];
    double rotationIncrement = totalRotation / (liftEnd - liftStart);
    
    for (double t = stepParameters.getTStart() + GLOBAL_DCM_TIME_STEP; t <= stepParameters.getTEnd(); t += GLOBAL_DCM_TIME_STEP * GLOBAL_STEP_FREQUENCY)
    {
        // Swing foot points.
        double rotateZ;
        double x;
        double y;
        double z;
        
        if (t < liftStart)
        {
            rotateZ = 0.0;
            x = -1 * nextStepNextFrameX;
            y = -1 * nextStepNextFrameY;
            z = 0;
        }
        else if (t > liftEnd)
        {
            rotateZ = totalRotation;
            x = nextNextStepPos[0];
            y = nextNextStepPos[1];
            z = 0;
        }
        else
        {
            rotateZ = (t - liftStart) * rotationIncrement;
            
            if (t <= liftStart + (liftEnd - liftStart) / 2)
            {
                x = coefficientsX[0] + coefficientsX[1] * t + coefficientsX[2] * pow(t, 2) + coefficientsX[3] * pow(t, 3);
                y = coefficientsY[0] + coefficientsY[1] * t + coefficientsY[2] * pow(t, 2) + coefficientsY[3] * pow(t, 3);
                z = coefficientsZ[0] + coefficientsZ[1] * t + coefficientsZ[2] * pow(t, 2) + coefficientsZ[3] * pow(t, 3);
                
            }
            else
            {
                x = coefficientsX[4] + coefficientsX[5] * t + coefficientsX[6] * pow(t, 2) + coefficientsX[7] * pow(t, 3);
                y = coefficientsY[4] + coefficientsY[5] * t + coefficientsY[6] * pow(t, 2) + coefficientsY[7] * pow(t, 3);
                z = coefficientsZ[4] + coefficientsZ[5] * t + coefficientsZ[6] * pow(t, 2) + coefficientsZ[7] * pow(t, 3);
            }
        }
        
        Transform footPos;
        footPos.rotateZ(rotateZ);
        footPos.translate(x, y, z);
        
        swingVector.push_back(footPos);
        
        // COM points.
        double xPos;
        double yPos;
        
        xPos = stepParameters.getX0Pos() * cosh(k * t) + (stepParameters.getX0Vel() / k) * sinh(k * t);
        yPos = stepParameters.getY0Pos() * cosh(k * t) + (stepParameters.getY0Vel() / k) * sinh(k * t) + stepParameters.getR();
                
        Transform comPos;
        comPos.rotateZ(rotateZ / 2);
        comPos.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comVector.push_back(comPos);
    }
     
    // Generate trajectories from COM and swing foot position vectors.
    // Note that for now, the time stamp is set to zero, since it is not current used.
    // May be prudent to remove this feature in the future.
    for (int i = 0; i < comVector.size(); i++)
    {
        Trajectory newTrajectory;
        newTrajectory.setTime(0.0);
        newTrajectory.setCOMTransform(comVector[i]);
        newTrajectory.setSwingFootTransform(swingVector[i]);
        
        nextStep.addTrajectory(newTrajectory);
    }
}

/** @brief   Optimizes pendulum parameters, specifically y0Pos and r, which correspond
 *           to the symmetry of a pendulum phase as well as the offset of the t0 position
 *           from the foot origin.
 *
 *  @param   currentStep   The step that we are calculating trajectories for.
 *  @param   nextStep      The next step in the sequence of steps.
 */

void StepHandler::optimizePendulumParameters(Point& currentStep, Point& nextStep)
{
    COMContainer currentStepParameters = currentStep.getTrajectoryParameters();
    COMContainer nextStepParameters = nextStep.getTrajectoryParameters();
    
    // Computation constants.
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    
    // Computation parameters.
    double x0Pos      = currentStepParameters.getX0Pos();
    double y0Pos      = currentStepParameters.getY0Pos();
    double y0Vel      = currentStepParameters.getY0Vel();
    double endTime    = currentStepParameters.getTEnd();
    double r          = currentStepParameters.getR();
    
    double x0PosBar     = nextStepParameters.getX0Pos();
    double beginTimeBar = nextStepParameters.getTStart();
    double rWeight      = nextStepParameters.getRWeight();
    double y0PosWeight  = 1 - rWeight;

    Transform currentFootTransform = currentStep.getFootPos();
    Transform nextFootTransform    = nextStep.getFootPos();
    
    std::vector<double> currentFootPos = position6D(currentFootTransform);
    std::vector<double> nextFootPos    = position6D(nextFootTransform);
    double thetaRNext                  = nextFootPos[5] * M_PI / 180;
    
    double s          = nextFootPos[1]; // y displacement between current and next footstep.
    double sNextFrame = nextFootPos[0] * sin(thetaRNext) + nextFootPos[1] * cos(thetaRNext);
    
    double rNextFrame = r * cos(thetaRNext);
    
    // Variables to solve for.
    double y0PosBar = -0.2;
    double y0VelBar;
    double rBar;
    
    // Newton parameters.
    double f;                               // Newton root function (f = y0PosWeight * y0PosBar - rWeight * rBar).
    double fPrime;                          // First derivative of Newton root function.
    double rBarPrime;
    double oldY0PosBar;
    
    // Minimize y0Pos and r according to y0PosWeight and rWeight through implementation of Newton's method.
    // x1 = x0 - f(x0) / f'(x0)
    do
    {
        // Compute f.
        // Solve for y0VelBar via velocity matching along y axis of next footstep.
        y0VelBar = ( x0Pos * k * sinh(k * endTime) * sin(thetaRNext) +
                    (y0Pos * k * sinh(k * endTime) + y0Vel * cosh(k * endTime)) * cos(thetaRNext) -
                     y0PosBar * k * sinh(k * beginTimeBar)) /
                   cosh(k * beginTimeBar);
        rBar     = x0Pos * cosh(k * endTime) * sin(thetaRNext) + (y0Pos * cosh(k * endTime) + (y0Vel / k) * sinh(k * endTime)) * cos(thetaRNext) -
                   y0PosBar * cosh(k * beginTimeBar) - (y0VelBar / k) * sinh(k * beginTimeBar) - sNextFrame + rNextFrame;
        f        = y0PosWeight * y0PosBar + rWeight * rBar; // should be absolute value but I don't feel like fixing this right now...
        
        // Compute fPrime (with respect to y0PosBar).
        rBarPrime = -1 * cosh(k * beginTimeBar) + pow(sinh(k * beginTimeBar), 2) / cosh(k * beginTimeBar);
        fPrime = y0PosWeight + rWeight * rBarPrime;

        // Update y0Pos value.
        oldY0PosBar = y0PosBar;
        y0PosBar = y0PosBar - f / fPrime;
    } while(fabs(y0PosBar - oldY0PosBar) > 0.001);
    
    // Perform last minute updates to beginTimeBar in order to improve position/velocity matching due to
    // approximations in implementing omnidirectional gait.
    // First iteratively adjust beginTimeBar such that there is no overlap.
    double xPosTEndCurrentFrame = x0Pos * cosh(k * endTime);
    double yPosTEndCurrentFrame = y0Pos * cosh(k * endTime) + (y0Vel / k) * sinh(k * endTime);
    
    double xPosTEndNextFrame = -1 * nextFootPos[0] + xPosTEndCurrentFrame * cos(thetaRNext) - yPosTEndCurrentFrame * sin(thetaRNext);
    double yPosTEndNextFrame = -1 * nextFootPos[1] + xPosTEndCurrentFrame * sin(thetaRNext) + yPosTEndCurrentFrame * cos(thetaRNext);

    std::cout << "xPosTEndNextFrame " << xPosTEndNextFrame << std::endl;
    std::cout << "yPosTEndNextFrame " << yPosTEndNextFrame << std::endl;

    
    double xVelTEndCurrentFrame = x0Pos * k * sinh(k * endTime);
    double yVelTEndCurrentFrame = y0Pos * k * sinh(k * endTime) + y0Vel * cosh(k * endTime);
    
    double xVelTEndNextFrame = xVelTEndCurrentFrame * cos(thetaRNext) - yVelTEndCurrentFrame * sin(thetaRNext);
    double yVelTEndNextFrame = xVelTEndCurrentFrame * sin(thetaRNext) + yVelTEndCurrentFrame * cos(thetaRNext);
    
    // Determine angle between x axis of new step frame and a line tangent to COM curve at transition.
    double angleToTempFrame = atan(fabs(yVelTEndNextFrame / xVelTEndNextFrame));
    
    if (xVelTEndNextFrame > 0)
    {
        angleToTempFrame *= -1;
    }
    
    double xPosTEndTangentFrame = xPosTEndNextFrame * cos(angleToTempFrame) - yPosTEndNextFrame * sin(angleToTempFrame);

    std::cout << "xPosTEnd Tangent frame : " << xPosTEndTangentFrame << std::endl;
    
    double xPosBarTBegin = x0PosBar * cosh(k * beginTimeBar);
    double yPosBarTBegin = y0PosBar * cosh(k * beginTimeBar) + (y0VelBar / k) * sinh(k * beginTimeBar);
    
    double xPosBarTBeginTangentFrame = xPosBarTBegin * cos(angleToTempFrame) - yPosBarTBegin * sin(angleToTempFrame);
    
    std::cout << "x pos bar t begin : " << xPosBarTBeginTangentFrame << std::endl;
    
    
    
    
    
    
    
    
    
    
    
    /*
    while (fabs(xPosBarTBeginTangentFrame) > fabs(xPosTEndTangentFrame))
    {
        beginTimeBar += 0.010;
        
        xPosBarTBegin = x0PosBar * cosh(k * beginTimeBar);
        yPosBarTBegin = y0PosBar * cosh(k * beginTimeBar) + (y0VelBar / k) * sinh(k * beginTimeBar);
        
        xPosBarTBeginTangentFrame = xPosBarTBegin * cos(angleToTempFrame) - yPosBarTBegin * sin(angleToTempFrame);
    }
    */
    
    
    
    
    
    
    
    // Adjust x0 position of next footstep based on x velocity at tEnd of current footstep.
    // Compute x velocity at tEnd of current footstep in next footstep frame.
    /*
    double xVelTEndCurrentFrame = x0Pos * k * sinh(k * endTime);
    double yVelTEndCurrentFrame = y0Pos * k * sinh(k * endTime) + y0Vel * cosh(k * endTime);
    
    double xVelTEndNextFrame = xVelTEndCurrentFrame * cos(thetaRNext) - yVelTEndCurrentFrame * sin(thetaRNext);
    double xDisp = xVelTEndNextFrame * GLOBAL_DCM_TIME_STEP;
    
    double xAdjustment = xPosTEndNextFrame + xDisp - xPosBarTBegin;
    
    // Adjust y0 position of next footstep based on y velocity at tEnd of current footstep.
    // Compute y velocity at tEnd of current footstep in next footstep frame.
    double yVelTEndNextFrame = xVelTEndCurrentFrame * sin(thetaRNext) + yVelTEndCurrentFrame * cos(thetaRNext);
    double yDisp = yVelTEndNextFrame * GLOBAL_DCM_TIME_STEP;
    
    double yPosBarTBegin = y0PosBar * cosh(k * beginTimeBar) + (y0VelBar / k) * sinh(k * beginTimeBar);

    double yAdjustment = yPosTEndNextFrame + yDisp - yPosBarTBegin;
    */
    
    // Update LIPM parameters for next step.
    nextStepParameters.setTStart(beginTimeBar);
    nextStepParameters.setY0Pos(oldY0PosBar);
    nextStepParameters.setY0Vel(y0VelBar);
    nextStepParameters.setR(rBar);
    
    nextStep.setTrajectoryParameters(nextStepParameters);
    
    return;
}

/** @brief   Computes the ending and beginning times for the current and next pendulum phases respectively,
 *           at the point of their intersection.
 */

void StepHandler::computeTransitionTime(Point& firstStep, Point& secondStep)
{
    // Extract pose of secondStep, relative to firstStep.
    Transform secondStepPose(secondStep.getFootPos());
    
    std::vector<double> secondStepPoseVect(position6D(secondStepPose));
    
    double thetaR = secondStepPoseVect[5] * M_PI / 180;
    
    // Relevant pendulum parameters.
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    double endTime = 0.2;   // End time of current pendulum phase.  Fill in an initial guess
                            // for correction later.
    double beginTime;       // Begin time of next pendulum phase.
    double deltaT;          // Incremental adjustment to guess for end time.
    
    do
    {
        // Compute beginning time of next phase.
        // Note that this computation is an approximation, disregarding any terms that rely on the Y0POS of the next
        // footstep.
        beginTime = (1 / k) * asinh(-1 * (GLOBAL_X0POS * k * sinh(k * endTime)) / (GLOBAL_X0POS * k * cos(-1 * thetaR)));
        
        // Compute the horizontal distance covered by the current and next pendulum phases at their
        // intersection (Tend and Tbegin respectively).
        // Note that this computation is an approximation, disregarding any terms that rely on the Y0POS of the next
        // footstep.
        double z = GLOBAL_X0POS * cosh(k * endTime) + GLOBAL_X0POS * cosh(k * beginTime) * cos(-1 * thetaR);
        
        // Horizontal velocity at intersection of two pendulum phases.
        double xEndVel = GLOBAL_X0POS * k * sinh(k * endTime);
        
        // Time correction.
        deltaT = (fabs(secondStepPoseVect[0]) - z) / (2 * xEndVel);
        
        // Adjust endTime.
        endTime += deltaT;
    } while(fabs(deltaT) > 0.0001);
    
    beginTime = (1 / k) * asinh(-1 * (GLOBAL_X0POS * k * sinh(k * endTime)) / (GLOBAL_X0POS * k * cos(-1 * thetaR)));

    COMContainer firstStepParams(firstStep.getTrajectoryParameters());
    COMContainer secondStepParams(secondStep.getTrajectoryParameters());
    
    firstStepParams.setTEnd(endTime);
    secondStepParams.setTStart(beginTime);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    secondStep.setTrajectoryParameters(secondStepParams);
}

/** @brief   Determine start time of first step pendulum phase, so that it begins at global x = 0 position.
 *
 *  @param   firstStep   The first step in the walk sequence.
 */

void StepHandler::computeStartTimeFirstStep(Point& firstStep)
{
    // Extract pose of firstStep, relative to implicitStep.
    Transform firstStepPose(firstStep.getFootPos());
    
    std::vector<double> firstStepPoseVect(position6D(firstStepPose));
    double thetaR = firstStepPoseVect[5] * M_PI / 180;
    
    // Relevant pendulum parameters.
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    
    // Solve quadratic equation to find u, where u = e^beginTime.
    double a = -1 * GLOBAL_X0POS * cos(-1 * thetaR) - (GLOBAL_Y0VEL / k) * sin(-1 * thetaR);
    double b =  2 * firstStepPoseVect[0] - GLOBAL_STEP_WIDTH;
    double c = -1 * GLOBAL_X0POS * cos(-1 * thetaR) + (GLOBAL_Y0VEL / k) * sin(-1 * thetaR);
    
    double u    = (-1 * b - sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);
    double ualt = (-1 * b + sqrt(pow(b, 2) - 4 * a * c)) / (2 * a);

    double beginTime    = log(u) / k;
    double beginTimeAlt = log(ualt) / k;
    
    if (beginTimeAlt < 0.0)
    {
        beginTime = beginTimeAlt;
    }
    
    COMContainer firstStepParams(firstStep.getTrajectoryParameters());
    firstStepParams.setTStart(beginTime);
    
    firstStep.setTrajectoryParameters(firstStepParams);
}

/** @brief   Determine the path length along a curve, given two functions that indicate the
 *           x and y speed at a given time.  The path length is determined through the use of a trapezoidal
 *           numerical integration technique.
 *
 *  @param   tMin   The start time for calculation.
 *  @param   tMax   The stop time of the calculation.
 *  @param   fdxdt  Coefficients to a function that returns the speed in the x direction along the curve at a given time.
 *  @param   fdydt  Coefficients to a function that returns the speed in the y direction along the curve at a given time.
 *
 *  @return  The path length between tMin and tMax.
 */

double StepHandler::integratePathLength(double tMin, double tMax,
                                        std::vector <double> fdxdt,
                                        std::vector <double> fdydt)
{
    // Specify the integration time-step.
    double dt = 0.001;
    
    double sum = 0.0;
    double t = tMin;
        
    // Perform trapezoidal integration method.
    for ( ; t < tMax; t += dt)
    {
        // X and Y velocity along Bezier curve at current time step.
        double dxdt = evaluateCoeff(fdxdt, t);
        double dydt = evaluateCoeff(fdydt, t);
        
        double magFun = sqrt(dxdt * dxdt + dydt * dydt);
        
        if ((t + dt) < tMax)
        {
            // X and Y velocity along Bezier curve at next time step.
            double dxdt2 = evaluateCoeff(fdxdt, (t + dt));
            double dydt2 = evaluateCoeff(fdydt, (t + dt));
            
            double magFun2 = sqrt(dxdt2 * dxdt2 + dydt2 * dydt2);
            
            sum += ((magFun + magFun2) / 2) * dt;
        }
        else
        {
            // X and Y velocity along Bezier curve at next time step.
            double dxdt2 = evaluateCoeff(fdxdt, tMax);
            double dydt2 = evaluateCoeff(fdydt, tMax);
            
            double magFun2 = sqrt(dxdt2 * dxdt2 + dydt2 * dydt2);

            sum += ((magFun + magFun2) / 2) * (tMax - t);
        }
    }
    
    return sum;
}

/** @brief   Determines the value to pass into the bezier path equation to achieve a specified arc length (s),
 *           out of the total path length (L).
 *
 *  @param   s      The arc length to achieve.
 *  @param   L      The total path length.
 *  @param   fdxdt  Coefficients of function that returns x velocity as a function of time.
 *  @param   fdydt  Coefficients of function that returns y velocity as a function of time.
 *
 *  @return  The parameter to pass into the bezier path equation to achieve the specified arc length (s).
 */

double StepHandler::bezierTimeWrapper(double s, double L,
                                      std::vector<double> fdxdt,
                                      std::vector<double> fdydt)
{
    // t0 = tmin + (s/L) * (tmax - tmin)
    double t0   = 0.0 + (s / L) * (1.0 - 0.0);
    double tOld = t0;
    double tNew = -1.0;

    // Use Newton's method to find time parameter value.
    double dxdt;
    double dydt;
    double fPrime;

    do
    {
        dxdt = evaluateCoeff(fdxdt, tOld);
        dydt = evaluateCoeff(fdydt, tOld);

        fPrime = sqrt(dxdt * dxdt + dydt * dydt);

        if (tNew != -1.0)
        {
            tOld = tNew;
        }

        tNew = tOld - (integratePathLength(0, tOld, fdxdt, fdydt) - s) / fPrime;
    } while (fabs(tNew - tOld) > 0.001);

    return tNew;
}
 
/** @brief   Evaluates a function with given coefficients at the given parameter.
 *
 *  @param   coeffVect      A vector with the function's coefficients.
 *  @param   param          The value to evaluate the function at.
 *
 *  @return  The evaluated functions equivalent value.
 */

double StepHandler::evaluateCoeff(std::vector <double> coeffVect, double param)
{
    int order = coeffVect.size() - 1;
    
    double output = 0.0;
    
    for (int i = order; i >= 0; i--)
    {        
        output += coeffVect[order - i] * pow(param, i);
    }

    return output;
}

                            /******************************************/
                            /** Swing Foot Trajectory Helper Methods **/
                            /******************************************/

/** @brief   Determines the coefficients to parameterize the swing foot.  The coefficients parameterize motion along an axis between 
 *           two footsteps, and are used in the following equation: x(t) = c0 + c1 * t + c2 * t^2 + c3 * t^3.  Note that 8
 *           coefficients are returned because this swing motion requires the foot to pass through three points, and a single
 *           equation characterizes the motion between two points.
 *
 *  @param   p1     The starting point.
 *  @param   p2     The midpoint.
 *  @param   p3     The ending point.
 *  @param   t1     The starting time.
 *  @param   t2     The midpoint time.
 *  @param   t3     The ending time.
 *
 *  @return  A vector containing the eight coefficients necessary to parameterize this motion.
 */

std::vector<double> StepHandler::computeSwingCoefficients(double p1, double p2, double p3, double t1, double t2, double t3)
{
    std::vector<double> coefficients;
    
    coefficients.push_back(-(2 * p1 * t1 * pow(t2, 4) + p2 * pow(t1, 4) * t2 - 2 * p1 * pow(t2, 4) * t3 + 2 * p2 * pow(t1, 4) * t3 - 3 * p3 * pow(t1, 4) * t2 -
                             3 * p1 * pow(t1, 2) * pow(t2, 3) + 2 * p1 * pow(t2, 3) * pow(t3, 2) - 2 * p2 * pow(t1, 3) * pow(t3, 2) - 3 * p3 * pow(t1, 2) * pow(t2, 3) +
                             6 * p3 * pow(t1, 3) * pow(t2, 2) + 4 * p1 * t1 * pow(t2, 3) * t3 - 4 * p2 * pow(t1, 3) * t2 * t3 - 6 * p1 * t1 * pow(t2, 2) * pow(t3, 2) +
                             3 * p1 * pow(t1, 2) * t2 * pow(t3, 2) + 3 * p2 * pow(t1, 2) * t2 * pow(t3, 2)) /
                            (2 * (t1 - t2) * (t1 - t3) * (t2 - t3) * (pow(t1, 2) - 2 * t1 * t2 + pow(t2, 2))));
    
    coefficients.push_back((3 * (p2 * pow(t1, 4) - p3 * pow(t1, 4) + 2 * p1 * t1 * pow(t2, 3) - 2 * p3 * t1 * pow(t2, 3) - 3 * p1 * pow(t1, 2) * pow(t2, 2) +
                            p1 * pow(t1, 2) * pow(t3, 2) - p2 * pow(t1, 2) * pow(t3, 2) + 3 * p3 * pow(t1, 2) * pow(t2, 2) - 2 * p1 * t1 * t2 * pow(t3, 2) +
                            2 * p1 * pow(t1, 2) * t2 * t3 + 2 * p2 * t1 * t2 * pow(t3, 2) - 2 * p2 * pow(t1, 2) * t2 * t3)) /
                           (2 * (t1 - t2) * (t1 - t3) * (t2 - t3) * (pow(t1, 2) - 2 * t1 * t2 + pow(t2, 2))));
    
    coefficients.push_back((3 * (2 * p3 * pow(t1, 3) - 2 * p2 * pow(t1, 3) - p1 * pow(t2, 3) + p3 * pow(t2, 3) + 2 * p1 * pow(t1, 2) * t2 - 2 * p1 * pow(t1, 2) * t3 +
                            p2 * pow(t1, 2) * t2 + p1 * t2 * pow(t3, 2) + 2 * p2 * pow(t1, 2) * t3 - 3 * p3 * pow(t1, 2) * t2 - p2 * t2 * pow(t3, 2))) /
                           (2 * (t1 - t2) * (t1 - t3) * (t2 - t3) * (pow(t1, 2) - 2 * t1 * t2 + pow(t2, 2))));
    
    coefficients.push_back(-(p1 * pow(t3, 2) - 3 * p2 * pow(t1, 2) - 3 * p1 * pow(t2, 2) + 3 * p3 * pow(t1, 2) - p2 * pow(t3, 2) + 3 * p3 * pow(t2, 2) + 4 * p1 * t1 * t2 -
                             4 * p1 * t1 * t3 + 2 * p2 * t1 * t2 + 2 * p1 * t2 * t3 + 4 * p2 * t1 * t3 - 6 * p3 * t1 * t2 - 2 * p2 * t2 * t3) /
                            (2 * (t1 - t2) * (t1 - t3) * (t2 - t3) * (pow(t1, 2) - 2 * t1 * t2 + pow(t2, 2))));

    coefficients.push_back((2 * p2 * t1 * pow(t3, 4) - 3 * p1 * t2 * pow(t3, 4) - 2 * p3 * t1 * pow(t2, 4) + p2 * t2 * pow(t3, 4) + 2 * p3 * pow(t2, 4) * t3 +
                            6 * p1 * pow(t2, 2) * pow(t3, 3) - 3 * p1 * pow(t2, 3) * pow(t3, 2) - 2 * p2 * pow(t1, 2) * pow(t3, 3) + 2 * p3 * pow(t1, 2) * pow(t2, 3) -
                            3 * p3 * pow(t2, 3) * pow(t3, 2) - 4 * p2 * t1 * t2 * pow(t3, 3) + 4 * p3 * t1 * pow(t2, 3) * t3 + 3 * p2 * pow(t1, 2) * t2 * pow(t3, 2) +
                            3 * p3 * pow(t1, 2) * t2 * pow(t3, 2) - 6 * p3 * pow(t1, 2) * pow(t2, 2) * t3) /
                           (2 * (t1 - t2) * (- pow(t2, 3) * t3 + t1 * pow(t2, 3) + 3 * pow(t2, 2) * pow(t3, 2) - 3 * t1 * pow(t2, 2) * t3 - 3 * t2 * pow(t3, 3) +
                            3 * t1 * t2 * pow(t3, 2) + pow(t3, 4) - t1 * pow(t3, 3))));
    
    coefficients.push_back((3 * (p1 * pow(t3, 4) - p2 * pow(t3, 4) + 2 * p1 * pow(t2, 3) * t3 - 2 * p3 * pow(t2, 3) * t3 - 3 * p1 * pow(t2, 2) * pow(t3, 2) +
                            p2 * pow(t1, 2) * pow(t3, 2) - p3 * pow(t1, 2) * pow(t3, 2) + 3 * p3 * pow(t2, 2) * pow(t3, 2) + 2 * p2 * t1 * t2 * pow(t3, 2) -
                            2 * p2 * pow(t1, 2) * t2 * t3 - 2 * p3 * t1 * t2 * pow(t3, 2) + 2 * p3 * pow(t1, 2) * t2 * t3)) /
                           (2 * (t1 - t2) * (- pow(t2, 3) * t3 + t1 * pow(t2, 3) + 3 * pow(t2, 2) * pow(t3, 2) - 3 * t1 * pow(t2, 2) * t3 - 3 * t2 * pow(t3, 3) +
                            3 * t1 * t2 * pow(t3, 2) + pow(t3, 4) - t1 * pow(t3, 3))));

    coefficients.push_back(-(3 * (p1 * pow(t2, 3) + 2 * p1 * pow(t3, 3) - 2 * p2 * pow(t3, 3) - p3 * pow(t2, 3) - p2 * pow(t1, 2) * t2 - 3 * p1 * t2 * pow(t3, 2) +
                             2 * p2 * t1 * pow(t3, 2) + p3 * pow(t1, 2) * t2 + p2 * t2 * pow(t3, 2) - 2 * p3 * t1 * pow(t3, 2) + 2 * p3 * t2 * pow(t3, 2))) /
                            (2 * (t1 - t2) * (- pow(t2, 3) * t3 + t1 * pow(t2, 3) + 3 * pow(t2, 2) * pow(t3, 2) - 3 * t1 * pow(t2, 2) * t3 - 3 * t2 * pow(t3, 3) +
                             3 * t1 * t2 * pow(t3, 2) + pow(t3, 4) - t1 * pow(t3, 3))));
    
    coefficients.push_back((3 * p1 * pow(t2, 2) - p2 * pow(t1, 2) + 3 * p1 * pow(t3, 2) + p3 * pow(t1, 2) - 3 * p2 * pow(t3, 2) - 3 * p3 * pow(t2, 2) - 2 * p2 * t1 * t2 -
                            6 * p1 * t2 * t3 + 4 * p2 * t1 * t3 + 2 * p3 * t1 * t2 + 2 * p2 * t2 * t3 - 4 * p3 * t1 * t3 + 4 * p3 * t2 * t3) /
                           (2 * (t1 - t2) * (- pow(t2, 3) * t3 + t1 * pow(t2, 3) + 3 * pow(t2, 2) * pow(t3, 2) - 3 * t1 * pow(t2, 2) * t3 - 3 * t2 * pow(t3, 3) +
                            3 * t1 * t2 * pow(t3, 2) + pow(t3, 4) - t1 * pow(t3, 3))));
    
    return coefficients;
}

                            /******************************************/
                            /************** Unit Testing **************/
                            /******************************************/

std::string StepHandler::runTests(void)
{
    std::string output("");
    
    /** evaluateCoeff **/
    // f = 0
    std::vector<double> coeffVect(1);
    
    coeffVect[0] = 0.0;
    
    double result = evaluateCoeff(coeffVect, 1.0);
    
    if (fabs(result - 0.0) < 0.001)
    {
        output.append("evaluateCoeff (f = 0.0) test passed!\n");
    }
    else
    {
        output.append("evaluateCoeff (f = 0.0) test FAILED!\n");
    }

    // f = 3
    coeffVect.clear();
    coeffVect.resize(1);
    
    coeffVect[0] = 3.0;
    
    result = evaluateCoeff(coeffVect, 1.0);
    
    if (fabs(result - 3.0) < 0.001)
    {
        output.append("evaluateCoeff (f = 3.0) test passed!\n");
    }
    else
    {
        output.append("evaluateCoeff (f = 3.0) test FAILED!\n");
    }
    
    // f = 3 * x + 1
    // x = 2
    coeffVect.clear();
    coeffVect.resize(2);
    
    coeffVect[0] = 3.0;
    coeffVect[1] = 1.0;
    
    result = evaluateCoeff(coeffVect, 2.0);
    
    if (fabs(result - 7.0) < 0.001)
    {
        output.append("evaluateCoeff (f = 3 * x + 1, x = 2) test passed!\n");
    }
    else
    {
        output.append("evaluateCoeff (f = 3 * x + 1, x = 2) test FAILED!\n");
    }

    // f = 4 * x^2 + 3 * x + 1
    // x = 2
    coeffVect.clear();
    coeffVect.resize(3);
    
    coeffVect[0] = 4.0;
    coeffVect[1] = 3.0;
    coeffVect[2] = 1.0;
    
    result = evaluateCoeff(coeffVect, 2.0);
    
    if (fabs(result - 23.0) < 0.001)
    {
        output.append("evaluateCoeff (f = 4 * x^2 + 3 * x + 1, x = 2) test passed!\n");
    }
    else
    {
        output.append("evaluateCoeff (f = 4 * x^2 + 3 * x + 1, x = 2) test FAILED!\n");
    }
    
    // f = 5 * x^3 - 4 * x^2 + 3 * x - 1
    // x = 2
    coeffVect.clear();
    coeffVect.resize(4);
    
    coeffVect[0] =  5.0;
    coeffVect[1] = -4.0;
    coeffVect[2] =  3.0;
    coeffVect[3] = -1.0;
    
    result = evaluateCoeff(coeffVect, 2.0);
    
    if (fabs(result - 29.0) < 0.001)
    {
        output.append("evaluateCoeff (f = 5 * x^3 - 4 * x^2 + 3 * x - 1, x = 2) test passed!\n");
    }
    else
    {
        output.append("evaluateCoeff (f = 5 * x^3 - 4 * x^2 + 3 * x - 1, x = 2) test FAILED!\n");
    }
    
    /** integratePathLength **/
    std::vector<double> coeffVectX;
    std::vector<double> coeffVectY;
    
    // Single axis paths.
    // vx = 0 / vy = 0 from t = 0 to x = 2
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(1);
    coeffVectY.resize(1);
    
    coeffVectX[0] = 0;
    
    coeffVectY[0] = 0;
    
    result = integratePathLength(0.0, 2.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 0.0) < 0.001)
    {
        output.append("integratePathLength (vx = 5 / vy = 2 from t = 0 -> 2) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 5 / vy = 2 from t = 0 -> 2) test FAILED!\n");
    }
    
    // vx = 2 / vy = 0 from t = 0 to x = 2
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(1);
    coeffVectY.resize(1);
    
    coeffVectX[0] = 2;
    
    coeffVectY[0] = 0;
    
    result = integratePathLength(0.0, 2.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 4.0) < 0.001)
    {
        output.append("integratePathLength (vx = 2 / vy = 0 from t = 0 -> 2) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 2 / vy = 0 from t = 0 -> 2) test FAILED!\n");
    }
    
    // vx = 0 / vy = 2 from t = 0 to t = 2
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(1);
    coeffVectY.resize(1);
    
    coeffVectX[0] = 0;
    
    coeffVectY[0] = 2;
    
    result = integratePathLength(0.0, 2.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 4.0) < 0.001)
    {
        output.append("integratePathLength (vx = 0 / vy = 2 from t = 0 -> 2) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 0 / vy = 2 from t = 0 -> 2) test FAILED!\n");
    }
    
    // vx = 2 * t / vy = 6 * t from t = 3 to t = 12
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(2);
    coeffVectY.resize(2);
    
    coeffVectX[0] = 2;
    coeffVectX[1] = 0;
    
    coeffVectY[0] = 6;
    coeffVectY[1] = 0;
    
    result = integratePathLength(3.0, 12.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 426.907) < 0.001)
    {
        output.append("integratePathLength (vx = 2 * t / vy = 6 * t from t = 3 -> 12) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 2 * t / vy = 6 * t from t = 3 -> 12) test FAILED!\n");
    }
    
    // vx = 2 * t - 2 / vy = 3 * t from t = 0.0 to t = 1.0
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(2);
    coeffVectY.resize(2);
    
    coeffVectX[0] =  2;
    coeffVectX[1] = -2;
    
    coeffVectY[0] = 3;
    coeffVectY[1] = 0;
    
    result = integratePathLength(0.0, 1.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 2.0450) < 0.001)
    {
        output.append("integratePathLength (vx = 2 * t - 2 / vy = 3 * t from t = 0 -> 1) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 2 * t - 2 / vy = 3 * t from t = 0 -> 1) test FAILED!\n");
    }
    
    // vx = 3 * t^2 - 2 * t / vy = 3 * t^3 from t = 0.0 to t = 1.0
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(3);
    coeffVectY.resize(4);
    
    coeffVectX[0] =  3;
    coeffVectX[1] = -2;
    coeffVectX[2] =  0;
    
    coeffVectY[0] = 3;
    coeffVectY[1] = 0;
    coeffVectY[2] = 0;
    coeffVectY[3] = 0;
    
    result = integratePathLength(0.0, 1.0, coeffVectX, coeffVectY);
    
    if (fabs(result - 0.8666) < 0.001)
    {
        output.append("integratePathLength (vx = 3 * t^2 - 2 * t / vy = 3 * t^3 from t = 0 -> 1) test passed!\n");
    }
    else
    {
        output.append("integratePathLength (vx = 3 * t^2 - 2 * t / vy = 3 * t^3 from t = 0 -> 1) test FAILED!\n");
    }

    /** bezierTimeWrapper **/
    coeffVectX.clear();
    coeffVectY.clear();
    
    coeffVectX.resize(2);
    coeffVectY.resize(2);
    
    coeffVectX[0] =  4;
    coeffVectX[1] = -2;
    
    coeffVectY[0] = 4;
    coeffVectY[1] = 0;
    
    double L = integratePathLength(0.0, 1.0, coeffVectX, coeffVectY);
    double s = L / 4;
    
    double tParam = bezierTimeWrapper(s, L, coeffVectX, coeffVectY);
    
    double computedPathLength = integratePathLength(0.0, tParam, coeffVectX, coeffVectY);
    
    if (fabs(computedPathLength - s) < 0.0001)
    {
        output.append("bezierTimeWrapper test one passed!\n");
    }
    else
    {
        output.append("bezierTimeWrapper test one FAILED!\n");
    }
    
    /** generateImplicitStep **/
    generateImplicitStep();
    
    Point copyOfLeftFootPoint(currentStepBuffer.front());
    Transform tempTransform(copyOfLeftFootPoint.getFootPos());
    
    if (copyOfLeftFootPoint.getFoot() == Point::left &&
        
        copyOfLeftFootPoint.getStepType() == Point::begin &&
        
        fabs(tempTransform(0,0) - 1.0)  < 0.001 && fabs(tempTransform(0,1) - 0.0) < 0.001 && fabs(tempTransform(0,2) - 0.0) < 0.001 && fabs(tempTransform(0,3) - (-1 * GLOBAL_STEP_WIDTH / 2)) < 0.001 &&
        fabs(tempTransform(1,0) - 0.0) < 0.001  && fabs(tempTransform(1,1) - 1.0) < 0.001 && fabs(tempTransform(1,2) - 0.0) < 0.001 && fabs(tempTransform(1,3) - 0.0) < 0.001                          &&
        fabs(tempTransform(2,0) - 0.0) < 0.001  && fabs(tempTransform(2,1) - 0.0) < 0.001 && fabs(tempTransform(2,2) - 1.0) < 0.001 && fabs(tempTransform(2,3) - 0.0) < 0.001                          &&
        fabs(tempTransform(3,0) - 0.0)  < 0.001 && fabs(tempTransform(3,1) - 0.0) < 0.001 && fabs(tempTransform(3,2) - 0.0) < 0.001 && fabs(tempTransform(3,3) - 1.0) < 0.001)
    {
        output.append("generateImplicitStep test passed!\n");
    }
    else
    {
        output.append("generateImplicitStep test FAILED!\n");
    }
    
    
    // add step
    // initialize motion
    // terminate motion
    // loop
    // execute motion

    return output;
}

std::string StepHandler::runVisualTestsPartOne(void)
{
    int numTests = 0;
    std::string output("");
    
    /** computeSwingCoefficients **/
    // Points (0, 0) (0.5, 0.5) (1, 0)
    numTests++;
    double p1X = 0.0;
    double p1Y = 0.0;
    
    double p2X = 0.5;
    double p2Y = 0.5;
    
    double p3X = 1.0;
    double p3Y = 0.0;
    
    
    double t1 = 0.0;
    double t2 = 0.5;
    double t3 = 1.0;
    
    std::vector<double> swingCoefficientsX(computeSwingCoefficients(p1X, p2X, p3X, t1, t2, t3));
    std::vector<double> swingCoefficientsY(computeSwingCoefficients(p1Y, p2Y, p3Y, t1, t2, t3));
    
    // Generate trajectory points.
    std::vector<double> xPosVect;
    std::vector<double> yPosVect;
    std::vector<double> zPosVect;
    
    for (double t = t1; t <= t3; t += 0.1)
    {
        double xPos;
        double yPos;
        
        if (t <= t2)
        {
            xPos = swingCoefficientsX[0] + swingCoefficientsX[1] * t + swingCoefficientsX[2] * pow(t, 2) + swingCoefficientsX[3] * pow(t, 3);
            yPos = swingCoefficientsY[0] + swingCoefficientsY[1] * t + swingCoefficientsY[2] * pow(t, 2) + swingCoefficientsY[3] * pow(t, 3);
        }
        else
        {
            xPos = swingCoefficientsX[4] + swingCoefficientsX[5] * t + swingCoefficientsX[6] * pow(t, 2) + swingCoefficientsX[7] * pow(t, 3);
            yPos = swingCoefficientsY[4] + swingCoefficientsY[5] * t + swingCoefficientsY[6] * pow(t, 2) + swingCoefficientsY[7] * pow(t, 3);
        }
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(0.0);
    }
    
    // Compose output string.
    output.append("computeSwingCoefficients test one - points (0, 0) (0.5, 0.5), (1, 0)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }
    
    // Points (0, 0) (0.25, 0.5) (1, 0)
    numTests++;
    p1X = 0.0;
    p1Y = 0.0;
    
    p2X = 0.25;
    p2Y = 0.5;
    
    p3X = 1.0;
    p3Y = 0.0;
    
    t1 = 0.0;
    t2 = 0.5;
    t3 = 1.0;
    
    swingCoefficientsX = computeSwingCoefficients(p1X, p2X, p3X, t1, t2, t3);
    swingCoefficientsY = computeSwingCoefficients(p1Y, p2Y, p3Y, t1, t2, t3);
    
    // Generate trajectory points.
    xPosVect.clear();
    yPosVect.clear();
    zPosVect.clear();
    
    for (double t = t1; t <= t3; t += 0.1)
    {
        double xPos;
        double yPos;
        
        if (t <= t2)
        {
            xPos = swingCoefficientsX[0] + swingCoefficientsX[1] * t + swingCoefficientsX[2] * pow(t, 2) + swingCoefficientsX[3] * pow(t, 3);
            yPos = swingCoefficientsY[0] + swingCoefficientsY[1] * t + swingCoefficientsY[2] * pow(t, 2) + swingCoefficientsY[3] * pow(t, 3);
        }
        else
        {
            xPos = swingCoefficientsX[4] + swingCoefficientsX[5] * t + swingCoefficientsX[6] * pow(t, 2) + swingCoefficientsX[7] * pow(t, 3);
            yPos = swingCoefficientsY[4] + swingCoefficientsY[5] * t + swingCoefficientsY[6] * pow(t, 2) + swingCoefficientsY[7] * pow(t, 3);
        }
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(0.0);
    }
    
    // Compose output string.
    output.append("computeSwingCoefficients test one - points (0, 0) (0.25, 0.5), (1, 0)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }
    
    // Points (0, 0) (0.75, 0.5) (1, 0)
    numTests++;
    p1X = 0.0;
    p1Y = 0.0;
    
    p2X = 0.75;
    p2Y = 0.5;
    
    p3X = 1.0;
    p3Y = 0.0;
    
    t1 = 0.0;
    t2 = 0.5;
    t3 = 1.0;
    
    swingCoefficientsX = computeSwingCoefficients(p1X, p2X, p3X, t1, t2, t3);
    swingCoefficientsY = computeSwingCoefficients(p1Y, p2Y, p3Y, t1, t2, t3);
    
    // Generate trajectory points.
    xPosVect.clear();
    yPosVect.clear();
    zPosVect.clear();
    
    for (double t = t1; t <= t3; t += 0.1)
    {
        double xPos;
        double yPos;
        
        if (t <= t2)
        {
            xPos = swingCoefficientsX[0] + swingCoefficientsX[1] * t + swingCoefficientsX[2] * pow(t, 2) + swingCoefficientsX[3] * pow(t, 3);
            yPos = swingCoefficientsY[0] + swingCoefficientsY[1] * t + swingCoefficientsY[2] * pow(t, 2) + swingCoefficientsY[3] * pow(t, 3);
        }
        else
        {
            xPos = swingCoefficientsX[4] + swingCoefficientsX[5] * t + swingCoefficientsX[6] * pow(t, 2) + swingCoefficientsX[7] * pow(t, 3);
            yPos = swingCoefficientsY[4] + swingCoefficientsY[5] * t + swingCoefficientsY[6] * pow(t, 2) + swingCoefficientsY[7] * pow(t, 3);
        }
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(0.0);
    }
    
    // Compose output string.
    output.append("computeSwingCoefficients test one - points (0, 0) (0.75, 0.5), (1, 0)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }
    
    // Points (0, 0) (0.5, 0.5) (1, 0)
    // Time (1.0, 1.5, 2.0)
    numTests++;
    p1X = 0.0;
    p1Y = 0.0;
    
    p2X = 0.5;
    p2Y = 0.5;
    
    p3X = 1.0;
    p3Y = 0.0;
    
    t1 = 1.0;
    t2 = 1.5;
    t3 = 2.0;
    
    swingCoefficientsX = computeSwingCoefficients(p1X, p2X, p3X, t1, t2, t3);
    swingCoefficientsY = computeSwingCoefficients(p1Y, p2Y, p3Y, t1, t2, t3);
    
    // Generate trajectory points.
    xPosVect.clear();
    yPosVect.clear();
    zPosVect.clear();
    
    for (double t = t1; t <= t3; t += 0.1)
    {
        double xPos;
        double yPos;
        
        if (t <= t2)
        {
            xPos = swingCoefficientsX[0] + swingCoefficientsX[1] * t + swingCoefficientsX[2] * pow(t, 2) + swingCoefficientsX[3] * pow(t, 3);
            yPos = swingCoefficientsY[0] + swingCoefficientsY[1] * t + swingCoefficientsY[2] * pow(t, 2) + swingCoefficientsY[3] * pow(t, 3);
        }
        else
        {
            xPos = swingCoefficientsX[4] + swingCoefficientsX[5] * t + swingCoefficientsX[6] * pow(t, 2) + swingCoefficientsX[7] * pow(t, 3);
            yPos = swingCoefficientsY[4] + swingCoefficientsY[5] * t + swingCoefficientsY[6] * pow(t, 2) + swingCoefficientsY[7] * pow(t, 3);
        }
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(0.0);
    }
    
    // Compose output string.
    output.append("computeSwingCoefficients test one - points (0, 0) (0.5, 0.5), (1, 0) - time (1.0, 1.5, 2.0)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }
    
    /** computeStartTimeFirstStep **/
    // No rotation.
    numTests++;
    
    xPosVect.clear();
    yPosVect.clear();
    zPosVect.clear();
    
    // Create first real step.
    Point firstStep(Point::right, Point::begin);
    Transform firstStepPos;
    
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    COMContainer firstStepParams;
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    
    stepsBuffer.push_back(firstStep);

    // Generate COM Points.
    firstStepParams = firstStep.getTrajectoryParameters();
    double phaseStartTime = firstStepParams.getTStart();
    
    double k = sqrt(GLOBAL_G / GLOBAL_COM_HEIGHT);
    
    std::cout << "start time : " << phaseStartTime << std::endl;

    
    for (double t = phaseStartTime; t <= 0; t += 0.01)
    {
        double xPos = GLOBAL_STEP_WIDTH / 2 - GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPos = GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(GLOBAL_COM_HEIGHT);
    }
    
    output.append("computeStartTimeFirstStep (No Rotation)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }
    
    // Rotate right foot by 30 degrees.
    numTests++;
    
    xPosVect.clear();
    yPosVect.clear();
    zPosVect.clear();
    
    // Create first real step.
    firstStepPos.clear();
    
    firstStepPos.rotateZ(30).translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    
    stepsBuffer.push_back(firstStep);
    
    // Generate COM Points.
    firstStepParams = firstStep.getTrajectoryParameters();
    phaseStartTime = firstStepParams.getTStart();
    
    std::vector<double> firstStepPoseVect(position6D(firstStepPos));
    double thetaR = firstStepPoseVect[5] * M_PI / 180;
    
    for (double t = phaseStartTime; t <= -1 * phaseStartTime; t += 0.01)
    {
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord;
        
        xPosVect.push_back(xPos);
        yPosVect.push_back(yPos);
        zPosVect.push_back(GLOBAL_COM_HEIGHT);
    }
    
    output.append("computeStartTimeFirstStep (+30 Degree Rotation)_");
    output.append(boost::lexical_cast<std::string>(xPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < xPosVect.size(); i++)
    {
        output.append(boost::lexical_cast<std::string>(xPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(yPosVect[i]));
        output.append("_");
        output.append(boost::lexical_cast<std::string>(zPosVect[i]));
        output.append("_");
    }

    /** generateImplicitStepTrajectories **/
    // No rotation.
    numTests++;
     
    stepsBuffer.clear();
    currentStepBuffer.clear();
     
    generateImplicitStep();
     
    // Create first real step.
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);

    generateImplicitStepTrajectories(currentStepBuffer.front());
     
    // Extract COM and swing foot trajectories.
    std::vector<Transform> comPosVect;
    std::vector<Transform> swingPosVect;

    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));

        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }

    // Generate trajectory for first step as well.
    for (double t = phaseStartTime; t <= 0; t += 0.01)
    {
        Transform tempTransform;

        double xPos = -1 * GLOBAL_X0POS * cosh(k * t);
        double yPos = GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);

        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }

    output.append("generateImplicitStepTrajectories Test (No Rotation)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");

    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));

        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }

    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));

        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
     
    // 30 degree rotation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStepPos.clear();
    firstStepPos.rotateZ(30);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    // Generate trajectory for first step as well.
    for (double t = phaseStartTime; t <= -1 * phaseStartTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_LENGTH / 2;
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }
    
    output.append("generateImplicitStepTrajectories Test (+30 Degree Rotation)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    // 30 degree rotation and 1 cm translation along x axis.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStepPos.clear();
    firstStepPos.rotateZ(30);
    firstStepPos.translate(GLOBAL_STEP_WIDTH + 0.010, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    
    // Generate trajectory for first step as well.
    phaseStartTime = firstStep.getTrajectoryParameters().getTStart();
    
    for (double t = phaseStartTime; t <= -1 * phaseStartTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2 + 0.010;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_LENGTH / 2;
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }
    
    output.append("generateImplicitStepTrajectories Test (+30 Degree Rotation and 0.010 cm translation along x axis)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    /** computeTransitionTime **/
    // No rotation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();

    // Create first step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);

    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create seconds step.
    Point secondStep(Point::left, Point::moving);
    Transform secondStepPos;
    
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    // Generate trajectories.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }

    // Generate trajectory for first step as well.
    COMContainer stepParams(stepsBuffer[0].getTrajectoryParameters());
    phaseStartTime      = stepParams.getTStart();
    double phaseEndTime = stepParams.getTEnd();
    
    for (double t = phaseStartTime; t <= phaseEndTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPos = -1 * GLOBAL_X0POS * cosh(k * t);
        double yPos = GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }

    output.append("computeTransitionTime (No rotation)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");

    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    // +5 degree rotation of first step.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(5.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create seconds step.
    secondStep = Point(Point::left, Point::moving);

    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    // Generate trajectories.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    // Generate trajectory for first step as well.
    stepParams     = stepsBuffer[0].getTrajectoryParameters();
    phaseStartTime = stepParams.getTStart();
    phaseEndTime   = stepParams.getTEnd();
    thetaR         = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (double t = phaseStartTime; t <= phaseEndTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_LENGTH / 2;
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }

    output.append("computeTransitionTime (+5 degree rotation of first step)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    // +10 degree rotation of first step.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(10.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create seconds step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    // Generate trajectories.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    // Generate trajectory for first step as well.
    stepParams     = stepsBuffer[0].getTrajectoryParameters();
    phaseStartTime = stepParams.getTStart();
    phaseEndTime   = stepParams.getTEnd();
    thetaR         = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (double t = phaseStartTime; t <= phaseEndTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_LENGTH / 2;
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }
    
    output.append("computeTransitionTime (+10 degree rotation of first step)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    // +30 degree rotation of first step.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(30.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create seconds step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    // Generate trajectories.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    // Extract COM and swing foot trajectories.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    // Generate trajectory for first step as well.
    stepParams     = stepsBuffer[0].getTrajectoryParameters();
    phaseStartTime = stepParams.getTStart();
    phaseEndTime   = stepParams.getTEnd();
    thetaR         = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (double t = phaseStartTime; t <= phaseEndTime; t += 0.01)
    {
        Transform tempTransform;
        
        double xPosNextCoord = -1 * GLOBAL_X0POS * cosh(k * t) + (GLOBAL_X0VEL / k) * sinh(k * t);
        double yPosNextCoord =      GLOBAL_Y0POS * cosh(k * t) + (GLOBAL_Y0VEL / k) * sinh(k * t);
        
        double xPos = cos(-1 * thetaR) * xPosNextCoord - sin(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_WIDTH / 2;
        double yPos = sin(-1 * thetaR) * xPosNextCoord + cos(-1 * thetaR) * yPosNextCoord + GLOBAL_STEP_LENGTH / 2;
        
        tempTransform.translate(xPos, yPos, GLOBAL_COM_HEIGHT);
        comPosVect.push_back(tempTransform);
    }
    
    output.append("computeTransitionTime (+30 degree rotation of first step)_");
    output.append(boost::lexical_cast<std::string>(comPosVect.size() + currentStepBuffer[0].getNumTrajectories()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        double thetaR = pos[5];
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[2]));
            output.append("_");
        }
    }
    
    /** generateGenericStepTrajectories **/
    // No rotation.
    numTests++;

    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
        
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);

    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);

    // Extract COM points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("generateGenericStepTrajectories Test (No rotation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");

    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }

        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with 10 degree rotation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(10.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    // Extract COM points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("generateGenericStepTrajectories Test (10 Degree Rotation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaR = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with 30 degree rotation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(30.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    // Extract COM points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("generateGenericStepTrajectories Test (30 Degree Rotation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaR = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with 30 degree rotation and 1.0 cm translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(30.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH + 0.010, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    stepsBuffer.push_back(secondStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    // Extract COM points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("generateGenericStepTrajectories Test (30 Degree Rotation and 1.0 cm translation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaR = position6D(firstStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2 + 0.010));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] * cos(-1 * thetaR) - pos[1] * sin(-1 * thetaR) + GLOBAL_STEP_WIDTH / 2 + 0.010));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[0] * sin(-1 * thetaR) + pos[1] * cos(-1 * thetaR) + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    /** optimizePendulumParameters **/
    // No rotation.
    // Test with RWeight = 0.0
    numTests++;

    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    COMContainer secondStepParams;
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);

    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    Point thirdStep(Point::right, Point::moving);
    Transform thirdStepPos;
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);

    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);

    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.0) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.1
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.1);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.1 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.1) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.2
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.2);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.2 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.2) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.3
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.3);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.3 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.3) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.4
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.4);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.4) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.5
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.5);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.5 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.5) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.6
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.6);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.6 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.6) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.7
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.7);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.7 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.7) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.8
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.8);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.8 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.8) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 0.9
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.9);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "rweight : 0.9 : R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;

    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 0.9) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with RWeight = 1.0
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(1.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test (RWeight = 1.0) No Rotation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    

    
    /** computeMotion **/
    /*
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    std::vector<Point> oldSteps;
    
    generateImplicitStep();

    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    stepsBuffer.push_back(thirdStep);
    
    // Create fourth real step.
    fourthStep = Point(Point::left, Point::moving);
    fourthStepPos.clear();
    fourthStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    fourthStep.setFootPos(fourthStepPos);
    stepsBuffer.push_back(fourthStep);
    
    // Trajectories for implicit and first step.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for second step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for third step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    oldSteps.push_back(currentStepBuffer.front());
        
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < oldSteps[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[2].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[2].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[3].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[3].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("computeMotion Test (No rotation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < oldSteps[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH + oldSteps[2].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (1.5) * GLOBAL_STEP_LENGTH + oldSteps[3].getTrajectoryParameters().getR()));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < oldSteps[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (1.5) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    
    
    
    
    
    
    */
    
    
    
    
    /** computeMotion (10 steps) **/
    /*
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    oldSteps.clear();
    
    generateImplicitStep();
    
    // Create steps.
    firstStep = Point(Point::right, Point::begin);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    secondStep = Point(Point::left, Point::moving);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    firstStep = Point(Point::right, Point::moving);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    secondStep = Point(Point::left, Point::moving);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    firstStep = Point(Point::right, Point::moving);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    secondStep = Point(Point::left, Point::moving);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    firstStep = Point(Point::right, Point::moving);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    secondStep = Point(Point::left, Point::moving);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    firstStep = Point(Point::right, Point::moving);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    secondStep = Point(Point::left, Point::moving);
    secondStep.setFootPos(secondStepPos);
    stepsBuffer.push_back(secondStep);
    
    firstStep = Point(Point::right, Point::moving);
    firstStep.setFootPos(firstStepPos);
    stepsBuffer.push_back(firstStep);
    
    // Trajectories for implicit and first step.
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for second step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for third step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for fourth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for fifth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for sixth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for seventh step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for eighth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for ninth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    // Trajectories for tenth step.
    computeMotion();
    
    oldSteps.push_back(currentStepBuffer.front());
    currentStepBuffer.clear();
    currentStepBuffer.push_back(stepsBuffer[0]);
    stepsBuffer.erase(stepsBuffer.begin());
    
    oldSteps.push_back(currentStepBuffer.front());
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < oldSteps[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[2].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[2].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < oldSteps[3].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[3].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[4].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[4].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[5].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[5].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[6].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[6].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[7].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[7].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[8].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[8].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    for (int i = 0; i < oldSteps[9].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(oldSteps[9].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("computeMotion Test (10 Steps)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < oldSteps[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH + oldSteps[2].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (1.5) * GLOBAL_STEP_LENGTH + oldSteps[3].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (2) * GLOBAL_STEP_LENGTH + oldSteps[4].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (2.5) * GLOBAL_STEP_LENGTH + oldSteps[5].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (3) * GLOBAL_STEP_LENGTH + oldSteps[6].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (3.5) * GLOBAL_STEP_LENGTH + oldSteps[7].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories() + oldSteps[8].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (4) * GLOBAL_STEP_LENGTH + oldSteps[8].getTrajectoryParameters().getR()));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories() + oldSteps[8].getNumTrajectories() +
                      oldSteps[9].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (4.5) * GLOBAL_STEP_LENGTH + oldSteps[9].getTrajectoryParameters().getR()));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < oldSteps[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (1.5) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (2) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (2.5) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (3) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (3.5) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories() + oldSteps[8].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (4) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        else if (i < (oldSteps[0].getNumTrajectories() + oldSteps[1].getNumTrajectories() + oldSteps[2].getNumTrajectories() +
                      oldSteps[3].getNumTrajectories() + oldSteps[4].getNumTrajectories() + oldSteps[5].getNumTrajectories() +
                      oldSteps[6].getNumTrajectories() + oldSteps[7].getNumTrajectories() + oldSteps[8].getNumTrajectories() +
                      oldSteps[9].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + (4.5) * GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    */

    // Post testing output manipulation.
    std::string tempOutput(boost::lexical_cast<std::string>(numTests));
    tempOutput.append("_");
    tempOutput.append(output);
    return tempOutput;
}

std::string StepHandler::runVisualTestsPartTwo(void)
{
    int numTests = 0;
    std::string output("");
    
    // Test with no rotation, but with feet translating to the right.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    Point firstStep(Point::right, Point::begin);
    
    Transform firstStepPos;
    firstStepPos.translate(GLOBAL_STEP_WIDTH + 0.002, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    COMContainer firstStepParams;
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    Point secondStep(Point::left, Point::moving);
    
    Transform secondStepPos;
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH + 0.002, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    COMContainer secondStepParams;
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    Point thirdStep(Point::right, Point::moving);
    Transform thirdStepPos;
    thirdStepPos.translate(GLOBAL_STEP_WIDTH + 0.002, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    std::vector<Transform> comPosVect;
    std::vector<Transform> swingPosVect;
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test No Rotation / Pure Translation To Right (2 mm / step)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2 + 0.002));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2 + 0.004));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2 + 0.002));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2 + 0.004));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    /*
    // Test with no rotation, but with feet translating to the right (1 cm / step).
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.translate(GLOBAL_STEP_WIDTH + 0.010, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH + 0.010, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.translate(GLOBAL_STEP_WIDTH + 0.010, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test No Rotation / Pure Translation To Right (1 cm / step)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2 + 0.010));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2 + 0.020));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            output.append(boost::lexical_cast<std::string>(pos[0] + GLOBAL_STEP_WIDTH / 2 + 0.010));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH / 2));
            output.append("_");
        }
        else
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2 + 0.020));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1] + GLOBAL_STEP_LENGTH));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with rotation (5 degrees) but no translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(5.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.rotateZ(5.0);
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.rotateZ(5.0);
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    std::cout << "test: R : " << stepsBuffer[1].getTrajectoryParameters().getR() << " y0Pos : " << stepsBuffer[1].getTrajectoryParameters().getY0Pos() << std::endl;
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test Rotation (5 deg / step) and no translation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    double thetaRFirst  = position6D(firstStepPos)[5] * M_PI / 180;
    double thetaRSecond = position6D(secondStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with rotation (10 degrees) but no translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(10.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.rotateZ(10.0);
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.rotateZ(10.0);
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test Rotation (10 deg / step) and no translation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaRFirst  = position6D(firstStepPos)[5] * M_PI / 180;
    thetaRSecond = position6D(secondStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with rotation (15 degrees) but no translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(15.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.rotateZ(15.0);
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.rotateZ(15.0);
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test Rotation (15 deg / step) and no translation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaRFirst  = position6D(firstStepPos)[5] * M_PI / 180;
    thetaRSecond = position6D(secondStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    // Test with rotation (30 degrees) but no translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(30.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.rotateZ(30.0);
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    thirdStepPos.clear();
    thirdStepPos.rotateZ(30.0);
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    stepsBuffer.push_back(thirdStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("optimizePendulumParameters Test Rotation (30 deg / step) and no translation_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaRFirst  = position6D(firstStepPos)[5] * M_PI / 180;
    thetaRSecond = position6D(secondStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    */
    
    /** Test four footsteps **/
    /*
    // Rotation of 10 degrees per footstep, but no translation.
    numTests++;
    
    stepsBuffer.clear();
    currentStepBuffer.clear();
    
    generateImplicitStep();
    
    // Create first real step.
    firstStep = Point(Point::right, Point::begin);
    
    firstStepPos.clear();
    firstStepPos.rotateZ(10.0);
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    firstStep.setFootPos(firstStepPos);
    
    firstStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    firstStepParams.setX0Vel(GLOBAL_X0VEL);
    firstStepParams.setY0Pos(GLOBAL_Y0POS);
    firstStepParams.setY0Vel(GLOBAL_Y0VEL);
    firstStepParams.setR(0.0);
    
    firstStep.setTrajectoryParameters(firstStepParams);
    
    computeStartTimeFirstStep(firstStep);
    stepsBuffer.push_back(firstStep);
    
    // Create second real step.
    secondStep = Point(Point::left, Point::moving);
    
    secondStepPos.clear();
    secondStepPos.rotateZ(10.0);
    secondStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    secondStep.setFootPos(secondStepPos);
    
    secondStepParams.setX0Pos(GLOBAL_X0POS);
    secondStepParams.setX0Vel(GLOBAL_X0VEL);
    secondStepParams.setRWeight(0.0);
    
    secondStep.setTrajectoryParameters(secondStepParams);
    stepsBuffer.push_back(secondStep);
    
    // Create third real step.
    thirdStep = Point(Point::right, Point::moving);
    
    thirdStepPos.clear();
    thirdStepPos.rotateZ(10.0);
    thirdStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    thirdStep.setFootPos(thirdStepPos);
    
    COMContainer thirdStepParams(thirdStep.getTrajectoryParameters());
    
    thirdStepParams.setX0Pos(-1 * GLOBAL_X0POS);
    thirdStepParams.setX0Vel(GLOBAL_X0VEL);
    thirdStepParams.setRWeight(0.0);
    
    thirdStep.setTrajectoryParameters(thirdStepParams);
    stepsBuffer.push_back(thirdStep);
    
    // Create fourth real step.
    Point fourthStep(Point::left, Point::moving);
    
    Transform fourthStepPos;
    fourthStepPos.rotateZ(10.0);
    fourthStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    fourthStep.setFootPos(fourthStepPos);
    
    stepsBuffer.push_back(fourthStep);
    
    computeTransitionTime(stepsBuffer[0], stepsBuffer[1]);
    computeTransitionTime(stepsBuffer[1], stepsBuffer[2]);
    computeTransitionTime(stepsBuffer[2], stepsBuffer[3]);
    
    generateImplicitStepTrajectories(currentStepBuffer.front());
    
    generateGenericStepTrajectories(currentStepBuffer.front(), stepsBuffer[0], stepsBuffer[1]);
    
    optimizePendulumParameters(stepsBuffer[0], stepsBuffer[1]);
    
    generateGenericStepTrajectories(stepsBuffer[0], stepsBuffer[1], stepsBuffer[2]);
    
    optimizePendulumParameters(stepsBuffer[1], stepsBuffer[2]);
    
    generateGenericStepTrajectories(stepsBuffer[1], stepsBuffer[2], stepsBuffer[3]);
    
    // Extract trajectory points.
    comPosVect.clear();
    swingPosVect.clear();
    
    for (int i = 0; i < currentStepBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(currentStepBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[0].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[0].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[1].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[1].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    for (int i = 0; i < stepsBuffer[2].getNumTrajectories(); i++)
    {
        Trajectory tempTraj(stepsBuffer[2].getTrajectory(i));
        
        comPosVect.push_back(tempTraj.getCOMTransform());
        swingPosVect.push_back(tempTraj.getSwingFootTransform());
    }
    
    output.append("Four successive steps (10 deg rotation / step and no translation)_");
    output.append(boost::lexical_cast<std::string>(2 * comPosVect.size()));
    output.append("_");
    
    thetaRFirst  = position6D(firstStepPos)[5] * M_PI / 180;
    thetaRSecond = position6D(secondStepPos)[5] * M_PI / 180;
    double thetaRThird  = position6D(thirdStepPos)[5] * M_PI / 180;
    
    for (int i = 0; i < comPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(comPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories() + stepsBuffer[1].getNumTrajectories()))
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of second step.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xThirdStepFrame = pos[0];
            double yThirdStepFrame = pos[1];
            
            double xSecondStepFrame = xThirdStepFrame * cos(-1 * thetaRThird) - yThirdStepFrame * sin(-1 * thetaRThird) + GLOBAL_STEP_WIDTH;
            double ySecondStepFrame = xThirdStepFrame * sin(-1 * thetaRThird) + yThirdStepFrame * cos(-1 * thetaRThird) + GLOBAL_STEP_LENGTH / 2;
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
    
    for (int i = 0; i < swingPosVect.size(); i++)
    {
        std::vector<double> pos(position6D(swingPosVect[i]));
        
        if (i < currentStepBuffer[0].getNumTrajectories())
        {
            output.append(boost::lexical_cast<std::string>(pos[0] - GLOBAL_STEP_WIDTH / 2));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(pos[1]));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories()))
        {
            // x and y points will need to be rotated to global frame, and then translated to global position.
            double xFirstStepFrame = pos[0];
            double yFirstStepFrame = pos[1];
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else if (i < (currentStepBuffer[0].getNumTrajectories() + stepsBuffer[0].getNumTrajectories() + stepsBuffer[1].getNumTrajectories()))
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xSecondStepFrame = pos[0];
            double ySecondStepFrame = pos[1];
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        else
        {
            // Process for converting x and y points to global frame.
            // Rotate and then translate to move into coordinate frame of second step.
            // Rotate and then translate to move into coordinate frame of first step.
            // Rotate and then translate to move into coordinate frame of global frame.
            double xThirdStepFrame = pos[0];
            double yThirdStepFrame = pos[1];
            
            double xSecondStepFrame = xThirdStepFrame * cos(-1 * thetaRThird) - yThirdStepFrame * sin(-1 * thetaRThird) + GLOBAL_STEP_WIDTH;
            double ySecondStepFrame = xThirdStepFrame * sin(-1 * thetaRThird) + yThirdStepFrame * cos(-1 * thetaRThird) + GLOBAL_STEP_LENGTH / 2;
            
            double xFirstStepFrame = xSecondStepFrame * cos(-1 * thetaRSecond) - ySecondStepFrame * sin(-1 * thetaRSecond) - GLOBAL_STEP_WIDTH;
            double yFirstStepFrame = xSecondStepFrame * sin(-1 * thetaRSecond) + ySecondStepFrame * cos(-1 * thetaRSecond) + GLOBAL_STEP_LENGTH / 2;
            
            double xGlobalFrame = xFirstStepFrame * cos(-1 * thetaRFirst) - yFirstStepFrame * sin(-1 * thetaRFirst) + GLOBAL_STEP_WIDTH / 2;
            double yGlobalFrame = xFirstStepFrame * sin(-1 * thetaRFirst) + yFirstStepFrame * cos(-1 * thetaRFirst) + GLOBAL_STEP_LENGTH / 2;
            
            output.append(boost::lexical_cast<std::string>(xGlobalFrame));
            output.append("_");
            output.append(boost::lexical_cast<std::string>(yGlobalFrame));
            output.append("_");
        }
        
        output.append(boost::lexical_cast<std::string>(pos[2]));
        output.append("_");
    }
     */
    
    // Post testing output manipulation.
    std::string tempOutput(boost::lexical_cast<std::string>(numTests));
    tempOutput.append("_");
    tempOutput.append(output);
    return tempOutput;
}









/*
std::string StepHandler::getParamValue(std::string paramName)
{
    if (paramName.compare("STEP-HEIGHT") == 0)
    {
        return boost::lexical_cast<std::string>(GLOBAL_STEP_HEIGHT);
    }
    else if (paramName.compare("SWING-LIFT-START-PHASE") == 0)
    {
        return boost::lexical_cast<std::string>(GLOBAL_SWING_LIFT_START_PHASE);
    }
    else if (paramName.compare("SWING-LIFT-END-PHASE") == 0)
    {
        return boost::lexical_cast<std::string>(GLOBAL_SWING_LIFT_END_PHASE);
    }
    else if (paramName.compare("STEP-FREQUENCY") == 0)
    {
        return boost::lexical_cast<std::string>(GLOBAL_STEP_FREQUENCY);
    }
    else if (paramName.compare("RWEIGHT") == 0)
    {
        return boost::lexical_cast<std::string>(GLOBAL_RWEIGHT);
    }
    
    return "";
}
*/

/*
std::string StepHandler::setParamValue(std::string paramName, std::string paramValue)
{
    if (paramName.compare("STEP-HEIGHT") == 0)
    {
        GLOBAL_STEP_HEIGHT = boost::lexical_cast<double>(paramValue);
    }
    else if (paramName.compare("SWING-LIFT-START-PHASE") == 0)
    {
        GLOBAL_SWING_LIFT_START_PHASE = boost::lexical_cast<double>(paramValue);
    }
    else if (paramName.compare("SWING-LIFT-END-PHASE") == 0)
    {
        GLOBAL_SWING_LIFT_END_PHASE = boost::lexical_cast<double>(paramValue);
    }
    else if (paramName.compare("STEP-FREQUENCY") == 0)
    {
        GLOBAL_STEP_FREQUENCY = boost::lexical_cast<double>(paramValue);
    }
    else if (paramName.compare("RWEIGHT") == 0)
    {
        GLOBAL_RWEIGHT = boost::lexical_cast<double>(paramValue);
    }
    
    return std::string("Success");
}
*/

/*
void StepHandler::startTestMotion(void)
{
    // Check if the test motion is already running.
    if (testMotionLoopRunning)
    {
        return;
    }
    
    // Create first step.
    Transform firstStepPos;
    firstStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    
    addStep(Point::right, firstStepPos, Point::begin);

    readyStance();
    
    // Check if test motion was already run once, in which case we only need to resume motion,
    // but do not need to initialize motion.
    if (!testMotionLoopRunOnce)
    {
        testMotionLoopRunOnce = true;
        initializeMotion();
    }
    else
    {
        pauseLoop = false;
    }
    
    continueTestMotion = true;
    boost::thread testMotionThread = boost::thread(&StepHandler::testMotionGenerator, this);
}
*/

/*
void StepHandler::testMotionGenerator(void)
{
    // Generate generic right and left steps.
    Transform rightStepPos;
    rightStepPos.translate(GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    
    Transform leftStepPos;
    leftStepPos.translate(-1 * GLOBAL_STEP_WIDTH, GLOBAL_STEP_LENGTH / 2, 0);
    
    testMotionLoopRunning = true;
    Point::Foot lastStep = Point::right;
    
    // Begin loop of infinite walk.
    while (true)
    {
        try
        {
            if (!continueTestMotion)
            {
                testMotionLoopRunning = false;
                break;
            }
            
            if (stepsBuffer.size() <= 2)
            {
                if (lastStep == Point::right)
                {
                    addStep(Point::left, leftStepPos, Point::moving);
                    lastStep = Point::left;
                }
                else
                {
                    addStep(Point::right, rightStepPos, Point::moving);
                    lastStep = Point::right;
                }
            }
        }
        catch (std::exception& e)
        {
            testMotionLoopRunning = false;
            break;
        }
    }
}
*/

/*
void StepHandler::stopTestMotion(void)
{
    // Kill test motion thread and pause the motion loop.
    continueTestMotion = false;
    pauseLoop = true;
    
    // Wait for both threads to stop.
    while (testMotionLoopRunning || !loopPaused)
    {
        usleep(200);
    }
    
    // Remove all steps in buffers.
    currentStepBuffer.clear();
    stepsBuffer.clear();
    
    restStance();
}
*/