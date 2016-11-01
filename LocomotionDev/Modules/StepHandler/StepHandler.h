/** @file    StepHandler.h
 *  @brief   Header file to StepHandler class, which interfaces behavior modules with locomotion.
 *  @author  Austin Small.
 */

#ifndef StepHandler_h_DEFINED
#define StepHandler_h_DEFINED

#include <math.h>
#include <vector>
#include <unistd.h>
#include <sys/time.h>
#include <ctime>

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/lexical_cast.hpp>

#include <alcommon/alproxy.h>
#include <alcommon/albroker.h>
#include <alcommon/almodule.h>

#include "../LocomotionDefines.h"
#include "../Representations/Point.h"
#include "../Representations/PointTests.h"
#include "../Representations/Trajectory.h"
#include "../Representations/TrajectoryTests.h"
#include "../Representations/Transform.h"
#include "../Representations/TransformTests.h"
#include "../Representations/COMContainer.h"
#include "../Representations/COMContainerTests.h"
#include "../Kinematics/KinematicsWrapper.h"
#include "../Kinematics/TestKinematics.h"
#include "../Utils/Iterp/Iterp.h"

// Output stream libraries included for testing only.
#include <iostream>
#include <fstream>

namespace AL
{
    class ALBroker;
}

/** @brief		StepHandler class interfaces behavior modules with locomotion.
 */
class StepHandler : public AL::ALModule
{
    public:
        // Methods
        StepHandler(boost::shared_ptr<AL::ALBroker> pBroker, const std::string &pName);
        virtual ~StepHandler();
        void exit(void);
        std::string communicate(std::string);
    
        void readyStance(void);
        void restStance(void);
    
        void addStep(Point::Foot argFoot, Transform& argTransform, Point::StepType argStepType);
        void initializeMotion(void);
    
    private:
        // Fields
        // NAOQI related fields.
        AL::ALProxy* shmProxy;
    
        // Array to store a mapping from array indices to actuator names.
        std::string indexToStringMap[GLOBAL_NUM_ACTUATORS];
    
        std::vector<Point> stepsBuffer;
        std::vector<Point> currentStepBuffer;
    
        // Main loop and loop state variables.
        boost::thread loopThread;
        bool pauseLoop;
        bool stopLoop;
        bool loopRunning;
        bool loopPaused;
    
        // Test motion loop state variables.
        bool continueTestMotion;
        bool testMotionLoopRunning;
        bool testMotionLoopRunOnce;
    
        /** Locomotion Parameters **/
        double GLOBAL_STEP_WIDTH;
        double GLOBAL_STEP_LENGTH;
        double GLOBAL_STEP_HEIGHT;
        double GLOBAL_COM_HEIGHT;
        double GLOBAL_COM_OFFSET_X;
        double GLOBAL_COM_OFFSET_Y;
        double GLOBAL_G;
        double GLOBAL_Y0POS;
        double GLOBAL_Y0VEL;
        double GLOBAL_X0POS;
        double GLOBAL_X0VEL;
        double GLOBAL_FIRST_SWING_LIFT_START_PHASE;
        double GLOBAL_FIRST_SWING_LIFT_END_PHASE;
        double GLOBAL_SWING_LIFT_START_PHASE;
        double GLOBAL_SWING_LIFT_END_PHASE;
        double GLOBAL_STEP_FREQUENCY;
        double GLOBAL_RWEIGHT;

        // Methods
        void loadLocomotionParameters(void);
    
        void loop(void);
        void computeMotion(void);
        void executeMotion(void);
        void generateImplicitStep(void);
        void generateImplicitStepTrajectories(Point& implicitStep);
        void generateGenericStepTrajectories(Point& currentStep, Point& nextStep, Point& nextNextStep);
        void optimizePendulumParameters(Point& currentStep, Point& nextStep);
        void updateSwingTrajectory(void);
    
        // Low level functional / helper methods.
        void computeTransitionTime(Point& firstStep, Point& secondStep);
        void computeStartTimeFirstStep(Point& firstStep);
        double integratePathLength(double tMin, double tMax,
                                   std::vector <double> fdxdt,
                                   std::vector <double> fdydt);
        double bezierTimeWrapper(double s, double L,
                                 std::vector<double> fdxdt,
                                 std::vector<double> fdydt);
        double evaluateCoeff(std::vector <double> coeffVect, double param);
        std::vector<double> computeSwingCoefficients(double p1, double p2, double p3, double t1, double t2, double t3);
    
        // Debugging methods.
        std::string runTests(void);
        std::string runVisualTestsPartOne(void);
        std::string runVisualTestsPartTwo(void);
        std::string runVisualTestsPartThree(void);
        std::string getParamValue(std::string paramName);
        std::string setParamValue(std::string paramName, std::string paramValue);
        void startTestMotion(void);
        void testMotionGenerator(void);
        void stopTestMotion(void);
};

enum LocomotionVariables { GLOBAL_STEP_WIDTH_ENUM,
                           GLOBAL_STEP_LENGTH_ENUM,
                           GLOBAL_STEP_HEIGHT_ENUM,
                           GLOBAL_COM_HEIGHT_ENUM,
                           GLOBAL_COM_OFFSET_X_ENUM,
                           GLOBAL_COM_OFFSET_Y_ENUM,
                           GLOBAL_G_ENUM,
                           GLOBAL_Y0POS_ENUM,
                           GLOBAL_Y0VEL_ENUM,
                           GLOBAL_X0POS_ENUM,
                           GLOBAL_X0VEL_ENUM,
                           GLOBAL_FIRST_SWING_LIFT_START_PHASE_ENUM,
                           GLOBAL_FIRST_SWING_LIFT_END_PHASE_ENUM,
                           GLOBAL_SWING_LIFT_START_PHASE_ENUM,
                           GLOBAL_SWING_LIFT_END_PHASE_ENUM,
                           GLOBAL_STEP_FREQUENCY_ENUM,
                           GLOBAL_RWEIGHT_ENUM,
                           NumLocomotionVariables};

#endif
