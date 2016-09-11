/** @file    Point.h
 *  @brief   Header file to Point class for representation of a footstep.
 *  @author  Austin Small.
 */

#ifndef Point_h_DEFINED
#define Point_h_DEFINED

#include <vector>
#include "Transform.h"
#include "Trajectory.h"
#include "COMContainer.h"

/** @brief		Point class represents a footstep.
 */
class Point
{
    public:
        // Enums
        enum Foot { left, right };
        enum StepType { begin, moving, end };
    
        // Methods
        Point(Foot argFoot, StepType argStepType);
        void setFootPos(Transform argFootPos);
        Transform getFootPos(void);
        void addTrajectory(Trajectory argTrajectory);
        Trajectory getTrajectory(int index);
        int getNumTrajectories();
        Point::Foot getFoot(void);
        Point::StepType getStepType(void);
        void setTrajectoryParameters(COMContainer argContainer);
        COMContainer getTrajectoryParameters();
        void setExtraTime(double argTime);
        double getExtraTime(void);
    
    private:
        // Fields
        Transform footPos;
        std::vector<Trajectory> trajectoryVector;
        Foot foot;
        StepType stepType;
        COMContainer trajParameters;
        double extraTime;
};

#endif
