/** @file    Point.cpp
 *  @brief   Point class for representation of a footstep.
 *  @author  Austin Small.
 */

#include "Point.h"

/** @brief   Point class constructor.
 */

Point::Point(Foot argFoot, StepType argStepType)
{
    foot = argFoot;
    stepType = argStepType;
    extraTime = 0.0;
    shouldUpdateSwingTrajectory = false;
}

/** @brief   Sets the footPos to a designated Transform.
 *
 *  @param   argFootPos     Transform representing new foot position.
 */

void Point::setFootPos(Transform argFootPos)
{
    footPos = argFootPos;
}

/** @brief   Returns a reference to the footPos Transform.
 *
 *  @return  A reference to the footPos Transform object.
 */

Transform Point::getFootPos(void)
{
    return footPos;
}

/** @brief   Adds new trajectories in conjuction with a specified time stamp.
 *
 *  @param   argTrajectory    New trajectory to add to trajectoryVector.
 */

void Point::addTrajectory(Trajectory argTrajectory)
{
    trajectoryVector.push_back(argTrajectory);
}

/** @brief   Returns a Trajectory containing a time stamp, as well as the relevant
 *           COM Transform and swing foot Transform.
 *
 *  @return  Returns a Trajectory as described above.
 */

Trajectory Point::getTrajectory(int index)
{
    return trajectoryVector[index];
}

/** @brief   Returns a Trajectory containing a time stamp, as well as the relevant
 *           COM Transform and swing foot Transform.
 *
 *  @return  Returns a Trajectory as described above.
 */

Trajectory Point::getTrajectoryFromEnd(int indexFromEnd)
{
    return trajectoryVector[trajectoryVector.size() - 1 - indexFromEnd];
}

/** @brief   Replaces a trajectory, specified by index from the end of the buffer, with a new
 *           trajectory.
 *
 *  @param   indexFromEnd   Index of trajectory to replace (relative to end of buffer).
 *  @param   newTrajectory  New trajectory to take place of old one.
 */

void Point::replaceTrajectoryFromEnd(int indexFromEnd, Trajectory newTrajectory)
{
    trajectoryVector[trajectoryVector.size() - 1 - indexFromEnd] = newTrajectory;
}

/** @brief   Returns the number of trajectories stored in the trajectory vector.
 *           Useful while indexing into the vector.
 *
 *  @return  Returns the number of trajectories stored in the trajectory vector.
 */

int Point::getNumTrajectories()
{
    return trajectoryVector.size();
}

/** @brief   Returns an enum detailing which foot (right or left) this point
 *           should be associated with.
 *  @return  Foot enumeration.
 */

Point::Foot Point::getFoot(void)
{
    return foot;
}

/** @brief   Returns an enum detailing whether the step is a beginning, moving, or end step.
 *  @return  StepType enumeration.
 */

Point::StepType Point::getStepType(void)
{
    return stepType;
}

/** @brief   Sets the LIPM parameters for this step.
 *
 *  @param   argContainer     A COMContainer that stores the relevant parameters.
 */

void Point::setTrajectoryParameters(COMContainer argContainer)
{
    trajParameters = argContainer;
}

/** @brief   Returns a COMContainer that stores the current step LIPM parameters.
 *
 *  @return  A COMContainer that stores the current step LIPM parameters.
 */

COMContainer Point::getTrajectoryParameters(void)
{
    return trajParameters;
}

/** @brief   Sets the extraTime for this step.
 *
 *  @param   argTime     Time between last computed COM trajectory point and time to realize
 *                       full trajectory.
 */

void Point::setExtraTime(double argTime)
{
    extraTime = argTime;
}

/** @brief   Returns extraTime for this step.
 *
 *  @return  Time between last computed COM trajectory point and tiem to realize full trajectory.
 */

double Point::getExtraTime(void)
{
    return extraTime;
}

/** @brief   Set state of shouldUpdateSwingTrajectory.  If true, swing trajectory will be recomputed.
 *
 *  @param   True if swing foot trajectory should be recomputed.  False otherwise.
 */

void Point::setShouldUpdateSwingTrajectory(bool argUpdate)
{
    shouldUpdateSwingTrajectory = argUpdate;
}

/** @brief   Get state of shouldUpdateSwingTrajectory.  If true, swing trajectory will be recomputed.
 *
 *  @return  True if swing foot trajectory should be recomputed.  False otherwise.
 */

bool Point::getShouldUpdateSwingTrajectory(void)
{
    return shouldUpdateSwingTrajectory;
}
