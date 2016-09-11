/** @file    Trajectory.cpp
 *  @brief   cpp file to Trajectory class, which stores a time stamp and associated
 *           COM and swing foot transforms.
 *  @author  Austin Small.
 */

#include "Trajectory.h"

/** @brief   Trajectory class constructor.
 */
    
Trajectory::Trajectory(void) { }

/** @brief   Clear values of Trajectory object.
 */

void Trajectory::clear(void)
{
    time = 0.0;
    comTransform.clear();
    swingFootTransform.clear();
}

/** @brief   Sets the time to a designated value.
 *
 *  @param   argTime    The designated time for execution of the Trajectory.
 */

void Trajectory::setTime(double argTime)
{
    time = argTime;
}

/** @brief   Returns the time.
 *
 *  @return  Returns the time parameter.
 */
    
double Trajectory::getTime(void)
{
    return time;
}

/** @brief   Sets the comTransofm to a designated Transform.
 *
 *  @param   argTransform   The designated Transform, which encodes position and orientation
 *                          for the COM.
 */
    
void Trajectory::setCOMTransform(Transform argTransform)
{
    comTransform = argTransform;
}
    
/** @brief   Returns the comTransform.
 *
 *  @return  Returns the comTransform parameter.
 */
    
Transform Trajectory::getCOMTransform(void)
{
    return comTransform;
}
    
/** @brief   Sets the swing foot Transform to a designated Transform.
 *
 *  @param   argTransform   The designated Transform, which encodes position and orientation for the
 *                          swing foot.     */
    
void Trajectory::setSwingFootTransform(Transform argTransform)
{
    swingFootTransform = argTransform;
}
    
/** @brief   Returns the swing foot Transform.
 *
 *  @return  Returns the swing foot Transform parameter.
 */
    
Transform Trajectory::getSwingFootTransform(void)
{
    return swingFootTransform;
}
