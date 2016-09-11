/** @file    COMContainer.cpp
 *  @brief   cpp file to COMContainer class, which stores COM trajectory parameters.
 *  @author  Austin Small.
 */

#include "COMContainer.h"

/** @brief   COMContainer class constructor.  All values are instantiated with zeros.
 */
    
COMContainer::COMContainer(void)
{
    x0Pos   = 0.0;
    x0Vel   = 0.0;
    y0Pos   = 0.0;
    y0Vel   = 0.0;
    r       = 0.0;
    rWeight = 0.0;
    tStart  = 0.0;
    tEnd    = 0.0;
}

/** @brief   Clears all values, setting them to default 0.0.
 *
 */

void COMContainer::clear(void)
{
    x0Pos   = 0.0;
    x0Vel   = 0.0;
    y0Pos   = 0.0;
    y0Vel   = 0.0;
    r       = 0.0;
    rWeight = 0.0;
    tStart  = 0.0;
    tEnd    = 0.0;
}

/** @brief   Sets the x0Pos to a designated value.
 *
 *  @param   argX0Pos   The designated x0Pos, which represents the horizontal offset
 *                      of the pendulum from the foot at t = 0.
 */
    
void COMContainer::setX0Pos(double argX0Pos)
{
    x0Pos = argX0Pos;
}

/** @brief   Returns the x0Pos.
 *
 *  @return  Returns the x0Pos parameter.
 */
    
double COMContainer::getX0Pos(void)
{
    return x0Pos;
}
    
/** @brief   Sets the x0Vel to a designated value.
 *
 *  @param   argX0Vel   The designated x0Vel, which represents the horizontal velocity
 *                      of the pendulum at t = 0.
 */
    
void COMContainer::setX0Vel(double argX0Vel)
{
    x0Vel = argX0Vel;
}
    
/** @brief   Returns the x0Vel.
 *
 *  @return  Returns the x0Vel parameter.
 */
    
double COMContainer::getX0Vel(void)
{
    return x0Vel;
}
    
/** @brief   Sets the y0Pos to a designated value.
 * 
 *  @param   argY0Pos   The designated y0Pos, which represents the longitudinal offset
 *                      of the pendulum from the foot at t = 0.
 */
    
void COMContainer::setY0Pos(double argY0Pos)
{
    y0Pos = argY0Pos;
}
    
/** @brief   Returns the y0Pos.
 *
 *  @return  Returns the y0Pos parameter.
 */
    
double COMContainer::getY0Pos(void)
{
    return y0Pos;
}

/** @brief   Sets the y0Vel to a designated value.
 *
 *  @param   argX0Vel   The designated y0Vel, which represents the longitudinal velocity
 *                      of the pendulum at t = 0.
 */
    
void COMContainer::setY0Vel(double argY0Vel)
{
    y0Vel = argY0Vel;
}

/** @brief   Returns the y0Vel.
 *
 *  @return  Returns the y0Vel parameter.
 */
    
double COMContainer::getY0Vel(void)
{
    return y0Vel;
}

/** @brief   Sets r - the longitudinal offset of the pendulum origin from the foot -
 *           to a designated value.
 *
 *  @param   argR   The designated longitudinal offset value.
 */
    
void COMContainer::setR(double argR)
{
    r = argR;
}
    
/** @brief   Gets r - the longitudinal offset of the pendulum origin from the foot.
 *
 *  @return  r   The designated longitudinal offset value.
 */
    
double COMContainer::getR(void)
{
    return r;
}

/** @brief   Sets rWeight to a designated value.  rWeight tells the LIPM solver how to preference
 *           the symmetry of a LIPM phase vs its position at t0 relative to the center of the foot.
 *           This value should range from [0, 1], with 1 giving r the highest weight over y0Pos.
 *
 *  @param   argRWeight   The designated weight value.
 */

void COMContainer::setRWeight(double argRWeight)
{
    rWeight = argRWeight;
}

/** @brief   Gets rWeight.
 *
 *  @return  rWeight   The designated rWeight value.
 */

double COMContainer::getRWeight()
{
    return rWeight;
}

/** @brief   Sets tStart - the start time of the pendulum's motion - to a designated value.
 *
 *  @param   argTStart   The designated start time.
 */
    
void COMContainer::setTStart(double argTStart)
{
    tStart = argTStart;
}

/** @brief   Gets tStart - the start time of the pendulum's motion.
 *
 *  @return  tStart   The designated start time.
 */
    
double COMContainer::getTStart()
{
    return tStart;
}

/** @brief   Sets tEnd - the end time of the pendulum's motion - to a designated value.
 *
 *  @param   argTEnd   The designated end time.
 */
    
void COMContainer::setTEnd(double argTEnd)
{
    tEnd = argTEnd;
}

/** @brief   Gets tEnd - the end time of the pendulum's motion.
 *
 *  @return  tEnd   The designated end time.
 */
    
double COMContainer::getTEnd()
{
    return tEnd;
}