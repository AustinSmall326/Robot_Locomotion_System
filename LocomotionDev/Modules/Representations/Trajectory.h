/** @file    Trajectory.h
 *  @brief   Header file to Trajectory class, which stores a time stamp and associated
 *           COM and swing foot transforms.
 *  @author  Austin Small.
 */

#ifndef Trajectory_h_DEFINED
#define Trajectory_h_DEFINED

#include "Transform.h"

/** @brief		Trajectory class stores a time stamp and the associated transforms for COM and swing foot.
 */
class Trajectory
{
    public:
        // Constructor.
        Trajectory(void);
    
        void clear(void);
    
        // Getter and setter methods.
        void setTime(double argTime);
        double getTime(void);
        void setCOMTransform(Transform argTransform);
        Transform getCOMTransform(void);
        void setSwingFootTransform(Transform argTransform);
        Transform getSwingFootTransform(void);
    
    private:
        // Fields.
        double time;
        Transform comTransform;
        Transform swingFootTransform;
};

#endif
