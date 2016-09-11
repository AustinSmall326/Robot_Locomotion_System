/** @file    PID.h
 *  @brief   Header file to PID class, which modulates joint stiffness in order to control joint angles.
 *  @author  Austin Small.
 */

#ifndef PID_H
#define PID_H

#include <cmath>

/** @brief		PID class allows modulation of joint stiffness in order to control joint angles.
 */
class PID
{
    public:
        // Methods.
        PID (int _state, double _Kp = 5.0, double _Kd = 5.0, double _Ki = 5.0);
        ~PID (void);
        double calculate (double setpoint, double pv, int direction, double dt);
    
        // Fields.
        enum controllerState { P_ON,
                               I_ON,
                               D_ON,
                               PI_ON,
                               ID_ON,
                               PD_ON,
                               PID_ON };
    
    private:
        // Fields.
        double max;
        double min;
        double Kp;
        double Kd;
        double Ki;
        double preError;
        double integral;
};

#endif