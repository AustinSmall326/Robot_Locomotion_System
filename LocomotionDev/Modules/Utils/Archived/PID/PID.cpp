/** @file    PID.cpp
 *  @brief   cpp file to PID class, which modulates joint stiffness in order to control joint angles.
 *  @author  Austin Small.
 */

#include "PID.h"

/** @brief   PID class constructor.
 *
 *  @param   dt         Sampling time step.
 *  @param   state      Which P/I/D components of controller are to be used.
 *  @param   Kp         Proportional gain.
 *  @param   Kd         Derivative gain.
 *  @param   Ki         Integral gain.
 */

PID::PID (int _state, double _Kp, double _Kd, double _Ki) :
    Kp(_Kp),
    Kd(_Kd),
    Ki(_Ki),
    max(1.0),
    min(0.0),
    preError(0.0),
    integral(0.0)
{
    switch(_state)
    {
        case P_ON   : Kd = 0.0;
                      Ki = 0.0;
                      break;
        case I_ON   : Kp = 0.0;
                      Kd = 0.0;
                      break;
        case D_ON   : Kp = 0.0;
                      Ki = 0.0;
                      break;
        case PI_ON  : Kd = 0.0;
                      break;
        case ID_ON  : Kp = 0.0;
                      break;
        case PD_ON  : Ki = 0.0;
                      break;
        case PID_ON : break;
    }
}

/** @brief   PID class destructor.
 *
 */

PID::~PID (void)
{ }

/** @brief   Method to compute joint stiffness.
 *
 *  @param   setpoint   Expected joint angle.
 *  @param   pv         Actual joint angle.
 *  @param   direction  Is the angle slope positive or negative.
 */

double PID::calculate (double setpoint, double pv, int direction, double dt)
{
    // Calculate error.
    double error = setpoint - pv;
    
    // Proportional term.
    double Pout = Kp * error;
    
    // Integral term.
    integral += error * dt;
    double Iout = Ki * integral;
    
    // Derivative term.
    double derivative = (error - preError) / dt;
    double Dout = Kd * derivative;
    
    // Calculate total output.
    double output = Pout + Iout + Dout;
    
    /*
    // Restrict to max / min.
    if (output > max)
    {
        output = max;
    }
    else if (output < min)
    {
        output = min;
    }
    */
    
    // Save error to previous error.
    preError = error;

    return output;
}