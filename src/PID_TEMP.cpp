#include "PIDController.h"


void PIDController::Init() {
    resetPID();

}

void PIDController::Reset()
{
    resetPID();
}

float PIDController::PID(float setpnt, float prev_setpnt, float err, float prev_err, 
        float p_gain, float i_gain, float prev_i_gain, float i_saturation, 
        float d_gain,
        float output_saturation, 
        float dt) {

    float p_ = p_gain * err;
    
    /* Anti-windup logic*/
    float i_dt = ( (err + prev_err) / 2. ) * dt;
    /* Integral saturation control */
    if (abs(prev_i_gain * i_dt) >= i_saturation) {
        i_gain = 0.0; // if i_gain goes beyond saturation limits, reset gain to zero
    } else {
        i_gain = i_gain;
    }
    /* Integral term */
    float i_ = prev_i_gain + i_gain * i_dt;
    /* Derivative kick compensation using derivative of set points */
    float d_ = d_gain * ( setpnt - prev_setpnt) * (1 / dt); 

    /* Calculate final PID output */
    float output = p_ + i_ + d_;

    /* Limit output based on physical actuator limits */
    if (output_saturation != 0.0) {
        if (abs(output) >= abs(output_saturation)) {
            output = sgn(output) * output_saturation; 
        }
    }
    return output;
}

// Source - https://stackoverflow.com/a/4609795
// Posted by user79758, modified by community. See post 'Timeline' for change history
// Retrieved 2026-03-21, License - CC BY-SA 4.0

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}


void PIDController::resetPID() {

    prevErrorRateX = 0.0;
    prevErrorRateY = 0.0;
    prevErrorRateZ = 0.0;
    prevIGainRateX = 0.0;
    prevIGainRateY = 0.0;
    prevIGainRateZ = 0.0;

    prevErrorAngleX = 0.0;
    prevErrorAngleY = 0.0;
    prevIGainAngleX = 0.0;
    prevIGainAngleY = 0.0;

    prevErrorVelZ = 0.0;
    prevIGainVelZ = 0.0;
}

