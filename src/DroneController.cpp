#include "DroneController.h"


void DroneController::Init() {
    resetPID();

}


void DroneController::Reset() {
    resetPID();
}

float DroneController::PID(float err_, float prev_err_, float p_gain, float i_gain,
            float prev_i_gain, float d_gain, float dt) {

    float p_ = p_gain * err_;
    float i_ = prev_i_gain + i_gain * ( ((err_ + prev_err_) * dt) / 2 );
    float d_ = d_gain * ( (err_ - prev_err_) / dt );

    float output = p_ + i_ + d_;
    
    /* clamp output for Integral anti-wind up*/
    if (output >= pid_out_hi_limit) {
        output = pid_out_hi_limit;
    } else if (output < pid_out_lo_limit ) {
        output = pid_out_lo_limit;    
    }

    return output;
}

void DroneController::resetPID() {

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

