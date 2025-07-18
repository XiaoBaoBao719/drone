#include "EKF.h"
#include <iostream>

int main() {
    // Create EKF for 2D position tracking (state: [x, y, vx, vy])
    ExtendedKalmanFilter ekf(4, 2); // 4 states, 2 observations
    
    // Set state transition function (constant velocity model)
    ekf.setStateTransitionFunction([](const std::vector<double>& x, 
                                     const std::vector<double>& u, 
                                     double dt) {
        return std::vector<double>{
            x[0] + x[2] * dt,  // x + vx * dt
            x[1] + x[3] * dt,  // y + vy * dt
            x[2],              // vx
            x[3]               // vy
        };
    });
    
    // Set observation function (position only)
    ekf.setObservationFunction([](const std::vector<double>& x) {
        return std::vector<double>{x[0], x[1]}; // [x, y]
    });
    
    // Set Jacobians...
    // Initialize and use the filter...
    
    return 0;
}