#include "DroneEKF.h"

// #include <BasicLinearAlgebra.h>

/*
    Simple EKF for a quadcopter drone

    State vector:
    [px, py, pz,           // 3-axis position (m)
     vx, vy, vz,           // 3-axis velocity (m/s)
     roll, pitch, yaw,     // 3-axis attitude Euler angles (rad)
     p, q, r]              // 3-axis rotation rates (rad/s)

    // [q0, q1, q2, q3, px, py, pz] where q is the quaternion orientation and p is position

    Asumptions:
    - input[] are four propeller speed values.
    - measurement[] is a 9 element array containing accelerometer (3), gyroscope (3),
      x-y camera position (2), and time of flight altitude (1) measurements. 
    - process and measurement noise are diagonal matrices represented as 1D arrays

*/

/* Helper function for angle wrapping */
static inline double wrap_angle(double angle) {
    if (angle > M_PI) angle -= (2.0 * M_PI);
    else if (angle < M_PI) angle += (2.0 * M_PI); 
    return angle;
}

static inline Vector3 rotate_body_z_to_world(const double q[4], double z_accel) {
    // Rotate a body-frame Z acceleration into world-frame using the quaternion
    Vector3 result;
    result(0) = 2 * (q[1]*q[3] - q[0]*q[2]) * z_accel; // x
    result(1) = 2 * (q[2]*q[3] + q[0]*q[1]) * z_accel; // y
    result(2) = (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3]) * z_accel; // z
    return result;
}

/**
 * Drone EKF constructor
 */
DroneEKF::DroneEKF(float accel_data[]) {
    // initialize state and covariance variables
    state = Vector12.Fill(0);
    state(0) = 1.0; // initial quaternion w=1
    covariance = Matrix7.Fill(1) * 1e-2;  // identity matrix with small uncertainty
    process_noise = Matrix7.Fill(1) * 1e-4; // small process noise
    measurement_noise = Matrix3.Fill(1) * 1e-2; // small measurement noise
    
    
}

/**
 * Constructor
 */
DroneEKF::DroneEKF(float accel_data[]) {
    state = Vector12::Zero();
    state(6) = 0.0;  // roll
    state(7) = 0.0;  // pitch
    state(8) = 0.0;  // yaw

    covariance = Matrix12::Identity() * 0.1;
    
    process_noise = Matrix12::Identity() * 1e-4;
    process_noise(0,0) = process_noise(1,1) = process_noise(2,2) = 1e-3;  // position
    process_noise(3,3) = process_noise(4,4) = process_noise(5,5) = 1e-3;  // velocity
    process_noise(6,6) = process_noise(7,7) = process_noise(8,8) = 1e-4;  // attitude
    process_noise(9,9) = process_noise(10,10) = process_noise(11,11) = 1e-4; // rotation rates

    measurement_noise = Matrix9::Identity() * 0.01;

    (void)accel_data;
}

/**
 * Initialize EKF with initial state and covariance
 */
void DroneEKF::initialize(const double initial_state[], const double initial_covariance[]) {
    for (int i = 0; i < 12; ++i) {
        state(i) = initial_state[i];
    }

    for (int r = 0; r < 12; ++r) {
        for (int c = 0; c < 12; ++c) {
            covariance(r,c) = initial_covariance[r*12 + c];
        }
    }
}

/**
 * Main EKF estimation step
 */
void DroneEKF::ekf_estimate(double prior_state[], double prior_covariance[], 
                             double input[], double measurement[], double dt) {
    // Load prior
    for (int i = 0; i < 12; ++i) {
        state(i) = prior_state[i];
    }
    for (int r = 0; r < 12; ++r) {
        for (int c = 0; c < 12; ++c) {
            covariance(r,c) = prior_covariance[r*12 + c];
        }
    }

    // Predict step
    pred_state(prior_state, input, dt);
    for (int i = 0; i < 12; ++i) {
        prior_state[i] = state(i);
    }
    pred_state_cov(nullptr, nullptr, dt);

    // Update step with measurements
    update(prior_state, measurement, nullptr);

    // Write back posterior
    for (int i = 0; i < 12; ++i) {
        prior_state[i] = state(i);
    }
    for (int r = 0; r < 12; ++r) {
        for (int c = 0; c < 12; ++c) {
            prior_covariance[r*12 + c] = covariance(r,c);
        }
    }
}

/**
 * Predict step
 */
void DroneEKF::predict(double prior[], double prior_covariance[]) {
    (void)prior; (void)prior_covariance;
}

/**
 * Predict state using motion model
 * input: [thrust1, thrust2, thrust3, thrust4]
 */
void DroneEKF::pred_state(double state_arr[], double input[], double dt) {
    const double mass = 1.2;  // kg
    const double g = 9.81;    // m/s^2

    // Extract current state
    double px = state(0), py = state(1), pz = state(2);
    double vx = state(3), vy = state(4), vz = state(5);
    double roll = state(6), pitch = state(7), yaw = state(8);
    double p = state(9), q = state(10), r = state(11);

    // Compute total thrust
    double total_thrust = 0.0;
    if (input) {
        total_thrust = input[0] + input[1] + input[2] + input[3];
    } else {
        total_thrust = mass * g;  // hover thrust
    }

    // Rotation matrix body to world
    double R[3][3];
    euler_to_rotation_matrix(roll, pitch, yaw, R);

    // Thrust vector in body frame (along z-axis)
    double ax_body = 0.0, ay_body = 0.0, az_body = total_thrust / mass;

    // Transform to world frame
    double ax_world = R[0][0]*ax_body + R[0][1]*ay_body + R[0][2]*az_body;
    double ay_world = R[1][0]*ax_body + R[1][1]*ay_body + R[1][2]*az_body;
    double az_world = R[2][0]*ax_body + R[2][1]*ay_body + R[2][2]*az_body - g;

    // Update position
    px += vx * dt + 0.5 * ax_world * dt * dt;
    py += vy * dt + 0.5 * ay_world * dt * dt;
    pz += vz * dt + 0.5 * az_world * dt * dt;

    // Update velocity
    vx += ax_world * dt;
    vy += ay_world * dt;
    vz += az_world * dt;

    // Gyroscope to Euler angle rates
    double cp = cos(pitch);
    double tp = tan(pitch);
    double roll_rate = p + q*sin(roll)*tp + r*cos(roll)*tp;
    double pitch_rate = q*cos(roll) - r*sin(roll);
    double yaw_rate = q*sin(roll)/cp + r*cos(roll)/cp;

    // Update attitude
    roll += roll_rate * dt;
    pitch += pitch_rate * dt;
    yaw += yaw_rate * dt;

    // Wrap angles
    yaw = wrap_angle(yaw);

    // Rotation rates remain unchanged (direct gyro measurement)
    // p, q, r unchanged in this model

    // Write back state
    state(0) = px; state(1) = py; state(2) = pz;
    state(3) = vx; state(4) = vy; state(5) = vz;
    state(6) = roll; state(7) = pitch; state(8) = yaw;
    state(9) = p; state(10) = q; state(11) = r;

    if (state_arr) {
        for (int i = 0; i < 12; ++i) {
            state_arr[i] = state(i);
        }
    }
}

/**
 * Compute Jacobian of motion model
 */
void DroneEKF::jacob_F(double state_arr[], double input[], double dt) {
    (void)state_arr; (void)input; (void)dt;
    // Simplified: F is approximately identity for small dt
}

/**
 * Update covariance after prediction
 */
void DroneEKF::pred_state_cov(double F[], double process_noise_arr[], double dt) {
    // Simplified covariance update: P = P + Q*dt
    // For full EKF: P = F*P*F^T + Q
    
    Matrix12 Q = process_noise;
    
    // Simple additive noise model
    covariance = covariance + Q * (dt * dt);
}

/**
 * Update step with measurements
 * measurement: [accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, cam_x, cam_y, tof_z]
 */
void DroneEKF::update(double prediction[], double measurement[], double measurement_noise_arr[]) {
    if (!measurement) return;

    // Measurement vector
    Vector9 z;
    for (int i = 0; i < 9; ++i) {
        z(i) = measurement[i];
    }

    // Predicted measurement: h(x)
    Vector9 h;
    
    // From accelerometer: world acceleration (estimated from state derivatives)
    double roll = state(6), pitch = state(7);
    double g = 9.81;
    h(0) = -g * sin(pitch);                    // accel_x
    h(1) = g * sin(roll) * cos(pitch);         // accel_y
    h(2) = g * cos(roll) * cos(pitch);         // accel_z

    // From gyroscope: rotation rates
    h(3) = state(9);   // p
    h(4) = state(10);  // q
    h(5) = state(11);  // r

    // From camera and TOF
    h(6) = state(0);   // px (camera x)
    h(7) = state(1);   // py (camera y)
    h(8) = state(2);   // pz (TOF z)

    // Measurement residual
    Vector9 y = z - h;

    // Observation Jacobian H (9x12)
    Matrix9x12 H = Matrix9x12::Zero();

    // Accel derivatives
    H(0,7) = -g * cos(pitch);
    H(1,6) = g * cos(roll) * cos(pitch);
    H(1,7) = -g * sin(roll) * sin(pitch);
    H(2,6) = -g * sin(roll) * cos(pitch);
    H(2,7) = -g * cos(roll) * sin(pitch);

    // Gyro derivatives
    H(3,9) = 1.0;
    H(4,10) = 1.0;
    H(5,11) = 1.0;

    // Position derivatives
    H(6,0) = 1.0;
    H(7,1) = 1.0;
    H(8,2) = 1.0;

    // Measurement noise
    Matrix9 R = measurement_noise;
    if (measurement_noise_arr) {
        for (int r = 0; r < 9; ++r) {
            for (int c = 0; c < 9; ++c) {
                R(r,c) = measurement_noise_arr[r*9 + c];
            }
        }
    }

    // Innovation covariance: S = H*P*H^T + R
    Matrix9 S = H * covariance * H.Transpose() + R;

    // Kalman gain: K = P*H^T*S^-1
    Matrix9 S_inv = S.Inverse();
    BLA::Matrix<12,9> K = covariance * H.Transpose() * S_inv;

    // State update: x = x + K*y
    Vector12 dx = K * y;
    state = state + dx;

    // Normalize yaw
    state(8) = wrap_angle(state(8));

    // Covariance update: P = (I - K*H)*P
    Matrix12 I = Matrix12::Identity();
    Matrix12 KH = K * H;
    covariance = (I - KH) * covariance;

    if (prediction) {
        for (int i = 0; i < 12; ++i) {
            prediction[i] = state(i);
        }
    }
}

/**
 * Compute innovation
 */
void DroneEKF::innovation(double measurement[], double predicted_measurement[]) {
    if (!measurement || !predicted_measurement) return;

    for (int i = 0; i < 9; ++i) {
        measurement[i] = measurement[i] - predicted_measurement[i];
    }
}

/**
 * Compute innovation covariance
 */
void DroneEKF::innovation_cov(double H[], double predicted_covariance[], double measurement_noise_arr[]) {
    (void)H; (void)predicted_covariance; (void)measurement_noise_arr;
}

/**
 * Compute observation Jacobian
 */
void DroneEKF::jacobian_H(double measurement[], double dt) {
    (void)measurement; (void)dt;
}

/**
 * Compute Kalman gain
 */
void DroneEKF::kalman_gain(double prediction_covariance[], double H[], double S[]) {
    (void)prediction_covariance; (void)H; (void)S;
}

/**
 * Update state estimate
 */
void DroneEKF::update_state(double predicted_state[], double K, double y, double measurement_covariance[]) {
    (void)K; (void)y; (void)measurement_covariance;
    if (predicted_state) {
        for (int i = 0; i < 12; ++i) {
            predicted_state[i] = state(i);
        }
    }
}

/**
 * Update state covariance
 */
void DroneEKF::update_state_covariance(double measurement_covariance[], double prior_state_covariance[], double K) {
    (void)measurement_covariance; (void)prior_state_covariance; (void)K;
}