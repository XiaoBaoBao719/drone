#ifndef DRONE_EKF_H
#define DRONE_EKF_H

/* standard libraries */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* custom lin alg library */
#include <BasicLinearAlgebra.h>

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like
// so:
// using namespace BLA;

typedef BLA::Matrix<3, 3>   Matrix3;
typedef BLA::Matrix<7, 7>   Matrix7;
typedef BLA::Matrix<3, 7>   Matrix3x7;
typedef BLA::Matrix<12, 12> Matrix12;
typedef BLA::Matrix<3>      Vector3;
typedef BLA::Matrix<4>      Vector4;
typedef BLA::Matrix<7>      Vector7;
typedef BLA::Matrix<9>      Vector9;
typedef BLA::Matrix<12>     Vector12;
typedef BLA::Matrix<4>      Quaternion;
// This class makes use of quaternions, NOT Euler angles for state

class DroneEKF
{
public:
    /**
     * @brief Constructor for the extended kalman filter
     */
    EKF(float accel_data[]);

    /**
     * @brief Destructor
     */
    ~EKF() = default;

    /**
     * @brief Initializes the EKF with an initial state and uncertainty
     * @param initial_state the initial state of the filter as a quaterion
     * @param initial_covariance the initial uncertainty for the filter's dynamic model
     */
    void initialize(const double initial_state[], const double  initial_covariance[]); 
    
    /**
     * @brief Performs a single iteration of EKF state estimation and returns an EKF
     *  estimated state and covariance for a given control input and set of measurements
     * @param prior_state state predicted by last EKF iteration
     * @param prior_covariance uncertainty from last EKF iteration
     * @param input used in the dynamic model predict step
     * @param measurement used for the observation model update step
     * @param dt time step
     */
    void ekf_estimate(double prior_state[], double prior_covariance[], 
                        double input[], double measurement[], double dt);

    /**
     * @brief Takes the prior belief, the propogated state of the dynamic model, 
     * and the dynamic model's process noise and computes the predicted belief
     * @param prior The predicted state from the most recent iteration of estimation
     * @param prior_covariance The most recent predicted state's uncertainty
     */
    void predict(double prior[], double prior_covariance[]);

    /**
     * @brief Calculates a state estimate based on the motion model
     * @param state measurements representing the model
     * @param input the control vector input to the model
     * @param dt time step
     * Computes the predicted state based on motion model's dynamics
     */
    void pred_state(double state[], double input[], double dt);

    /**
     * @brief Calculates the Jacobian of the motion model
     * @param state estimate of current model's state
     * @param input the control vector input to the model
     * @param dt time step
     */
    void jacob_F(double state[], double input[], double dt);

    /**
     * @brief Updates the covariance matrix for the dynamic model
     * @param jacobian the dynamic model's jacobian
     * @param process_noise matrix that describes the noise of the dynamic model
     * @param dt time step
     */
    void pred_state_cov(double F[], double process_noise[], double dt);


    /**
     * @brief Updates the state estimate predicted by the dynamic model
     * by incorporating measurements from the measurement model and the
     * measurement model's noise
     * @param prediction
     * @param measurement
     * @param measurement_noise
     */
    void update(double prediction[], double measurement[], double measurement_noise[]);

    // void compute_measurement_model(double measurement[], double state[]);

    /**
     * @brief Calculates the measurement innovation (residual) which reflects
     * the deviation between the measurement and the predicted state
     * @param measurement the reading from a sensor
     * @param predicted_measurement estimated reading for sensor based on motion model
     */
    void innovation(double measurement[], double predicted_measurement[]);

    /**
     * @brief Calculates the covariance (uncertainty) in the innovation
     * @param jacobian H_observation jacobian
     * @param predicted_covariance estimated covariance from the predict step
     * @param measurement_noise R estimated covariance of the sensor
     */
    void innovation_cov(double H[], double predicted_covariance[], double measurement_noise[]);

    /**
     * @brief Calculates the jacobian of the Observation matrix
     */
    void jacobian_H(double measurement[], double dt);

    // void compute_innovation_jacobian(double innovation[], double dt);

    /**
     * @brief Computes the Kalman gain balancing factor that optimally minimizes
     * the posteriori error covariance
     */
    void kalman_gain(double prediction_covariance[], double H[], 
                                double S[]);

    /**
     * @brief Updates the mean state estimate
     * @param K Kalman gain
     * @param y Innovation (residual)
     */
    void update_state(double predicted_state[], double K, double y, 
                                double measurement_covariance[]);

    /**
     * @brief Updates the mean covariance of the state estimate
     * @param K Kalman gain
     */
    void update_state_covariance(double measurement_covariance[], double prior_state_covariance[],
                                    double K);


private:

    Vector12 state;
    Matrix7 covariance;

    Vector9 measurements;
    Matrix7 process_noise;
    Matrix3 measurement_noise;

};

#endif  // DRONE_EKF_H
