#ifndef EXTENDED_KALMAN_FILTER_H
#define EXTENDED_KALMAN_FILTER_H

/* standard libraries */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* custom math libs */
#include <BasicLinearAlgebra.h>

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like
// so:
// using namespace BLA;

typedef BLA::Matrix<3,3> Matrix3;
typedef BLA::Matrix<7,7> Matrix7;
typedef BLA::Matrix<3,7> Matrix3x7;
typedef BLA::Matrix<3>   Vector3;
typedef BLA::Matrix<4>   Vector4;
typedef BLA::Matrix<7>   Vector7;
typedef BLA::Matrix<4>   Quaternion; 

// This class makes use of quaternions, NOT Euler angles for state

class EKF
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
     * @brief Calculates the evolution of the state over time for a dynamic model
     * @param state measurements representing the model
     * @param input the control vector input to the model
     * @param dt time step
    //  * @return Computes the predicted state based on motion model dynamics
     */
    void compute_motion_model(double state[], double input[], double dt);

    /**
     * @brief Calculates the Jacobian of the dynamic model
     * @param state estimate of current model's state
     * @param input the control vector input to the model
     * @param dt time step
     */
    void compute_dynamic_jacobian(double state[], double input[], double dt);

    /**
     * @brief Updates the covariance matrix for the dynamic model
     * @param jacobian the dynamic model's jacobian
     * @param process_noise matrix that describes the noise of the dynamic model
     * @param dt time step
     */
    void update_dynamic_covariance(double jacobian[], double process_noise[], double dt);


    /**
     * @brief Updates the state estimate predicted by the dynamic model
     * by incorporating measurements from the measurement model and the
     * measurement model's noise
     * @param prediction
     * @param measurement
     * @param measurement_noise
     */
    void update(double prediction[], double measurement[], double measurement_noise[]);

    void compute_measurement_model(double measurement[], double state[]);

    void compute_innovation(double measurement[], double predicted_measurement[]);

    void compute_measurement_jacobian(double measurement[], double dt);

    void compute_innovation_jacobian(double innovation[], double dt);

    void compute_kalman_gain(double prediction_covariance[], double measurement_jacobian[], 
                                double innovation_covariance[]);

    void update_state(double predicted_state[], double kalman_gain, double innovation, 
                                double measurement_covariance[]);

    void update_state_covariance(double measurement_covariance[], double prior_state_covariance[],
                                    double kalman_gain);

    

private:

    Vector7 state;
    Matrix7 covariance;
    Matrix7 process_noise;
    Matrix3 measurement_noise;

};

#endif  // EXTENDED_KALMAN_FILTER_H
