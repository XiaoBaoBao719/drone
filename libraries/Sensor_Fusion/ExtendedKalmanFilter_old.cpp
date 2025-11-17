#include <stdio.h>
#include "EKF.h"

// #include "ExtendedKalmanFilter.h"
#include <iostream>
#include <cmath>
#include <algorithm>

ExtendedKalmanFilter::ExtendedKalmanFilter(int state_dim, int obs_dim, int control_dim)
    : state_dim_(state_dim), obs_dim_(obs_dim), control_dim_(control_dim), initialized_(false) {
    
    if (state_dim <= 0 || obs_dim <= 0) {
        throw std::invalid_argument("State and observation dimensions must be positive");
    }
    
    // Initialize state and covariance with zeros
    state_.resize(state_dim_, 0.0);
    covariance_.resize(state_dim_, std::vector<double>(state_dim_, 0.0));
    
    // Initialize noise matrices with identity matrices
    process_noise_ = createIdentityMatrix(state_dim_);
    measurement_noise_ = createIdentityMatrix(obs_dim_);
    
    // Initialize innovation vectors
    innovation_.resize(obs_dim_, 0.0);
    innovation_covariance_.resize(obs_dim_, std::vector<double>(obs_dim_, 0.0));
}

void ExtendedKalmanFilter::initialize(const std::vector<double>& initial_state, 
                                     const std::vector<std::vector<double>>& initial_covariance) {
    validateVectorDimension(initial_state, state_dim_, "initial_state");
    validateMatrixDimensions(initial_covariance, state_dim_, state_dim_, "initial_covariance");
    
    state_ = initial_state;
    covariance_ = initial_covariance;
    initialized_ = true;
}

void ExtendedKalmanFilter::setStateTransitionFunction(const StateTransitionFunc& func) {
    state_transition_func_ = func;
}

void ExtendedKalmanFilter::setObservationFunction(const ObservationFunc& func) {
    observation_func_ = func;
}

void ExtendedKalmanFilter::setStateTransitionJacobian(const JacobianFunc& func) {
    state_transition_jacobian_ = func;
}

void ExtendedKalmanFilter::setObservationJacobian(const ObservationJacobianFunc& func) {
    observation_jacobian_ = func;
}

void ExtendedKalmanFilter::setProcessNoiseCovariance(const std::vector<std::vector<double>>& Q) {
    validateMatrixDimensions(Q, state_dim_, state_dim_, "Process noise covariance Q");
    process_noise_ = Q;
}

void ExtendedKalmanFilter::setMeasurementNoiseCovariance(const std::vector<std::vector<double>>& R) {
    validateMatrixDimensions(R, obs_dim_, obs_dim_, "Measurement noise covariance R");
    measurement_noise_ = R;
}

void ExtendedKalmanFilter::predict(const std::vector<double>& control_input, double dt) {
    if (!initialized_) {
        throw std::runtime_error("Filter must be initialized before prediction");
    }
    
    if (!state_transition_func_ || !state_transition_jacobian_) {
        throw std::runtime_error("State transition function and Jacobian must be set");
    }
    
    std::vector<double> control = control_input;
    if (control.empty() && control_dim_ > 0) {
        control.resize(control_dim_, 0.0);
    }
    
    // Predict state: x_k|k-1 = f(x_k-1|k-1, u_k, dt)
    state_ = state_transition_func_(state_, control, dt);
    
    // Compute Jacobian: F_k = ∂f/∂x evaluated at x_k-1|k-1
    auto F = state_transition_jacobian_(state_, control, dt);
    validateMatrixDimensions(F, state_dim_, state_dim_, "State transition Jacobian F");
    
    // Predict covariance: P_k|k-1 = F_k * P_k-1|k-1 * F_k^T + Q_k
    auto F_T = transposeMatrix(F);
    auto temp = multiplyMatrices(covariance_, F_T);
    auto FPF_T = multiplyMatrices(F, temp);
    covariance_ = addMatrices(FPF_T, process_noise_);
}

void ExtendedKalmanFilter::update(const std::vector<double>& measurement) {
    if (!initialized_) {
        throw std::runtime_error("Filter must be initialized before update");
    }
    
    if (!observation_func_ || !observation_jacobian_) {
        throw std::runtime_error("Observation function and Jacobian must be set");
    }
    
    validateVectorDimension(measurement, obs_dim_, "measurement");
    
    // Predict measurement: z_k|k-1 = h(x_k|k-1)
    auto predicted_measurement = observation_func_(state_);
    validateVectorDimension(predicted_measurement, obs_dim_, "predicted_measurement");
    
    // Compute innovation: y_k = z_k - z_k|k-1
    innovation_ = subtractVectors(measurement, predicted_measurement);
    
    // Compute observation Jacobian: H_k = ∂h/∂x evaluated at x_k|k-1
    auto H = observation_jacobian_(state_);
    validateMatrixDimensions(H, obs_dim_, state_dim_, "Observation Jacobian H");
    
    // Compute innovation covariance: S_k = H_k * P_k|k-1 * H_k^T + R_k
    auto H_T = transposeMatrix(H);
    auto temp = multiplyMatrices(covariance_, H_T);
    auto HPH_T = multiplyMatrices(H, temp);
    innovation_covariance_ = addMatrices(HPH_T, measurement_noise_);
    
    // Compute Kalman gain: K_k = P_k|k-1 * H_k^T * S_k^-1
    auto S_inv = invertMatrix(innovation_covariance_);
    auto PH_T = multiplyMatrices(covariance_, H_T);
    auto K = multiplyMatrices(PH_T, S_inv);
    
    // Update state: x_k|k = x_k|k-1 + K_k * y_k
    auto K_y = multiplyMatrixVector(K, innovation_);
    state_ = addVectors(state_, K_y);
    
    // Update covariance: P_k|k = (I - K_k * H_k) * P_k|k-1
    auto I = createIdentityMatrix(state_dim_);
    auto KH = multiplyMatrices(K, H);
    auto I_minus_KH = subtractMatrices(I, KH);
    covariance_ = multiplyMatrices(I_minus_KH, covariance_);
}

// Matrix operation helper functions
std::vector<std::vector<double>> ExtendedKalmanFilter::multiplyMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) {
    
    int rows_A = A.size();
    int cols_A = A[0].size();
    int cols_B = B[0].size();
    
    std::vector<std::vector<double>> result(rows_A, std::vector<double>(cols_B, 0.0));
    
    for (int i = 0; i < rows_A; ++i) {
        for (int j = 0; j < cols_B; ++j) {
            for (int k = 0; k < cols_A; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::addMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) {
    
    int rows = A.size();
    int cols = A[0].size();
    
    std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::subtractMatrices(
    const std::vector<std::vector<double>>& A, 
    const std::vector<std::vector<double>>& B) {
    
    int rows = A.size();
    int cols = A[0].size();
    
    std::vector<std::vector<double>> result(rows, std::vector<double>(cols));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::transposeMatrix(
    const std::vector<std::vector<double>>& A) {
    
    int rows = A.size();
    int cols = A[0].size();
    
    std::vector<std::vector<double>> result(cols, std::vector<double>(rows));
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[j][i] = A[i][j];
        }
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::invertMatrix(
    const std::vector<std::vector<double>>& A) {
    
    int n = A.size();
    
    // Create augmented matrix [A | I]
    std::vector<std::vector<double>> augmented(n, std::vector<double>(2 * n, 0.0));
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            augmented[i][j] = A[i][j];
        }
        augmented[i][n + i] = 1.0;  // Identity matrix
    }
    
    // Gauss-Jordan elimination
    for (int i = 0; i < n; ++i) {
        // Find pivot
        int pivot_row = i;
        for (int k = i + 1; k < n; ++k) {
            if (std::abs(augmented[k][i]) > std::abs(augmented[pivot_row][i])) {
                pivot_row = k;
            }
        }
        
        // Swap rows
        if (pivot_row != i) {
            std::swap(augmented[i], augmented[pivot_row]);
        }
        
        // Check for singular matrix
        if (std::abs(augmented[i][i]) < 1e-10) {
            throw std::runtime_error("Matrix is singular and cannot be inverted");
        }
        
        // Scale pivot row
        double pivot = augmented[i][i];
        for (int j = 0; j < 2 * n; ++j) {
            augmented[i][j] /= pivot;
        }
        
        // Eliminate column
        for (int k = 0; k < n; ++k) {
            if (k != i) {
                double factor = augmented[k][i];
                for (int j = 0; j < 2 * n; ++j) {
                    augmented[k][j] -= factor * augmented[i][j];
                }
            }
        }
    }
    
    // Extract inverse matrix
    std::vector<std::vector<double>> inverse(n, std::vector<double>(n));
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            inverse[i][j] = augmented[i][n + j];
        }
    }
    
    return inverse;
}

std::vector<double> ExtendedKalmanFilter::multiplyMatrixVector(
    const std::vector<std::vector<double>>& A, 
    const std::vector<double>& v) {
    
    int rows = A.size();
    int cols = A[0].size();
    
    std::vector<double> result(rows, 0.0);
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i] += A[i][j] * v[j];
        }
    }
    
    return result;
}

std::vector<double> ExtendedKalmanFilter::subtractVectors(
    const std::vector<double>& a, 
    const std::vector<double>& b) {
    
    int size = a.size();
    std::vector<double> result(size);
    
    for (int i = 0; i < size; ++i) {
        result[i] = a[i] - b[i];
    }
    
    return result;
}

std::vector<double> ExtendedKalmanFilter::addVectors(
    const std::vector<double>& a, 
    const std::vector<double>& b) {
    
    int size = a.size();
    std::vector<double> result(size);
    
    for (int i = 0; i < size; ++i) {
        result[i] = a[i] + b[i];
    }
    
    return result;
}

std::vector<std::vector<double>> ExtendedKalmanFilter::createIdentityMatrix(int size) {
    std::vector<std::vector<double>> identity(size, std::vector<double>(size, 0.0));
    
    for (int i = 0; i < size; ++i) {
        identity[i][i] = 1.0;
    }
    
    return identity;
}

void ExtendedKalmanFilter::validateMatrixDimensions(
    const std::vector<std::vector<double>>& matrix, 
    int expected_rows, 
    int expected_cols, 
    const std::string& matrix_name) {
    
    if (matrix.size() != static_cast<size_t>(expected_rows)) {
        throw std::invalid_argument(matrix_name + " has incorrect number of rows");
    }
    
    if (!matrix.empty() && matrix[0].size() != static_cast<size_t>(expected_cols)) {
        throw std::invalid_argument(matrix_name + " has incorrect number of columns");
    }
}

void ExtendedKalmanFilter::validateVectorDimension(
    const std::vector<double>& vector, 
    int expected_size, 
    const std::string& vector_name) {
    
    if (vector.size() != static_cast<size_t>(expected_size)) {
        throw std::invalid_argument(vector_name + " has incorrect dimension");
    }
}