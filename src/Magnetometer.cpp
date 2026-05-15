#include "Magnetometer.h"

/**
 * Constructor - Initialize calibration matrices with default values
 */
Magnetometer::Magnetometer()
    : hard_iron{0.0, 0.0, 0.0},
      soft_iron{1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0},
      eigen_values{0.0, 0.0, 0.0},
      eigen_vectors{1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0}
{
    // Initialize raw data buffer
    for (int i = 0; i < MAX_SAMPLES; i++) {
        mag_data_raw[i][0] = 0.0;
        mag_data_raw[i][1] = 0.0;
        mag_data_raw[i][2] = 0.0;
    }
}

/**
 * Initialize the MLX90393 magnetometer sensor
 */
bool Magnetometer::begin() {
    byte status = mlx.begin();
    
    uint8_t maxRetries = 10;
    uint8_t retryCount = 0;
    
    while (mlx.checkStatus(0) != MLX90393::STATUS_OK && retryCount < maxRetries) {
        Serial.print("Failed to connect to MLX90393. (attempt ");
        Serial.print(retryCount + 1);
        Serial.print("/");
        Serial.print(maxRetries);
        Serial.println("). Retrying...");
        retryCount++;
        delay(500);
    }
    
    if (retryCount >= maxRetries) {
        Serial.println("FATAL: MLX90393 initialization failed after max retries!");
        return false;
    }
    
    Serial.print("Magnetometer initialized with status: 0x");
    if (status < 0x10) Serial.print("0");
    Serial.println(status, BIN);
    
    return true;
}

/**
 * Configure sensor parameters
 */
void Magnetometer::configureSensor(uint8_t gainSel, uint8_t resX, uint8_t resY, uint8_t resZ,
                                   uint8_t overSampling, uint8_t digitalFiltering) {
    mlx.setGainSel(gainSel);
    mlx.setResolution(resX, resY, resZ);
    mlx.setOverSampling(overSampling);
    mlx.setDigitalFiltering(digitalFiltering);
}

/**
 * Read raw magnetometer data and store in buffer
 */
bool Magnetometer::readRawData(int num_samples) {
    if (num_samples > MAX_SAMPLES) {
        Serial.println("Error: Requested samples exceed buffer size");
        return false;
    }
    
    for (int i = 0; i < num_samples; i++) {
        if (!mlx.readData(mlx_data)) {
            Serial.print("Error reading magnetometer data at sample ");
            Serial.println(i);
            return false;
        }
        
        mag_data_raw[i][0] = mlx_data.x;
        mag_data_raw[i][1] = mlx_data.y;
        mag_data_raw[i][2] = mlx_data.z;
        
        delay(1);
    }
    
    return true;
}

/**
 * Compute hard-iron offsets as the mean of all readings
 */
BLA::Matrix<3, 1> Magnetometer::computeHardIronOffsets(int num_samples) {
    BLA::Matrix<3, 1> offsets{0.0, 0.0, 0.0};
    
    float total_x = 0.0, total_y = 0.0, total_z = 0.0;
    
    for (int i = 0; i < num_samples; i++) {
        total_x += mag_data_raw[i][0];
        total_y += mag_data_raw[i][1];
        total_z += mag_data_raw[i][2];
    }
    
    offsets(0) = total_x / num_samples;
    offsets(1) = total_y / num_samples;
    offsets(2) = total_z / num_samples;
    
    hard_iron = offsets;
    
    return offsets;
}

/**
 * Compute covariance matrix from centered data
 */
BLA::Matrix<3, 3> Magnetometer::computeCovariance(float mag_data_centered[MAX_SAMPLES][3], int num_samples) {
    BLA::Matrix<3, 3> covariance{0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0,
                                 0.0, 0.0, 0.0};
    
    for (int i = 0; i < num_samples; i++) {
        BLA::Matrix<3, 1> sample{mag_data_centered[i][0],
                                  mag_data_centered[i][1],
                                  mag_data_centered[i][2]};
        
        BLA::Matrix<1, 3> sample_T = ~sample;
        BLA::Matrix<3, 3> outer_product = sample * sample_T;
        
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                covariance(row, col) += outer_product(row, col);
            }
        }
    }
    
    return covariance;
}

/**
 * Perform eigenvalue decomposition on 3x3 symmetric matrix
 * using closed-form analytical solution
 */
void Magnetometer::eigenDecomposition3x3(const BLA::Matrix<3, 3>& matrix,
                                         BLA::Matrix<3, 1>& eigenvalues,
                                         BLA::Matrix<3, 3>& eigenvectors) {
    // Extract symmetric matrix components
    float a = matrix(0, 0);
    float b = matrix(1, 1);
    float c = matrix(2, 2);
    float d = matrix(0, 1);
    float e = matrix(1, 2);
    float f = matrix(0, 2);
    
    // Compute intermediate values for characteristic polynomial
    float x1 = a * a + b * b + c * c - a * b - a * c - b * c + 3.0 * (d * d + f * f + e * e);
    
    float x2 = -(2.0 * a - b - c) * (2.0 * b - a - c) * (2.0 * c - a - b) +
               9.0 * ((2.0 * c - a - b) * d * d + (2.0 * b - a - c) * f * f + (2.0 * a - b - c) * e * e) -
               54.0 * (d * e * f);
    
    // Compute phase angle
    float phi = 0.0;
    float discriminant = 4.0 * x1 * x1 * x1 - x2 * x2;
    
    if (discriminant < 0) {
        Serial.println("Warning: Negative discriminant in eigenvalue decomposition");
        discriminant = 0;
    }
    
    if (x2 > 0) {
        phi = atan(sqrt(discriminant) / x2);
    } else if (x2 < 0) {
        phi = atan(sqrt(discriminant) / x2) + PI;
    } else if (x2 == 0) {
        phi = PI / 2.0;
    }
    
    // Compute eigenvalues
    float lambda_1 = (a + b + c - 2.0 * sqrt(x1) * cos(phi / 3.0)) / 3.0;
    float lambda_2 = (a + b + c + 2.0 * sqrt(x1) * cos((phi - PI) / 3.0)) / 3.0;
    float lambda_3 = (a + b + c + 2.0 * sqrt(x1) * cos((phi + PI) / 3.0)) / 3.0;
    
    eigenvalues(0) = lambda_1;
    eigenvalues(1) = lambda_2;
    eigenvalues(2) = lambda_3;
    
    // Compute eigenvectors using eigenvector formula for Hermitian matrices
    // Set s1 = 1.0 and solve for m and s3
    
    float m1, m2, m3;
    
    // Avoid division by zero
    float denom1 = f * (b - lambda_1) - d * e;
    float denom2 = f * (b - lambda_2) - d * e;
    float denom3 = f * (b - lambda_3) - d * e;
    
    if (fabs(denom1) > 1e-6) {
        m1 = (d * (c - lambda_1) - e * f) / denom1;
    } else {
        m1 = 0.0;
    }
    
    if (fabs(denom2) > 1e-6) {
        m2 = (d * (c - lambda_2) - e * f) / denom2;
    } else {
        m2 = 0.0;
    }
    
    if (fabs(denom3) > 1e-6) {
        m3 = (d * (c - lambda_3) - e * f) / denom3;
    } else {
        m3 = 0.0;
    }
    
    // Compute s3 components
    float v1_1 = (lambda_1 - c - e * m1) / (f + 1e-10);
    float v2_1 = (lambda_2 - c - e * m2) / (f + 1e-10);
    float v3_1 = (lambda_3 - c - e * m3) / (f + 1e-10);
    
    // Build eigenvector matrix (eigenvectors as columns)
    eigenvectors(0, 0) = v1_1;  eigenvectors(0, 1) = v2_1;  eigenvectors(0, 2) = v3_1;
    eigenvectors(1, 0) = m1;    eigenvectors(1, 1) = m2;    eigenvectors(1, 2) = m3;
    eigenvectors(2, 0) = 1.0;   eigenvectors(2, 1) = 1.0;   eigenvectors(2, 2) = 1.0;
}

/**
 * Compute soft-iron calibration matrix using eigenvalue decomposition
 */
BLA::Matrix<3, 3> Magnetometer::computeSoftIronOffsets(int num_samples) {
    // Create centered data (hard-iron offsets removed)
    float mag_data_centered[MAX_SAMPLES][3];
    
    for (int i = 0; i < num_samples; i++) {
        mag_data_centered[i][0] = mag_data_raw[i][0] - hard_iron(0);
        mag_data_centered[i][1] = mag_data_raw[i][1] - hard_iron(1);
        mag_data_centered[i][2] = mag_data_raw[i][2] - hard_iron(2);
    }
    
    // Compute covariance matrix
    BLA::Matrix<3, 3> covariance = computeCovariance(mag_data_centered, num_samples);
    
    // Perform eigenvalue decomposition
    BLA::Matrix<3, 1> eigenvals{0.0, 0.0, 0.0};
    BLA::Matrix<3, 3> eigenvecs{1.0, 0.0, 0.0,
                                 0.0, 1.0, 0.0,
                                 0.0, 0.0, 1.0};
    
    eigenDecomposition3x3(covariance, eigenvals, eigenvecs);
    
    eigen_values = eigenvals;
    eigen_vectors = eigenvecs;
    
    // Build eigenvector matrix V (normalize eigenvectors)
    BLA::Matrix<3, 3> V = eigenvecs;
    
    // Compute V^T (since V is orthogonal, V^-1 = V^T)
    BLA::Matrix<3, 3> V_inv = ~V;
    
    // Compute diagonal matrix D^-0.5
    float c1 = 1.0 / sqrt(fabs(eigenvals(0)) + 1e-10);
    float c2 = 1.0 / sqrt(fabs(eigenvals(1)) + 1e-10);
    float c3 = 1.0 / sqrt(fabs(eigenvals(2)) + 1e-10);
    
    BLA::Matrix<3, 3> D_inv_sqrt{c1,  0.0, 0.0,
                                 0.0, c2,  0.0,
                                 0.0, 0.0, c3};
    
    // Compute soft-iron correction matrix: C = V * D^-0.5 * V^-1
    BLA::Matrix<3, 3> soft_iron_matrix = V * D_inv_sqrt * V_inv;
    
    soft_iron = soft_iron_matrix;
    
    return soft_iron_matrix;
}

/**
 * Read and return calibrated magnetometer data
 */
BLA::Matrix<3, 1> Magnetometer::readCalibratedData() {
    if (!mlx.readData(mlx_data)) {
        Serial.println("Error reading magnetometer data");
        BLA::Matrix<3, 1> error{0.0, 0.0, 0.0};
        return error;
    }
    
    BLA::Matrix<3, 1> mag_raw{mlx_data.x, mlx_data.y, mlx_data.z};
    BLA::Matrix<3, 1> mag_corrected = soft_iron * (mag_raw - hard_iron);
    
    return mag_corrected;
}

/**
 * Compute magnetic heading from calibrated data
 */
float Magnetometer::computeHeading(const BLA::Matrix<3, 1>& mag_data, float mag_declination) {
    float heading_rad = atan2(mag_data(1), mag_data(0));
    float heading_deg = heading_rad * 180.0 / PI;
    float heading_deg_corrected = heading_deg + mag_declination;
    
    // Normalize to 0-360 range
    if (heading_deg_corrected > 360.0) {
        heading_deg_corrected -= 360.0;
    } else if (heading_deg_corrected < 0.0) {
        heading_deg_corrected += 360.0;
    }
    
    return heading_deg_corrected;
}

/**
 * Get current calibration data
 */
Magnetometer::CalibrationData Magnetometer::getCalibrationData() const {
    CalibrationData calib;
    calib.hard_iron_offsets = hard_iron;
    calib.soft_iron_matrix = soft_iron;
    calib.eigen_values = eigen_values;
    calib.eigen_vectors = eigen_vectors;
    return calib;
}

/**
 * Set calibration data
 */
void Magnetometer::setCalibrationData(const CalibrationData& calib_data) {
    hard_iron = calib_data.hard_iron_offsets;
    soft_iron = calib_data.soft_iron_matrix;
    eigen_values = calib_data.eigen_values;
    eigen_vectors = calib_data.eigen_vectors;
}
