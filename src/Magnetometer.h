#ifndef MAGNETOMETER_H
#define MAGNETOMETER_H

#include <Wire.h>
#include <MLX90393.h>
#include "BasicLinearAlgebra.h"
#include <vector>

using namespace BLA;

/**
 * @class Magnetometer
 * @brief Handles magnetometer calibration and data acquisition from MLX90393 sensor
 * 
 * Performs hard-iron and soft-iron calibration of the magnetometer using eigenvalue
 * decomposition. Provides methods to read raw magnetometer data, compute calibration
 * matrices, and return calibrated magnetic heading.
 */
class Magnetometer {
public:
    static constexpr int MAX_SAMPLES = 1000;
    static constexpr float PI = 3.14159265358979323;
    
    /**
     * @struct CalibrationData
     * @brief Stores calibration results from magnetometer
     */
    struct CalibrationData {
        BLA::Matrix<3, 1> hard_iron_offsets;
        BLA::Matrix<3, 3> soft_iron_matrix;
        BLA::Matrix<3, 1> eigen_values;
        BLA::Matrix<3, 3> eigen_vectors;
    };

    /**
     * Constructor - initializes the magnetometer
     */
    Magnetometer();

    /**
     * Initialize the MLX90393 magnetometer sensor
     * @return true if initialization successful, false otherwise
     */
    bool begin();

    /**
     * Configure sensor gain, resolution, oversampling, and filtering
     * @param gainSel Gain selection (0-7)
     * @param resX X-axis resolution
     * @param resY Y-axis resolution
     * @param resZ Z-axis resolution
     * @param overSampling Oversampling factor (0-3)
     * @param digitalFiltering Digital filter setting (0-7)
     */
    void configureSensor(uint8_t gainSel, uint8_t resX, uint8_t resY, uint8_t resZ,
                         uint8_t overSampling, uint8_t digitalFiltering);

    /**
     * Read raw magnetometer data from sensor and store in internal buffer
     * @param num_samples Number of samples to read
     * @return true if read successful, false otherwise
     */
    bool readRawData(int num_samples);

    /**
     * Compute hard-iron calibration offsets from buffered data
     * Hard-iron offsets are computed as the mean of all magnetometer readings
     * @return Matrix<3,1> containing X, Y, Z offsets
     */
    BLA::Matrix<3, 1> computeHardIronOffsets(int num_samples);

    /**
     * Compute soft-iron calibration matrix using eigenvalue decomposition
     * Performs 3x3 eigen decomposition on covariance matrix of centered data
     * @param num_samples Number of samples to use in computation
     * @return 3x3 soft-iron correction matrix
     */
    BLA::Matrix<3, 3> computeSoftIronOffsets(int num_samples);

    /**
     * Read a single magnetometer sample and apply calibration
     * @return 3x1 Matrix containing calibrated X, Y, Z magnetic field
     */
    BLA::Matrix<3, 1> readCalibratedData();

    /**
     * Compute magnetic heading from calibrated magnetometer data
     * @param mag_data Calibrated 3x1 magnetometer vector
     * @param mag_declination Magnetic declination in degrees
     * @return Magnetic heading in degrees (0-360)
     */
    float computeHeading(const BLA::Matrix<3, 1>& mag_data, float mag_declination = 0.0);

    /**
     * Get current calibration data
     * @return CalibrationData structure containing all calibration matrices
     */
    CalibrationData getCalibrationData() const;

    /**
     * Set calibration data (for loading previously calibrated values)
     * @param calib_data Calibration data to apply
     */
    void setCalibrationData(const CalibrationData& calib_data);

private:
    MLX90393 mlx;
    MLX90393::txyz mlx_data;

    // Calibration matrices
    BLA::Matrix<3, 1> hard_iron;
    BLA::Matrix<3, 3> soft_iron;
    BLA::Matrix<3, 1> eigen_values;
    BLA::Matrix<3, 3> eigen_vectors;

    // Raw data buffer
    float mag_data_raw[MAX_SAMPLES][3];

    /**
     * Compute eigenvalues and eigenvectors of a 3x3 symmetric matrix
     * using closed-form analytical solution
     * @param matrix 3x3 symmetric covariance matrix
     * @param eigenvalues Output 3x1 matrix of eigenvalues
     * @param eigenvectors Output 3x3 matrix of eigenvectors (as columns)
     */
    void eigenDecomposition3x3(const BLA::Matrix<3, 3>& matrix,
                               BLA::Matrix<3, 1>& eigenvalues,
                               BLA::Matrix<3, 3>& eigenvectors);

    /**
     * Compute covariance matrix from centered magnetometer data
     * @param mag_data_centered Magnetometer data with hard-iron offsets removed
     * @param num_samples Number of samples
     * @return 3x3 covariance matrix
     */
    BLA::Matrix<3, 3> computeCovariance(float mag_data_centered[MAX_SAMPLES][3], int num_samples);
};

#endif // MAGNETOMETER_H
