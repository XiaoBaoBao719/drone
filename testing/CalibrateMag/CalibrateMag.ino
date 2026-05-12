#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <Arduino.h>
#include "BasicLinearAlgebra.h"

#include <vector>

using namespace std;
using namespace BLA;

/* Hard-iron calibration offsets */

BLA::Matrix<3> hard_iron = { 0.0,   0.0,    0.0 };
BLA::Matrix<3, 3> soft_iron = { 1.0,   0.0,   0.0,
                                0.0,   1.0,   0.0,
                                0.0,   0.0,   1.0 };

BLA::Matrix<3> eigen_values = { 0.0, 0.0, 0.0 };    // X, Y, Z eigenvalues of the covariance matrix of the magnetometer data
BLA::Matrix<3> eigen_vectors = { 0.0, 0.0, 0.0};   // X, Y, Z eigenvectors of the covariance matrix of the magnetometer data

const int MAX_SAMPLES = 1000;
float mag_data[MAX_SAMPLES][3]; // X, Y, Z magnetometer data for each sample
BLA::Matrix<MAX_SAMPLES, 3> mag_data_matrix; // Matrix representation of magnetometer data for covariance computation
// mag_data_matrix.Fill(0.0);

/**
 * Magnetic declination from magnetic-declination.com
 * East is positive (+), west is negative (-)
 * mag_decl = (+/-)(deg + min/60 + sec/3600)
 * Set to 0 to get magnetic heading instead of geographic heading
 */
constexpr float mag_decl = 0.0;

MLX90393 mlx;
MLX90393::txyz mlx_data; //Create a structure, called data, of four floats (t, x, y, and z)

/**
 * Reads raw magnetometer data from the MLX90393 sensor and stores it in the provided 2D array.
 * The array should have dimensions [MAX_SAMPLES][3] to hold the X, Y, and Z magnetometer readings for each sample.
 */
void readMagRaw(float mag_data[MAX_SAMPLES][3], int num_samples) {
  
  mlx.readData(mlx_data);

  for (int i = 0; i < num_samples; i++) {
    mag_data[i][0] = mlx_data.x;
    mag_data[i][1] = mlx_data.y;
    mag_data[i][2] = mlx_data.z;

    delay(1);
  }
}



/**
 * Collects magnetometer data and computes hard-iron offsets as the average of the magnetometer readings
 * Returns a matrix of magnetometer hard-iron bias offsets for each axis (X, Y, Z)
 */
BLA::Matrix<3, 1> computeHardIronOffsets(float mag_data_raw[MAX_SAMPLES][3], int num_samples) {
  BLA::Matrix<3, 1> offsets; // Matrix representation of magnetometer data for covariance computation

  float total_x, total_y, total_z = 0;

  for (int i = 0; i < num_samples; i++) {
    total_x += mag_data[i][0];
    total_y += mag_data[i][1];
    total_z += mag_data[i][2];
  }

  offsets(0) = total_x / num_samples;
  offsets(1) = total_y / num_samples;
  offsets(2) = total_z / num_samples;

  // hard_iron(0) = total_x[0];
  // hard_iron(1) = total_y[1];
  // hard_iron(2) = total_z[2];

  return offsets;
}

/**
 * mag_data_raw is a 3 x num_samples matrix of magnetometer data with hard-iron offsets removed
 */
void computeSoftIronOffsets(float mag_data_raw[MAX_SAMPLES][3], int num_samples) {

  BLA::Matrix<3,3> cov_sums;
  cov_sums.Fill(0);

  std::vector<BLA::Matrix<3, 3> > cov_data_sets;

  // Compute covariance matrix of magnetometer data
  // Compute eigenvalues and eigenvectors of covariance matrix
  // Compute soft-iron correction matrix from eigenvalues and eigenvectors

  // Create magnetometer data matrix with hard-iron offsets removed
  // std::vector<BLA::Matrix<3, 1> > mag_data_offsets_removed; // Vector of magnetometer data with hard-iron offsets removed for each sample
  BLA::Matrix<MAX_SAMPLES, 3, BLA::Matrix <3, 1>> mag_data_offsets_removed;

  for (int i = 0; i < num_samples; i++) {
    BLA::Matrix<3, 1> offsets_removed;
    offsets_removed(0) = mag_data_raw[i][0] - hard_iron(0);
    offsets_removed(1) = mag_data_raw[i][1] - hard_iron(1);
    offsets_removed(2) = mag_data_raw[i][2] - hard_iron(2);
    // mag_data_offsets_removed.push_back(offsets_removed);
    mag_data_offsets_removed(i) = offsets_removed;
  }

  // Traverse each data point and compute eigenvalues and eigenvectors of the covariance matrix of the magnetometer data
  for (int i = 0; i < num_samples; i++) {
    BLA::Matrix<3, 1> offsets_removed = mag_data_offsets_removed(i);

    BLA::Matrix<1, 3> offsets_removed_T = ~offsets_removed;  // calculate the transpose
    BLA::Matrix<3, 3> cov = offsets_removed * offsets_removed_T;  // calculate the covariance matrix

    // cov_data_sets.push_back(cov);
    cov_sums(0, 0) += cov(0, 0);
    cov_sums(0, 1) += cov(0, 1);
    cov_sums(0, 2) += cov(0, 2);
    cov_sums(1, 0) += cov(1, 0);
    cov_sums(1, 1) += cov(1, 1);
    cov_sums(1, 2) += cov(1, 2);
    cov_sums(2, 0) += cov(2, 0);
    cov_sums(2, 1) += cov(2, 1);
    cov_sums(2, 2) += cov(2, 2);
  }

  // Now, calculate the trace of the 3x3 covariance sums
  float trace = cov_sums(0, 0) + cov_sums(1, 1) + cov_sums(2, 2);

  // Calculate the determinant of the 3x3 covariance matrix
  /**
   * 
   */
  float determinant = cov_sums(0, 0)*cov_sums(1, 1)*cov_sums(2, 2) + 
                      cov_sums(0, 1)*cov_sums(1, 2)*cov_sums(0, 3) +
                      cov_sums(0, 2)*cov_sums(1, 0)*cov_sums(2, 1) -
                      cov_sums(0, 2)*cov_sums(1, 1)*cov_sums(2, 0) -
                      cov_sums(0, 1)*cov_sums(1, 0)*cov_sums(2, 2) -
                      cov_sums(0, 0)*cov_sums(1, 2)*cov_sums(2, 1);

  // Solve for eigenvalues from det and trace of characteristic equation
  //  C(x)  =  x^2 - tr(B)x + det(B) = 0
  //  C(x) = (x-x1)(x-x2)      where  x1 and x2 are the eigenvalues, use quadratic formula to get roots

  float x1 = ( -trace + sqrt( trace * trace - 4.0 * determinant) ) / 2.0;
  float x2 = ( -trace - sqrt( trace * trace - 4.0 * determinant) ) / 2.0;
  float scale = sqrt(x1 / x2);

  // Use the derived eigenvalues to calculate the ellipsoid skew
  
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("MLX90393 Calibration");
  Wire.begin();
  //Connect to sensor with I2C address jumpers set: A1 = 1
  //Use DRDY pin connected to A3
  //Returns byte containing status bytes
  byte status = mlx.begin();

  uint8_t maxRetries = 0;
  uint8_t retryCount = 0;
  while ( mlx.checkStatus(0) != MLX90393::STATUS_OK ) {
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
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);  
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  Serial.print("Magnetometer initialized with status: 0x");
  if(status < 0x10) Serial.print("0"); //Pretty output
  Serial.println(status, BIN);

  // Configure sensor
  mlx.setGainSel(1);
  mlx.setResolution(0, 0, 0); //x, y, z
  mlx.setOverSampling(0);
  mlx.setDigitalFiltering(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}