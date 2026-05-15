#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <Arduino.h>
#include "BasicLinearAlgebra.h"

#include <vector>

using namespace std;
using namespace BLA;

// constexpr float PI = 3.14159265358979323;

/* Hard-iron calibration offsets */

BLA::Matrix<3> hard_iron = { 0.0,   0.0,    0.0 };
BLA::Matrix<3, 3> soft_iron = { 1.0,   0.0,   0.0,
                                0.0,   1.0,   0.0,
                                0.0,   0.0,   1.0 };

BLA::Matrix<3> eigen_values = { 0.0, 0.0, 0.0 };    // X, Y, Z eigenvalues of the covariance matrix of the magnetometer data
BLA::Matrix<3> eigen_vectors = { 0.0, 0.0, 0.0};   // X, Y, Z eigenvectors of the covariance matrix of the magnetometer data

const int MAX_SAMPLES = 256;
float mag_data[MAX_SAMPLES][3]; // X, Y, Z magnetometer data for each sample
// BLA::Matrix<MAX_SAMPLES, 3> mag_data_matrix; // Matrix representation of magnetometer data for covariance computation - removed to save RAM
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
  for (int i = 0; i < num_samples; i++) {
    mlx.readData(mlx_data);
    mag_data[i][0] = mlx_data.x;
    mag_data[i][1] = mlx_data.y;
    mag_data[i][2] = mlx_data.z;

    Serial.print("Sample "); Serial.print(i + 1);
    Serial.print(": X = ");  Serial.print(mag_data[i][0]);
    Serial.print(", Y = ");  Serial.print(mag_data[i][1]);
    Serial.print(", Z = ");  Serial.println(mag_data[i][2]);

    delay(50);
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
// void computeSoftIronOffsets2x2(float mag_data_raw[MAX_SAMPLES][3], int num_samples) {

//   BLA::Matrix<3,3> cov_sums;
//   cov_sums.Fill(0);

//   // std::vector<BLA::Matrix<3, 3> > cov_data_sets;

//   // Compute covariance matrix of magnetometer data
//   // Compute eigenvalues and eigenvectors of covariance matrix
//   // Compute soft-iron correction matrix from eigenvalues and eigenvectors

//   // Create magnetometer data matrix with hard-iron offsets removed
//   // std::vector<BLA::Matrix<3, 1> > mag_data_offsets_removed; // Vector of magnetometer data with hard-iron offsets removed for each sample
//   BLA::Matrix<MAX_SAMPLES, 3, BLA::Matrix <3, 1>> mag_data_offsets_removed;

//   for (int i = 0; i < num_samples; i++) {
//     BLA::Matrix<3, 1> offsets_removed;
//     offsets_removed(0) = mag_data_raw[i][0] - hard_iron(0);
//     offsets_removed(1) = mag_data_raw[i][1] - hard_iron(1);
//     offsets_removed(2) = mag_data_raw[i][2] - hard_iron(2);
//     // mag_data_offsets_removed.push_back(offsets_removed);
//     mag_data_offsets_removed(i) = offsets_removed;
//   }

//   // Traverse each data point and compute eigenvalues and eigenvectors of the covariance matrix of the magnetometer data
//   for (int i = 0; i < num_samples; i++) {
//     BLA::Matrix<3, 1> offsets_removed = mag_data_offsets_removed(i);

//     BLA::Matrix<1, 3> offsets_removed_T = ~offsets_removed;  // calculate the transpose
//     BLA::Matrix<3, 3> cov = offsets_removed * offsets_removed_T;  // calculate the covariance matrix

//     // cov_data_sets.push_back(cov);
//     cov_sums(0, 0) += cov(0, 0);
//     cov_sums(0, 1) += cov(0, 1);
//     cov_sums(0, 2) += cov(0, 2);
//     cov_sums(1, 0) += cov(1, 0);
//     cov_sums(1, 1) += cov(1, 1);
//     cov_sums(1, 2) += cov(1, 2);
//     cov_sums(2, 0) += cov(2, 0);
//     cov_sums(2, 1) += cov(2, 1);
//     cov_sums(2, 2) += cov(2, 2);
//   }

//   // Now, calculate the trace of the 3x3 covariance sums
//   float trace = cov_sums(0, 0) + cov_sums(1, 1) + cov_sums(2, 2);

//   // Calculate the determinant of the 3x3 covariance matrix
//   /**
//    * 
//    */
//   float determinant = cov_sums(0, 0)*cov_sums(1, 1)*cov_sums(2, 2) + 
//                       cov_sums(0, 1)*cov_sums(1, 2)*cov_sums(0, 3) +
//                       cov_sums(0, 2)*cov_sums(1, 0)*cov_sums(2, 1) -
//                       cov_sums(0, 2)*cov_sums(1, 1)*cov_sums(2, 0) -
//                       cov_sums(0, 1)*cov_sums(1, 0)*cov_sums(2, 2) -
//                       cov_sums(0, 0)*cov_sums(1, 2)*cov_sums(2, 1);

//   // Solve for eigenvalues from det and trace of characteristic equation
//   //  C(x)  =  x^2 - tr(B)x + det(B) = 0
//   //  C(x) = (x-x1)(x-x2)      where  x1 and x2 are the eigenvalues, use quadratic formula to get roots

//   float x1 = ( -trace + sqrt( trace * trace - 4.0 * determinant) ) / 2.0;
//   float x2 = ( -trace - sqrt( trace * trace - 4.0 * determinant) ) / 2.0;
//   float scale = sqrt(x1 / x2);

//   // Use the derived eigenvalues to calculate the ellipsoid skew
  
// }

BLA::Matrix<3, 3> computeSoftIronOffsets3x3(float mag_data_raw[MAX_SAMPLES][3], int num_samples, BLA::Matrix<3, 1> hard_iron) {
  // Compute covariance matrix of magnetometer data
  // Compute eigenvalues and eigenvectors of covariance matrix
  // Compute soft-iron correction matrix from eigenvalues and eigenvectors
  BLA::Matrix<3,3> covariance { 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0 };

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

  // Traverse each data point and sum the covariance matrices per data point 
  // to get the covariance matrix of the whole magnetometer data
  for (int i = 0; i < num_samples; i++) {
    BLA::Matrix<3, 1> offsets_removed = mag_data_offsets_removed(i);
    BLA::Matrix<1, 3> offsets_removed_T = ~offsets_removed;  // calculate the transpose
    BLA::Matrix<3, 3> cov = offsets_removed * offsets_removed_T;  // calculate the covariance matrix

    // cov_data_sets.push_back(cov);
    covariance(0, 0) += cov(0, 0);
    covariance(0, 1) += cov(0, 1);
    covariance(0, 2) += cov(0, 2);
    covariance(1, 0) += cov(1, 0);
    covariance(1, 1) += cov(1, 1);
    covariance(1, 2) += cov(1, 2);
    covariance(2, 0) += cov(2, 0);
    covariance(2, 1) += cov(2, 1);
    covariance(2, 2) += cov(2, 2);
  }

  /**
   * Now perform eigen decomposition on an assumed 3x3 Hermitian matrix
   * to derive the eigenvalues and eigenvectors of the covariance matrix of the magnetometer data 
   */ 

  // Setup some letters for the 3x3 hermitian matrix defined by:
  /**
   * | a d f |
   * | d b e |
   * | f e c |
   * where a, b, c are real-valued and d, e, f are complex-valued
   */
  float a, b, c, d, e, f;
  a = covariance(0, 0);
  b = covariance(1, 1);
  c = covariance(2, 2);
  d = covariance(0, 1); 
  e = covariance(1, 2);
  f = covariance(0, 2);

  // closed-form solution for x1 and x2 
  float x1 = a*a + b*b + c*c - a*b - a*c - b*c + 3.0 * (abs(d*d) + abs(f*f) + abs(e*e));
  float x2 = -(2.0*a - b - c) * (2.0*b - a - c) * (2.0*c - a - b) +
              9.0*( (2.0*c - a - b)*abs(d*d) + (2.0*b - a - c)*abs(f*f) + (2.0*a - b - c)*abs(e*e) ) -
              54.0*(d*e*f);
  
  float phi = 0.0;   // ϕ 
  if (x2 > 0) {
    phi = atan( sqrt(4.0*x1*x1*x1 - x2*x2) / x2 );
  } else if (x2 < 0) {
    phi = atan( sqrt(4.0*x1*x1*x1 - x2*x2) / x2 ) + PI;
  } else if (x2 == 0) {
    phi = PI / 2.0;
  }

  // eigenvalues then are
  float lambda_1 = (a + b + c - 2.0 * sqrt(x1) * cos( phi / 3.0 )) / 3.0;
  float lambda_2 = (a + b + c + 2.0 * sqrt(x1) * cos( (phi - PI) / 3.0 )) / 3.0;
  float lambda_3 = (a + b + c + 2.0 * sqrt(x1) * cos( (phi + PI) / 3.0 )) / 3.0;

  /** 
   * With those three eigenvalues, we can determine the three 3x1 eigenvectors by setting s1 = 1.0
   */
  float m1 = ( d*(c - lambda_1) - e*f ) / ( f*(b - lambda_1) - d*e );
  float m2 = ( d*(c - lambda_2) - e*f ) / ( f*(b - lambda_2) - d*e );
  float m3 = ( d*(c - lambda_3) - e*f ) / ( f*(b - lambda_3) - d*e );
  
  BLA::Matrix<3, 1> v1;
  BLA::Matrix<3, 1> v2;
  BLA::Matrix<3, 1> v3;

  float v1_1 = ( lambda_1 - c - e * m1 ) / f;
  float v2_1 = ( lambda_2 - c - e * m2 ) / f;
  float v3_1 = ( lambda_3 - c - e * m3 ) / f;

  v1 = { v1_1, 
          m1,
          1.0 };
  v2 = { v2_1,
          m2, 
          1.0 };
  v3 = { v3_1,
          m3,
          1.0 };

  // Assemble the soft-iron correction matrix from the eigenvalues and eigenvectors
  // soft_iron = V * D^-0.5 * V^-1
  BLA::Matrix<3, 3> V = { v1(0), v2(0), v3(0),
                          v1(1), v2(1), v3(1),
                          v1(2), v2(2), v3(2) };

  BLA::Matrix<3, 3> V_inv = ~V; // since V is orthogonal, V^-1 = V^T

  /**  We know that for a centered ellipsoid, the coordinate axes obeys the formula
  * Sum from i=1 to d, (z_i^2 / R_i^2) = 1, where d is the dimension of the ellipsoid, 
  * z_i is the coordinate along the i-th axis, and R_i are the semi-axis lengths
  * (or radii) along the i-th axis.
  *  By this analogy, we can see that if the eigenvectors are the principal axes of the ellipsoid, 
  *  the semi-axis lengths are then given by 1 / sqrt(eigenvalues)
  */

  float c1 = 1.0 / sqrt(lambda_1);
  float c2 = 1.0 / sqrt(lambda_2);
  float c3 = 1.0 / sqrt(lambda_3);

  BLA::Matrix<3, 3> D_inv_sqrt = { c1, 0.0, 0.0,
                                  0.0, c2,  0.0,
                                  0.0, 0.0, c3 };

  // Finally, the soft-iron correction matrix (i.e. Jordan Canonical form)
  BLA::Matrix<3, 3> C = V * D_inv_sqrt * V_inv;

  return C;
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

  uint8_t maxRetries = 10;
  uint8_t retryCount = 0;
  while ( mlx.checkStatus(status) != MLX90393::STATUS_OK ) {
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

  delay(1000);

  /* ++++ Collect magnetometer data ++++ */
  readMagRaw(mag_data, MAX_SAMPLES);

  /* ++++ Compute hard-iron offsets ++++ */
  BLA::Matrix<3, 1> hard_iron_offsets = computeHardIronOffsets(mag_data, MAX_SAMPLES);
  Serial.println("Hard-iron offsets:");
  Serial.print("X: "); Serial.print(hard_iron_offsets(0));
  Serial.print(", Y: "); Serial.print(hard_iron_offsets(1));
  Serial.print(", Z: "); Serial.println(hard_iron_offsets(2));

  /* ++++ Compute soft-iron correction matrix ++++ */
  BLA::Matrix<3, 3> soft_iron_correction_matrix = computeSoftIronOffsets3x3(mag_data, MAX_SAMPLES, hard_iron_offsets);
  Serial.println("Soft-iron correction matrix:");
  Serial.print(soft_iron_correction_matrix(0, 0)); Serial.print(" "); Serial.print(soft_iron_correction_matrix(0, 1)); Serial.print(" "); Serial.println(soft_iron_correction_matrix(0, 2));
  Serial.print(soft_iron_correction_matrix(1, 0)); Serial.print(" "); Serial.print(soft_iron_correction_matrix(1, 1)); Serial.print(" "); Serial.println(soft_iron_correction_matrix(1, 2));
  Serial.print(soft_iron_correction_matrix(2, 0)); Serial.print(" "); Serial.print(soft_iron_correction_matrix(2, 1)); Serial.print(" "); Serial.println(soft_iron_correction_matrix(2, 2));

  hard_iron = hard_iron_offsets;
  soft_iron = soft_iron_correction_matrix;
  Serial.println("Calibration complete.");
  
}

float low_limit = 0.0;
float high_limit = 360.0;

void loop() {
  // put your main code here, to run repeatedly:
  // Subtract out the hard-iron offsets and then apply soft-iron affine transformation to 
  // the raw magnetometer data to compute calibrated magnetometer data. 
  // Secondly, compute the heading from the X and Y components of the output. 
  // Finally add the magnetic declination (obtained from GPS lookup table) to the heading
  // to obtain the geographic heading.

  // if (mlx.readData(mlx_data)) {
  mlx.readData(mlx_data);
  BLA::Matrix<3, 1> mag_raw = { mlx_data.x, mlx_data.y, mlx_data.z };
  BLA::Matrix<3, 1> mag_corrected = soft_iron * (mag_raw - hard_iron);

  float heading_rad = (atan2(mag_corrected(1), mag_corrected(0)));
  float heading_deg = heading_rad * (180.0 / PI);
  float heading_deg_corrected = heading_deg + mag_decl;

  if (heading_deg_corrected < 0.0) {
    heading_deg_corrected += 360.0;
  }

  Serial.print("low_bound:");          Serial.print(low_limit);
  Serial.print(",high_bound:");        Serial.print(high_limit);
  Serial.print(",heading_corrected:"); Serial.println(heading_deg_corrected); 

  delay(10);
}