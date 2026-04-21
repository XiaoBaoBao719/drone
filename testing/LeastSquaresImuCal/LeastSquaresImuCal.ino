#include <Wire.h>
#include <Arduino.h>
#include <MPU6050Plus.h>
#include <BasicLinearAlgebra.h>
// #include "SignalProcessing/LowPassFilter.h"
// #include "SignalProcessing/MovingAvg.h"

using namespace BLA;


/****************** SUMMARY ******************
 * This is a testbed for calibrating the IMU using least squares optimization. The goal is to find the optimal set of calibration parameters (offsets, scale factors, and misalignment angles) that minimize the error between the measured IMU data and the expected values based on known orientations and motions.

  The code will:
  - Collect raw IMU data from the MPU6050 sensor.
  - Collect raw measurements at six predefined orientations (e.g., flat, on each side, etc.) to capture the sensor's response in different conditions.
  - Apply a least squares optimization algorithm to estimate the calibration parameters.
  - Output the estimated parameters for use in correcting future IMU readings.

    Calibration Model:
    Assume that raw measurements (rx, ry, rz for accelerometer) can be converted into 
    calibrated values (cx, cy, cz) using matrix multiplication by a 4-by-4 matrix:

    [cx cy cz]^T = A * [rx ry rz]^T

    Where A is the calibration matrix:
    A   = [ a b c 
            d e f 
            g h i ]

    The parameters a, b, c, d, e, f, g, h, i represent scale factors and misalignment angles, 
    while j, k, l represent offsets. 
   
************************************************/

int LED_PIN = 13;

float x_offset, y_offset, z_offset;
int64_t x_xup, y_xup, z_xup;
int64_t x_yup, y_yup, z_yup;
int64_t x_zup, y_zup, z_zup;
int64_t x_xdown, y_xdown, z_xdown;
int64_t x_ydown, y_ydown, z_ydown;
int64_t x_zdown, y_zdown, z_zdown;

typedef struct {
   int64_t x;
   int64_t y;
   int64_t z;
} dataPoint;

dataPoint xup, yup, zup;
dataPoint xdown, ydown, zdown;

float a, b, c, d, e, f, g, h, i;  // Scale factors and misalignment parameters

BLA::Matrix<3, 3> A = { 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1};            // Calibration matrix that transforms raw measurements into calibrated values
BLA::Matrix<3> offsets = { 0, 0, 0 };            // offsets vector
BLA::Matrix<3> x;            // solution vector (calibrated values)

MPU6050Plus imu;                // MPU6050 wrapper that provides filtered angle measurements and raw data access



void collectData(MPU6050Plus* imu, dataPoint* dp) {
    long measCount = 0, buff_ax = 0, buff_ay = 0, buff_az = 0;
    const int numMeasurements = 1000; // Number of measurements to average for each orientation
    const int numSkip = 50; // Number of initial measurements to skip for filter settling

    while (measCount < numMeasurements + numSkip) {
        if (numSkip < measCount && measCount <= numMeasurements + numSkip) 
        {
            /* read raw imu accelerometer measurements and update */
            buff_ax += imu->getAccXRaw();
            buff_ay += imu->getAccYRaw();
            buff_az += imu->getAccZRaw();
        }
        measCount++;
    }
        
    /* normalize and update the 18 measurements */
    dp->x = buff_ax / numMeasurements;
    dp->y = buff_ay / numMeasurements;
    dp->z = buff_az / numMeasurements;
}

void waitForUserInput(const char* prompt) {

    int response = 0;

    Serial.println(prompt);
    // Clear the input buffer
    while (Serial.available() > 0) {
        Serial.read();
    }
    while (Serial.available() == 0) {
        response = Serial.read();
        delay(500);
        Serial.println("You entered: " + String(response));
        Serial.println("Advancing...");
    }
}

void setup() {
    Serial.begin(9600);
    Wire1.begin();
    Wire1.setClock(400000); // 400 kHz I2C clock

    // Initialize MPU6050
    imu.initialize(0x68, &Wire1, 0.001); // 1 kHz sample rate
    // Verify connection
    while (!imu.testConnection()) {
        Serial.println("MPU connection failed! Retrying...");
        delay(500);
    }

    imu.setCalibrated(false); // Start with uncalibrated state

    Serial.println("MPU initialized and verified.");

    pinMode(LED_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_PIN,LOW);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000);

    /* Collect raw IMU data and wait for user input at each of the six orientations */
    /* X+ orientation */
    waitForUserInput("Place the IMU in the X+ orientation and press any key to continue...");
    collectData(&imu, &xup);

    /* Y+ orientation */
    waitForUserInput("Place the IMU in the Y+ orientation and press any key to continue...");
    collectData(&imu, &yup);

    /* Z+ orientation */
    waitForUserInput("Place the IMU in the Z+ orientation and press any key to continue...");
    collectData(&imu, &zup);

    /* X- orientation */
    waitForUserInput("Place the IMU in the X- orientation and press any key to continue...");
    collectData(&imu, &xdown);

    /* Y- orientation */
    waitForUserInput("Place the IMU in the Y- orientation and press any key to continue...");
    collectData(&imu, &ydown);

    /* Z- orientation */
    waitForUserInput("Place the IMU in the Z- orientation and press any key to continue...");
    collectData(&imu, &zdown);


    /* Calculate the 12 calibration parameters */
    /* Calculate the offsets */
    x_offset = (xup.x + xdown.x + yup.x + ydown.x + zup.x + zdown.x) / 6;
    y_offset = (xup.y + xdown.y + yup.y + ydown.y + zup.y + zdown.y) / 6;
    z_offset = (xup.z + xdown.z + yup.z + ydown.z + zup.z + zdown.z) / 6;
    /* Calculate the 9 calibration parameters */
    a = (xup.x - x_offset) + (-xdown.x + x_offset) / 2; // Scale factor for X-axis
    b = (yup.x - x_offset) + (-ydown.x + x_offset) / 2; // Misalignment of Y-axis on X-axis
    c = (zup.x - x_offset) + (-zdown.x + x_offset) / 2; // Misalignment of Z-axis on X-axis
    d = (xup.y - y_offset) + (-xdown.y + y_offset) / 2; // Misalignment of X-axis on Y-axis
    e = (yup.y - y_offset) + (-ydown.y + y_offset) / 2; // Scale factor for Y-axis
    f = (zup.y - y_offset) + (-zdown.y + y_offset) / 2; // Misalignment of Z-axis on Y-axis
    g = (xup.z - z_offset) + (-xdown.z + z_offset) / 2; // Misalignment of X-axis on Z-axis
    h = (yup.z - z_offset) + (-ydown.z + z_offset) / 2; // Misalignment of Y-axis on Z-axis
    i = (zup.z - z_offset) + (-zdown.z + z_offset) / 2; // Scale factor for Z-axis

    /* Output calibration and offsets matrix */
    A = { a, b, c,
          d, e, f,
          g, h, i };
    offsets = { x_offset, y_offset, z_offset };

    Serial.println("Calibration matrix A:");
    Serial.print(A(0, 0)); Serial.print("\t"); Serial.print(A(0, 1)); Serial.print("\t"); Serial.println(A(0, 2));
    Serial.print(A(1, 0)); Serial.print("\t"); Serial.print(A(1, 1)); Serial.print("\t"); Serial.println(A(1, 2));
    Serial.print(A(2, 0)); Serial.print("\t"); Serial.print(A(2, 1)); Serial.print("\t"); Serial.println(A(2, 2));
    Serial.println("Offsets vector b:");
    Serial.print(offsets(0)); Serial.print("\t"); Serial.print(offsets(1)); Serial.print("\t"); Serial.println(offsets(2));
    
    waitForUserInput("Calibration complete. Press any key to test on raw measurements...");
}


void loop() {
    float raw_x = imu.getAccXRaw();
    float raw_y = imu.getAccYRaw();
    float raw_z = imu.getAccZRaw();

    /* Apply calibration to raw measurements */
    x = { raw_x, raw_y, raw_z };

    auto A_decomp = A;
    auto decomp = LUDecompose(A_decomp);
    // if (!decomp) {
    //     Serial.println("Calibration matrix is singular! Cannot apply calibration.");
    //     return;
    // }
    Matrix<3, 1> acc_calibrated = LUSolve(decomp, x - offsets);

    Serial.print("Raw Accel: (");
    Serial.print(x); Serial.println("), ");

    Serial.print("calibrated Accel: (");
    Serial.print(acc_calibrated); Serial.println(")");
}