#ifndef MPU6050Plus_h
#define MPU6050Plus_h

#include "MPU6050.h"
#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Filters.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* MPU6050 specific registers */
#define MPU6050_ADRR                    0x68
// #define MPU6050_SMPLRT_DIV_REGISTER     0x19
// #define MPU6050_CONFIG_REGISTER         0x1a
// #define MPU6050_GYRO_CONFIG             0x1b
// #define MPU6050_ACCEL_CONFIG            0x1c


#define M_PI (3.14159)
#define G (9.80665) // gravity constant - m/s^2
#define RAD_2_DEG ( M_PI / 180.0 )

//  * AFS_SEL | Full Scale Range | LSB Sensitivity
//  * --------+------------------+----------------
//  * 0       | +/- 2g           | 16384 LSB/mg
//  * 1       | +/- 4g           | 8192 LSB/mg
//  * 2       | +/- 8g           | 4096 LSB/mg
//  * 3       | +/- 16g          | 2048 LSB/mg
#define SENSITIVITY_ACCEL (16384)
//  * FS_SEL | Full Scale Range   | LSB Sensitivity
//  * -------+--------------------+----------------
//  * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
//  * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
//  * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
//  * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s
#define SENSITIVITY_GYRO (131)

#define NUM_CALIB_CYCLES (20)

enum IMU_OUTPUT_TYPE {
    YPR,
    EULER_ANGLE,
    QUATERNION
};

/* Filtering settings */
const float FILTER_SAMPLING_FREQ = 2;
const float FILTER_AMPLITUDE = 100;
const float FILTER_OFFSET = 100;
const float FILTER_WINDOW_LENGTH = 20.0 / FILTER_SAMPLING_FREQ;

/* Orientation - Quaterion form TODO*/
struct quaternion{
    float w = 0.0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

typedef struct EulerRPY{
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

/* this class keeps track of imu measurements for a single timestamp 
 * both raw, converted, and filtered measurements are all tracked
*/
struct ImuPoint {
    float ax_raw = 0.0;
    float ay_raw = 0.0;
    float az_raw = 0.0;

    float gx_raw = 0.0;
    float gy_raw = 0.0;
    float gz_raw = 0.0;

    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

    float gx = 0.0;
    float gy = 0.0;
    float gz = 0.0;

    float curr_ts = 0.0;

    bool isValid();
};

/*
*  This class is a wrapper to help with configuring the mpu instance
*  and passing measurements in a useable format
*/
class MPU6050Plus
{
private:
    MPU6050 *mpu;              // pointer to mpu instance created by main
    FilterOnePole *filters[6]; // create (RC) filters for each measurement channel
    ImuPoint *currMeas;
    ImuPoint *lastMeas;
    float fused_meas[3]; // rpy (x,y,z) format
    float start_time;
    float dT = 0.001; // imu sampling timestep

    float accX, accY, accZ, gyroX, gyroY, gyroZ;
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;

public:
    MPU6050Plus();

    void initialize(MPU6050 *mpu, float sampleT);

    // void getMeasurement(ImuPoint *point);
    // void getMeasurementRaw(float data[]);
    void updateMeasurement();

    void filterMeasurements(float data[]);

    EulerRPY getRPY();

    void showRawMeasurement(ImuPoint *point);

    void showVals(float data[]);

    /* Data Getters */
    float getAccX() { return accX; }
    float getAccY() { return accY; }
    float getAccZ() { return accZ; }
    float getGyroX() { return gyroX; }
    float getGyroY() { return gyroY; }
    float getGyroZ() { return gyroZ; }

    float getAngleX() { return angleX; }
    float getAngleY() { return angleY; }
    float getAngleZ() { return angleZ; }
    
    float getAngleAccX() { return angleAccX; }
    float getAngleAccY() { return angleAccY; }
};

#endif  // endif MPU6050Plus_h