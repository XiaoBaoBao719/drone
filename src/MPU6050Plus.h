#include <MPU6050.h>
// #include <Arduino.h>
#include <BasicLinearAlgebra.h>
#include <Filters.h>

#include <stdint.h>
#include <math.h>

#define M_PI (3.14159)
#define G (9.80665) // gravity constant - m/s^2
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
    EULER,
    QUATERNION
};

/* Filtering settings */
const float FILTER_SAMPLING_FREQ = 2;
const float FILTER_AMPLITUDE = 100;
const float FILTER_OFFSET = 100;
const float FILTER_WINDOW_LENGTH = 20.0 / FILTER_SAMPLING_FREQ;

/* IMU offsets */
int16_t offset_ax = 0;
int16_t offset_ay = 0;
int16_t offset_az = 1788;
int16_t offset_gx = 220;
int16_t offset_gy = 76;
int16_t offset_gz = -85;

/* Raw IMU data */
int16_t ax_raw, ay_raw, az_raw;
int16_t gx_raw, gy_raw, gz_raw;

/* Orientation - Yaw Pitch Roll form */
float ypr[3];

/* Orientation - Quaterion form TODO*/
typedef struct q{
    float w = 0.0;
    float x = 0.0;
    float y = 0.0;
    float z = 0.0;
};

/* this class keeps track of imu measurements for a single timestamp 
 * both raw, converted, and filtered measurements are all tracked
*/
struct ImuPoint {
    float ax_raw;
    float ay_raw;
    float az_raw;

    float gx_raw;
    float gy_raw;
    float gz_raw;

    float ax;
    float ay;
    float az;

    float gx;
    float gy;
    float gz;

    float curr_ts;
};

/*
*  This class is a wrapper to help with configuring the mpu instance
*  and passing measurements in a useable format
*/
struct MPU6050Plus {
    MPU6050 *mpu;   // pointer to mpu instance created by main
    FilterOnePole *filters[6]; // create (RC) filters for each measurement channel
    ImuPoint currMeas;
    ImuPoint lastMeas;
    float start_time;

    MPU6050Plus( MPU6050 *mpu );

    void applyIMUOffsets(MPU6050 *mpu);

    void getMeasurement(MPU6050 *mpu, ImuPoint *point );

    void getMeasurementRaw(MPU6050 *mpu, float data[] );

    void getGyroRaw(MPU6050 *mpu, float *gx_, float *gy_, float *gz_);

    void getYawPitchRoll(MPU6050 *mpu, ImuPoint *point);

    void filterMeasurements(float data[]);
};