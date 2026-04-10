#ifndef MPU6050PLUS_H_
#define MPU6050PLUS_H_

#include "I2Cdev.h"
#include <Arduino.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>
#include <Filters.h>
#include <LowPassFilter.h>

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/* MPU6050 specific registers */
#define MPU6050_ADRR                    0x68

#define MPU6050_RA_GYRO_CONFIG      0x1B
#define MPU6050_RA_ACCEL_CONFIG     0x1C
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_XG_OFFS_USRH     0x13
#define MPU6050_RA_YG_OFFS_USRH     0x15
#define MPU6050_RA_ZG_OFFS_USRH     0x17
#define MPU6050_RA_XA_OFFS_H        0x06
#define MPU6050_RA_YA_OFFS_H        0x08
#define MPU6050_RA_ZA_OFFS_H        0x0A
#define MPU6050_RA_WHO_AM_I         0x75
#define MPU6050_RA_SMPLRT_DIV       0x19
#define MPU6050_RA_CONFIG           0x1A
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_PWR_MGMT_2       0x6C

#define MPU6050_PWR1_DEVICE_RESET_BIT 7
#define MPU6050_PWR1_SLEEP_BIT        6
#define MPU6050_PWR1_CLKSEL_BIT       0
#define MPU6050_PWR1_CLKSEL_LENGTH    3

#define MPU6050_CLOCK_INTERNAL       0x00
#define MPU6050_CLOCK_PLL_XGYRO      0x01
#define MPU6050_WHO_AM_I_BIT        6
#define MPU6050_WHO_AM_I_LENGTH     6

#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define MPU6050_ACONFIG_AFS_SEL_BIT         4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH      2


#define M_PI (3.14159)
#define G (9.80665) // gravity constant - m/s^2
#define RAD_2_DEG ( 180.0 / M_PI )
#define DEG_2_RAD ( M_PI / 180.0 )

#define LSB_RANGE (32768.0) // 16 bit signed int range

#define SENSITIVITY_ACCEL (16384)
//  * AFS_SEL | Full Scale Range | LSB Sensitivity
//  * --------+------------------+----------------
//  * 0       | +/- 2g           | 16384 LSB/mg
//  * 1       | +/- 4g           | 8192 LSB/mg
//  * 2       | +/- 8g           | 4096 LSB/mg
//  * 3       | +/- 16g          | 2048 LSB/mg

// #define SENSITIVITY_GYRO (131)
# define SENSITIVITY_GYRO (250.0 / 32768.0) // deg/s per LSB for +/-250 dps setting
//  * FS_SEL | Full Scale Range   | LSB Sensitivity
//  * -------+--------------------+----------------
//  * 0      | +/- 250 degrees/s  | 131 LSB/deg/s
//  * 1      | +/- 500 degrees/s  | 65.5 LSB/deg/s
//  * 2      | +/- 1000 degrees/s | 32.8 LSB/deg/s
//  * 3      | +/- 2000 degrees/s | 16.4 LSB/deg/s

#define LP_FILTER_DEGREE 5

/* State Estimate Params */
const float COMPLEMENTARY_ALPHA = 0.95; // 0.05;

enum GYRO_SCALE {
    MPU_GYR_250,
    MPU_GYR_500,
    MPU_GYR_1000,
    MPU_GYR_2000
};

enum ACCEL_SCALE {
    MPU_ACC_2G,
    MPU_ACC_4G,
    MPU_ACC_8G,
    MPU_ACC_16G
};

enum IMU_OUTPUT_TYPE {
    YPR,
    EULER_ANGLE,
    QUATERNION
};

/* Filtering settings */
const float FILTER_SAMPLING_FREQ = 1000;
const float FILTER_AMPLITUDE = 100;
const float FILTER_OFFSET = 100;
const float FILTER_WINDOW_LENGTH = 20.0 / FILTER_SAMPLING_FREQ;

const float AX_LP_CUTOFF_FREQ = 5.0;
const float AY_LP_CUTOFF_FREQ = 5.0;
const float AZ_LP_CUTOFF_FREQ = 5.0;
const float GX_LP_CUTOFF_FREQ = 5.0;
const float GY_LP_CUTOFF_FREQ = 5.0;
const float GZ_LP_CUTOFF_FREQ = 5.0;

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
    FilterOnePole *filters[6];                  // create (RC) filters for each measurement channel
    LowPassFilter<2> *lpFilters[6];            // create 2nd order low pass filters for each measurement channel

    ImuPoint *currMeas;
    ImuPoint *lastMeas;
    float fused_meas[3];                        // rpy (x,y,z) format
    static constexpr float start_time = 0.0;
    float dT = 0.01;                            // imu sampling timestep 100 Hz default
    static constexpr unsigned long preInterval = 0;

    bool invertedX = false;
    bool invertedY = false;
    bool invertedZ = false;

    float rawAccX, rawAccY, rawAccZ;
    float rawGyroX, rawGyroY, rawGyroZ;
    float accX, accY, accZ;         // meters / sec ^ 2
    float filtAccX, filtAccY, filtAccZ;
    float gyroX, gyroY, gyroZ;      // degrees / sec
    float angleAccX, angleAccY;
    float angleX, angleY, angleZ;
    float angleGyroX = 0.0, angleGyroY = 0.0, angleGyroZ = 0.0;
    float quaternion[4] = {0.0, 0.0, 0.0, 0.0}; // w, x, y, z format

    /* MPU scaling factors */
    float gyroScaleFactor;
    uint16_t gyroScale;

    float accScaleFactor;
    uint8_t accScale;

    /* IMU offsets */
    static constexpr uint16_t ACCEL_DEADZONE = 8; // 8 LSBs is ~0.25 mg, which is roughly the noise level of the aceelerometer
    static constexpr uint16_t GYRO_DEADZONE = 1; // 1 LSB is ~0.0076 deg/s, which is roughly the noise level of the gyros
    static constexpr uint16_t CAL_BUFFER_LEN = 100; // number of measurements to collect for calibration
    static constexpr uint16_t NUM_ELEM_SKIP = 20; // number of initial measurements to skip for calibration (to let the filter settle)
    int32_t offset_ax = 0;
    int32_t offset_ay = 0;
    int32_t offset_az = 0;
    int32_t offset_gx = 0;
    int32_t offset_gy = 0;
    int32_t offset_gz = 0;

    // Accelerometer LP filter history buffers
    float accXHist[LP_FILTER_DEGREE] = {0.0};
    float accYHist[LP_FILTER_DEGREE] = {0.0};
    float accZHist[LP_FILTER_DEGREE] = {0.0};
    short lpFilterIndex = 0;
    uint8_t buffer[6] = {0};
    uint8_t getDeviceID();

public:
    MPU6050Plus();
    MPU6050Plus(uint8_t devAddr_, TwoWire *wireObj_, float sampleT = 0.01);
    void initialize(uint8_t devAddr, TwoWire *wireObj, float sampleT);
    bool initialize();
    bool testConnection();

    // MPU6050 driver methods implemented using I2Cdev
    void getAcceleration(int16_t* x, int16_t* y, int16_t* z);
    void getRotation(int16_t* x, int16_t* y, int16_t* z);
    void setFullScaleGyroRange(uint8_t range);
    void setFullScaleAccelRange(uint8_t range);
    void setClockSource(uint8_t source);
    void setSleepEnabled(bool enabled);
    void setXGyroOffset(int16_t offset);
    void setYGyroOffset(int16_t offset); 
    void setZGyroOffset(int16_t offset);
    void setXAccelOffset(int16_t offset);
    void setYAccelOffset(int16_t offset);
    void setZAccelOffset(int16_t offset);

    void filterMeasurements(float data[]);
    void getMeasurementAvgs(float data[], size_t size);
    void calibrate_(float data[], size_t size);
    bool calibrateIMU();
    // void getMeasurement(ImuPoint *point);
    // void getMeasurementRaw(float data[]);
    void updateRawMeasurements();

    /* State estimation algorithms */
    void complementaryFilter();         // calculate and update attitude using complementary filter
    // void kalmanFilter();                // calculate and update attitude using kalman filter

    /* Gyro axis-inversion setters */
    void invertAxis(int axis);
    void invertX() { invertedX = !invertedX; }
    void invertY() { invertedY = !invertedY; }
    void invertZ() { invertedZ = !invertedZ; }
    void printInvertedAxes();

    // EulerRPY getRPY();

    void showRawMeasurement(ImuPoint *point);
    void showVals(float data[]);

    void configureGyroScale(GYRO_SCALE scale);
    void configureAccScale(ACCEL_SCALE scale);

    /* Data Getters */

    inline float getAccXRaw() { return rawAccX; }
    inline float getAccYRaw() { return rawAccY; }
    inline float getAccZRaw() { return rawAccZ; }
    inline float getGyroXRaw() { return rawGyroX; }
    inline float getGyroYRaw() { return rawGyroY; }
    inline float getGyroZRaw() { return rawGyroZ; }

    inline float getAccX() { return accX; }
    inline float getAccY() { return accY; }
    inline float getAccZ() { return accZ; }
    inline float getGyroX() { return gyroX; }
    inline float getGyroY() { return gyroY; }
    inline float getGyroZ() { return gyroZ; }
    inline float getAngleAccX() { return angleAccX; }
    inline float getAngleAccY() { return angleAccY; }

    float getAngleX() { return angleX; }        // angle X in degs
    float getAngleY() { return angleY; }        // angle y in degs
    float getAngleZ() { return angleZ; }        // angle z in degs

    float* angleAxisQuaternion();

    inline int16_t getOffsetGyroX() { return offset_gx; }
    inline int16_t getOffsetGyroY() { return offset_gy; }
    inline int16_t getOffsetGyroZ() { return offset_gz; }
    inline int16_t getOffsetAccX()  { return offset_ax; }
    inline int16_t getOffsetAccY()  { return offset_ay; }
    inline int16_t getOffsetAccZ()  { return offset_az; }

    constexpr float getGyroScaleFactor() { return gyroScaleFactor; }
    constexpr uint16_t getGyroScale()    { return gyroScale; }
    constexpr float getAccScaleFactor()  { return accScaleFactor; }
    constexpr uint8_t getAccScale()     { return accScale; }

protected:
    uint8_t devAddr;
    void *wireObj;
};

#endif  // endif MPU6050PLUS_H_