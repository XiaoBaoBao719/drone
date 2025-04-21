#include <MPU6050Plus.h>
#include <Arduino.h>

const float COMPLEMENTARY_ALPHA = 0.2;

/* IMU offsets */
int16_t offset_ax = 0;
int16_t offset_ay = 0;
int16_t offset_az = 1788;
int16_t offset_gx = 220;
int16_t offset_gy = 76;
int16_t offset_gz = -85;

MPU6050Plus::MPU6050Plus() {
    ImuPoint* currMeas_;
    ImuPoint* lastMeas_;
    currMeas = currMeas_;
    lastMeas = lastMeas_;
}

void MPU6050Plus::initialize(MPU6050 *mpu_, float sampleT)
{
    mpu = mpu_;

    // Apply offsets
    mpu->setXGyroOffset(offset_gx);
    mpu->setYGyroOffset(offset_gy);
    mpu->setZGyroOffset(offset_gz);

    mpu->setXAccelOffset(offset_ax);
    mpu->setYAccelOffset(offset_ay);
    mpu->setZAccelOffset(offset_az);

    // Create filters for each measurement channel
    for (size_t n = 0; n < sizeof(filters)/sizeof(filters[0]); n++ )
    {
        FilterOnePole filterLowpass( LOWPASS, FILTER_SAMPLING_FREQ );
        filters[n] = &filterLowpass;
    }

    // Update current time
    start_time = (float) ( micros() ) * 1e-6;
    // Update sampling period for taking measurements
    dT = sampleT;
}

void MPU6050Plus::updateMeasurement() {
    int16_t _ax_raw, _ay_raw, _az_raw;
    int16_t _gx_raw, _gy_raw, _gz_raw;
    float angleGyroX, angleGyroY, angleGyroZ;
    float data[6];

    mpu->getAcceleration(&_ax_raw,&_ay_raw,&_az_raw);
    /* convert from analog counts to m/s^2 */
    accX = _ax_raw * G / SENSITIVITY_ACCEL;  // x
    accY = _ay_raw * G / SENSITIVITY_ACCEL;  // y
    accZ = _az_raw * G / SENSITIVITY_ACCEL;  // z

    mpu->getRotation(&_gx_raw,&_gy_raw,&_gz_raw);
    /* convert from analog counts to deg / s */
    gyroX = _gx_raw / SENSITIVITY_GYRO;
    gyroY = _gy_raw / SENSITIVITY_GYRO;
    gyroZ = _gz_raw / SENSITIVITY_GYRO;

    /* Filter measurements */
    this->filterMeasurements( data );

    /* Calculate angleAcc measurements */
    angleAccY = atan2( -currMeas->ax , ( sqrt( currMeas->ay*currMeas->ay + currMeas->az*currMeas->az )) ); // Calculate pitch tilt angle from accel in radians
    angleAccX = atan2(currMeas->ay , currMeas->az);  // Calculate the roll tilt angle from accel in radians
    // TODO: Calcualte the yaw angle from a magnetometer reading

    /* Calculate angleGyro measurements */
    angleGyroX += gyroX * dT;           // 'roll'
    angleGyroY += gyroY * dT;           // 'pitch'
    angleGyroZ += gyroZ * dT;           // 'yaw'

    /* Sensor fusion of gyro and accel angles */
    // TODO: EKF sensor fusion
    // Ccomplementary filter sensor fusion
    angleX = COMPLEMENTARY_ALPHA * angleGyroX + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX;
    angleY = COMPLEMENTARY_ALPHA * angleGyroY + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY;
    angleZ = angleGyroY;
}

void MPU6050Plus::filterMeasurements(float data[]) {
    // Update the measurement based on it's respective filter (ps. they are all low pass filters)
    for (size_t index = 0; index < sizeof(data)/sizeof(data[0]); index++) {
        data[index] = filters[index]->input( data[index] );
    }
}

bool ImuPoint::isValid(){
    // Check that all values are real values
    return ( !(isnan(ax) || isinf(ax)) &&
             !(isnan(ay) || isinf(ay)) &&
             !(isnan(az) || isinf(az)) &&
             !(isnan(gx) || isinf(ax)) &&
             !(isnan(gy) || isinf(gy)) &&
             !(isnan(gz) || isinf(gz)) );
}

// void MPU6050Plus::showRawMeasurement(ImuPoint *point) {
//     if (this->currMeas->isValid()) {
//         Serial.print("x: ");
//         Serial.print(currMeas->ax_raw);
//         Serial.print(" y: ");
//         Serial.print(currMeas->ay_raw);
//         Serial.print(" z: ");
//         Serial.print(currMeas->az_raw);
//         Serial.println();
//         Serial.print("gx: ");
//         Serial.print(currMeas->gx_raw);
//         Serial.print(" gy: ");
//         Serial.print(currMeas->gy_raw);
//         Serial.print(" gz: ");
//         Serial.print(currMeas->gz_raw);
//         Serial.println();

//     } else {
//         Serial.print("No Current Measurement!");
//         Serial.println();
//     }
// }
// void MPU6050Plus::showVals(float data[]) {
//         Serial.print("x: ");
//         Serial.print(data[0]);
//         Serial.print(" y: ");
//         Serial.print(data[1]);
//         Serial.print(" z: ");
//         Serial.print(data[2]);
//         Serial.println();
//         Serial.print("gx: ");
//         Serial.print(data[3]);
//         Serial.print(" gy: ");
//         Serial.print(data[4]);
//         Serial.print(" gz: ");
//         Serial.print(data[5]);
//         Serial.println();
// }