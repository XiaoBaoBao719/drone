#include <MPU6050Plus.h>
#include <Arduino.h>

MPU6050Plus::MPU6050Plus( MPU6050 *mpu , float sampleT) {
    // Initialize the mpu 6050 instance
    mpu->initialize();
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
    start_time = float( micros() ) * 1e-6;
    // Update sampling period for taking measurements
    dT = sampleT;
}

void MPU6050Plus::getMeasurementRaw(MPU6050 *mpu, float data[] ) {
    mpu->getAcceleration(&ax_raw,&ay_raw,&az_raw);
    /* convert from analog counts to m/s^2 */
    data[0] = ax_raw * G / SENSITIVITY_ACCEL;  // x
    data[1] = ay_raw * G / SENSITIVITY_ACCEL;  // y
    data[2] = az_raw * G / SENSITIVITY_ACCEL;  // z

    mpu->getRotation(&gx_raw,&gy_raw,&gz_raw);
    /* convert from analog counts to deg / s */
    data[3] = gx_raw / SENSITIVITY_GYRO;
    data[4] = gy_raw / SENSITIVITY_GYRO;
    data[5] = gz_raw / SENSITIVITY_GYRO;
}

void MPU6050Plus::getMeasurement(MPU6050 *mpu, ImuPoint *point) {
    // Get raw values, convert them, and run them through a low pass (RC) filter
    float data[6];
    getMeasurementRaw( mpu, data );
    // Update Imu point
    point->ax_raw = data[0];
    point->ay_raw = data[1];
    point->az_raw = data[2];
    point->gx_raw = data[3];
    point->gx_raw = data[4];
    point->gx_raw = data[5];
    // Pass measurements through low pass filter
    filterMeasurements( data );
    // Update Imu point
    point->ax = data[0];
    point->ay = data[1];
    point->az = data[2];
    point->gx = data[3];
    point->gy = data[4];
    point->gz = data[5];
}

float* MPU6050Plus::getRollPitchYaw(MPU6050 *mpu) {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    
    float* data = (float*) malloc(3 * sizeof(float));
    float roll = 0.0;
    float pitch = 0.0;
    float yaw = 0.0;
    // Calculate pitch tilt angle from accel in radians
    pitch = atan2( -currMeas.ax , ( sqrt( currMeas.ay*currMeas.ay + currMeas.az*currMeas.az )) );
    // Calculate the roll tilt angle from accel in radians
    roll = atan2(currMeas.ay , currMeas.az);
    // Grab the imu measurement's time stamp
    float ts = currMeas.curr_ts;
    // Calcualte the pitch tilt angle via gyro integration

    // Return final tilt angles
    return data;
}

float* MPU6050Plus::integrateGyro() {
    float* data = (float*) malloc(3 * sizeof(float));   // explicit casting of malloc
    data[0] = lastMeas.gx + currMeas.gx * dT;
    data[1] = lastMeas.gy + currMeas.gy * dT;
    data[2] = lastMeas.gz + currMeas.gz * dT;
    return data;
}

void MPU6050Plus::filterMeasurements(float data[]) {
    // Update the measurement based on it's respective filter (ps. they are all low pass filters)
    for (size_t index = 0; index < sizeof(data)/sizeof(data[0]); index++) {
        data[index] = filters[index]->input( data[index] );
    }
}

void update() {

}