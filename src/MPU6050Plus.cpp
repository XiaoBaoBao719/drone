#include <MPU6050Plus.h>
#include <Arduino.h>

const float COMPLEMENTARY_ALPHA = 0.2;

MPU6050Plus::MPU6050Plus(MPU6050 *mpu_, float sampleT)
{
    mpu = mpu_;
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
    start_time = (float) ( micros() ) * 1e-6;
    // Update sampling period for taking measurements
    dT = sampleT;
}

void MPU6050Plus::getMeasurementRaw(float data[] ) {
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

void MPU6050Plus::getMeasurement(ImuPoint *point) {
    // Get raw values, convert them, and run them through a low pass (RC) filter
    float data[6];
    getMeasurementRaw( data );
    // Update Imu point
    point->ax_raw = data[0];
    point->ay_raw = data[1];
    point->az_raw = data[2];
    point->gx_raw = data[3];
    point->gx_raw = data[4];
    point->gx_raw = data[5];
    // Pass accel measurements through low pass filter
    filterMeasurements( data );
    // Update Imu point
    point->ax = data[0];
    point->ay = data[1];
    point->az = data[2];
    point->gx = data[3];
    point->gy = data[4];
    point->gz = data[5];
}

void MPU6050Plus::filterMeasurements(float data[]) {
    // Update the measurement based on it's respective filter (ps. they are all low pass filters)
    for (size_t index = 0; index < sizeof(data)/sizeof(data[0]); index++) {
        data[index] = filters[index]->input( data[index] );
    }
}

float* MPU6050Plus::getAccelRPY() {
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
    // TODO: Calcualte the yaw angle from a magnetometer reading

    // Return final tilt angles
    return data;
}

float* MPU6050Plus::integrateGyro() {
    float* data = (float*) malloc(3 * sizeof(float));   // explicit casting of malloc
    data[0] = lastMeas.gx + currMeas.gx * dT;           // 'roll'
    data[1] = lastMeas.gy + currMeas.gy * dT;           // 'pitch'
    data[2] = lastMeas.gz + currMeas.gz * dT;           // 'yaw'
    return data;
}

EulerRPY MPU6050Plus::getRPY() {
    EulerRPY rpy;
    bool is_valid = !(isnanf(fused_meas[0]) || isinff(fused_meas[0])) &&
                    !(isnanf(fused_meas[1]) || isinff(fused_meas[1])) &&
                    !(isnanf(fused_meas[2]) || isinff(fused_meas[2]));

    if (is_valid && (fused_meas[0] && fused_meas[1] && fused_meas[2])) {
        // Convert from euler to degrees
        rpy.x = fused_meas[0] * (M_PI / 180.0);
        rpy.y = fused_meas[1] * (M_PI / 180.0);
        rpy.z = fused_meas[2] * (M_PI / 180.0);
    }
}

bool ImuPoint::isValid(){
    // Check that all values are real values
    return ( !(isnanf(ax) || isinff(ax)) &&
             !(isnanf(ay) || isinff(ay)) &&
             !(isnanf(az) || isinff(az)) &&
             !(isnanf(gx) || isinff(ax)) &&
             !(isnanf(gy) || isinff(gy)) &&
             !(isnanf(gz) || isinff(gz)) );
}


void MPU6050Plus::update() {
    float roll_ = 0.0;       
    float pitch_ = 0.0;      
    float yaw_ = 0.0;
    // get an imu point and update current measurement
    getMeasurement( &currMeas );
    // get the time delta
    double ts_now = millis();
    if (start_time - ts_now > 0.0) 
    // check if current measurement is valid, get gyro-based tilt attitude
    if ( currMeas.isValid() ) {
        float *gyro_data = integrateGyro();
        float *accel_data = getAccelRPY();
        // Perform sensor fusion on both types of sensor measurements
        // TODO: EKF sensor fusion
        // Ccomplementary filter sensor fusion
        for (size_t n = 0; n < sizeof(fused_meas)/sizeof(fused_meas[0]); n++)
        {
            fused_meas[n] = COMPLEMENTARY_ALPHA * gyro_data[n] + (1.0 - COMPLEMENTARY_ALPHA) * accel_data[n];
        }

        // Update lastMeas data
        lastMeas.gx = currMeas.gx;
        lastMeas.gy = currMeas.gy;
        lastMeas.gz = currMeas.gz;
    }
}