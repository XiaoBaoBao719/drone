#include <MPU6050Plus.h>
#include <Arduino.h>

const float COMPLEMENTARY_ALPHA = 0.05;
const float CALIB_BIAS_CYCLES = 200;

/* IMU offsets */
int16_t offset_ax = 0;
int16_t offset_ay = 0;
int16_t offset_az = 1788;
int16_t offset_gx = 220;
int16_t offset_gy = 76;
int16_t offset_gz = -85;

void wrap(float* in) {
    if (*in <= -180.0) {
        *in = 180.0;
    } else if (*in >= 180.0) {
        *in = -180.0;
    }
}

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
    // dT = sampleT;
    preInterval = millis();

    
    /* Z axis bias calibration */
    this->calcBias();
    delay(100);
}

void MPU6050Plus::updateMeasurement() {
    int16_t _ax_raw, _ay_raw, _az_raw;
    int16_t _gx_raw, _gy_raw, _gz_raw;
    float angleGyroX, angleGyroY, angleGyroZ;
    float data[6];

    mpu->getAcceleration(&_ax_raw,&_ay_raw,&_az_raw);
    /* Convert: Counts -> m/s^2 */
    accX = _ax_raw * G / SENSITIVITY_ACCEL;  // x
    accY = _ay_raw * G / SENSITIVITY_ACCEL;  // y
    accZ = _az_raw * G / SENSITIVITY_ACCEL;  // z

    mpu->getRotation(&_gx_raw,&_gy_raw,&_gz_raw);
    /* Convert: Counts -> deg/s */
    gyroX = _gx_raw / SENSITIVITY_GYRO;
    gyroY = _gy_raw / SENSITIVITY_GYRO;
    gyroZ = _gz_raw / SENSITIVITY_GYRO;

    /* Apply (RC) Low Pass filter */
    // this->filterMeasurements( data );

    /* Calculate angleAcc measurements */
    angleAccY = atan2( -accX , ( sqrt( accY * accY + accZ * accZ )) ) * RAD_2_DEG; // Calculate pitch tilt angle from accel in radians
    angleAccX = atan2( accY , accZ ) * RAD_2_DEG;  // Calculate the roll tilt angle from accel in radians
    // TODO: Calcualte the yaw angle from a magnetometer reading

    unsigned long Tnew = millis();
    float dt = (Tnew - preInterval) * 1e-3;
    preInterval = Tnew;

    /* Calculate angleGyro measurements */
    angleGyroX = gyroX * dT;           // 'roll'
    angleGyroY = gyroY * dT;           // 'pitch'
    angleGyroZ = gyroZ * dT; // biasGyroZ;           // 'yaw'

    gZdT = gyroZ * dT;

    /* Sensor fusion of gyro and accel angles */
    // TODO: EKF sensor fusion

    // Ccomplementary filter sensor fusion  (X and Y axis inverted)
    angleY = COMPLEMENTARY_ALPHA * angleGyroX + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX - biasGyroX;
    angleX = COMPLEMENTARY_ALPHA * angleGyroY + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY - biasGyroY;
    angleX = angleX * -1;   // invert X-axis for this convention
    angleZ = angleGyroZ;           // add angleGyroZ since it is initially zero each loop

    // // Ccomplementary filter sensor fusion       (X - foward, Y - facing left )
    // angleX = COMPLEMENTARY_ALPHA * angleGyroX + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX - biasGyroX;
    // angleY = COMPLEMENTARY_ALPHA * angleGyroY + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY - biasGyroY;
    // angleZ += angleGyroZ;           // add angleGyroZ since it is initially zero each loop
}

void MPU6050Plus::filterMeasurements(float data[]) {
    // Update the measurement based on it's respective filter (ps. they are all low pass filters)
    for (size_t index = 0; index < sizeof(data)/sizeof(data[0]); index++) {
        data[index] = filters[index]->input( data[index] );
    }
}

void MPU6050Plus::calcBias() {
    angleZ = 0.0;
    float start = angleZ;
    float total = 0.0;
    float avg;

    float angleVals[] = {0.0, 0.0, 0.0};     // x, y, z
    float startVals[] = {0.0, 0.0, 0.0};     // x, y, z
    float totalVals[] = {0.0, 0.0, 0.0};     // x_total, y_total, z_total
    float avgs[]      = {0.0, 0.0, 0.0};     // x_avg, y_avg, z_avg

    float diffX, diffY, diffZ;

    for (int i = 0; i < CALIB_BIAS_CYCLES; i++) {
        this->updateMeasurement();
        // float temp = angleZ;
        angleVals[0] = angleX;
        angleVals[1] = angleY;
        angleVals[2] = angleZ;

        // float diff = temp - start;
        diffX = angleVals[0] - startVals[0];
        diffY = angleVals[1] - startVals[1];
        diffZ = angleVals[2] - startVals[2];

        // total += diff;
        totalVals[0] += diffX;
        totalVals[1] += diffY;
        totalVals[2] += diffZ;

        delay(1);
    }

    // avg = total / (float)CALIB_BIAS_CYCLES;
    avgs[0] = totalVals[0] / (float) CALIB_BIAS_CYCLES;
    avgs[1] = totalVals[1] / (float) CALIB_BIAS_CYCLES;
    avgs[2] = totalVals[2] / (float) CALIB_BIAS_CYCLES;

    biasGyroX = avgs[0];
    biasGyroY = avgs[1];
    biasGyroZ = avgs[2];

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

void MPU6050Plus::showRawMeasurement(ImuPoint *point) {
    if (this->currMeas->isValid()) {
        Serial.print("ax: ");
        Serial.print(currMeas->ax_raw);
        Serial.print(" ay: ");
        Serial.print(currMeas->ay_raw);
        Serial.print(" az: ");
        Serial.print(currMeas->az_raw);
        Serial.println();
        Serial.print("gx: ");
        Serial.print(currMeas->gx_raw);
        Serial.print(" gy: ");
        Serial.print(currMeas->gy_raw);
        Serial.print(" gz: ");
        Serial.print(currMeas->gz_raw);
        Serial.println();

    } else {
        Serial.print("No Current Measurement!");
        Serial.println();
    }
}

void MPU6050Plus::showVals(float data[]) {
        Serial.print("ax: ");
        Serial.print(data[0]);
        Serial.print(" ay: ");
        Serial.print(data[1]);
        Serial.print(" az: ");
        Serial.print(data[2]);
        Serial.println();
        Serial.print("gx: ");
        Serial.print(data[3]);
        Serial.print(" gy: ");
        Serial.print(data[4]);
        Serial.print(" gz: ");
        Serial.print(data[5]);
        Serial.println();
}