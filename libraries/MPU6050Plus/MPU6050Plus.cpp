#include <MPU6050Plus.h>
#include <Arduino.h>

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

    mpu->setXGyroOffset(0);
    mpu->setYGyroOffset(0);
    mpu->setZGyroOffset(0);
    
    mpu->setXAccelOffset(0);
    mpu->setYAccelOffset(0);
    mpu->setZAccelOffset(0);
    /* Z axis bias calibration */
    // this->calcOffsets();
    delay(100);
}

void MPU6050Plus::updateRawMeasurements() {
    int16_t _ax_raw, _ay_raw, _az_raw;
    int16_t _gx_raw, _gy_raw, _gz_raw;
    float angleGyroX, angleGyroY, angleGyroZ;
    float data[6];

    mpu->getAcceleration(&_ax_raw,&_ay_raw,&_az_raw);

    rawAccX = _ax_raw - offset_ax;
    rawAccY = _ay_raw - offset_ay;
    rawAccZ = _az_raw - offset_az;

    /* Convert: Counts -> m/s^2 */
    accX = _ax_raw * G / SENSITIVITY_ACCEL;  // x
    accY = _ay_raw * G / SENSITIVITY_ACCEL;  // y
    accZ = _az_raw * G / SENSITIVITY_ACCEL;  // z

    // Calculate filtered accelerometer values as the average of
    // the new reading and the last LP_FILTER_DEGREE filtered readings
    float _filtAccX = accX;
    float _filtAccY = accZ;
    float _filtAccZ = accZ;
    for(int i = 0; i < LP_FILTER_DEGREE; i++)
    {
        _filtAccX += accXHist[i];
        _filtAccY += accYHist[i];
        _filtAccZ += accZHist[i];
    }
    
    filtAccX = _filtAccX / (LP_FILTER_DEGREE + 1);
    filtAccY = _filtAccY / (LP_FILTER_DEGREE + 1);
    filtAccZ = _filtAccZ / (LP_FILTER_DEGREE + 1);

    // Update the history arrays with the new filtered value
    accXHist[lpFilterIndex] = filtAccX;
    accYHist[lpFilterIndex] = filtAccY;
    accZHist[lpFilterIndex] = filtAccZ;

    // make the history buffers act as ring buffers
    lpFilterIndex++;
    if(lpFilterIndex == LP_FILTER_DEGREE)
    {
        lpFilterIndex = 0;
    }

    mpu->getRotation(&_gx_raw,&_gy_raw,&_gz_raw);

    rawGyroX = _gx_raw - offset_gx;
    rawGyroY = _gy_raw - offset_gy;
    rawGyroZ = _gz_raw - offset_gz;

    /* Convert: Counts -> deg/s */
    gyroX = _gx_raw / SENSITIVITY_GYRO;
    gyroY = _gy_raw / SENSITIVITY_GYRO;
    gyroZ = _gz_raw / SENSITIVITY_GYRO;
}

void MPU6050Plus::updateEstimates() {
    // float angleGyroX = 0.0, angleGyroY = 0.0, angleGyroZ = 0.0;
    /* Apply (RC) Low Pass filter */
    // this->filterMeasurements( data );

    /* Calculate angleAcc measurements */
    angleAccY = atan2( -filtAccX, ( sqrt( filtAccY* filtAccY+ filtAccZ * filtAccZ )) ) * RAD_2_DEG; // Calculate pitch tilt angle from accel in radians
    angleAccX = atan2( filtAccY , filtAccZ) * RAD_2_DEG;  // Calculate the roll tilt angle from accel in radians
    // TODO: Calcualte the yaw angle from a magnetometer reading

    unsigned long Tnew = millis();
    float dt = (Tnew - preInterval) * 1e-3;
    preInterval = Tnew;

    /* Integrate gyro measurements to update estimate of gyro angle */
    angleGyroX += gyroX * dT;           // 'roll'
    angleGyroY += gyroY * dT;           // 'pitch'
    angleGyroZ += gyroZ * dT; // biasGyroZ;           // 'yaw'

    /* Sensor fusion of gyro and accel angles */
    // TODO: EKF sensor fusion

    // Ccomplementary filter sensor fusion  (X and Y axis swapped w/ X inverted)
    angleY = (COMPLEMENTARY_ALPHA * angleGyroX) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX;
    angleX = (COMPLEMENTARY_ALPHA * angleGyroY) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY;
    angleX = angleX * -1;   // invert X-axis for this convention
    angleZ = (COMPLEMENTARY_ALPHA * angleGyroZ);           // add angleGyroZ since it is initially zero each loop

    if ( angleX > 180.0 ) {
        angleX = angleX - 360.0;
    } else if (angleX <= -180.0) {
        angleX = angleX + 360.0;
    }

    if ( angleY > 180.0 ) {
        angleY = angleY - 360.0;
    } else if (angleY <= -180.0) {
        angleY = angleY + 360.0;
    }

    if ( angleZ > 180.0 ) {
        angleZ = angleZ - 360.0;
    } else if (angleZ <= -180.0) {
        angleZ = angleZ + 360.0;
    }


    // // Ccomplementary filter sensor fusion       (X - foward, Y - facing left )
    // angleX = COMPLEMENTARY_ALPHA * angleGyroX + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX;
    // angleY = COMPLEMENTARY_ALPHA * angleGyroY + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY;
    // angleZ = COMPLEMENTARY_ALPHA * angleGyroZ;           // add angleGyroZ since it is initially zero each loop
}

void MPU6050Plus::filterMeasurements(float data[])
{
    // Update the measurement based on it's respective filter (ps. they are all low pass filters)
    for (size_t index = 0; index < sizeof(data) / sizeof(data[0]); index++)
    {
        data[index] = filters[index]->input(data[index]);
    }
}

void MPU6050Plus::calcOffsets()
{
    float rawAcc[] = {0.0, 0.0, 0.0}; // x, y, z
    float rawGyr[] = {0.0, 0.0, 0.0}; // x, y, z
    float totals[] = {0.0, 0.0, 0.0}; // x_total, y_total, z_total
    int32_t mean_acc_x = 0.0, mean_acc_y = 0.0, mean_acc_z = 0.0;
    int32_t mean_gyr_x = 0.0, mean_gyr_y = 0.0, mean_gyr_z = 0.0;

    float diffX, diffY, diffZ;

    /* Reset the IMU's offsets */
    mpu->setXGyroOffset(0);
    mpu->setYGyroOffset(0);
    mpu->setZGyroOffset(0);
    mpu->setXAccelOffset(0);
    mpu->setYAccelOffset(0);
    mpu->setZAccelOffset(0);

    for (int i = 0; i < NUM_CALIB_CYCLES + 200; i++)
    {

        this->updateRawMeasurements();

        if (i >= 200)
        { // Start calibrating after first 100 cycles to allow sensors to settle

            rawAcc[0] += rawAccX;
            rawAcc[1] += rawAccY;
            rawAcc[2] += rawAccZ;

            rawGyr[0] += rawGyroX;
            rawGyr[1] += rawGyroY;
            rawGyr[2] += rawGyroZ;

            // rawAcc[0] += rawAccX / NUM_CALIB_CYCLES;
            // rawAcc[1] += rawAccY / NUM_CALIB_CYCLES;
            // rawAcc[2] += rawAccZ / NUM_CALIB_CYCLES;

            // rawGyr[0] += rawGyroX / NUM_CALIB_CYCLES;
            // rawGyr[1] += rawGyroY / NUM_CALIB_CYCLES;
            // rawGyr[2] += rawGyroZ / NUM_CALIB_CYCLES;

            Serial.print("x_avg:");
            Serial.print(rawAcc[0]);
            Serial.print(",y_avg:");
            Serial.print(rawAcc[1]);
            Serial.print(",z_avg:");
            Serial.println(rawAcc[2]);
        }

        delay(10);
    }

    mean_acc_x = rawAcc[0] / NUM_CALIB_CYCLES;
    mean_acc_y = rawAcc[1] / NUM_CALIB_CYCLES;
    mean_acc_z = rawAcc[2] / NUM_CALIB_CYCLES;

    mean_gyr_x = rawGyr[0] / NUM_CALIB_CYCLES;
    mean_gyr_y = rawGyr[1] / NUM_CALIB_CYCLES;
    mean_gyr_z = rawGyr[2] / NUM_CALIB_CYCLES;

    // // Apply offsets
    // mpu->setXGyroOffset((mean_gyr_x));
    // mpu->setYGyroOffset((mean_gyr_y));
    // mpu->setZGyroOffset((mean_gyr_z));

    // mpu->setXAccelOffset((mean_acc_x));
    // mpu->setYAccelOffset((mean_acc_y));
    // mpu->setZAccelOffset((mean_acc_z));

    /* Set IMU offsets */
    offset_ax = mean_acc_x;
    offset_ay = mean_acc_y;
    offset_az = mean_acc_z;
    offset_gx = mean_gyr_x;
    offset_gy = mean_gyr_y;
    offset_gz = mean_gyr_z;

    delay(1);
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