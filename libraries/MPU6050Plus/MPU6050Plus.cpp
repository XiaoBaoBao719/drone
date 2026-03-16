#include <MPU6050Plus.h>
#include <Arduino.h>

static inline float wrapAngleDeg(float angle) {
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
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
    dT = sampleT;
    preInterval = millis();

    gyroScaleFactor = 131.0;            // default to +/- 250 deg/s
    configureGyroScale(MPU_GYR_2000);    // default to +/- 250 deg/s
    configureAccScale(MPU_ACC_2G);      // default to +/- 2g

    invertedY = true;
    invertedZ = true;

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

void MPU6050Plus::configureGyroScale(GYRO_SCALE scale)
{
    uint8_t fs_select;
    int gyroScale;

    switch (scale)
    {
    case MPU_GYR_250:
        fs_select = 0;
        gyroScaleFactor = 131.0;
        gyroScale = 250;
        break;
    case MPU_GYR_500:
        fs_select = 1;
        gyroScaleFactor = 65.5;
        gyroScale = 500;
        break;
    case MPU_GYR_1000:
        fs_select = 2;
        gyroScaleFactor = 32.8;
        gyroScale = 1000;
        break;
    case MPU_GYR_2000:
        fs_select = 3;
        gyroScaleFactor = 16.4;
        gyroScale = 2000;
        break;
    default:
        // Invalid scale, default to 250 dps
        gyroScaleFactor = 131.0;
        gyroScale = 250;
        break;
    }

    mpu->setFullScaleGyroRange(fs_select);
    // Serial.print("Gyro scale set to +/- %d degrees/s with sensitivity of %.1f LSB/deg/s\n", gyroScale, gyroScaleFactor);
}

void MPU6050Plus::configureAccScale(ACCEL_SCALE scale)
{
    uint8_t afs_select;
    int accScale;

    switch (scale)
    {
    case MPU_ACC_2G:
        afs_select = 0;
        accScaleFactor = 16384.0;
        accScale = 2;
        break;
    case MPU_ACC_4G:
        afs_select = 1;
        accScaleFactor = 8192.0;
        accScale = 4;
        break;
    case MPU_ACC_8G:
        afs_select = 2;
        accScaleFactor = 4096.0;
        accScale = 8;
        break;
    case MPU_ACC_16G:
        afs_select = 3;
        accScaleFactor = 2048.0;
        accScale = 16;
        break;
    default:
        // Invalid scale, default to +/- 2g
        accScaleFactor = 16384.0;
        accScale = 2;
        break;
    }

    mpu->setFullScaleAccelRange(afs_select);
    // Serial.print("Accelerometer scale set to +/- %d g with sensitivity of %.1f LSB/g\n", accScale, accScaleFactor);
}

void MPU6050Plus::updateRawMeasurements() {
    int16_t _ax_raw, _ay_raw, _az_raw;
    int16_t _gx_raw, _gy_raw, _gz_raw;
    float angleGyroX, angleGyroY, angleGyroZ;
    float data[6];

    /* +++++ Read raw accel values +++++ */
    mpu->getAcceleration(&_ax_raw,&_ay_raw,&_az_raw);

    rawAccX = _ax_raw;
    rawAccY = _ay_raw;
    rawAccZ = _az_raw;

    /* Convert: Accelerometer Counts -> m/s^2 
    Raw accelerometer values, which are in units of Counts per "g" (9.81m/s^2), 
    are divided by the accelerometer sensitivity, also in units of Counts per g (9.81m/s^2).

    The product is multiplied by the gravity constant in order to get back to units of m/s^2 
    */
    accX = (_ax_raw / accScaleFactor) * G;  // x
    accY = (_ay_raw / accScaleFactor) * G;  // y
    accZ = (_az_raw / accScaleFactor) * G;  // z

    // Calculate filtered accelerometer values as the average of
    // the new reading and the last LP_FILTER_DEGREE filtered readings
    float _filtAccX = accX;
    float _filtAccY = accY;
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

    /* +++++ Read raw gyro values +++++ */
    mpu->getRotation(&_gx_raw,&_gy_raw,&_gz_raw);

    rawGyroX = _gx_raw - offset_gx;
    rawGyroY = _gy_raw - offset_gy;
    rawGyroZ = _gz_raw - offset_gz;

    /* Convert: Counts -> deg/s */
    gyroX = _gx_raw / gyroScaleFactor;
    gyroY = _gy_raw / gyroScaleFactor;
    gyroZ = _gz_raw / gyroScaleFactor;
}

/**
 * @brief Update the attitude estimates using a complementary filter
 */
void MPU6050Plus::complementaryFilter() {
    // float angleGyroX = 0.0, angleGyroY = 0.0, angleGyroZ = 0.0;
    /* Apply (RC) Low Pass filter */
    // this->filterMeasurements( data );

    /* Calculate angleAcc measurements using filtered Acc values */
    angleAccY = atan2( -filtAccX, 
                       sqrt( filtAccY * filtAccY + filtAccZ * filtAccZ ) ) * RAD_2_DEG;    // Calculate pitch tilt angle from accel in degs
    
    // angleAccX = atan2(filtAccY, filtAccZ) * RAD_2_DEG;                                     // Calculate the roll tilt angle from accel in degs
    angleAccX = atan2( filtAccY , sqrt( filtAccZ * filtAccZ + filtAccX * filtAccX ) ) * RAD_2_DEG;  // Calculate the roll tilt angle from accel in degs

    // unsigned long Tnew = millis();
    // float dt = (Tnew - preInterval) * 1e-3;
    // preInterval = Tnew;

    /* Integrate gyro measurements to update estimate of gyro angle */
    angleX += gyroX * dT;           // 'roll'       degs
    angleY += gyroY * dT;           // 'pitch'      degs
    angleZ += gyroZ * dT;           // 'yaw'        degs

    // ++++++++++ IF X-Y INVERTED ++++++++++
    // angleY = (COMPLEMENTARY_ALPHA * angleGyroX) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX;
    // angleX = (COMPLEMENTARY_ALPHA * angleGyroY) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY;
    // angleX = angleX * -1;   // invert X-axis for this convention
    // angleZ = (COMPLEMENTARY_ALPHA * angleGyroZ);           // add angleGyroZ since it is initially zero each loop

    // ++++++++++ IF X-Y NOT INVERTED ++++++++++
    // Update angles with accelerometer values
    angleX = (COMPLEMENTARY_ALPHA * angleX) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccX;
    angleY = (COMPLEMENTARY_ALPHA * angleY) + (1.0 - COMPLEMENTARY_ALPHA) * angleAccY;


    // angleX = (invertedX) ? -1 * angleX : angleX;
    // angleY = (invertedY) ? -1 * angleY : angleY;
    // angleZ = (invertedZ) ? -1 * angleZ : angleZ;
    // Wrap angles to -180 to +180 deg
    // angleX = wrapAngleDeg(angleX);
    // angleY = wrapAngleDeg(angleY);
    // angleZ = wrapAngleDeg(angleZ);
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

    Serial.println("Calibration complete.");
    Serial.print("Accel Offsets - X:");
    Serial.print(mean_acc_x);
    Serial.print(", Y:");
    Serial.print(mean_acc_y);
    Serial.print(", Z:");
    Serial.println(mean_acc_z);

    // // Apply offsets
    mpu->setXGyroOffset((mean_gyr_x));
    mpu->setYGyroOffset((mean_gyr_y));
    mpu->setZGyroOffset((mean_gyr_z));

    mpu->setXAccelOffset((mean_acc_x));
    mpu->setYAccelOffset((mean_acc_y));
    mpu->setZAccelOffset((mean_acc_z));

    Serial.println("Applied Offsets:");
    mpu->PrintActiveOffsets();

    /* Set IMU offsets */
    offset_ax = mean_acc_x;
    offset_ay = mean_acc_y;
    offset_az = mean_acc_z;
    offset_gx = mean_gyr_x;
    offset_gy = mean_gyr_y;
    offset_gz = mean_gyr_z;

    delay(1);
}

float* MPU6050Plus::angleAxisQuaternion() {
    /* Convert the current roll, pitch, yaw angles to a quaternion */
    // Convert angle axis values from degrees to radians
    float roll = angleX * DEG_2_RAD;
    float pitch = angleY * DEG_2_RAD;
    float yaw = angleZ * DEG_2_RAD;

    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);

    float qw = cr * cp * cy + sr * sp * sy;
    float qx = sr * cp * cy - cr * sp * sy;
    float qy = cr * sp * cy + sr * cp * sy;
    float qz = cr * cp * sy - sr * sp * cy;

    quaternion[0] = qw;
    quaternion[1] = qx;
    quaternion[2] = qy;
    quaternion[3] = qz;

    return quaternion;
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

void MPU6050Plus::printInvertedAxes() {
    Serial.print("invertX: ");
    Serial.print(invertedX);
    Serial.print(" invertY: ");
    Serial.print(invertedX);
    Serial.print(" invertZ: ");
    Serial.println(invertedX);
}