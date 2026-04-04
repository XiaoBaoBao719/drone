#include <MPU6050Plus.h>
#include <I2Cdev.h>
#include <Arduino.h>

MPU6050Plus::MPU6050Plus()
{
    devAddr = MPU6050_ADRR;
    wireObj = &Wire;
    dT = 0.01;
    currMeas = nullptr;
    lastMeas = nullptr;
    gyroScaleFactor = 131.0;
    gyroScale = 250;
    accScaleFactor = 16384.0;
    accScale = 2;
    for (size_t i = 0; i < sizeof(filters) / sizeof(filters[0]); ++i) {
        filters[i] = nullptr;
    }
}

MPU6050Plus::MPU6050Plus(uint8_t devAddr_, TwoWire *wireObj_, float sampleT)
{
    initialize(devAddr_, wireObj_, sampleT);
}

void MPU6050Plus::initialize(uint8_t devAddr_, TwoWire *wireObj_, float sampleT)
{
    devAddr = devAddr_;
    wireObj = wireObj_;
    dT = sampleT;                           // Update sampling period for taking measurements

    configureGyroScale(MPU_GYR_250);        // default to +/- 250 deg/s
    configureAccScale(MPU_ACC_2G);          // default to +/- 2g

    /* Create filters for each measurement channel */
    for (size_t n = 0; n < sizeof(filters)/sizeof(filters[0]); n++ ) {
        filters[n] = new FilterOnePole(LOWPASS, FILTER_SAMPLING_FREQ);
    }
}

void MPU6050Plus::configureGyroScale(GYRO_SCALE scale)
{
    uint8_t fs_select;

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

    setFullScaleGyroRange(fs_select);
    // Serial.print("Gyro scale set to +/- %d degrees/s with sensitivity of %.1f LSB/deg/s\n", gyroScale, gyroScaleFactor);
}

void MPU6050Plus::configureAccScale(ACCEL_SCALE scale)
{
    uint8_t afs_select;

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

    setFullScaleAccelRange(afs_select);
    // Serial.print("Accelerometer scale set to +/- %d g with sensitivity of %.1f LSB/g\n", accScale, accScaleFactor);
}

uint8_t MPU6050Plus::getDeviceID() {
    uint8_t id = 0;
    I2Cdev::readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer, I2Cdev::readTimeout, wireObj);
    id = buffer[0];
    return id;
}

bool MPU6050Plus::testConnection() {
    return getDeviceID() == 0x34;
}

void MPU6050Plus::updateRawMeasurements() {
    int16_t _ax_raw, _ay_raw, _az_raw;
    int16_t _gx_raw, _gy_raw, _gz_raw;
    float angleGyroX, angleGyroY, angleGyroZ;
    float data[6];

    /* +++++ Read raw accel values +++++ */
    getAcceleration(&_ax_raw,&_ay_raw,&_az_raw);

    rawAccX = _ax_raw - this->getOffsetAccX();
    rawAccY = _ay_raw - this->getOffsetAccY();
    rawAccZ = _az_raw - this->getOffsetAccZ();

    /* Convert: Accelerometer Counts -> m/s^2 
    Raw accelerometer values, which are in units of Counts per "g" (9.81m/s^2), 
    are divided by the accelerometer sensitivity, also in units of Counts per g (9.81m/s^2).

    The product is multiplied by the gravity constant in order to get back to units of m/s^2 
    */
    accX = (_ax_raw / this->getAccScaleFactor() ) * G;  // x
    accY = (_ay_raw / this->getAccScaleFactor() ) * G;  // y
    accZ = (_az_raw / this->getAccScaleFactor() ) * G;  // z

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
    getRotation(&_gx_raw,&_gy_raw,&_gz_raw);

    rawGyroX = _gx_raw - this->getOffsetGyroX();
    rawGyroY = _gy_raw - this->getOffsetGyroY();
    rawGyroZ = _gz_raw - this->getOffsetGyroZ();

    /* Convert: Counts -> deg/s */
    gyroX = _gx_raw / this->getGyroScaleFactor();
    gyroY = _gy_raw / this->getGyroScaleFactor();
    gyroZ = _gz_raw / this->getGyroScaleFactor();
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
    // Update the measurement based on it's respective filter (all 6 channels)
    for (size_t index = 0; index < 6; index++)
    {
        if (filters[index] != nullptr) {
            data[index] = filters[index]->input(data[index]);
        }
    }
}

/**
 * 
 */
void MPU6050Plus::getMeasurementAvgs(float data[], size_t size)
{
    assert(size == 6);          // run time assertion that data is exactly a six element array
    long measCount = 0, buff_ax = 0, buff_ay = 0, buff_az = 0,
          buff_gx  = 0, bugg_gy = 0, buff_gz = 0;
          
    while ( measCount < (CAL_BUFFER_LEN + NUM_ELEM_SKIP + 1) ) 
    {  // The first NUM_ELEM_SKIP measurements are discarded to allow sensor to settle
        this->updateRawMeasurements();

        if (measCount > NUM_ELEM_SKIP && measCount <= (CAL_BUFFER_LEN + NUM_ELEM_SKIP))
        {
            buff_ax = buff_ax + rawAccX;
            buff_ay = buff_ay + rawAccY;
            buff_az = buff_az + rawAccZ;
            buff_gx = buff_gx + rawGyroX;
            buff_gy = buff_gy + rawGyroY;
            buff_gz = buff_gz + rawGyroZ;
        }

        measCount++;
        delay(2);
    }
    /* data frame format = { accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z }*/
    data[0] = buff_ax / CAL_BUFFER_LEN;
    data[1] = buff_ay / CAL_BUFFER_LEN;
    data[2] = buff_az / CAL_BUFFER_LEN;
    data[3] = buff_gx / CAL_BUFFER_LEN;
    data[4] = buff_gy / CAL_BUFFER_LEN;
    data[5] = buff_gz / CAL_BUFFER_LEN;
}

/**
 * 
 */
void MPU6050Plus::calibrate_(float data[], size_t size)
{
    /* Reset the IMU's offsets */
    offset_ax = -1 * data[0] / this->getAccScale();
    offset_ay = -1 * data[1] / this->getAccScale();
    
    /** 
     * Since this vector already points in the opposite direction
     * subtract from accel scale factor (i.e. 16384 counts).
     */ 
    offset_az = (this->getAccScaleFactor() -  data[2] ) / this->getAccScale();
    
    offset_gx = -1 * data[3] / this->getGyroScale();
    offset_gy = -1 * data[4] / this->getGyroScale();
    offset_gz = -1 * data[5] / this->getGyroScale();
    
    uint8_t numSensorsReady = 0;
    while (true) {
            
        setXAccelOffset(offset_ax);
        setYAccelOffset(offset_ay);
        setZAccelOffset(offset_az);
        setXGyroOffset(offset_gx);
        setYGyroOffset(offset_gy);
        setZGyroOffset(offset_gz);

        this->getMeasurementAvgs(data, size);

        if ( abs( data[0] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_ax = offset_ax - data[0] / ACCEL_DEADZONE;

        if ( abs( data[1] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_ay = offset_ay - data[1] / ACCEL_DEADZONE;

        if ( abs( this->getAccScaleFactor() - data[2] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_az = offset_az + (this->getAccScaleFactor() - data[2] ) / ACCEL_DEADZONE;

        if ( abs( data[3] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_gx = offset_gx - data[3] / (GYRO_DEADZONE + 1);
    
        if ( abs( data[4] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_gy = offset_gy - data[4] / (GYRO_DEADZONE + 1);
    
        if ( abs( data[5] ) <= ACCEL_DEADZONE ) numSensorsReady++;
        else    offset_gz = offset_gz - data[5] / (GYRO_DEADZONE + 1);
    
        if (numSensorsReady == 6)    // there are six total 'sensors' on the IMU
            break;
    }
}

/**
 * 
 */
void MPU6050Plus::calibrateIMU() {
    uint8_t elements = 6;
    float data[elements] = {0.f};

    getMeasurementAvgs(data, elements);
    calibrate_(data, elements);
    Serial.println("Calibration complete.");
}

/**
 * @brief
 */
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

void MPU6050Plus::getAcceleration(int16_t* x, int16_t* y, int16_t* z) {
    I2Cdev::readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 6, buffer, I2Cdev::readTimeout, wireObj);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050Plus::getRotation(int16_t* x, int16_t* y, int16_t* z) {
    I2Cdev::readBytes(devAddr, MPU6050_RA_GYRO_XOUT_H, 6, buffer, I2Cdev::readTimeout, wireObj);
    *x = (((int16_t)buffer[0]) << 8) | buffer[1];
    *y = (((int16_t)buffer[2]) << 8) | buffer[3];
    *z = (((int16_t)buffer[4]) << 8) | buffer[5];
}

void MPU6050Plus::setFullScaleGyroRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range, wireObj);
}

void MPU6050Plus::setFullScaleAccelRange(uint8_t range) {
    I2Cdev::writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range, wireObj);
}

void MPU6050Plus::setXGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset, wireObj);
}

void MPU6050Plus::setYGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset, wireObj);
}

void MPU6050Plus::setZGyroOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset, wireObj);
}

void MPU6050Plus::setXAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_XA_OFFS_H, offset, wireObj);
}

void MPU6050Plus::setYAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_YA_OFFS_H, offset, wireObj);
}

void MPU6050Plus::setZAccelOffset(int16_t offset) {
    I2Cdev::writeWord(devAddr, MPU6050_RA_ZA_OFFS_H, offset, wireObj);
}

void MPU6050Plus::showVals(float data[]) {
    Serial.print("ax: ");   Serial.print(data[0]);
    Serial.print(" ay: ");  Serial.print(data[1]);
    Serial.print(" az: ");  Serial.print(data[2]);
    Serial.println();

    Serial.print("gx: ");   Serial.print(data[3]);
    Serial.print(" gy: ");  Serial.print(data[4]);
    Serial.print(" gz: ");  Serial.print(data[5]);
    Serial.println();
}

void MPU6050Plus::printInvertedAxes() {
    Serial.print("invertX: ");  Serial.print(invertedX);
    Serial.print(" invertY: "); Serial.print(invertedY);
    Serial.print(" invertZ: "); Serial.print(invertedZ);
    Serial.println();
}

static extern bool ImuPoint::isValid(){
    // Check that all values are real values
    return ( !(isnan(ax) || isinf(ax)) &&
             !(isnan(ay) || isinf(ay)) &&
             !(isnan(az) || isinf(az)) &&
             !(isnan(gx) || isinf(gx)) &&
             !(isnan(gy) || isinf(gy)) &&
             !(isnan(gz) || isinf(gz)) );
}

static inline float wrapAngleDeg(float angle) {
    if (angle > 180.0) {
        angle -= 360.0;
    } else if (angle < -180.0) {
        angle += 360.0;
    }
    return angle;
}