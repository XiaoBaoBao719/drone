// #include "Wire.h"
#include <Wire.h>
// extern TwoWire Wire1;
// #include "Arduino.h"
// #include "Arduino_LED_Matrix.h"  //Include the LED_Matrix library

// custom dependencies
#include <MovingAvg.h>
#include <MPU6050Plus.h>
#include <Arduino.h>

#define LED_PIN LED_BUILTIN
#define BUTTON_PIN D2
#define MPU_ADDR (0x68)

// make an instance of the library:
// ArduinoLEDMatrix matrix;

volatile byte ledState = LOW;
volatile int counts = 0;

// clock
unsigned long interval = 10;  // 10 ms update rate
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
uint32_t print_ts;
uint32_t loopTimer;


// MPU6050Plus mpu;
// MPU6050 mpu(0x68, &Wire1);
MPU6050Plus imu;
EulerRPY rpy;

// const float IMU_SAMPLE_FREQ_MS = 1000;  // millisecs
float IMU_SAMPLE_FREQ = 0.05;    // time interval between imu points (secs)

/* DIGITAL SIGNAL PROCESSING */
MovingAvg<5> movingAvg;


float euler_to_quaternion(float *q_, float yaw, float pitch, float roll) {

  roll = roll * DEG_2_RAD;
  pitch = pitch * DEG_2_RAD;
  yaw = yaw * DEG_2_RAD;

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

  q_[0] = qw;
  q_[1] = qx;
  q_[2] = qy;
  q_[3] = qz;
}
  

void setup() {
  Serial.begin(9600);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  // matrix.loadSequence(LEDMATRIX_ANIMATION_SPINNING_COIN);
  // matrix.begin();
  // matrix.play(true);

  /* I2C devices setup section */
  // Serial.println(F("Initialize I2C devices..."));
  // i2cSetup();
  Wire1.begin();
  Wire1.setClock(400000); // 400 kHz I2C clock

  imu.initialize(MPU_ADDR, &Wire1, IMU_SAMPLE_FREQ);
  
  // Verify connection with WHO_AM_I before proceeding
  uint8_t maxRetries = 10;
  uint8_t retryCount = 0;
  while (!imu.initialize() && retryCount < maxRetries) {
    Serial.print("WHO_AM_I verification failed (attempt ");
    Serial.print(retryCount + 1);
    Serial.print("/");
    Serial.print(maxRetries);
    Serial.println("). Retrying...");
    retryCount++;
    delay(500);
  }
  
  if (retryCount >= maxRetries) {
    Serial.println("FATAL: IMU initialization failed after max retries!");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  Serial.println("IMU initialized and verified.");

  /* Create MPU6050 Plus */
  // imu.initialize(&mpu, IMU_SAMPLE_FREQ);  print_ts = 0;

  // Flip imu Y-axis since default coodinate frame is ENU
  /* Print Gyro and Accel configs */

  /* Perform calibration */
  // imu.calibrateIMU();
  // imu.calcOffsets();
  // mpu.CalibrateAccel(20);
  // mpu.CalibrateGyro(20);
    // imu.setOffsetAccX(-981);
    // imu.setOffsetAccY(-307);
    // imu.setOffsetAccZ(1209);
    // imu.setOffsetGyroX(-4);
    // imu.setOffsetGyroY(-36);
    // imu.setOffsetGyroZ(-36);
  imu.resetCalibration();

  imu.setOffsetAccX(-997);
  imu.setOffsetAccY(-305);
  imu.setOffsetAccZ(1221);
  imu.setOffsetGyroX(-1);
  imu.setOffsetGyroY(-33);
  imu.setOffsetGyroZ(-39);


  imu.setCalibrated(true);

  // imu.invertY();
  imu.invertZ();

  imu.printInvertedAxes();

  // Serial.print("offset_acc_x:");
  // Serial.print(imu.getOffsetAccX());
  // Serial.print(",offset_acc_y:");
  // Serial.print(imu.getOffsetAccY());
  // Serial.print(",offset_acc_z:");
  // Serial.println(imu.getOffsetAccZ());

  movingAvg = MovingAvg<5>();

  delay(3000);
}

float q_[4];

void loop() {

  // Update measurements at 100 hz
  currMillis = millis();
  if (currMillis - prevMillis >= interval) {
    prevMillis = currMillis;

    imu.updateRawMeasurements();
    imu.complementaryFilter();
  }

  if ((millis() - print_ts) > interval) {
    // Serial.print("raw_acc_X:");
    // Serial.print(imu.getAccXRaw());
    // Serial.print(",raw_acc_Y:");
    // Serial.print(imu.getAccYRaw());
    // Serial.print(",raw_acc_Z:");
    // Serial.println(imu.getAccZRaw());

    // Serial.print("acc_X:");
    // Serial.print(imu.getAccX());
    // Serial.print(",acc_Y:");
    // Serial.print(imu.getAccY());
    // Serial.print(",acc_Z:");
    // Serial.println(imu.getAccZ());

    // Serial.print("gyro_X:");
    // Serial.print(imu.getGyroX());
    // Serial.print(",gyro_Y:");
    // Serial.print(imu.getGyroY());
    // Serial.print(",gyro_Z:");
    // Serial.print(imu.getGyroZ());
    // Serial.println();

    // Serial.print("X:");
    // Serial.print(imu.getAngleX());
    
    Serial.print(",Y:");
    Serial.print(imu.getAngleY());

    /* Experimental moving avg filtering */
    // float rawAngle = imu.getAngleY();
    // float output;
    // movingAvg.filter(rawAngle, output);
    // Serial.print(output);

    // Serial.print(",Z:");
    // Serial.print(imu.getAngleZ());
    Serial.println();

    // imu.printInvertedAxes();

    // float* q = imu.angleAxisQuaternion();
    // euler_to_quaternion(q_, imu.getAngleZ(), imu.getAngleY(), imu.getAngleX());
    // Serial.print(q_[0], 2); Serial.print(",");
    // Serial.print(q_[1], 2); Serial.print(",");
    // Serial.print(q_[2], 2); Serial.print(",");
    // Serial.print(q_[3], 2); Serial.print("\n");

    //   Serial.print("biasx:");
    // Serial.print(imu.getBiasGyroX());
    // Serial.print("biasy:");
    // Serial.print(imu.getBiasGyroY());
    // Serial.print("biasz:");
    // Serial.println(imu.getBiasGyroZ());
    print_ts = millis();
  }

  // wait for control timer loop to finish
  // while (micros() - loopTimer < IMU_SAMPLE_FREQ_MS) {
  //   loopTimer = micros();
  // }
}

void doThing() {
  // pass
}