// #include "Wire.h"
#include <Wire.h>
// extern TwoWire Wire1;
// #include "Arduino.h"
// #include "Arduino_LED_Matrix.h"  //Include the LED_Matrix library

// custom dependencies
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

// void i2cSetup() {
//   // join I2C bus (I2Cdev library doesn't do this automatically)
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//   Wire1.begin();
//   Wire1.setClock(200000);
//   // TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
// #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//   Fastwire::setup(200, true);
// #endif
// }


// # Source - https://stackoverflow.com/q/53033620
// # Posted by Amir, modified by community. See post 'Timeline' for change history
// # Retrieved 2026-03-12, License - CC BY-SA 4.0

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
  imu.calibrateIMU();
  // imu.calcOffsets();
  // mpu.CalibrateAccel(20);
  // mpu.CalibrateGyro(20);

  // imu.invertZ();

  imu.printInvertedAxes();

  Serial.println("+++++++++++++++++++++ IMU  CALIBRATION OFFSETS ++++++++++++++++++++++++");
  Serial.print("offset_acc_x:");  Serial.print(imu.getOffsetAccX());    Serial.print("\t");
  Serial.print(",offset_acc_y:"); Serial.print(imu.getOffsetAccY());    Serial.print("\t");
  Serial.print(",offset_acc_z:"); Serial.print(imu.getOffsetAccZ());        Serial.print("\n");
  Serial.print(",offset_gyro_x:");Serial.print(imu.getOffsetGyroX());   Serial.print("\t");
  Serial.print(",offset_gyro_y:");Serial.print(imu.getOffsetGyroY());   Serial.print("\t");
  Serial.print(",offset_gyro_z:");Serial.print(imu.getOffsetGyroZ());   Serial.print("\n");
  Serial.println("++++++++++++++++++++++++++++++ OFFSETS ++++++++++++++++++++++++");
  delay(3000);
}

float q_[4];

void loop() {

 
}
