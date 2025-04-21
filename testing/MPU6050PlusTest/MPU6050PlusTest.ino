#include "Wire.h"
// #include "Arduino.h"
#include "Arduino_LED_Matrix.h"  //Include the LED_Matrix library

// custom dependencies
#include <MPU6050Plus.h>

#define LED_PIN LED_BUILTIN
#define BUTTON_PIN D2

// make an instance of the library:
ArduinoLEDMatrix matrix;

volatile byte ledState = LOW;
volatile int counts = 0;

// clock
uint32_t print_ts;
uint32_t loopTimer;

// MPU6050 mpu(0x69);
MPU6050 mpu;
MPU6050Plus imu;
EulerRPY rpy;

const float IMU_SAMPLE_FREQ_MS = 1000;  // millisecs
const float IMU_SAMPLE_FREQ = 0.001;    // time interval between imu points (secs)

void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000);
  // TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void setup() {
  Serial.begin(115200);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);
  // matrix.loadSequence(LEDMATRIX_ANIMATION_SPINNING_COIN);
  // matrix.begin();
  // matrix.play(true);

  /* I2C devices setup section */
  Serial.println(F("Initialize I2C devices..."));
  i2cSetup();

  /* IMU setup section */
  Serial.println(F("Initialize MPU..."));
  mpu.initialize();
  Serial.println("Testing MPU connection...");
  // Check if MPU is ready to send bytes
  while (mpu.testConnection() == false) {
    Serial.println("MPU connection failed! Retrying...");
    mpu.testConnection();
    delay(500);
  }

  /* Create MPU6050 Plus */
  imu.initialize(&mpu, IMU_SAMPLE_FREQ);
  print_ts = 0;
}

void loop() {
  imu.updateMeasurement();

  if ((millis() - print_ts) > 10) {
    Serial.print("x: ");
    Serial.print(imu.getAngleX());
    Serial.print(" y: ");
    Serial.print(imu.getAngleY());
    Serial.print(" z: ");
    Serial.print(imu.getAngleZ());
    Serial.println();
    print_ts = millis();
  }

  // wait for control timer loop to finish
  while (micros() - loopTimer < IMU_SAMPLE_FREQ_MS) {
    loopTimer = micros();
  }
}

void doThing() {
  // pass
}