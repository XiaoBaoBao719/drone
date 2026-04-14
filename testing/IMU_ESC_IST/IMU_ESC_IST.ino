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











bool mpuIntStatus = true;

float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

float angleRateX = 0.0;
float angleRateY = 0.0;
float angleRateZ = 0.0;


void updateSensors() {
#if SITL_MODE
    read_sitl_imu();
    // Use simulated IMU data
    angleX = sitl_imu.gyro[0] * 180.0 / PI;  // Convert rad/s to deg/s
    angleY = sitl_imu.gyro[1] * 180.0 / PI;
    angleZ = sitl_imu.gyro[2] * 180.0 / PI;
#else

    imu.updateRawMeasurements(); // read from IMU wrapper
    imu.complementaryFilter();       // get attitude estimate

    angleX = imu.getAngleX();
    angleY = imu.getAngleY();
    angleZ = imu.getAngleZ();

    angleRateX = imu.getGyroX();
    angleRateY = imu.getGyroY();
    angleRateZ = imu.getGyroZ();

#endif

#if DEBUG_ANGLE
    Serial.print(",angleX:");        Serial.print(angleX);       Serial.print(" ");
    Serial.print(",angleY:");            Serial.print(angleY);           Serial.print(" ");
    Serial.print(",angleZ:");           Serial.print(angleZ);          Serial.print(" ");
    Serial.println();
#endif
}


void updateMotors() {
    /* Constrain max throttle input */
    // filtInputThrottle = constrain(filtInputThrottle, MIN_THROTTLE, MAX_THROTTLE);
    /* Convert PID outputs into motor commands */
    mixMotors(rcThrottle, pidRollRate, pidPitchRate, pidYawRate);

#if DEBUG_MOTOR
    Serial.print(",m1:");
    Serial.print(motors.cmd[0]);
    Serial.print(",m2:");
    Serial.print(motors.cmd[1]);
    Serial.print(",m3:");
    Serial.print(motors.cmd[2]);
    Serial.print(",m4:");
    Serial.print(motors.cmd[3]);
    Serial.println();
#endif

    /* Send the motor speed commands to individual ESCS */
    if (receiverValueRaw[CH_THR - 1] < MIN_THROTTLE)
    {
#if SITL_MODE
      motor_pwm[0] = ESC_STOP;
      motor_pwm[1] = ESC_STOP;
      motor_pwm[2] = ESC_STOP;
      motor_pwm[3] = ESC_STOP;
      write_sitl_pwm();
#else
      /* Command STOP to ESCs! */
      m1_esc.speed(ESC_STOP);
      m2_esc.speed(ESC_STOP);
      m3_esc.speed(ESC_STOP);
      m4_esc.speed(ESC_STOP);
#endif
      resetPID();
    }
    else
    {
#if SITL_MODE
      motor_pwm[0] = (uint16_t)motors.cmd[0];
+      motor_pwm[1] = (uint16_t)motors.cmd[1];
      motor_pwm[2] = (uint16_t)motors.cmd[2];
      motor_pwm[3] = (uint16_t)motors.cmd[3];
      write_sitl_pwm();
#else
      /* Command motor speeds to ESCS! */
      m1_esc.speed(motors.cmd[0]);
      m2_esc.speed(motors.cmd[1]);
      m3_esc.speed(motors.cmd[2]);
      m4_esc.speed(motors.cmd[3]);
#endif
    }
}

void updateRC() {
  readReceiver(); // read PPM from the RC remote

    /* String debug statements */
    #if DEBUG_INPUT
      Serial.print(",Throttle:");        Serial.print(rcThrottle);       Serial.print(" ");
      Serial.print(",Roll:");            Serial.print(rcRoll);           Serial.print(" ");
      Serial.print(",Pitch:");           Serial.print(rcPitch);          Serial.print(" ");
      Serial.print(",Yaw:");             Serial.print(rcYaw);            Serial.print(" ");
      Serial.println();
    #endif
}



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
#if SITL_MODE
  setup_sitl_serial();
#endif
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  /** ============================================================ 
   *                    Motor Setup 
   *  ============================================================
   */
  Serial.println(F("Arming ESCS..."));
  pinMode(LED_BUILTIN, HIGH);
  // delay(500);
  m1_esc.arm();  m1_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 1 armed.");
  m2_esc.arm();  m2_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 2 armed.");
  m3_esc.arm();  m3_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 3 armed.");
  m4_esc.arm();  m4_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 4 armed.");
  pinMode(LED_BUILTIN, HIGH);
  delay(3500);

  /** ============================================================
   *                    I2C Setup
   *  ============================================================
   */
  /* I2C devices setup section */
  // Serial.println(F("Initialize I2C devices..."));
  // i2cSetup();
  Wire1.begin();
  Wire1.setClock(400000); // 400 kHz I2C clock

  /** ============================================================
   *                    MPU6050 Setup
   *  ============================================================
   */
  
  /* Create IMU data wrapper */
  imu.initialize(MPU_ADDR, &Wire1, IMU_SAMPLE_FREQ_SEC);

  Serial.println("Testing IMU connection...");
  // Check if MPU is ready to send bytes
  while (imu.testConnection() == false)
  {
    Serial.println("MPU connection failed! Retrying...");
    imu.testConnection();
    delay(500);
  }
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
  /* Calibrate the IMU accel/gyro offsets if not known */
  // bool calibrationResult = imu.calibrateIMU();
  // imu.setCalibrated(calibrationResult);

  /* If the calibration offsets are known, apply them here */
  imu.setOffsetAccX(-981);
  imu.setOffsetAccY(-307);
  imu.setOffsetAccZ(1209);
  imu.setOffsetGyroX(-4);
  imu.setOffsetGyroY(-36);
  imu.setOffsetGyroZ(-36);
  imu.setCalibrated(true);
  Serial.println("IMU calibrated");

  /** ============================================================
   *                   RC PPM Receiver Setup
   *  ============================================================
   */ 
  ppm.begin(PPM_INTERRUPT, false);

  /* Safety check: if throttle above threshold do not advance */
  uint32_t init_throttle = MAX_INIT_THROTTLE;
  readReceiver();
  while (init_throttle >= MAX_INIT_THROTTLE) {
    Serial.println(init_throttle);
    init_throttle = rcThrottle;
    readReceiver();
    delay(100);
  }
  Serial.println(F("RC reciever ready!"));
  delay(100);

  /** ===========================================================
   *                    End of Setup.
   *  ===========================================================
   */
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println(F("Flight Controller Setup Complete!"));
}

float q_[4];

void loop() {

  /* Update the RC receiver data at a rate of 10 Hz */
  currRcRecTimer = millis();
  if ((currRcRecTimer - prevRcRecTimer) > RCREC_FREQ_MS)
  {
    updateRC();
    prevRcRecTimer = currRcRecTimer;
  }


  /* Update the IMU data at a rate of 500 Hz*/
  currImuTimer = millis();
  if ((currImuTimer - prevImuTimer) > IMU_SAMPLE_FREQ_MS)
  {
    updateSensors();
    prevImuTimer = currImuTimer; // update prev timer
  }


  /* Update the motors loop and motors at a rate of 500 Hz */
  currControlTimer = millis();
  if ((currControlTimer - prevControlTimer) > CONTROL_LOOP_TIME_MS)
  {
    // updateControllers();
    updateMotors();
    prevControlTimer = currControlTimer;
  } /* End of control loop */
}