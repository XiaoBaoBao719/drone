#include "ESC.h"
#include "Wire.h"
#include "ppm.h"

#include <MPU6050Plus.h>

// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* FS-iA6B channel map
 *  Ch 1 -  Yaw
 *  Ch 2 -  Pitch
 *  Ch 3 -  Throttle
 *  Ch 4 -  Roll
 *  Ch 5 -  L knob
 *  Ch 6 -  R knob
 */

// PPM channel layout (update for your situation)
#define THROTTLE 3
#define ROLL 1
#define PITCH 2
#define YAW 4
#define SWITCH3WAY_1 5
#define BUTTON 6
#define SWITCH3WAY_2 7  // (NOT USED) trim-pot for left/right motor mix  (face trim)
#define POT 8           // (NOT USED) trim-pot on the (front left edge trim)

#define MAN_LP_FILTER_DEGREE (8)
float inputThrottleHist[MAN_LP_FILTER_DEGREE] = { 0.0 };
static float rcThrottle = 0.0f;  // moving average filtered rc value
static float rcThrottleRaw = 0.0f;

short throttle = 0;  // temporary var that holds the raw ppm counts

float filtInputThrottle = 0.0;
short lpFilterIndex = 0;

// Loop interval time
const long motor_interval = 4;  // 4 ms  update rate
unsigned long previousMillis = 0;

// Initialize a PPMReader on digital pin 3 with 6 expected channels.
byte interruptPin = 3;
byte channelAmount = 6;

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
/* ESC & MOTOR control vars */

#define MOTOR_1_PWM (5)
#define MOTOR_2_PWM (6)
#define MOTOR_3_PWM (9)
#define MOTOR_4_PWM (10)

int motorPins[] = { 5, 6, 9, 10 };

#define SPEED_MIN (1000)
#define SPEED_MAX (1800)
#define ESC_STOP (500)
#define MIN_ARM_TIME (500)


#define MIN_THROTTLE (1050)
#define MAX_THROTTLE (SPEED_MAX)

ESC m1_esc(MOTOR_1_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m2_esc(MOTOR_2_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m3_esc(MOTOR_3_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);
ESC m4_esc(MOTOR_4_PWM, SPEED_MIN, SPEED_MAX, MIN_ARM_TIME);


ESC motorESCs[] = { m1_esc, m2_esc, m3_esc, m4_esc };
int oESC;

/* Quadcopter Motor Layout

   (M4)   (Front-X)   (M1)
      \\            //
       \\   ----   //
         |        |
(Left-Y) |  (Z)   |   (Right)
         |        |
       //   ----   \\
      //            \\
   (M3)   (Back)   (M2)

   M1 - CCW
   M2 - CW
   M3 - CCW
   M4 - CW
*/

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/** ------------   IMU  VARIABLES AND FUNCTIONS -----------------------
*
*
* -------------------------------------------------------------------- */

#define LED_PIN LED_BUILTIN
#define BUTTON_PIN D2
#define MPU_ADDR (0x68)

// make an instance of the library:
// ArduinoLEDMatrix matrix;

volatile byte ledState = LOW;
volatile int counts = 0;

// clock
unsigned long imu_interval = 10;  // 1 ms update rate
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
unsigned long imu_print_interval = 1;
uint32_t loopTimer;

// MPU6050 mpu(0x68, &Wire1);
MPU6050Plus imu;
EulerRPY rpy;

// const float IMU_SAMPLE_FREQ_MS = 1000;  // millisecs
float IMU_SAMPLE_FREQ = 0.001;  // time interval between imu points (secs)

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
  // put your setup code here, to run once:
  Serial.begin(9600);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  /* ------------------------  MOTORS ESC ------------------------- */
  // PPM on pin for iFlyReceiver
  ppm.begin(interruptPin, false);

  /* Motor setup */
  // Serial.println("Calibrating ESCS...");
  // m1_esc.calib();
  // m1_esc.stop();
  // m2_esc.calib();
  // m2_esc.stop();
  // m3_esc.calib();
  // m3_esc.stop();
  // m4_esc.calib();
  // m4_esc.stop();

  Serial.println(F("Arming ESCS..."));
  delay(2000);
  m1_esc.arm();
  // m1_esc.speed(SPEED_MIN);
  Serial.println("ESC 1 armed.");
  delay(1000);
  m2_esc.arm();
  // m2_esc.speed(SPEED_MIN);
  Serial.println("ESC 2 armed.");
  delay(1000);
  m3_esc.arm();
  // m3_esc.speed(SPEED_MIN);
  Serial.println("ESC 3 armed.");
  delay(1000);
  m4_esc.arm();
  // m4_esc.speed(SPEED_MIN);
  Serial.println("ESC 4 armed.");

  /**
  *   -----------------------  IMU SETUP -------------------------
  */
  Wire1.begin();
  Wire1.setClock(400000);  // 400 kHz I2C clock

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

  /* Perform calibration */
  // imu.calibrateIMU();
  imu.setOffsetAccX(-981);
  imu.setOffsetAccY(-307);
  imu.setOffsetAccZ(1209);
  imu.setOffsetGyroX(-4);
  imu.setOffsetGyroY(-36);
  imu.setOffsetGyroZ(-36);
  imu.setCalibrated(true);
  imu.printInvertedAxes();

  // Serial.print("offset_acc_x:");
  // Serial.print(imu.getOffsetAccX());
  // Serial.print(",offset_acc_y:");
  // Serial.print(imu.getOffsetAccY());
  // Serial.print(",offset_acc_z:");
  // Serial.println(imu.getOffsetAccZ());

  digitalWrite(LED_BUILTIN, HIGH);
  delay(3000);

  /* -----------------------  END OF SETUP -------------------------- */
}

void loop() {
  // put your main code here, to run repeatedly:

  // Update measurements at 100 hz
  float imuCurrMillis = millis();
  if (imuCurrMillis - prevMillis > imu_interval) {

    imu.updateRawMeasurements();
    imu.complementaryFilter();

    // if (imuCurrMillis - prevMillis > imu_print_interval) {

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

    Serial.print("gyro_X:");
    Serial.print(imu.getGyroX());
    Serial.print(",gyro_Y:");
    Serial.print(imu.getGyroY());
    Serial.print(",gyro_Z:");
    Serial.print(imu.getGyroZ());
    Serial.println();

    // Serial.print("X:");
    // Serial.print(imu.getAngleX());
    // Serial.print(",Y:");
    // Serial.print(imu.getAngleY());
    // Serial.print(",Z:");
    // Serial.print(imu.getAngleZ());
    // Serial.println();

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
    // }

    prevMillis = imuCurrMillis;
  }

  // limit ppm read to specific frequency
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > motor_interval) {

    /* READ THE RAW PPM VALUE FROM RC RECIEVER HERE */
    rcThrottleRaw = constrain(ppm.read_channel(THROTTLE), MIN_THROTTLE, MAX_THROTTLE);

    /* Process the throttle rc channel with a low-pass filter */
    // filtInputThrottle = lowPassFilter(rcThrottle, filtInputThrottle, THROTTLE_LP_FILTER_COEFF);

    float filtInputThrottle = rcThrottleRaw;

    for (int i = 0; i < LP_FILTER_DEGREE; i++) {
      filtInputThrottle += inputThrottleHist[i];
    }

    filtInputThrottle = filtInputThrottle / (LP_FILTER_DEGREE + 1);

    inputThrottleHist[lpFilterIndex] = filtInputThrottle;

    // make the history buffers act as ring buffers
    lpFilterIndex++;
    if (lpFilterIndex == LP_FILTER_DEGREE) {
      lpFilterIndex = 0;
    }
    /* Set rcThrottle to the current index in the low pass filter */
    rcThrottle = inputThrottleHist[lpFilterIndex];

    // Map input between allowable PWM band
    m1_esc.speed(rcThrottle);
    m2_esc.speed(rcThrottle);
    m3_esc.speed(rcThrottle);
    m4_esc.speed(rcThrottle);
    // Serial.print("esc_out:");
    // Serial.println(esc_speed_out);

    previousMillis = currentMillis;
  }
}
