#include "ESC.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
#include "PPMReader.h"
#include "Arduino_LED_Matrix.h"   //Include the LED_Matrix library

// custom dependencies
#include <MPU6050Plus.h>

/* BEGIN GLOBAL CONSTANTS */

// #define DEBUG_EULER
#define LED_PIN (13)
#define PPM_INTERRUPT (3)
#define MPU_INTERRUPT (2)
bool blink_state = false;

// make an instance of the library:
ArduinoLEDMatrix matrix;

const int NUM_CHANNELS = 6;
PPMReader ppm_(PPM_INTERRUPT, NUM_CHANNELS);
int32_t receiverValue[NUM_CHANNELS] = { 0.0 };
int32_t long value = 0.0;

// MPU6050 mpu(0x69);
MPU6050 mpu;          // Used for initiating a connection with the MPU drivers
MPU6050Plus imu;      // Wrapper that converts raw IMU values into filtered angle measurements
EulerRPY rpy;         // IMU converted values expressed in an Euler transform

const float IMU_SAMPLE_FREQ_MS = 1000;    // millisecs
const float IMU_SAMPLE_FREQ = 0.001;      // time interval between imu points (secs)

/* orientation data */
Quaternion q;         // [w, x ,y ,z]
VectorInt16 aa;       // [x, y, z]
VectorInt16 aaReal;   // [x, y, z] gravity-free measurements
VectorInt16 aaWorld;  // [x, y, z] world-frame accel measurements
VectorFloat gravity;  // [x, y, z] gravity vector
float euler[3];       // [psi, theta, phi] Euler conversion
float ypr[3];         // [yaw, pitch, roll] Angle conversion w/ gravity vector

/* ESC & MOTOR control vars */

#define MOTOR_1_PWM (5)
#define MOTOR_2_PWM (6)
#define MOTOR_3_PWM (9)
#define MOTOR_4_PWM (10)

#define ESC_SPEED_MIN (1000)
#define ESC_SPEED_MAX (1200)
#define ESC_STOP (500)
#define MIN_ARM_TIME (500)

/* FS-iA6B channel map
 *  Ch 1 -  Yaw
 *  Ch 2 -  Pitch
 *  Ch 3 -  Throttle
 *  Ch 4 -  Roll
 *  Ch 5 -  L knob
 *  Ch 6 -  R knob
 */
#define CH_YAW (1)
#define CH_ROLL (4)
#define CH_THR (3)
#define CH_PITCH (2)
#define CH_L_KNOB (5)
#define CH_R_KNOB (6)

int thr_in;
int roll_in;
int pitch_in;
int yaw_in;

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

ESC m1_esc(MOTOR_1_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC m2_esc(MOTOR_2_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC m3_esc(MOTOR_3_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC m4_esc(MOTOR_4_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);

/* Rate Mode flight control vars */

float rateX, rateY, rateZ;
float rateCalibX, rateCalibY, rateCalibZ;
int16_t* imu_offsets;
int rateCalibNumber;

#define LOOP_TIME_MS (4000)      // ms
#define LOOP_TIME_SEC  (0.004)     // seconds
uint32_t loopTimer;

float setptRateX, setptRateY, setptRateZ;
float inputX, inputY, inputZ, input_throttle;
float errorRateX, errorRateY, errorRateZ;

float prevErrorRateX, prevErrorRateY, prevErrorRateZ;
float prevIntRateX, prevIntRateY, prevIntRateZ;
float PID_prev[] = { 0.0, 0.0, 0.0 };  // [PID_output, prev_I, prev_error]

// the following PID gains are w.r.t. the gyro rate
float PGainX = 0.6;
float PGainY = 0.6;
float PGainZ = 2.0;

float IGainX = 3.5;
float IGainY = 3.5;
float IGainZ = 12;

float DGainX = 0.03;
float DGainY = 0.03;
float DGainZ = 0.0;

// motor input values
float motor_one_speed, motor_two_speed, motor_three_speed, motor_four_speed;
#define MIN_THROTTLE (1180)
#define MAX_THROTTLE (2000)
#define MOTOR_CONSTANT (5) // (7)
bool stopMotors = false;

/* END GLOBAL CONSTANTS */

float pid(float err_, float prev_err_, float p_gain, float i_gain,
          float prev_i_gain, float d_gain) {
  // float pid_state[3] = { 0.0 };
  float p_ = p_gain * err_;
  float i_ = prev_i_gain + i_gain * (((err_ + prev_err_) * LOOP_TIME_SEC) / 2);
  float d_ = d_gain * ((err_ - prev_err_) / LOOP_TIME_SEC);

  // Serial.print("\t");
  // Serial.print(String(p_));
  // Serial.print("\t");
  // Serial.print(String(i_));
  // Serial.print("\t");
  // Serial.print(String(d_));
  // Serial.println();

  float pid_output = p_ + i_ + d_;
  /* constrain output to avoid Integral windup */
  if (pid_output >= 400) {
    pid_output = 400;
  } else if (pid_output < -400) {
    pid_output = -400;
  }

  return pid_output;
}

void resetPID(void) {
  prevErrorRateX = 0.0;
  prevErrorRateY = 0.0;
  prevErrorRateZ = 0.0;
  prevIntRateX = 0.0;
  prevIntRateY = 0.0;
  prevIntRateZ = 0.0;
}

void readReceiver(void) {
  /* read latest valid values from all channels */
  for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
    value = ppm_.latestValidChannelValue(channel, 0);
    receiverValue[channel - 1] = value;
    // Serial.print(String(value) + " ");
  }
  // Serial.println();
}

// void calibrateIMU(void) {
//   // apply arbitrary initial gyro offsets
//   mpu.setXGyroOffset(220);
//   mpu.setYGyroOffset(76);
//   mpu.setZGyroOffset(-85);
//   mpu.setZAccelOffset(1788);

//   mpu.CalibrateAccel(6);
//   mpu.CalibrateGyro(6);
//   imu_offsets = mpu.GetActiveOffsets();
//   mpu.PrintActiveOffsets();
// }

// void readMPURaw() {
//   /* Read raw accel/gyro data from the module. Other methods commented*/
//   mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//   //mpu.getAcceleration(&ax, &ay, &az);
//   //mpu.getRotation(&gx, &gy, &gz);

//   /*Print the obtained data on the defined format*/
//   #ifdef OUTPUT_READABLE_ACCELGYRO
//     Serial.print("a/g:\t");
//     Serial.print(ax); Serial.print("\t");
//     Serial.print(ay); Serial.print("\t");
//     Serial.print(az); Serial.print("\t");
//     Serial.print(gx); Serial.print("\t");
//     Serial.print(gy); Serial.print("\t");
//     Serial.println(gz);
//   #endif

//   #ifdef OUTPUT_BINARY_ACCELGYRO
//     Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
//     Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
//     Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
//     Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
//     Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
//     Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
//   #endif

//   /*Blink LED to indicate activity*/
//   blinkState = !blinkState;
//   digitalWrite(LED_BUILTIN, blinkState);
// }

// void updateIMUData(void) {
//   if (fifoBuffer) {
//     // convert fifo buffer values
//     // mpu.dmpGetQuaternion(&q, fifoBuffer);
//     // mpu.dmpGetGravity(&gravity, &q);
//     // mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//     // mpu.dmpGetEuler(euler, &q);
// #ifdef DEBUG_MODE_QUAT
//     Serial.print("quat\t");
//     Serial.print(q.w);
//     Serial.print("\t");
//     Serial.print(q.x);
//     Serial.print("\t");
//     Serial.print(q.y);
//     Serial.print("\t");
//     Serial.println(q.z);
// #endif

// #ifdef DEBUG_YAWPITCHROLL
//     Serial.print("ypr\t");
//     Serial.print(ypr[0] * 180 / M_PI);
//     Serial.print("\t");
//     Serial.print(ypr[1] * 180 / M_PI);
//     Serial.print("\t");
//     Serial.println(ypr[2] * 180 / M_PI);
// #endif

// #ifdef DEBUG_EULER
//     // display Euler angles in degrees
//     // mpu.dmpGetQuaternion(&q, fifoBuffer);
//     // mpu.dmpGetEuler(euler, &q);
//     Serial.print("euler\t");
//     Serial.print(euler[0] * 180 / M_PI);
//     Serial.print("\t");
//     Serial.print(euler[1] * 180 / M_PI);
//     Serial.print("\t");
//     Serial.println(euler[2] * 180 / M_PI);
// #endif
//   }
// }

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
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


const int TEMP_MIN = 1010;
// void stopMotors(void) {
//   // motor_one_speed = TEMP_MIN;
//   // motor_two_speed = TEMP_MIN;
//   // motor_three_speed = TEMP_MIN;
//   // motor_four_speed = TEMP_MIN;
//   m1_esc.stop();
//   m2_esc.stop();
//   m3_esc.stop();
//   m4_esc.stop();
//   resetPID();
//   stopMotors = true;
// }

void setup() {
  Serial.begin(115200);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);
  matrix.loadSequence(LEDMATRIX_ANIMATION_SPINNING_COIN);
  matrix.begin();
  matrix.play(true);

  /* Motor setup */
  Serial.println(F("Arming ESCS..."));
  delay(2000);
  m1_esc.arm();
  m1_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 1 armed.");
  delay(1000);
  m2_esc.arm();
  m2_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 2 armed.");
  delay(1000);
  m3_esc.arm();
  m3_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 3 armed.");
  delay(1000);
  m4_esc.arm();
  m4_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 4 armed.");

  delay(5000);

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

  /* Create IMU data wrapper */
  imu.initialize(&mpu, IMU_SAMPLE_FREQ);

  // Calibrate the IMU accel/gyro offsets
  // calibrateIMU();    // --- no need as the mpu wrapper takes care of calibration with preset values

  /* RC PPM Receiver Setup */
  /* Safety check: if throttle above threshold do not advance */
  readReceiver();
  uint32_t init_throttle = receiverValue[CH_THR - 1];
  while (init_throttle >= 1100) {
    readReceiver();
    delay(100);
  }
  Serial.println(F("RC reciever ready!"));

  // Start the RTC


  loopTimer = micros();
}

bool mpuIntStatus = true;

float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

void loop() {
  // Serial.println("looping...");
  // if (mpuIntStatus) {
  //   imu.updateMeasurement();      // read from IMU wrapper  
  //   readReceiver();               // read PPM from the RC remote
  // }

  imu.updateMeasurement();      // read from IMU wrapper  
  readReceiver();               // read PPM from the RC remote

  if (receiverValue[CH_THR - 1] < 1180) {
    return;
  }
  input_throttle = 0.15 * (receiverValue[CH_THR - 1]);
  inputX = 0.15 * (receiverValue[CH_ROLL - 1] - 1500);   // roll
  inputY = 0.15 * (receiverValue[CH_PITCH - 1] - 1500);  // pitch
  inputZ = 0.15 * (receiverValue[CH_YAW - 1] - 1500);    // yaw

  // Serial.print(String(input_throttle)+"\n");
  // Serial.print(String(inputX)+"\n");
  // Serial.print(String(inputY)+"\n");
  // Serial.print(String(inputZ)+"\n");
  Serial.println("inX: " + String(inputX) + "\tinY: " + String(inputY) + "\tinZ: " + String(inputZ));

  angleX = imu.getAngleX();
  angleY = imu.getAngleY();
  angleZ = imu.getAngleZ();

  // Caculate error
  // errorRateX = inputX - (euler[2] * 180/M_PI);      // this converts from euler to degrees
  // errorRateZ = inputZ - (euler[0] * 180/M_PI);
  // errorRateY = inputY - (euler[1] * 180/M_PI);
  errorRateX = inputX - angleX;
  errorRateY = inputY - angleY;
  errorRateZ = inputZ - angleZ;

  // Serial.println("ErrX: " + String(errorRateX) + "\tErrY: " + String(errorRateY) + "\tErrZ: " + String(errorRateZ));

  float pid_out_x = pid(errorRateX, prevErrorRateX, PGainX,
                        IGainX, prevIntRateX, DGainX);
  float pid_out_y = pid(errorRateY, prevErrorRateY, PGainY,
                        IGainY, prevIntRateY, DGainY);
  float pid_out_z = pid(errorRateZ, prevErrorRateZ, PGainZ,
                        IGainZ, prevIntRateZ, DGainZ);

  // Serial.print(String(pid_out_x));
  // Serial.print(String(pid_out_y));
  // Serial.print(String(pid_out_z));
  // Serial.println("thr: " + String(input_throttle) + "\tpidX: " + String(pid_out_x) + "\tpidY: " + String(pid_out_y) + "\tpidZ: " + String(pid_out_z));
  // Serial.println();

  /* update prev_error and prev_I gain */
  prevErrorRateX = errorRateX;
  prevErrorRateY = errorRateY;
  prevErrorRateZ = errorRateZ;
  prevIntRateX = IGainX;
  prevIntRateY = IGainY;
  prevIntRateZ = IGainZ;

  if (input_throttle > 1800)
    input_throttle = 1800;  // limit max throttle

  /* Convert into motor commands based on quadcopter dynamics model */
  motor_one_speed = MOTOR_CONSTANT*(input_throttle - pid_out_x - pid_out_y - pid_out_z);
  motor_two_speed = MOTOR_CONSTANT*(input_throttle - pid_out_x + pid_out_y + pid_out_z);
  motor_three_speed = MOTOR_CONSTANT*(input_throttle + pid_out_x + pid_out_y - pid_out_z);
  motor_four_speed = MOTOR_CONSTANT*(input_throttle + pid_out_x - pid_out_y + pid_out_z);

  Serial.print("m1= ");
  Serial.print(String(motor_one_speed));
  Serial.print("\tm2= ");
  Serial.print(String(motor_two_speed));
  Serial.print("\tm3= ");
  Serial.print(String(motor_three_speed));
  Serial.print("\tm4= ");
  Serial.print(String(motor_four_speed));
  Serial.println();

  // /* limit motor speed from max */
  if (motor_one_speed >= MAX_THROTTLE) {
    motor_one_speed = 1999;
  } else if (motor_one_speed < MIN_THROTTLE) {
    motor_one_speed = MIN_THROTTLE;
  }
  if (motor_two_speed >= MAX_THROTTLE) {
    motor_two_speed = 1999;
  } else if (motor_two_speed < MIN_THROTTLE) {
    motor_two_speed = MIN_THROTTLE;
  }
  if (motor_three_speed >= MAX_THROTTLE) {
    motor_three_speed = 1999;
  } else if (motor_three_speed < MIN_THROTTLE) {
    motor_three_speed = MIN_THROTTLE;
  }
  if (motor_four_speed >= MAX_THROTTLE) {
    motor_four_speed = 1999;
  } else if (motor_four_speed < MIN_THROTTLE) {
    motor_four_speed = MIN_THROTTLE;
  }

/* Send the motor speed commands to individual ESCS */

  // Serial.print("Throttle= ");
  // Serial.println(String(receiverValue[CH_THR - 1]));

  // if (receiverValue[CH_THR - 1] < 1280) {
  //   m1_esc.speed(ESC_STOP);
  //   m2_esc.speed(ESC_STOP);
  //   m3_esc.speed(ESC_STOP);
  //   m4_esc.speed(ESC_STOP);
  // } else {
  //   // /* Output motor speeds to ESCS */
  // m1_esc.speed(motor_one_speed);
  // m2_esc.speed(motor_two_speed);
  // m3_esc.speed(motor_three_speed);
  // m4_esc.speed(motor_four_speed);
  // }

  // Serial.print(String(motor_one_speed));
  // Serial.print("\t");
  // Serial.print(String(motor_two_speed));
  // Serial.print("\t");
  // Serial.print(String(motor_three_speed));
  // Serial.print("\t");
  // Serial.print(String(motor_four_speed));
  // Serial.println();

  

  /* Wait for control loop to finish. */
  while (micros() - loopTimer < LOOP_TIME_MS) {
    loopTimer = micros();
  }
}
