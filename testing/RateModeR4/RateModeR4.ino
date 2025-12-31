#include "ESC.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "PPMReader.h"
#include "ppm.h"
// #include "Arduino_LED_Matrix.h"   //Include the LED_Matrix library

// custom dependencies
#include <MPU6050Plus.h>

/* BEGIN GLOBAL CONSTANTS */

// debug update freq
#define DEBUG_MODE (false)
#define DEBUG_ANGLE (true)

const int DEBUG_UPDATE_FREQ_MS = 1000; // millisecs
unsigned long prevMillis = 0;
unsigned long currMillis = 0;

// #define DEBUG_EULER
#define LED_PIN (13)
#define PPM_INTERRUPT (3)
#define MPU_INTERRUPT (2)
bool blink_state = false;

// make an instance of the library:
// ArduinoLEDMatrix matrix;

MPU6050 mpu(0x68, &Wire1); // Set i2c addr to 0x68 (default) and use Qwiic connector (Wire1)
// MPU6050 mpu;          // Used for initiating a connection with the MPU drivers
MPU6050Plus imu; // Wrapper that converts raw IMU values into filtered angle measurements
EulerRPY rpy;    // IMU converted values expressed in an Euler transform

// #define IMU_SAMPLE_FREQ_MS (2.0)    // millisecs   (500 Hz)
// #define IMU_SAMPLE_FREQ_S (0.002)      // time interval between imu points (secs)
#define IMU_SAMPLE_FREQ_MS (4.0)    // millisecs   (500 Hz)
#define IMU_SAMPLE_FREQ_S (0.004)      // time interval between imu points (secs)
unsigned long prevImuTimer;
unsigned long currImuTimer;

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
#define ESC_SPEED_MAX (2000)
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
#define CH_YAW (4)
#define CH_ROLL (1)
#define CH_THR (3)
#define CH_PITCH (2)
#define CH_L_KNOB (5)
#define CH_R_KNOB (6)

#define RCREC_FREQ_MS (10)    // millisecs   (100 Hz)
#define RCREC_FREQ_S (0.01)      // seconds    (100 Hz)
unsigned long prevRcRecTimer;
unsigned long currRcRecTimer;

const int NUM_CHANNELS = 6;
// PPMReader ppm_(PPM_INTERRUPT, NUM_CHANNELS);
int32_t receiverValue[NUM_CHANNELS] = { 0.0 };
int32_t long value = 0.0;

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

#define CONTROL_LOOP_TIME_MS (10)      // milliseconds (100 Hz)
#define CONTROL_LOOP_TIME_SEC  (0.01)     // seconds   (100 Hz)
unsigned long prevControlTimer;
unsigned long currControlTimer;

float setptRateX, setptRateY, setptRateZ, prev_setptRateX, prev_setptRateY, prev_setptRateZ;
float inputX, inputY, inputZ, input_throttle;
float errorRateX, errorRateY, errorRateZ;

float prevErrorRateX, prevErrorRateY, prevErrorRateZ;
float prevIGainX, prevIGainY, prevIGainZ;
float PID_prev[] = { 0.0, 0.0, 0.0 };  // [PID_output, prev_I, prev_error]

/* Attitude Controller PID gains */
float PGainX = 1.67;
float PGainY = 1.67; //0.167;
float PGainZ = 1.67;

float IGainX = 0.0015;
float IGainY = 0.0015;
float IGainZ = 0.0012;

float DGainX = 0.01; //0.0302;
float DGainY = 0.01; //0.0302;
float DGainZ = 0.01; //3.0;

// motor input values
float motor_one_speed, motor_two_speed, motor_three_speed, motor_four_speed;
bool stopMotors = false;

#define MIN_THROTTLE (1050) 
#define MAX_THROTTLE (2000)
#define MOTOR_CONSTANT (1.025) // (7)
// Cheap COTS motors have variable performance and are further tuned here:
#define M1_CONST (1.0)//(0.98)
#define M2_CONST (1.0)//(0.95)
#define M3_CONST (0.95)
#define M4_CONST (0.80)
// TODO: characterize individual motor constants via thrust test stand

/* END GLOBAL CONSTANTS */
const float I_SATURATION = 100.0;

float pid(float setpt, float prev_setpt, float err_, float prev_err_, float p_gain, float i_gain,
          float prev_i_gain, float d_gain) {
  /* Proportional */
  float p_ = p_gain * err_;

  /* Derivative kick compensation by using derivative of setpoint */
  float d_ = d_gain * (setpt - prev_setpt) / CONTROL_LOOP_TIME_MS;

  /* Anti-wind up logic */
  float i_dt = ((err_ + prev_err_) / 2) * CONTROL_LOOP_TIME_MS; // total error since last time step

  if (abs(prev_i_gain + i_dt) >= I_SATURATION) {
    i_gain = 0.0;     // if i_gain is outside saturation limits, set i_gain to zero 
  } else {
    i_gain = i_gain;  // if i_gain is within saturation limits, proceed with normal i_gain
  }

  float i_ = prev_i_gain + i_gain * i_dt;

  /* PID output */
  float pid_output = p_ + i_ - d_;

#if DEBUG_MODE
  Serial.print("\t");
  Serial.print(String(p_));
  Serial.print("\t");
  Serial.print(String(i_));
  Serial.print("\t");
  Serial.print(String(d_));
  Serial.println();
#endif

  /* Constrain output to avoid motor saturation */
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
  prevIGainX = 0.0;
  prevIGainY = 0.0;
  prevIGainZ = 0.0;
}

void readReceiver(void) {
  /* read latest valid values from all channels */

  receiverValue[CH_YAW - 1] = ppm.read_channel(CH_YAW);
  receiverValue[CH_PITCH - 1] = ppm.read_channel(CH_PITCH);
  receiverValue[CH_THR - 1] = ppm.read_channel(CH_THR);
  receiverValue[CH_ROLL - 1] = ppm.read_channel(CH_ROLL);
  receiverValue[CH_L_KNOB - 1] = ppm.read_channel(CH_L_KNOB);
  receiverValue[CH_R_KNOB - 1] = ppm.read_channel(CH_R_KNOB);

  // Print the values for the Arduino Serial Plotter
  #if DEBUG_MODE
    Serial.print("Raw_Throttle:");        Serial.print(receiverValue[2]);       Serial.print(" ");
    Serial.print("Raw_Roll:");            Serial.print(receiverValue[3]);           Serial.print(" ");
    Serial.print("Raw_Pitch:");           Serial.print(receiverValue[1]);          Serial.print(" ");
    Serial.print("Raw_Yaw:");             Serial.print(receiverValue[0]);            Serial.print(" ");
    Serial.print("Switch_3way_1:");   Serial.print(receiverValue[4]);   Serial.print(" ");
    Serial.print("Switch_3way_2:");   Serial.print(receiverValue[5]);   Serial.print(" ");
    Serial.println();
  #endif

  /* deprecated - used in #include PPMReader.h */
  // for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
  //   value = ppm_.latestValidChannelValue(channel, 0);
  //   receiverValue[channel - 1] = value;
  //   // Serial.print(String(value) + " ");
  // }
  // Serial.println();
}

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
void i2cSetup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire1.begin();
  Wire1.setClock(400000);
  // TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

void setup() {
  Serial.begin(115200);
  /* Startup animation */
  pinMode(LED_BUILTIN, HIGH);

  /* Motor setup */
  Serial.println(F("Arming ESCS..."));
  pinMode(LED_BUILTIN, HIGH);
  // delay(500);
  m1_esc.arm();
  m1_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 1 armed.");
  pinMode(LED_BUILTIN, LOW);
  // delay(500);
  m2_esc.arm();
  m2_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 2 armed.");
  pinMode(LED_BUILTIN, LOW);
  // delay(500);
  m3_esc.arm();
  m3_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 3 armed.");
  pinMode(LED_BUILTIN, LOW);
  // delay(500);
  m4_esc.arm();
  m4_esc.speed(ESC_SPEED_MIN);
  Serial.println("ESC 4 armed.");
  pinMode(LED_BUILTIN, HIGH);
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
  imu.initialize(&mpu, IMU_SAMPLE_FREQ_S);
  Serial.println("IMU initialized");

  // Calibrate the IMU accel/gyro offsets
  // imu.calcOffsets();
  mpu.CalibrateAccel(20);
  mpu.CalibrateGyro(20);

  /* RC PPM Receiver Setup */
  ppm.begin(PPM_INTERRUPT, false);

  /* Safety check: if throttle above threshold do not advance */

  readReceiver();
  
  uint32_t init_throttle = 1100;

  while (init_throttle >= 1100) {
    Serial.println(init_throttle);
    init_throttle = receiverValue[CH_THR - 1];
    readReceiver();
    delay(100);
  }
  Serial.println(F("RC reciever ready!"));

  // Start the RTC


  // loopTimer = micros();
}

bool mpuIntStatus = true;

float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

// Apply filtering to the input joystick values
#define MAN_LP_FILTER_DEGREE (8)
float inputThrottleHist[MAN_LP_FILTER_DEGREE] = {0.0};
float inputXHist[MAN_LP_FILTER_DEGREE] = {0.0};
float inputYHist[MAN_LP_FILTER_DEGREE] = {0.0};
float inputZHist[MAN_LP_FILTER_DEGREE] = {0.0};
short lpFilterIndex = 0;

float filtInputThrottle = 0.0;
float filtInputX = 0.0;
float filtInputY = 0.0;
float filtInputZ = 0.0;

void loop() {

  /* Update the RC receiver data at a rate of 10 Hz */
  currRcRecTimer = millis();
  if ((currRcRecTimer - prevRcRecTimer) > RCREC_FREQ_MS)
  {
    readReceiver(); // read PPM from the RC remote

    // if (receiverValue[CH_THR - 1] < MIN_THROTTLE) {
    //   return;
    // }
    // input_throttle = 0.15 * (receiverValue[CH_THR - 1]);
    input_throttle = receiverValue[CH_THR - 1];
    inputX = 0.15 * (receiverValue[CH_ROLL - 1] - 1500);  // roll
    inputY = 0.15 * (receiverValue[CH_PITCH - 1] - 1500); // pitch
    inputZ = 0.15 * (receiverValue[CH_YAW - 1] - 1500);   // yaw

    // Calculate filtered input joystick values as the average of
    // the new reading and the last LP_FILTER_DEGREE filtered readings
    float _filtInputThrottle = input_throttle;
    float _filtInputX = inputX;
    float _filtInputY = inputY;
    float _filtInputZ = inputZ;
    for (int i = 0; i < MAN_LP_FILTER_DEGREE; i++)
    {
      _filtInputThrottle += inputThrottleHist[i];
      _filtInputX += inputXHist[i];
      _filtInputY += inputYHist[i];
      _filtInputZ += inputZHist[i];
    }

    filtInputThrottle = _filtInputThrottle / (MAN_LP_FILTER_DEGREE + 1);
    filtInputX = _filtInputX / (MAN_LP_FILTER_DEGREE + 1);
    filtInputY = _filtInputY / (MAN_LP_FILTER_DEGREE + 1);
    filtInputZ = _filtInputZ / (MAN_LP_FILTER_DEGREE + 1);

    // Update the history arrays with the new filtered value
    inputThrottleHist[lpFilterIndex] = filtInputThrottle;
    inputXHist[lpFilterIndex] = filtInputX;
    inputYHist[lpFilterIndex] = filtInputY;
    inputZHist[lpFilterIndex] = filtInputZ;

    // make the history buffers act as ring buffers
    lpFilterIndex++;
    if (lpFilterIndex == MAN_LP_FILTER_DEGREE)
    {
      lpFilterIndex = 0;
    }

    /* String debug statements */
    #if DEBUG_MODE
      Serial.print(",Throttle:");        Serial.print(filtInputThrottle);       Serial.print(" ");
      Serial.print(",Roll:");            Serial.print(filtInputX);           Serial.print(" ");
      Serial.print(",Pitch:");           Serial.print(filtInputY);          Serial.print(" ");
      Serial.print(",Yaw:");             Serial.print(filtInputZ);            Serial.print(" ");
      Serial.println();
    #endif

    prevRcRecTimer = currRcRecTimer;
  }

  /* Update the IMU data at a rate of 500 Hz*/
  currImuTimer = millis();
  if ((currImuTimer - prevImuTimer) > IMU_SAMPLE_FREQ_MS)
  {
    imu.updateRawMeasurements(); // read from IMU wrapper
    imu.updateEstimates();       // get attitude estimate

    angleX = imu.getAngleX();
    angleY = imu.getAngleY();
    angleZ = imu.getAngleZ();

    #if DEBUG_ANGLE
      Serial.print(",angleX:");        Serial.print(angleX);       Serial.print(" ");
      Serial.print(",angleY:");            Serial.print(angleY);           Serial.print(" ");
      Serial.print(",angleZ:");           Serial.print(angleZ);          Serial.print(" ");
      Serial.println();
    #endif

    prevImuTimer = currImuTimer; // update prev timer
  }


  /* Update the control loop at a rate of 500 Hz */
  currControlTimer = millis();
  if ((currControlTimer - prevControlTimer) > CONTROL_LOOP_TIME_MS)
  {
    /* Calculate error terms */
    errorRateX = filtInputX - angleX;
    errorRateY = filtInputY - angleY;
    errorRateZ = filtInputZ - angleZ;

    /* Calculate PID outputs */
    float pid_out_x = pid(filtInputX, prev_setptRateX, errorRateX, prevErrorRateX, 
                          PGainX,
                          IGainX, prevIGainX, 
                          DGainX);
    float pid_out_y = pid(filtInputY, prev_setptRateY, errorRateY, prevErrorRateY, 
                          PGainY,
                          IGainY, prevIGainY, 
                          DGainY);
    float pid_out_z = pid(filtInputZ, prev_setptRateZ, errorRateZ, prevErrorRateZ, 
                          PGainZ,
                          IGainZ, prevIGainZ, 
                          DGainZ);

    // Serial.println("thr: " + String(input_throttle) + "\tpidX: " + String(pid_out_x) + "\tpidY: " + String(pid_out_y) + "\tpidZ: " + String(pid_out_z));
    // Serial.println();

    /* Update prev_error and prev_I gains */
    prev_setptRateX = filtInputX;
    prev_setptRateY = filtInputY;
    prev_setptRateZ = filtInputZ;
    prevErrorRateX = errorRateX;
    prevErrorRateY = errorRateY;
    prevErrorRateZ = errorRateZ;
    prevIGainX = IGainX;
    prevIGainY = IGainY;
    prevIGainZ = IGainZ;

    /* Constrain max throttle input */
    if (filtInputThrottle >= MAX_THROTTLE)
      filtInputThrottle = MAX_THROTTLE; 

    /* Convert PID outputs into motor commands based on quadcopter dynamics model */
    motor_one_speed = MOTOR_CONSTANT * (filtInputThrottle - pid_out_x - pid_out_y - pid_out_z) * M1_CONST;
    motor_two_speed = MOTOR_CONSTANT * (filtInputThrottle - pid_out_x + pid_out_y + pid_out_z) * M2_CONST;
    motor_three_speed = MOTOR_CONSTANT * (filtInputThrottle + pid_out_x + pid_out_y - pid_out_z) * M3_CONST;
    motor_four_speed = MOTOR_CONSTANT * (filtInputThrottle + pid_out_x - pid_out_y + pid_out_z) * M4_CONST;

    /* Constrain individual motor speeds from max */
    if (motor_one_speed >= MAX_THROTTLE)
    {
      motor_one_speed = MAX_THROTTLE;
    }
    else if (motor_one_speed < MIN_THROTTLE)
    {
      motor_one_speed = MIN_THROTTLE;
    }
    if (motor_two_speed >= MAX_THROTTLE)
    {
      motor_two_speed = MAX_THROTTLE;
    }
    else if (motor_two_speed < MIN_THROTTLE)
    {
      motor_two_speed = MIN_THROTTLE;
    }
    if (motor_three_speed >= MAX_THROTTLE)
    {
      motor_three_speed = MAX_THROTTLE;
    }
    else if (motor_three_speed < MIN_THROTTLE)
    {
      motor_three_speed = MIN_THROTTLE;
    }
    if (motor_four_speed >= MAX_THROTTLE)
    {
      motor_four_speed = MAX_THROTTLE;
    }
    else if (motor_four_speed < MIN_THROTTLE)
    {
      motor_four_speed = MIN_THROTTLE;
    }

#if DEBUG_MODE
    Serial.print("pid_out_x:");
    Serial.print(pid_out_x);
    Serial.print(",pid_out_y:");
    Serial.print(pid_out_y);
    Serial.print(",pid_out_z:");
    Serial.print(pid_out_z);
    Serial.println();

    Serial.print("m1:");
    Serial.print(motor_one_speed);
    Serial.print(",m2:");
    Serial.print(motor_two_speed);
    Serial.print(",m3:");
    Serial.print(motor_three_speed);
    Serial.print(",m4:");
    Serial.print(motor_four_speed);
    Serial.println();
#endif

    /* Send the motor speed commands to individual ESCS */
    if (receiverValue[CH_THR - 1] < MIN_THROTTLE)
    {
      m1_esc.speed(ESC_STOP);
      m2_esc.speed(ESC_STOP);
      m3_esc.speed(ESC_STOP);
      m4_esc.speed(ESC_STOP);
      resetPID();
    }
    else
    {
      // /* Output motor speeds to ESCS */
      m1_esc.speed(motor_one_speed);
      m2_esc.speed(motor_two_speed);
      m3_esc.speed(motor_three_speed);
      m4_esc.speed(motor_four_speed);
    }

    prevControlTimer = currControlTimer;
  } /* End of control loop */
}
