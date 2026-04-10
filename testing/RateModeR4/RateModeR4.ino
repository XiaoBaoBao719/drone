// #if defined(ESP32)
//   #include <ESP32Servo.h>
// #else
//   #include "ESC.h"
// #endif
#include "ESC.h"
#include "Wire.h"
// #include "I2Cdev.h"
// #include "MPU6050.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "PPMReader.h"
#include "ppm.h"
// #include "Arduino_LED_Matrix.h"   //Include the LED_Matrix library

// custom dependencies
#include <MPU6050Plus.h>
#include <ArduinoJson.h>  // For SITL JSON parsing

/* BEGIN GLOBAL CONSTANTS */

// SITL mode toggle
#define SITL_MODE (false)  // Set to true for simulation testing

/* BEGIN GLOBAL CONSTANTS */

// debug update freq
#define DEBUG_MODE  (false)
#define DEBUG_ANGLE (false)
#define DEBUG_MOTOR (false)
#define DEBUG_PID   (true)
#define DEBUG_RC_RAW    (false)
#define DEBUG_RC_RPY    (false)

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

// MPU6050 mpu(0x68, &Wire1); // Set i2c addr to 0x68 (default) and use Qwiic connector (Wire1)
// MPU6050 mpu;          // Used for initiating a connection with the MPU drivers
#define MPU_ADDR (0x68) 
MPU6050Plus imu; // Wrapper that converts raw IMU values into filtered angle measurements
EulerRPY rpy;    // IMU converted values expressed in an Euler transform

#define IMU_SAMPLE_FREQ_MS (1.0)    // millisecs   (1,000 Hz)
#define IMU_SAMPLE_FREQ_SEC (0.001)      // time interval between imu points (secs)
// #define IMU_SAMPLE_FREQ_MS (4.0)    // millisecs   (250 Hz)
// #define IMU_SAMPLE_FREQ_SEC (0.004)      // time interval between imu points (secs)
unsigned long prevImuTimer;
unsigned long currImuTimer;

/* orientation data */
// Quaternion q;         // [w, x ,y ,z]
// VectorInt16 aa;       // [x, y, z]
// VectorInt16 aaReal;   // [x, y, z] gravity-free measurements
// VectorInt16 aaWorld;  // [x, y, z] world-frame accel measurements
// VectorFloat gravity;  // [x, y, z] gravity vector
float euler[3];       // [psi, theta, phi] Euler conversion
float ypr[3];         // [yaw, pitch, roll] Angle conversion w/ gravity vector

/* ESC & MOTOR control vars */

#define MOTOR_1_PWM (5)
#define MOTOR_2_PWM (6)
#define MOTOR_3_PWM (9)
#define MOTOR_4_PWM (10)

#define ESC_SPEED_MIN (1000)
#define ESC_SPEED_MAX (1800)
#define ESC_STOP (500)
#define MIN_ARM_TIME (500)

#define MIN_THROTTLE (1050)
#define MAX_THROTTLE (ESC_SPEED_MAX)
#define MOTOR_CONSTANT (1.025) // (7)
// Cheap COTS motors have variable performance and are further tuned here:
#define M1_CONST (1.0)//(0.98)
#define M2_CONST (1.0)//(0.95)
#define M3_CONST (1.0)
#define M4_CONST (1.0)
// TODO: characterize individual motor constants via thrust test stand

static constexpr uint16_t MAX_INIT_THROTTLE = 1100;

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
// #if ESP32
//   ESP32PWM m1_esc;
//   ESP32PWM m2_esc;
//   ESP32PWM m3_esc;
//   ESP32PWM m4_esc;
// #else
  ESC m1_esc(MOTOR_1_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
  ESC m2_esc(MOTOR_2_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
  ESC m3_esc(MOTOR_3_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
  ESC m4_esc(MOTOR_4_PWM, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
// #endif

/* SITL serial interface */
#if SITL_MODE
StaticJsonDocument<256> imu_doc;
HardwareSerial& sitl_serial = Serial1;  // Use Serial1 for SITL

struct SITLImuData {
    float accel[3];
    float gyro[3];
    float mag[3];
} sitl_imu;

uint16_t motor_pwm[4] = {1500, 1500, 1500, 1500};  // PWM values to send to simulator
#endif

/* Rate Mode flight control vars */

float rateX, rateY, rateZ;
float rateCalibX, rateCalibY, rateCalibZ;
int16_t* imu_offsets;
int rateCalibNumber;

#define CONTROL_LOOP_TIME_MS (1)      // milliseconds (1 kHz)
#define CONTROL_LOOP_TIME_SEC  (0.001)     // seconds   (1,000 Hz)

enum ControlMode {
  RATE_MODE,
  ANGLE_MODE,
  POSITION_MODE
};

ControlMode controlMode = RATE_MODE; 
unsigned long prevControlTimer;
unsigned long currControlTimer;

/* Angle Mode State Vars */
static float pidRollAngle;
static float pidPitchAngle;
static float pidYawAngle;

/* Angle Mode Controller PID Gains */
/* Converts desired attitude angles to rate setpoints (cascaded control) */
static constexpr float ANGLE_ROLL_KP   = 3.8f;   // Attitude to rate conversion (±45° -> ~±190 deg/s)
static constexpr float ANGLE_PITCH_KP  = 3.8f;   // Symmetric with roll
static constexpr float ANGLE_YAW_KP    = 1.5f;   // Lower for conservative yaw control

static constexpr float ANGLE_ROLL_KI   = 0.015f; // Corrects steady-state angle bias
static constexpr float ANGLE_PITCH_KI  = 0.015f;
static constexpr float ANGLE_YAW_KI    = 0.004f; // Very small (gyro drift sensitivity)

static constexpr float ANGLE_ROLL_KD   = 0.10f;  // Damping on attitude rate
static constexpr float ANGLE_PITCH_KD  = 0.10f;
static constexpr float ANGLE_YAW_KD    = 0.05f;  // Minimal damping for yaw

static constexpr float ANGLE_ROLL_I_LIMIT   = 50.0f;   // Conservative vs rate loop
static constexpr float ANGLE_PITCH_I_LIMIT  = 50.0f;
static constexpr float ANGLE_YAW_I_LIMIT    = 30.0f;

static constexpr float ANGLE_PID_LIMIT = 200.0f; // Rate setpoint limit (±200 deg/s)


/* Rate Mode State Vars */
float setptRateX, setptRateY, setptRateZ;
float inputX, inputY, inputZ, input_throttle;

static float pidRollRate;
static float pidPitchRate;
static float pidYawRate;

/* Rate Mode Controller PID Gains */
/* Gyroscope direct feedback (inner loop) - 1000 Hz control */
static constexpr float ROLL_KP    = 0.90f;   // +12.5% - tighter response for 1kHz loop & 250g mass
static constexpr float PITCH_KP   = 0.90f;   // Symmetric with roll
static constexpr float YAW_KP     = 0.55f;   // +10% - faster yaw response

static constexpr float ROLL_KI    = 0.015f;  // +50% - compensates motor drag & ESC non-linearity
static constexpr float PITCH_KI   = 0.015f;
static constexpr float YAW_KI     = 0.008f;  // +100% - better steady-state

static constexpr float ROLL_KD    = 0.060f;  // +20% - enhanced damping for 2-blade props
static constexpr float PITCH_KD   = 0.060f;
static constexpr float YAW_KD     = 0.002f;  // +100% - increased damping

static constexpr float ROLL_I_LIMIT   = 100.0f;  // Integral saturation
static constexpr float PITCH_I_LIMIT  = 100.0f;
static constexpr float YAW_I_LIMIT    = 150.0f;  // -25% - more conservative

static constexpr float PID_LIMIT  = 300.0f;  // ±300 motor command units


/* END GLOBAL CONSTANTS */

struct PIDState {
  float integral;
  float prevMeasurement;
  float prevError;
};

static PIDState rollAnglePID = {};
static PIDState pitchAnglePID = {};
static PIDState yawAnglePID = {};

static PIDState rollRatePID   = {};
static PIDState pitchRatePID  = {};
static PIDState yawRatePID    = {};

static float pid(float setpt, float measurement,
                  float kp, float ki, float kd,
                  float dt, float iLimit, float pidLimit,
                  PIDState &state)
{
  /* Error calculation */
  float error = setpt - measurement;

  /* Proportional */
  float p_ = kp * error;

  /* Derivative kick compensation by using derivative of measurements */
  float d_ = -kd * ((measurement - state.prevMeasurement) / dt);
  
  /* Integral Anti-wind up with C */
  float i_dt = ((error + state.prevError) / 2) * dt; // total error since last time step

  float i_ = constrain( state.integral + ki * i_dt, 
                        -iLimit, iLimit);
  
  state.prevError = error;
  state.prevMeasurement = measurement;
  state.integral = i_;
  
  /* PID output */
  float pid_output = constrain(p_ + i_ + d_, -pidLimit, pidLimit);

// #if DEBUG_PID
//   Serial.print("\t");
//   Serial.print(String(p_));
//   Serial.print("\t");
//   Serial.print(String(i_));
//   Serial.print("\t");
//   Serial.print(String(d_));
//   Serial.println();
// #endif

  return pid_output;
}

void resetPID(void) {
  rollRatePID.prevError   = 0.0;
  pitchRatePID.prevError  = 0.0;
  yawRatePID.prevError    = 0.0;

  rollRatePID.integral  = 0.0;
  pitchRatePID.integral = 0.0;
  yawRatePID.integral   = 0.0;
}

// ============================================================
// MOTORS
// ============================================================

/** Scalar motor commands, indexed 0-3 -> M1-M4. */
struct MotorOutputs {
    float cmd[4];
};

static MotorOutputs motors = {{ static_cast<float>(ESC_SPEED_MIN),
                                static_cast<float>(ESC_SPEED_MIN),
                                static_cast<float>(ESC_SPEED_MIN),
                                static_cast<float>(ESC_SPEED_MIN) }};

/**
 * @brief Convert PID outputs into motor commands based on quadcopter dynamics model.
 * Assumes NED (North-East-Down) coordinate convention:
 * - X-axis: North (forward/backward)
 * - Y-axis: East (right/left)
 * - Z-axis: Down (upward thrust)
 *
 * Quadcopter motor layout (X configuration):
 * - M1: Front-Right (+X, +Y, CCW)
 * - M2: Back-Left (-X, -Y, CW)
 * - M3: Front-Left (+X, -Y, CCW)
 * - M4: Back-Right (-X, +Y, CW)
 *
 * Motor mixing formula (as implemented in code):
 * - Motor 1 (Front-Right, CCW): Throttle - Roll + Pitch + Yaw
 * - Motor 2 (Back-Left, CW):    Throttle - Roll - Pitch - Yaw
 * - Motor 3 (Front-Left, CCW):  Throttle + Roll - Pitch + Yaw
 * - Motor 4 (Back-Right, CW):   Throttle + Roll + Pitch - Yaw
 */

static void mixMotors(float throttle, float rollOut,
                      float pitchOut, float yawOut)
{
  motors.cmd[0] = M1_CONST * (throttle - rollOut + pitchOut + yawOut);
  motors.cmd[1] = M2_CONST * (throttle - rollOut - pitchOut - yawOut);
  motors.cmd[2] = M3_CONST * (throttle + rollOut - pitchOut + yawOut);
  motors.cmd[3] = M4_CONST * (throttle + rollOut + pitchOut - yawOut);

  for (int i = 0; i < sizeof(motors.cmd) / sizeof(motors.cmd[0]); i++)
  {
    motors.cmd[i] = constrain(motors.cmd[i],
                              static_cast<float>(MIN_THROTTLE),
                              static_cast<float>(ESC_SPEED_MAX));
  }
}

#if SITL_MODE
void setup_sitl_serial() {
    sitl_serial.begin(115200);  // Match Python simulator
}

void read_sitl_imu() {
    if (!sitl_serial.available()) return;
    
    // Read JSON from simulator
    String json_str = sitl_serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(imu_doc, json_str);
    
    if (!error) {
        // Parse IMU data
        JsonArray accel = imu_doc["accel"];
        JsonArray gyro = imu_doc["gyro"];
        
        sitl_imu.accel[0] = accel[0];
        sitl_imu.accel[1] = accel[1];
        sitl_imu.accel[2] = accel[2];
        
        sitl_imu.gyro[0] = gyro[0];
        sitl_imu.gyro[1] = gyro[1];
        sitl_imu.gyro[2] = gyro[2];
    }
}

void write_sitl_pwm() {
    // Send 4 PWM values (8 bytes: 4 uint16_t)
    sitl_serial.write((byte*)motor_pwm, 8);
}
#endif

// =======================================================================
//  RATE SETPOINT LIMITS
// =======================================================================

/**
 * Maximum body rates commanded by full stick deflection.
 * These are intentionally conservative for initial flight testing.
 * Increase gradually once the platform is confirmed stable.
 */
static constexpr float MAX_ROLL_ANGLE     = 45.0f;
static constexpr float MAX_PITCH_ANGLE    = 45.0f;
static constexpr float MAX_YAW_ANGLE      = 20.0f;

static constexpr float MAX_ROLL_RATE_DPS  = 200.0f;  ///< deg/s at full stick
static constexpr float MAX_PITCH_RATE_DPS = 200.0f;  ///< deg/s at full stick
static constexpr float MAX_YAW_RATE_DPS   = 120.0f;  ///< deg/s at full stick

// ============================================================
// RC RECEIVER
// ============================================================

// RC pulse geometry (us)
static constexpr float RC_CENTER   = 1500.0f;  ///< Neutral / centre pulse
static constexpr float RC_RANGE    = 500.0f;   ///< Centre -> full deflection
static constexpr float RC_DEADBAND = 20.0f;    ///< Ignore noise within +-us

/* FS-iA6B channel map
 *  Ch 1 -  Yaw
 *  Ch 2 -  Pitch
 *  Ch 3 -  Throttle
 *  Ch 4 -  Roll
 *  Ch 5 -  L knob
 *  Ch 6 -  R knob
 */
static constexpr int CH_YAW     = 4;
static constexpr int CH_ROLL    = 1;
static constexpr int CH_THR     = 3;
static constexpr int CH_PITCH   = 2;
static constexpr int CH_L_KNOB  = 5;
static constexpr int CH_R_KNOB  = 6;

static constexpr float RCREC_FREQ_MS = 4.;    // millisecs   (100 Hz)
static constexpr float RCREC_FREQ_S  = 0.04;     // seconds    (100 Hz)
static unsigned long prevRcRecTimer         = 0.0;
static unsigned long currRcRecTimer         = 0.0;

/* Digital Low-pass filter */

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
// short lpFilterIndex_throttleHist = 0;

static constexpr int RC_NUM_CHANNELS = 6;
// PPMReader ppm_(PPM_INTERRUPT, NUM_CHANNELS);
static int32_t receiverValueRaw[RC_NUM_CHANNELS] = { 0.0 };
// int32_t long value = 0.0;
static float rcThrottle      = 0.0f;
static float rcThrottleRaw   = 0.0f;
static float rcRoll          = 0.0f;
static float rcPitch         = 0.0f;
static float rcYaw           = 0.0f;

static void readReceiver(void) {
  /* read latest valid values from all channels */

  receiverValueRaw[CH_YAW - 1] = ppm.read_channel(CH_YAW);
  receiverValueRaw[CH_PITCH - 1] = ppm.read_channel(CH_PITCH);
  receiverValueRaw[CH_THR - 1] = ppm.read_channel(CH_THR);
  receiverValueRaw[CH_ROLL - 1] = ppm.read_channel(CH_ROLL);
  receiverValueRaw[CH_L_KNOB - 1] = ppm.read_channel(CH_L_KNOB);
  receiverValueRaw[CH_R_KNOB - 1] = ppm.read_channel(CH_R_KNOB);

  if (controlMode == RATE_MODE) {
    rcThrottleRaw = constrain(receiverValueRaw[CH_THR - 1], MIN_THROTTLE, MAX_THROTTLE);
    rcRoll = scaleRcToAngleRate(receiverValueRaw[CH_ROLL - 1], MAX_ROLL_RATE_DPS);
    rcPitch = scaleRcToAngleRate(receiverValueRaw[CH_PITCH - 1], MAX_PITCH_RATE_DPS);
    rcYaw = scaleRcToAngleRate(receiverValueRaw[CH_YAW - 1], MAX_YAW_RATE_DPS);
  } else if (controlMode == ANGLE_MODE) {
    rcThrottleRaw = constrain(receiverValueRaw[CH_THR - 1], MIN_THROTTLE, MAX_THROTTLE);
    rcRoll = scaleRcToAngleRate(receiverValueRaw[CH_ROLL - 1], MAX_ROLL_ANGLE);
    rcPitch = scaleRcToAngleRate(receiverValueRaw[CH_PITCH - 1], MAX_PITCH_ANGLE);
    rcYaw = scaleRcToAngleRate(receiverValueRaw[CH_YAW - 1], MAX_YAW_ANGLE);
  }

  /* Process the throttle rc channel with a low-pass filter */
  // filtInputThrottle = lowPassFilter(rcThrottle, filtInputThrottle, THROTTLE_LP_FILTER_COEFF);

  float filtInputThrottle = rcThrottleRaw;
  float filtInputX        = rcRoll;
  float filtInputY        = rcPitch;
  float filtInputZ        = rcYaw;

  for (int i = 0; i < LP_FILTER_DEGREE; i++)
  {
    filtInputThrottle += inputThrottleHist[i];
    filtInputX        += inputXHist[i];
    filtInputY        += inputYHist[i];
    filtInputZ        += inputZHist[i];
  }

  filtInputThrottle = filtInputThrottle / (LP_FILTER_DEGREE + 1);
  filtInputX        = filtInputX / (LP_FILTER_DEGREE + 1);
  filtInputY        = filtInputY / (LP_FILTER_DEGREE + 1);
  filtInputZ        = filtInputZ / (LP_FILTER_DEGREE + 1);

  inputThrottleHist[lpFilterIndex] = filtInputThrottle;
  inputXHist[lpFilterIndex] = filtInputX;
  inputYHist[lpFilterIndex] = filtInputY;
  inputZHist[lpFilterIndex] = filtInputZ;

  // make the history buffers act as ring buffers
  lpFilterIndex++;
  if (lpFilterIndex == LP_FILTER_DEGREE)
  {
    lpFilterIndex = 0;
  }
  /* Set rcThrottle to the current index in the low pass filter */
  rcThrottle = inputThrottleHist[lpFilterIndex];
  rcRoll     = inputXHist[lpFilterIndex];
  rcPitch    = inputYHist[lpFilterIndex];yawRatePID;
  rcYaw      = inputZHist[lpFilterIndex];

  // Print the values for the Arduino Serial Plotter
  #if DEBUG_RC_RAW
    Serial.print("Raw_Throttle:");        Serial.print(receiverValueRaw[2]);       Serial.print(" ");
    Serial.print("Raw_Roll:");            Serial.print(receiverValueRaw[3]);           Serial.print(" ");
    Serial.print("Raw_Pitch:");           Serial.print(receiverValueRaw[1]);          Serial.print(" ");
    Serial.print("Raw_Yaw:");             Serial.print(receiverValueRaw[0]);            Serial.print(" ");
    Serial.print("Switch_3way_1:");   Serial.print(receiverValueRaw[4]);   Serial.print(" ");
    Serial.print("Switch_3way_2:");   Serial.print(receiverValueRaw[5]);   Serial.print(" ");
    Serial.println();
  #endif

  #if DEBUG_RC_RPY
    Serial.print("Throttle:");            Serial.print(rcThrottle);       Serial.print(" ");
    Serial.print("Rc_Roll:");            Serial.print(rcRoll);           Serial.print(" ");
    Serial.print("Rc_Pitch:");           Serial.print(rcPitch);          Serial.print(" ");
    Serial.print("Rc_Yaw:");             Serial.print(rcYaw);            Serial.print(" ");
    Serial.println();
  #endif

  /* deprecated - used in #include PPMReader.h */
  // for (int channel = 1; channel <= NUM_CHANNELS; channel++) {
  //   value = ppm_.latestValidChannelValue(channel, 0);
  //   receiverValueRaw[channel - 1] = value;
  //   // Serial.print(String(value) + " ");
  // }
  // Serial.println();
}

static inline float deadband(float value, float deadband)
{
  if (fabsf(value) < deadband)
    return 0.0f;
  else
    return value;
}

static float scaleRcToAngleRate(int32_t rawUs, float maxRate)
{
    // calculate how far off from center stick is and apply deadband around it.
    const float dist_from_center = deadband(static_cast<float>(rawUs) - RC_CENTER,
                                            RC_DEADBAND);
    // scale distance by RC_RANGE itself (1000 to 2000 us)
    return (dist_from_center / RC_RANGE) * maxRate;
}

// ================================================================
// ===                      i2c SETUP Items                     ===
// ================================================================
// void i2cSetup() {
//   // join I2C bus (I2Cdev library doesn't do this automatically)
// #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//   Wire1.begin();
//   Wire1.setClock(500000);
//   // TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
// #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//   Fastwire::setup(500, true);
// #endif
// }

// ===============================================================
// ===                Global State Variables                   ===
// ===============================================================

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

void updateControllers() {

  pidRollAngle  = pid(rcRoll, angleX, ANGLE_ROLL_KD, ANGLE_ROLL_KI, ANGLE_ROLL_KD,
                          CONTROL_LOOP_TIME_SEC, ANGLE_ROLL_I_LIMIT, ANGLE_PID_LIMIT, 
                          rollAnglePID);

  pidPitchAngle = pid(rcPitch, angleY, ANGLE_PITCH_KD, ANGLE_PITCH_KI, ANGLE_PITCH_KD,
                          CONTROL_LOOP_TIME_SEC, ANGLE_PITCH_I_LIMIT, ANGLE_PID_LIMIT,
                          pitchAnglePID);

  pidYawAngle   = pid(rcYaw, angleZ, ANGLE_YAW_KD, ANGLE_YAW_KI, ANGLE_YAW_KD,
                          CONTROL_LOOP_TIME_SEC, ANGLE_YAW_I_LIMIT, ANGLE_PID_LIMIT,
                          yawAnglePID);

  pidRollRate   = pid(pidRollAngle, angleRateX, ROLL_KP, ROLL_KI, ROLL_KD,
                          CONTROL_LOOP_TIME_SEC, ROLL_I_LIMIT, PID_LIMIT, 
                          rollRatePID);

  pidPitchRate  = pid(pidPitchAngle, angleRateY, PITCH_KP, PITCH_KI, PITCH_KD,
                           CONTROL_LOOP_TIME_SEC, PITCH_I_LIMIT, PID_LIMIT, 
                           pitchRatePID);

  pidYawRate    = pid(pidYawAngle, angleRateZ, YAW_KP, YAW_KI, YAW_KD,
                         CONTROL_LOOP_TIME_SEC, YAW_I_LIMIT, PID_LIMIT, 
                         yawRatePID);
  // Serial.println("thr: " + String(input_throttle) + "\tpidX: " + String(pid_out_x) + "\tpidY: " + String(pid_out_y) + "\tpidZ: " + String(pid_out_z));
  // Serial.println();

#if DEBUG_PID
  // Serial.print("throttle:");    Serial.print(rcThrottle);
  Serial.print(",pid_angle_x:");  Serial.print(pidRollAngle);
  Serial.print(",pid_angle_y:");  Serial.print(pidPitchAngle);
  Serial.print(",pid_angle_z:");  Serial.print(pidYawAngle);

  Serial.print(",pid_out_x:");  Serial.print(pidRollRate);
  Serial.print(",pid_out_y:");  Serial.print(pidPitchRate);
  Serial.print(",pid_out_z:");  Serial.print(pidYawRate);
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
  // Calibrate the IMU accel/gyro offsets
  imu.calibrateIMU();
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


  /* Update the control loop and motors at a rate of 500 Hz */
  currControlTimer = millis();
  if ((currControlTimer - prevControlTimer) > CONTROL_LOOP_TIME_MS)
  {
    updateControllers();
    updateMotors();
    prevControlTimer = currControlTimer;
  } /* End of control loop */
}
