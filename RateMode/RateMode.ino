#include "ESC.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "PPMReader.h"

#define DEBUG_MODE
#define LED_PIN (13)
#define PPM_INTERRUPT (2)
#define MPU_INTERRUPT (3)
bool blink_state = false;

const int NUM_CHANNELS = 6;
PPMReader ppm_(PPM_INTERRUPT, NUM_CHANNELS);
unsigned long receiverValue[NUM_CHANNELS] = {0.0};
unsigned long value = 0.0;

MPU6050 mpu(0x69);

/* MPU control status vars */
bool dmpReady = false;  // if mpu dmp successful init
uint8_t mpuIntStatus;   // interrupt status byte from MPU
uint8_t devStatus;      // status after operation {0 = success, !0 = error}
uint8_t packetSize;     // expected DMP packet size (default 42 bytes)
uint8_t fifoCount;      // count of all bytes in the fifo buffer
uint8_t fifoBuffer[64]; // FIFO buffer storage

/* orientation data */
Quaternion q;        // [w, x ,y ,z]
VectorInt16 aa;      // [x, y, z]
VectorInt16 aaReal;  // [x, y, z] gravity-free measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3];      // [psi, theta, phi] Euler conversion
float ypr[3];        // [yaw, pitch, roll] Angle conversion w/ gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

/* ESC & MOTOR control vars */

#define CCW_R (3)
#define CW_L (5)
#define CCW_L (6)
#define CW_R (9)

#define ESC_SPEED_MIN (1000)
#define ESC_SPEED_MAX (2000)
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
#define CH_ROLL (2)
#define CH_THR (3)
#define CH_PITCH (4)
#define CH_L_KNOB (5)
#define CH_R_KNOB (6)

int thr_in;
int roll_in;
int pitch_in;
int yaw_in;

/* Quadcopter Motor Layout

   (CW)   (Front)   (CCW)
      \\            //
       \\   ----   //
         |        |
(Left)   |        |   (Right)
         |        |
       //   ----   \\
      //            \\
   (CCW)   (Back)   (CW)
*/

ESC esc_ccw_r(CCW_R, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC esc_cw_l(CW_L, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC esc_ccw_l(CCW_L, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);
ESC esc_cw_r(CW_R, ESC_SPEED_MIN, ESC_SPEED_MAX, MIN_ARM_TIME);

/* Rate Mode flight control vars */

float rateX, rateY, rateZ;
float rateCalibX, rateCalibY, rateCalibZ;
int16_t imu_offsets[6] = {0.0};
int rateCalibNumber;

uint32_t loopTimer;

float setptRateX, setptRateY, setptRateZ;
float inputX, inputY, inputZ;
float errorRateX, errorRateY, errorRateZ;

float prevErrorRateX, prevErrorRateY, prevErrorRateZ;
float prevIntRateX, prevIntRateY, prevIntRateZ;
float PID_prev[] = {0.0, 0.0, 0.0}; // [PID_output, prev_I, prev_error]

// the following PID gains are w.r.t. the gyro rate
float PGainX = 0.6;
float PGainY = PGainX;
float PGainZ = 2.0;

float IGainX = 3.5;
float IGainY = PGainX;
float IGainZ = 12;

float DGainX = 0.03;
float DGainY = DGainX;
float DGainZ = 0.0;

// motor input values
float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

float pid(float err_, float prev_err_, float p_gain, float i_gain,
          float prev_i_gain, float d_gain)
{
  float pid_state[3] = {0.0};
  float p_ = p_gain * err_;
  float i_ = prev_i_gain + i_gain * (((err_ + prev_err_) * loopTimer) / 2);
  float d_ = d_gain * ((err_ - prev_err_) / loopTimer);

  float pid_output = p_ + i_ + d_;
  /* constrain output to avoid Integral windup */
  if (pid_output >= 400)
  {
    pid_output = 400;
  }
  else if (pid_output < 0)
  {
    pid_output = 0;
  }

  pid_state[0] = pid_output;
  pid_state[1] = i_;
  pid_state[2] = err_;
  return pid_state;
}

void resetPID(void)
{
  prevErrorRateX = 0.0;
  prevErrorRateY = 0.0;
  prevErrorRateZ = 0.0;
  prevIntRateX = 0.0;
  prevIntRateY = 0.0;
  prevIntRateZ = 0.0;
}

void readReceiver(void)
{
  /* read latest valid values from all channels */
  for (int channel = 1; channel <= NUM_CHANNELS; channel++)
  {
    value = ppm_.latestValidChannelValue(channel, 0);
    receiverValue[channel - 1] = value;
  }
}

void calibrateIMU(void)
{
  // apply arbitrary initial gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  offsets = mpu.GetActiveOffsets();
  mpu.PrintActiveOffsets();
}

void updateIMU(void)
{
  if (!dmpReady)
    return;
  // read latest packet
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    // convert fifo buffer values
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
#ifdef DEBUG_MODE
    // Serial.print("quat\t");
    // Serial.print(q.w);
    // Serial.print("\t");
    // Serial.print(q.x);
    // Serial.print("\t");
    // Serial.print(q.y);
    // Serial.print("\t");
    // Serial.println(q.z);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180 / M_PI);
#endif
  }
}

void setup()
{
  // join I2Cbus thread
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock.

  Serial.begin(115200);
  Serial.println(F("Initialize I2C devices..."));
  mpu.initialize();
  pinMode(MPU_INTERRUPT, INPUT);

  // check IMU connection
  Serial.println(F("Testing device connection..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection success!") : F("MPU6050 connection failed."));

  Serial.println(F("Initializing DMP..."));
  // continue looping until good response from dmp
  devStatus = mpu.dmpInitialize();
  while (devStatus != 0)
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    Serial.println("Re-attempt dmp initialize...");
    delay(100);
    devStatus = mpu.dmpInitialize();
  }

  // calibrate IMU and activate dmp
  calibrateIMU();
  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // activate arduino interrupt detection
  Serial.print(digitalPinToInterrupt(MPU_INTERRUPT));
  Serial.println(F(")..."));
  attachInterrupt(digitalPinToInterrupt(MPU_INTERRUPT), dmpDataReady, RISING);
  mpuIntStatus = mpu.getIntStatus();
  Serial.println(F("DMP ready!"));
  dmpReady = true;
  // get expected DMP packet size
  packetSize = mpu.dmpGetFIFOPacketSize();

  /* Motor setup */
  esc_ccw_r.arm();
  esc_cw_l.arm();
  esc_ccw_l.arm();
  esc_cw_r.arm();

  /* RC PPM Receiver Setup */
  /* Safety check: if throttle above threshold do not advance */
  readReceiver();
  uint32_t init_throttle = receiverValue[CH_THR - 1];
  while (init_throttle >= 1100)
  {
    readReceiver();
    delay(100);
  }

  pinMode(LED_PIN, OUTPUT);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
