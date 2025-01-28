#include "ESC.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PPMReader>

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
uint8_t fifoBuffer[64];  // FIFO buffer storage

/* orientation data */
Quaternion q;           // [w, x ,y ,z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z] gravity-free measurements
VectorInt16 aaWorld;    // [x, y, z] world-frame accel measurements
VectorFloat gravity;    // [x, y, z] gravity vector
float euler[3];         // [psi, theta, phi] Euler conversion
float ypr[3];           // [yaw, pitch, roll] Angle conversion w/ gravity vector

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

 
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
float PID_prev[] = {0.0,0.0,0.0};

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

float pid_(float err_, float prev_err_, float p_gain, float i_gain, 
              float prev_i_gain, float d_gain)
{
    float p_ = p_gain * err_;
    float i_ = prev_i_gain + i_gain * (((err_ + prev_err_) * loopTimer) / 2);
    float d_ = d_gain * ((err_ - prev_err_) / loopTimer);

    return p_ + i_ + d_;
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
  if (!dmpReady) return;  
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

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}
