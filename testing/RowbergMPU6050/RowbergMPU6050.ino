
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include <Arduino.h>

#define MPU_INT (D2)
#define MPU_PACK_SIZE (42)  // default packet size for the mpu6050 fifo buffer

#define DEBUG_MODE
// Assign MPU6050 I2c address
MPU6050 mpu(0x69);     

/* MPU control variables */
bool dmpReady = false;  // true if  mpu dmp successful init
bool mpuConnect = false; // true if mpu is connected
uint8_t mpuIntStatus;   // interrupt status byte from MPU
uint8_t devStatus;      // status after operation {0 = success, !0 = error}
uint8_t packetSize;     // expected DMP packet size (default 42 bytes)
uint8_t fifoCount;      // count of all bytes in the fifo buffer
uint8_t fifoBuffer[64]; // FIFO buffer storage

/* Orientation Data */
Quaternion q;        // [w, x ,y ,z]
VectorInt16 aa;      // [x, y, z]
VectorInt16 aaReal;  // [x, y, z] gravity-free measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel measurements
VectorFloat gravity; // [x, y, z] gravity vector
float euler[3];      // [psi, theta, phi] Euler conversion
float ypr[3];        // [yaw, pitch, roll] Angle conversion w/ gravity vector

/* MPU calibration */
int16_t* imu_offsets;

volatile bool mpuIntStatus = false;

// void ARDUINO_ISR_ATTR isr() {
void interruptISR() {
    mpuIntStatus = true;
}

void calibrateIMU(void) {
  // apply arbitrary initial gyro offsets
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
}

void readDMP() {
    static uint64_t LastGoodPacketTime;
    mpuIntStatus = false;
    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    // Serial.println(String(fifoCount) + "   " + String(packetSize));
    if ((!fifoCount) || (fifoCount % packetSize))
    { // failed Reset, wait until next timer pass
        digitalWrite(LED_BUILTIN, HIGH);
        digitalWrite(LED_RED, HIGH);
        mpu.resetFIFO();
        // Serial.println(F("Resetting FIFO..."));
    }
    else
    {
        // while (fifoCount >= packetSize)
        // {
        //     mpu.getFIFOBytes(fifoBuffer, packetSize);
        //     fifoCount -= packetSize;
        // }
        while (fifoCount < packetSize)
            fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        // Convert the fifo packet into an orientiation format
        get_imu_data();
        LastGoodPacketTime = millis();
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // blink the led to indicate processing packets
    }
}

void get_imu_data(void)
{
    if (!dmpReady)
        return;
    // if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) func not present for esp32 vers?
    // {
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
    // }
}

// ================================================================
// ===                      i2c setup                    ===
// ================================================================
void i2c_setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    // join I2C bus
    Wire.begin();
    // Set I2C to update at 400kHz clock
    Wire.setClock(400000);
    #ifdef __AVR__  
    TWBR = 24;  // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #endif
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
}

// ================================================================
// ===                      MPU6050 setup                     ===
// ================================================================
bool mpu6050_setup() {
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    // if (devStatus != 0)
    //     return false;
    if (!mpu.testConnection()) {
      Serial.print("mpu connection failed!");
      return false;
    } else {
      Serial.print("mpu connection successful!");
    }

    // calibrate IMU and activate dmp
    calibrateIMU();
    // Enable DMP
    Serial.println(F("Enabling DMP..."));
    // mpu.setDMPEnabled(true);
    // Enable Arduino interrupt detection
    Serial.println(F("Enabling ext. interrupt on pin 2 of Uno"));
    Serial.print("mpu.getInterruptDrive=  ");
    Serial.println(mpu.getInterruptDrive());
// activate arduino interrupt detection
// Serial.print(digitalPinToInterrupt(MPU_INTERRUPT));
// Serial.println(F(")..."));
#if defined ESP32
    pinMode(MPU_INT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(MPU_INT), interruptISR, CHANGE);
#elif defined ARDUINO
    attachInterrupt(digitalPinToInterrupt(MPU_INT), dmpDataReady, CHANGE);
#endif

    // get expected DMP packet size
    // packetSize = mpu.dmpGetFIFOPacketSize();  doesn't work on esp32 atm
    packetSize = MPU_PACK_SIZE;
    delay(1000);

    mpu.resetFIFO();
    mpuIntStatus = mpu.getIntStatus();
    // dmpReady = true;
    mpuConnect = false; // wait for the next interrupt

    // Serial.println(F("DMP ready!"));
    Serial.println(F("mpu ready!"));
    return true;
}

void setup() {
    
    // Set USB Serial baud rate
    Serial.begin(115200);
    // Set up I2C connection
    i2c_setup();
    // Attempt set up MPU6050
    static int MPUInitCtr = 0;
    while (!mpu6050_setup())
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        // char *StatStr[5]{"No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};
        MPUInitCtr++;

        Serial.print(F("MPU connection Try #"));
        Serial.println(MPUInitCtr);
        // Serial.print(F("DMP Initialization failed (code "));
        // Serial.print(StatStr[devStatus]);
        Serial.println(F(")"));

        // if (MPUInitCtr >= 10)
        //     return; // only try 10 times
        delay(1000);
    }
}

void loop() {
    if (mpuIntStatus) {
        readDMP();
    }
}