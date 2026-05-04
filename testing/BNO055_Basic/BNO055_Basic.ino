#include <Wire.h>

#define RST             11    // BNO RST pin number
#define BNO_ADDR        0x28  // I2C address of first BNO
#define PAGE_ID         0x07  // BNO register: page select
#define ACC_DATA_X_LSB  0x08  // BNO page 0 register: Acceleration Data X LSB
#define OPR_MODE        0x3D  // BNO page 0 register: Operation Mode <3:0>
#define ACC_CONFIG      0x08  // BNO page 1 register: Accelerometer Config
#define MODE_AMG        0x07  // non-fusion mode with accel/gyro/mag

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data)  // write one BNO register
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // send stop
}

void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length)  // read multiple BNO registers into buffer
{
  for (uint32_t n=0; n<length; n++)
  {
    if ((n & 31) == 0)  // transfer up to 32 bytes at a time
    {
      Wire.beginTransmission(i2c_addr);
      Wire.write(reg+n);
      Wire.endTransmission(false);  // send restart
      Wire.requestFrom(i2c_addr, min(length-n, 32));
    }
    *buf++ = Wire.read();
  }
}

void setup(void)
{
  Serial.begin(9600);        // initialize serial port

  Wire.begin();                // initialize I2C

  digitalWrite(RST,0);
  pinMode(RST, OUTPUT);        // assert BNO RST
  delay(1);
  pinMode(RST, INPUT_PULLUP);  // deassert BNO RST
  delay(800);                  // allow time for BNO to boot

  bno_write(BNO_ADDR, PAGE_ID, 1);          // register page 1
  bno_write(BNO_ADDR, ACC_CONFIG, 0x0C);    // accel 2g range (default value 0x0D)
  bno_write(BNO_ADDR, PAGE_ID, 0);          // register page 0
  bno_write(BNO_ADDR, OPR_MODE, MODE_AMG);  // operating mode
  delay(10);                                // allow time for BNO to switch modes
}

void loop(void)
{
  struct
  {
    int16_t acc_x, acc_y, acc_z;
  } s;

  bno_read_multiple(BNO_ADDR, ACC_DATA_X_LSB, (uint8_t*)&s, sizeof s);  // read registers ACC_DATA_X_LSB through ACC_DATA_Z_MSB

  Serial.print("Accelerometer (100 * m/s^2): ");
  Serial.print(s.acc_x);  Serial.print(" ");
  Serial.print(s.acc_y);  Serial.print(" ");
  Serial.print(s.acc_z);  Serial.println("");

  delay(50);
}
