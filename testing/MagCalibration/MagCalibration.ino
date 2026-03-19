#include <Wire.h>
#include <MLX90393.h> //From https://github.com/tedyapo/arduino-MLX90393 by Theodore Yapo
#include <Arduino.h>



MLX90393 mlx;
MLX90393::txyz data; //Create a structure, called data, of four floats (t, x, y, and z)


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("MLX90393 Calibration");
  Wire.begin();
  //Connect to sensor with I2C address jumpers set: A1 = 1
  //Use DRDY pin connected to A3
  //Returns byte containing status bytes
  byte status = mlx.begin();

  // Configure sensor
  mlx.setGainSel(1);
  mlx.setResolution(0, 0, 0); //x, y, z
  mlx.setOverSampling(0);
  mlx.setDigitalFiltering(0);
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
