# Open-Quadcopter
A reliable and simple quadcopter + flight controller from scratch. Completely hackable and easy to modify.

**Flight Control System:**
- Renesas 32-bit ARM Cortex-M4 MCU embedded on an Arduino Nano R4 

**Flight Command System:**
- Embedded Compute: Raspberry Pi 4 (2GB) SBC running OS Ubuntu 22.01

**Propulsion:**
- Electronic Speed Controllers: BrushlessTM 30A 2-4S lipo; bec 5v/2A
- Motors: Duckietown DX2205 2300 KV Brushless DC 3-Phase Motors

**Sensors:**
- Adafruit MPU6050 (Inertial Measurement Unit)
- Sparkfun MLX90393 (Triple-Axis Magnetometer)
- Adafruit VL53L4CX (Vertical Time-of-Flight sensing)
- Sony IMX219 Camera Module (Optical Flow for X-Y Attitude Sensing)
- (planned) Sparkfun GP1818MMK (56 channel) (GPS module)

**Advanced sensors:**
- s(planned) Intel realsense d435i
