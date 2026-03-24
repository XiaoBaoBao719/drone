# SITL (Software In The Loop) Setup for Drone Controller

This directory contains the SITL simulator for testing the Arduino drone controller code over Serial.

## Features

- **Real-time 3D Visualization**: Shows drone position, orientation, and flight trajectory
- **Physics Simulation**: 6DOF quadcopter dynamics with realistic motor thrust models
- **Serial Communication**: Bidirectional IMU/motor PWM exchange with Arduino
- **Data Logging**: Complete simulation state history for analysis

## Setup Instructions

### 1. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 2. Modify Arduino Code
- In `RateModeR4.ino`, set `#define SITL_MODE (true)`
- Install ArduinoJson library via Library Manager
- Upload the modified code to your Arduino R4

### 3. Connect Arduino to Computer
- Connect Arduino via USB (creates COM port, e.g., COM3)
- Note the COM port number

### 4. Run the Simulator
```bash
python sitl_simulator.py
```
- Edit the `port='COM3'` in the script to match your Arduino's COM port
- The simulator will run for 30 seconds with real-time 3D plotting
- Data is logged to `sitl_log.json`

## 3D Visualization

The simulator displays:
- **Red quadcopter model**: Current position and orientation
- **Blue trajectory line**: Flight path history
- **Green dot**: Starting position
- **Red dot**: Current position
- **Dynamic axes**: Auto-scaling based on flight envelope

## How It Works

1. **Simulator** runs quadcopter physics in Python (250 Hz)
2. **Arduino** runs your controller code with SITL mode enabled
3. **Serial Bridge**:
   - Simulator sends IMU data (accel, gyro, mag) as JSON
   - Arduino sends PWM commands (4 motors) as binary data
4. **Real-time Plot**: Updates every 10 simulation steps
5. **Closed Loop**: Your PID controller runs on Arduino, physics in Python

## Tuning the Simulator

- Adjust `mass`, `L`, `kT`, `kD` in `QuadcopterDynamics` to match your real drone
- Tune noise models to match real sensor characteristics
- Validate against real hardware IMU data

## Debugging

- Check Serial connection (COM port correct?)
- Verify Arduino receives JSON IMU data
- Monitor PWM values sent back to simulator
- Use `sitl_log.json` to analyze simulation results

## Next Steps

- Test with your RC receiver simulation (currently uses neutral values)
- Tune physics parameters to match your real drone
- Add real-time plotting of control signals
- Compare SITL results with real flight data</content>
<parameter name="filePath">c:\Users\xbao2\workspaces\drone\simulation\README.md