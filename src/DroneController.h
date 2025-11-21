#include <string.h>

#include "Wire.h"
#include "Arduino.h"
#include <BasicLinearAlgebra.h>

#include "MPU6050Plus.h"
#include "Sensor_Fusion/DroneEKF.h"

class DroneController 
{

public:

    DroneController(std::string name) {
        
        _name = name;
        
        Init();

    };

    ~DroneController();

    void Init();

    void Reset();

    void UpdateEstimates(Vector3 pos, Vector3 vel, Quaternion attitude);

    float PID(float err_, float prev_err_, float p_gain, float i_gain,
        float prev_i_gain, float d_gain, float dt);

    void resetPID();

    float* CalculateMotorCommands(float inputThrottle, Vector3 pid_outs, float motor_out_cmds[]);
    Vector4 CalculateMotorCommands(float inputThrottle, Vector3 pid_outs);

    void readReceiver();



    // ---------------- PARAMETERS --------------

    std::string _name;

    /*              System Model Parameters     */
    float L;                    // span length (m) from center to motor
    float mass;                 // total mass  (+ est. payload mass)
    float Ixx, Iyy, Izz;        // moments of inertia
    float kappa;                // est. torque (N-m) produced per motor

    /*              Controller Gains    */ 
    // the following PID gains are w.r.t. the gyro (x-y-z) angular rate
    float PGainX_w = 0.6;
    float PGainY_w = 0.6;
    float PGainZ_w = 2.0;

    float IGainX_w = 3.5;
    float IGainY_w = 3.5;
    float IGainZ_w = 12;

    float DGainX_w = 0.03;
    float DGainY_w = 0.03;
    float DGainZ_w = 0.0;

    // the follow PID gains are w.r.t. angle (x-y) position
    float PGain_Roll = 2;
    float PGain_Pitch = 2;
    float PGain_Yaw = 0;

    float IGain_Roll = 0;
    float IGain_Pitch = 0;
    float IGain_Yaw = 0;

    float DGain_Roll = 0;
    float DGain_Pitch = 0;
    float DGain_Yaw = 0;

    // the followung PID gains w.r.t altitude (+z) rate
    float PGain_VelZ = 3.5;
    float IGain_VelZ = 0.0015;
    float DGain_VelZ = 0.01;

    // Controller Limits
    float pid_out_hi_limit = 400;
    float pid_out_lo_limit = -400;

    // Controller Params

    // ---  Angular rate params ---
    float rateX, rateY, rateZ;
    float rateCalibX, rateCalibY, rateCalibZ;
    int16_t* imu_offsets;
    int rateCalibNumber;

    float setptRateX, setptRateY, setptRateZ;
    float errorRateX, errorRateY, errorRateZ;
    float prevErrorRateX, prevErrorRateY, prevErrorRateZ;
    float prevIGainRateX, prevIGainRateY, prevIGainRateZ;

    float PID_prev[3] = { 0.0, 0.0, 0.0 };  // [PID_output, prev_I, prev_error]
    // Vector3 PID_prev;

    // --- Angle position params ---
    float angleX, angleY;

    float setptAngleX, setptAngleY;
    float errorAngleX, errorAngleY;
    float prevErrorAngleX, prevErrorAngleY;
    float prevIGainAngleX, prevIGainAngleY; 

    // --- Altitude rate params ---
    float velZ;

    float setptVelZ;
    float errorVelZ;
    float prevErrorVelZ;
    float prevIGainVelZ;


    /*           State Estimate          */ 
    // attitude date
    Quaternion q;         // [w, x ,y ,z]
    VectorInt16 aa;       // [x, y, z]
    VectorInt16 aaReal;   // [x, y, z] gravity-free measurements
    VectorInt16 aaWorld;  // [x, y, z] world-frame accel measurements
    VectorFloat gravity;  // [x, y, z] gravity vector
    float euler[3];       // [psi, theta, phi] Euler conversion
    float ypr[3];         // [yaw, pitch, roll] Angle conversion w/ gravity vector

    /* position and velocity */
    Vector3 vel_est;
    Vector3 pos_est;


    // Teleop Parameters
    float inputX, inputY, inputZ, input_throttle;

    int mode;       // 1 - Rate Mode
                    // 2 - Velocity Mode
                    // 3 - Trajectory Mode


};