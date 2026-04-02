
const RAW = `

/**
 * @file    RateMode.ino
 * @brief   Rate-mode flight controller for X-frame quadcopter
 *
 * @target  Arduino Nano R4 (Renesas RA4M1, 48 MHz Cortex-M4)
 *          Control loop budget @ 250 Hz: 192 000 cycles -- well within limits.
 *
 * @imu     MPU6050 via I2C @ 400 kHz (Wire1 / Qwiic connector)
 * @rc      6-channel PPM receiver (FS-iA6B or compatible)
 *
 * Coordinate Frame (body-fixed, NED):
 *   X -> Forward (nose)     Roll  (+) -> right wing down
 *   Y -> Right (starboard)  Pitch (+) -> nose up
 *   Z -> Down               Yaw   (+) -> nose right (CW from above)
 *
 * Motor Layout (X-frame, top-down):
 *
 *      M4 (FL, CW) <------ [FRONT / +X] ------> M1 (FR, CCW)
 *            \\                                       /
 *             \\                                     /
 *  [Left / -Y] x------------------------------------x [Right / +Y]
 *             /                                     \\
 *            /                                       \\
 *      M3 (BL, CCW)                             M2 (BR, CW)
 *
 * Mixing Matrix:
 *   M1 (FR, CCW) = Thr - Roll + Pitch + Yaw
 *   M2 (BR, CW)  = Thr - Roll - Pitch - Yaw
 *   M3 (BL, CCW) = Thr + Roll - Pitch + Yaw
 *   M4 (FL, CW)  = Thr + Roll + Pitch - Yaw
 *
 *   Roll  (+) -> right wing down : left motors up (M3, M4), right down (M1, M2)
 *   Pitch (+) -> nose up         : front motors up (M1, M4), rear down (M2, M3)
 *   Yaw   (+) -> CW from above   : CCW props up (M1, M3), CW props down (M2, M4)
 *
 * Defects Fixed from RateModeR4.ino:
 *   1. Gyro sign convention: configurable GYRO_*_SIGN macros; verify
 *      on the bench before first flight (see SECTION 6 below).
 *   2. Motor trim constants reset to 1.0; characterise on thrust stand.
 *   3. Integral accumulator: PIDState.integral tracks the running sum,
 *      not the gain constant -- i term now accumulates correctly.
 *   4. Derivative-on-measurement replaces derivative-of-setpoint,
 *      giving damping against disturbances even when sticks are still.
 *   5. All PID time deltas use DT_S (seconds), not raw milliseconds.
 *   6. Motor mixing matrix corrected for proper roll/pitch/yaw signs.
 *   7. Timing upgraded from millis() to micros() for 4 us resolution.
 *   8. CH_ROLL / CH_YAW comment-vs-define mismatch resolved.
 *   9. i_gain = i_gain no-op removed; anti-windup uses constrain().
 */

// -----------------------------------------------------------------------
//  INCLUDES
// -----------------------------------------------------------------------
#include "ESC.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "ppm.h"
#include <MPU6050Plus.h>

// =======================================================================
//  SECTION 1 -- DEBUG CONFIGURATION
// =======================================================================

/** Set any flag to (true) to enable that serial output stream. */
static constexpr bool DBG_RC     = false;  ///< Raw + scaled RC inputs
static constexpr bool DBG_IMU    = true;   ///< Gyro rates (serial plotter)
static constexpr bool DBG_PID    = false;  ///< Per-axis P / I / D components
static constexpr bool DBG_MOTORS = false;  ///< Final motor PWM commands

// =======================================================================
//  SECTION 2 -- TIMING CONFIGURATION
// =======================================================================

/**
 * Loop periods in microseconds. The Nano R4 runs at 48 MHz.
 *
 * At 250 Hz:  1 iteration = 4 000 us = 192 000 clock cycles.
 * The MPU6050 DLPF mode supports up to 1 kHz; 250 Hz is conservative
 * and leaves ample budget for I2C reads (~400 us at 400 kHz) and math.
 */
static constexpr uint32_t CONTROL_PERIOD_US = 4000U;   ///< 250 Hz
static constexpr uint32_t IMU_PERIOD_US     = 4000U;   ///< 250 Hz
static constexpr uint32_t RC_PERIOD_US      = 10000U;  ///< 100 Hz

/** Scalar time step for PID math -- always in seconds, never milliseconds. */
static constexpr float DT_S = static_cast<float>(CONTROL_PERIOD_US) * 1e-6f;

// =======================================================================
//  SECTION 3 -- HARDWARE PIN CONFIGURATION
// =======================================================================

// --- ESC PWM output pins ---
static constexpr uint8_t MOTOR_1_PIN = 5;   ///< Front-Right (CCW)
static constexpr uint8_t MOTOR_2_PIN = 6;   ///< Back-Right  (CW)
static constexpr uint8_t MOTOR_3_PIN = 9;   ///< Back-Left   (CCW)
static constexpr uint8_t MOTOR_4_PIN = 10;  ///< Front-Left  (CW)

// --- ESC / PWM limits (us) ---
static constexpr int ESC_STOP     = 900;   ///< Below-min disarm command
static constexpr int ESC_MIN      = 1000;  ///< Motor just spinning
static constexpr int ESC_MAX      = 2000;  ///< Full throttle
static constexpr int ESC_ARM_MS   = 500;   ///< ESC arming pulse duration

/**
 * Flying throttle window.  THROTTLE_MAX is deliberately kept below
 * ESC_MAX so the PID always has headroom to increase a motor without
 * saturating the output -- critical for stability on asymmetric manoeuvres.
 */
static constexpr int THROTTLE_MIN = 1050;  ///< Below this -> disarm motors
static constexpr int THROTTLE_MAX = 1800;  ///< PID headroom above this point

// --- Interrupt pins ---
static constexpr uint8_t PPM_INT_PIN = 3;  ///< PPM signal from RC receiver
static constexpr uint8_t MPU_INT_PIN = 2;  ///< MPU6050 data-ready (reserved)

// =======================================================================
//  SECTION 4 -- RC RECEIVER CONFIGURATION
// =======================================================================

/**
 * Physical PPM channel assignments -- FS-iA6B standard mapping.
 * Values are 1-indexed channel numbers as labelled on the receiver.
 *
 * If your transmitter is mapped differently, edit ONLY these defines.
 * Verify by checking Serial output with DBG_RC = true.
 */
static constexpr uint8_t CH_ROLL  = 1;
static constexpr uint8_t CH_PITCH = 2;
static constexpr uint8_t CH_THR   = 3;
static constexpr uint8_t CH_YAW   = 4;
static constexpr uint8_t CH_AUX1  = 5;
static constexpr uint8_t CH_AUX2  = 6;
static constexpr uint8_t NUM_CHANNELS = 6;

// RC pulse geometry (us)
static constexpr float RC_CENTER   = 1500.0f;  ///< Neutral / centre pulse
static constexpr float RC_RANGE    = 500.0f;   ///< Centre -> full deflection
static constexpr float RC_DEADBAND = 20.0f;    ///< Ignore noise within +-us

// =======================================================================
//  SECTION 5 -- RATE SETPOINT LIMITS
// =======================================================================

/**
 * Maximum body rates commanded by full stick deflection.
 * These are intentionally conservative for initial flight testing.
 * Increase gradually once the platform is confirmed stable.
 */
static constexpr float MAX_ROLL_RATE_DPS  = 200.0f;  ///< deg/s at full stick
static constexpr float MAX_PITCH_RATE_DPS = 200.0f;  ///< deg/s at full stick
static constexpr float MAX_YAW_RATE_DPS   = 120.0f;  ///< deg/s at full stick

// =======================================================================
//  SECTION 6 -- IMU CONFIGURATION
// =======================================================================

/**
 * Gyro axis sign corrections.
 *
 * The MPU6050 raw axes may not align with your NED body frame depending
 * on physical mounting orientation.  Use +1 or -1 to flip each axis.
 *
 * HOW TO VERIFY (do this before first flight, props off):
 *   1. Enable DBG_IMU = true and open the Serial Plotter.
 *   2. Hold the craft level.  All three rates should read near 0.
 *   3. Slowly roll the craft to the RIGHT (right wing down).
 *      -> gyroRoll should be POSITIVE.  If negative, set GYRO_ROLL_SIGN = -1.
 *   4. Slowly pitch the craft NOSE UP.
 *      -> gyroPitch should be POSITIVE.  If negative, set GYRO_PITCH_SIGN = -1.
 *   5. Rotate the craft CLOCKWISE when viewed from above (yaw right).
 *      -> gyroYaw should be POSITIVE.  If negative, set GYRO_YAW_SIGN = -1.
 */
static constexpr int8_t GYRO_ROLL_SIGN  = +1;
static constexpr int8_t GYRO_PITCH_SIGN = +1;
static constexpr int8_t GYRO_YAW_SIGN   = +1;

static constexpr float IMU_DT_S = static_cast<float>(IMU_PERIOD_US) * 1e-6f;

// =======================================================================
//  SECTION 7 -- PID GAINS
// =======================================================================

/**
 * Rate-mode PID gains.
 *
 * TUNING PROCEDURE (props off first, then low altitude hover):
 *   Step 1 -- Set I = D = 0.  Raise P until the craft starts to oscillate
 *             rapidly on that axis, then halve P.
 *   Step 2 -- Slowly raise D until oscillations dampen noticeably.
 *   Step 3 -- Add a small I to eliminate drift at sustained rates.
 *
 * Roll and pitch are usually symmetric on a well-built X-frame.
 * Yaw typically needs a higher P and very little D.
 */
static constexpr float ROLL_KP  = 0.80f;
static constexpr float ROLL_KI  = 0.002f;
static constexpr float ROLL_KD  = 0.018f;

static constexpr float PITCH_KP = 0.80f;
static constexpr float PITCH_KI = 0.002f;
static constexpr float PITCH_KD = 0.018f;

static constexpr float YAW_KP   = 1.40f;
static constexpr float YAW_KI   = 0.003f;
static constexpr float YAW_KD   = 0.004f;

// Integral accumulator clamp (deg*s units)
static constexpr float I_LIMIT_ROLL  = 80.0f;
static constexpr float I_LIMIT_PITCH = 80.0f;
static constexpr float I_LIMIT_YAW   = 40.0f;

// Symmetric PID output clamp (us equivalent)
// Must be significantly smaller than (THROTTLE_MAX - THROTTLE_MIN) / 2
// so that no single axis can saturate the throttle range.
static constexpr float PID_OUT_LIMIT = 300.0f;

// =======================================================================
//  SECTION 8 -- MOTOR TRIM
// =======================================================================

/**
 * Per-motor output scale factor.
 *
 * Leave at 1.0 until you have measured each motor's thrust on a stand
 * and confirmed it deviates from the mean by more than ~2 %.
 * A factor of 0.95 on one motor represents a 5 % thrust reduction on
 * that arm -- it is a blunt tool and is NOT a substitute for PID tuning.
 *
 * Example: if M4 produces 5 % more thrust at the same PWM as the others,
 * set M4_TRIM = 1.0 / 1.05 = 0.952.
 */
static constexpr float M1_TRIM = 1.0f;  ///< Front-Right
static constexpr float M2_TRIM = 1.0f;  ///< Back-Right
static constexpr float M3_TRIM = 1.0f;  ///< Back-Left
static constexpr float M4_TRIM = 1.0f;  ///< Front-Left

// =======================================================================
//  SECTION 9 -- DATA TYPES
// =======================================================================

/**
 * @brief  Persistent state for one PID axis.
 *
 * FIX: The original code stored (prevIGainX = IGainX) -- the constant
 * gain -- overwriting the accumulated integral on every iteration.
 * This struct correctly separates the gain (a constant) from the
 * accumulated integral value (a running state variable).
 */
struct PIDState {
    float integral;        ///< Running integral accumulator (physical units)
    float prevMeasurement; ///< Previous gyro reading for D-on-measurement
};

/** Scalar motor commands, indexed 0-3 -> M1-M4. */
struct MotorOutputs {
    float cmd[4];
};

// =======================================================================
//  SECTION 10 -- HARDWARE OBJECTS
// =======================================================================

MPU6050     mpu(0x68, &Wire1);  ///< Wire1 = Qwiic connector on Nano R4
MPU6050Plus imu;                ///< Calibrated angle / rate wrapper

ESC m1_esc(MOTOR_1_PIN, ESC_MIN, ESC_MAX, ESC_ARM_MS);
ESC m2_esc(MOTOR_2_PIN, ESC_MIN, ESC_MAX, ESC_ARM_MS);
ESC m3_esc(MOTOR_3_PIN, ESC_MIN, ESC_MAX, ESC_ARM_MS);
ESC m4_esc(MOTOR_4_PIN, ESC_MIN, ESC_MAX, ESC_ARM_MS);

// =======================================================================
//  SECTION 11 -- GLOBAL STATE
// =======================================================================

// --- Loop timers (us) ---
static uint32_t prevControlUs = 0U;
static uint32_t prevImuUs     = 0U;
static uint32_t prevRcUs      = 0U;

// --- RC ---
static int32_t rcRaw[NUM_CHANNELS] = {};  ///< Raw PPM pulse widths (us)
static float rcThrottle = 0.0f;           ///< Throttle: us [1000, THROTTLE_MAX]
static float rcRoll     = 0.0f;           ///< Roll rate setpoint  (deg/s)
static float rcPitch    = 0.0f;           ///< Pitch rate setpoint (deg/s)
static float rcYaw      = 0.0f;           ///< Yaw rate setpoint   (deg/s)

// --- IMU ---
static float gyroRoll  = 0.0f;  ///< Body roll rate  (deg/s, sign-corrected)
static float gyroPitch = 0.0f;  ///< Body pitch rate (deg/s, sign-corrected)
static float gyroYaw   = 0.0f;  ///< Body yaw rate   (deg/s, sign-corrected)

// --- PID state ---
static PIDState pidRoll  = {};
static PIDState pidPitch = {};
static PIDState pidYaw   = {};

// --- Motor outputs ---
static MotorOutputs motors = {{ static_cast<float>(ESC_MIN),
                                 static_cast<float>(ESC_MIN),
                                 static_cast<float>(ESC_MIN),
                                 static_cast<float>(ESC_MIN) }};

// =======================================================================
//  SECTION 12 -- UTILITY FUNCTIONS
// =======================================================================

/** Return 0 if |value| < deadband, else return value unchanged. */
static inline float applyDeadband(float value, float deadband)
{
    return (fabsf(value) < deadband) ? 0.0f : value;
}

/**
 * @brief  Map a raw RC us value to a rate setpoint in deg/s.
 * Applies deadband first so the craft does not drift when sticks are
 * centred, then scales linearly to [-maxRate, +maxRate].
 */
static float scaleRcToRate(int32_t rawUs, float maxRate)
{
    const float centred = applyDeadband(
        static_cast<float>(rawUs) - RC_CENTER, RC_DEADBAND);
    return (centred / RC_RANGE) * maxRate;
}

// =======================================================================
//  SECTION 13 -- RC RECEIVER
// =======================================================================

static void readReceiver()
{
    rcRaw[CH_ROLL  - 1] = ppm.read_channel(CH_ROLL);
    rcRaw[CH_PITCH - 1] = ppm.read_channel(CH_PITCH);
    rcRaw[CH_THR   - 1] = ppm.read_channel(CH_THR);
    rcRaw[CH_YAW   - 1] = ppm.read_channel(CH_YAW);
    rcRaw[CH_AUX1  - 1] = ppm.read_channel(CH_AUX1);
    rcRaw[CH_AUX2  - 1] = ppm.read_channel(CH_AUX2);

    rcThrottle = static_cast<float>(
        constrain(rcRaw[CH_THR - 1], ESC_MIN, THROTTLE_MAX));

    rcRoll  = scaleRcToRate(rcRaw[CH_ROLL  - 1], MAX_ROLL_RATE_DPS);
    rcPitch = scaleRcToRate(rcRaw[CH_PITCH - 1], MAX_PITCH_RATE_DPS);
    rcYaw   = scaleRcToRate(rcRaw[CH_YAW   - 1], MAX_YAW_RATE_DPS);

    if (DBG_RC) {
        Serial.print("Thr:");   Serial.print(rcThrottle);
        Serial.print(",Roll:");  Serial.print(rcRoll);
        Serial.print(",Pitch:"); Serial.print(rcPitch);
        Serial.print(",Yaw:");   Serial.println(rcYaw);
    }
}

// =======================================================================
//  SECTION 14 -- IMU
// =======================================================================

static void updateImu()
{
    imu.updateRawMeasurements();
    imu.complementaryFilter();

    // Apply sign corrections to align IMU axes with NED body frame.
    // FIX: An inverted roll axis causes positive feedback -- the craft rolls
    // right, the controller sees a left-roll error, commands more right roll,
    // and the craft flips. Verify GYRO_*_SIGN per SECTION 6 before flight.
    gyroRoll  = GYRO_ROLL_SIGN  * imu.getGyroX();  // deg/s
    gyroPitch = GYRO_PITCH_SIGN * imu.getGyroY();  // deg/s
    gyroYaw   = GYRO_YAW_SIGN   * imu.getGyroZ();  // deg/s

    if (DBG_IMU) {
        Serial.print(",gyroRoll:");  Serial.print(gyroRoll,  2);
        Serial.print(",gyroPitch:"); Serial.print(gyroPitch, 2);
        Serial.print(",gyroYaw:");   Serial.println(gyroYaw, 2);
    }
}

// =======================================================================
//  SECTION 15 -- PID CONTROLLER
// =======================================================================

/**
 * @brief  Compute one PID iteration (derivative-on-measurement).
 *
 * D-on-measurement prevents the large spike that occurs when the setpoint
 * steps sharply (stick input). The gyro signal is continuous, so the
 * derivative term damps disturbances smoothly at all times.
 *
 * Anti-windup: the integral accumulator is symmetrically clamped.
 * When the limit is hit, new error area is discarded.
 *
 * FIX: Original passed prevIGainX = IGainX (the gain constant) back into
 * prev_i_gain each loop, so the accumulated integral was reset every
 * iteration. PIDState.integral now correctly persists across calls.
 *
 * FIX: Original used CONTROL_LOOP_TIME_MS (integer ms) as the time delta,
 * making I and D gains wrong by 1000x. All math now uses DT_S (seconds).
 *
 * @param setpt       Desired rate (deg/s)
 * @param measurement Measured gyro rate (deg/s)
 * @param kp, ki, kd  PID gains
 * @param dt          Loop period in seconds
 * @param iLimit      Symmetric integral clamp
 * @param outLimit    Symmetric output clamp (us equivalent)
 * @param state       Persistent axis state (modified in-place)
 * @return            Clamped PID output
 */
static float pidCompute(float setpt, float measurement,
                         float kp, float ki, float kd,
                         float dt, float iLimit, float outLimit,
                         PIDState &state)
{
    const float error = setpt - measurement;

    // --- Proportional ---
    const float p = kp * error;

    // --- Integral with anti-windup clamping ---
    state.integral = constrain(state.integral + ki * error * dt,
                               -iLimit, iLimit);

    // --- Derivative on measurement ---
    // FIX: original computed D as kd*(setpt - prev_setpt)/dt, which only
    // fires on stick movement and gives zero damping against disturbances.
    const float d = -kd * (measurement - state.prevMeasurement) / dt;

    state.prevMeasurement = measurement;

    const float output = p + state.integral + d;

    if (DBG_PID) {
        Serial.print("  P:"); Serial.print(p, 3);
        Serial.print(" I:");  Serial.print(state.integral, 3);
        Serial.print(" D:");  Serial.print(d, 3);
    }

    return constrain(output, -outLimit, outLimit);
}

static void resetPID()
{
    // Seed prevMeasurement with current gyro so the first D term is zero,
    // not a cold-start spike from (measurement - 0).
    pidRoll  = { 0.0f, gyroRoll  };
    pidPitch = { 0.0f, gyroPitch };
    pidYaw   = { 0.0f, gyroYaw   };
}

// =======================================================================
//  SECTION 16 -- MOTOR MIXING
// =======================================================================

/**
 * @brief  Translate throttle + PID outputs into per-motor PWM commands.
 *
 * FIX: Original mixing had incorrect signs for pitch (M2, M3 swapped)
 * and yaw (all four signs wrong). Matrix is derived from first principles
 * using the motor layout diagram and NED sign convention above.
 *
 *   M1 (FR, CCW) = Thr - Roll + Pitch + Yaw
 *   M2 (BR, CW)  = Thr - Roll - Pitch - Yaw
 *   M3 (BL, CCW) = Thr + Roll - Pitch + Yaw
 *   M4 (FL, CW)  = Thr + Roll + Pitch - Yaw
 */
static void mixMotors(float throttle, float rollOut,
                       float pitchOut, float yawOut)
{
    motors.cmd[0] = M1_TRIM * (throttle - rollOut + pitchOut + yawOut);
    motors.cmd[1] = M2_TRIM * (throttle - rollOut - pitchOut - yawOut);
    motors.cmd[2] = M3_TRIM * (throttle + rollOut - pitchOut + yawOut);
    motors.cmd[3] = M4_TRIM * (throttle + rollOut + pitchOut - yawOut);

    for (int i = 0; i < 4; i++) {
        motors.cmd[i] = constrain(motors.cmd[i],
                                  static_cast<float>(THROTTLE_MIN),
                                  static_cast<float>(ESC_MAX));
    }

    if (DBG_MOTORS) {
        Serial.print("M1:"); Serial.print(motors.cmd[0]);
        Serial.print(",M2:"); Serial.print(motors.cmd[1]);
        Serial.print(",M3:"); Serial.print(motors.cmd[2]);
        Serial.print(",M4:"); Serial.println(motors.cmd[3]);
    }
}

static void writeMotors()
{
    m1_esc.speed(static_cast<int>(motors.cmd[0]));
    m2_esc.speed(static_cast<int>(motors.cmd[1]));
    m3_esc.speed(static_cast<int>(motors.cmd[2]));
    m4_esc.speed(static_cast<int>(motors.cmd[3]));
}

static void disarmMotors()
{
    m1_esc.speed(ESC_STOP);
    m2_esc.speed(ESC_STOP);
    m3_esc.speed(ESC_STOP);
    m4_esc.speed(ESC_STOP);
    resetPID();
}

// =======================================================================
//  SECTION 17 -- CONTROL LOOP
// =======================================================================

static void runControlLoop()
{
    const float rollOut  = pidCompute(rcRoll,  gyroRoll,
                                       ROLL_KP,  ROLL_KI,  ROLL_KD,
                                       DT_S, I_LIMIT_ROLL,  PID_OUT_LIMIT,
                                       pidRoll);

    const float pitchOut = pidCompute(rcPitch, gyroPitch,
                                       PITCH_KP, PITCH_KI, PITCH_KD,
                                       DT_S, I_LIMIT_PITCH, PID_OUT_LIMIT,
                                       pidPitch);

    const float yawOut   = pidCompute(rcYaw,   gyroYaw,
                                       YAW_KP,   YAW_KI,   YAW_KD,
                                       DT_S, I_LIMIT_YAW,   PID_OUT_LIMIT,
                                       pidYaw);

    // Safety gate: disarm if throttle is below the arming threshold
    if (rcRaw[CH_THR - 1] < THROTTLE_MIN) {
        disarmMotors();
        return;
    }

    mixMotors(rcThrottle, rollOut, pitchOut, yawOut);
    writeMotors();
}

// =======================================================================
//  SECTION 18 -- SETUP
// =======================================================================

void setup()
{
    // USB-CDC on Nano R4 requires waiting for host enumeration
    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println(F("[FC] -- Rate Mode Flight Controller --"));
    Serial.println(F("[FC] Target: Arduino Nano R4  IMU: MPU6050  250 Hz"));

    // Wire1 is the Qwiic / secondary I2C port on the Nano R4
    Wire1.begin();
    Wire1.setClock(400000);  // 400 kHz fast mode

    // -- ESC arming sequence -------------------------------------------
    Serial.println(F("[FC] Arming ESCs..."));
    m1_esc.arm(); m1_esc.speed(ESC_MIN);
    m2_esc.arm(); m2_esc.speed(ESC_MIN);
    m3_esc.arm(); m3_esc.speed(ESC_MIN);
    m4_esc.arm(); m4_esc.speed(ESC_MIN);
    Serial.println(F("[FC] ESCs armed. Waiting for tones..."));
    delay(3500);

    // -- MPU6050 init --------------------------------------------------
    Serial.println(F("[FC] Initialising MPU6050..."));
    mpu.initialize();

    uint8_t retries = 0;
    while (!mpu.testConnection()) {
        if (++retries > 10) {
            Serial.println(F("[FC] FATAL: MPU6050 not found. Halting."));
            while (true) { delay(1000); }
        }
        Serial.println(F("[FC] MPU6050 not found -- check wiring. Retrying..."));
        delay(500);
    }
    Serial.println(F("[FC] MPU6050 connected."));

    // -- IMU calibration -----------------------------------------------
    Serial.println(F("[FC] Calibrating IMU -- keep craft flat and still..."));
    mpu.CalibrateAccel(20);
    mpu.CalibrateGyro(20);
    imu.initialize(&mpu, IMU_DT_S);
    Serial.println(F("[FC] IMU calibrated."));

    // -- PPM receiver --------------------------------------------------
    ppm.begin(PPM_INT_PIN, false);
    Serial.println(F("[FC] PPM receiver ready."));

    // -- Safety gate: throttle must be low before arming ---------------
    Serial.println(F("[FC] Waiting for throttle to be lowered..."));
    readReceiver();
    while (rcRaw[CH_THR - 1] >= THROTTLE_MIN) {
        readReceiver();
        delay(100);
    }
    Serial.println(F("[FC] Throttle low -- flight controller ACTIVE."));
    Serial.println(F("[FC] -- Verify GYRO_*_SIGN with DBG_IMU=true before flight --"));

    const uint32_t now = micros();
    prevControlUs = now;
    prevImuUs     = now;
    prevRcUs      = now;
}

// =======================================================================
//  SECTION 19 -- MAIN LOOP
// =======================================================================

/**
 * Three independent rate-limited tasks, each gated on elapsed us.
 *
 * Task priorities (highest -> lowest):
 *   IMU update      250 Hz -- stale gyro data destabilises control
 *   Control loop    250 Hz -- runs immediately after each IMU update
 *   RC read         100 Hz -- human inputs change slowly
 *
 * FIX: Original used millis() (1 ms resolution). micros() on the RA4M1
 * is hardware timer-backed and gives 1 us resolution at 48 MHz.
 */
void loop()
{
    const uint32_t now = micros();

    // IMU @ 250 Hz
    if ((now - prevImuUs) >= IMU_PERIOD_US) {
        updateImu();
        prevImuUs = now;
    }

    // Control loop @ 250 Hz
    if ((now - prevControlUs) >= CONTROL_PERIOD_US) {
        runControlLoop();
        prevControlUs = now;
    }

    // RC receiver @ 100 Hz
    if ((now - prevRcUs) >= RC_PERIOD_US) {
        readReceiver();
        prevRcUs = now;
    }
}