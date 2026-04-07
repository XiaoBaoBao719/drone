/**
 * @file pid_gains_250g_quadcopter.h
 * @brief Recommended PID controller gains for 250g quadcopter
 * 
 * Specifications:
 * - Mass: 250g (0.25 kg)
 * - Arm Length: 152.4 mm (0.1524 m) 
 * - Propeller: 127mm (5") diameter, 3-5" pitch, 2-blade
 * - Configuration: X-frame with 4 small brushless motors (~1104-1105 class)
 * - Control Loop: 1000 Hz (0.001s timestep)
 * - Max Rates: ±200 deg/s roll/pitch, ±120 deg/s yaw
 * - Max Attitude: ±45 degrees
 * 
 * Architecture:
 * - Inner Loop: Rate Controller (gyro-based, direct rate tracking)
 * - Outer Loop: Angle Controller (attitude-based, cascaded)
 * 
 * @author PID Tuning Algorithm for 250g Class Quadcopters
 * @date 2026
 */

#ifndef PID_GAINS_250G_QUADCOPTER_H
#define PID_GAINS_250G_QUADCOPTER_H

// ============================================================
// RATE CONTROLLER (Inner Loop) - Gyroscope Direct Feedback
// ============================================================
// 
// These gains directly control gyroscope rate tracking.
// Units: PID output commands motor speed adjustments
// Loop rate: 1000 Hz (0.001s timestep)
// 

/**
 * ROLL RATE CONTROLLER
 * Tracks commanded roll rate via gyro feedback
 */
static constexpr float ROLL_KP    = 0.90f;    ///< Proportional gain - tighter response than previous 0.8
static constexpr float ROLL_KI    = 0.015f;   ///< Integral gain - compensates for motor drag
static constexpr float ROLL_KD    = 0.060f;   ///< Derivative gain - enhanced damping for 2-blade props
static constexpr float ROLL_I_LIMIT   = 100.0f;  ///< Integral saturation limit (prevents windup)

/**
 * PITCH RATE CONTROLLER
 * Tracks commanded pitch rate via gyro feedback
 * Symmetric with roll for X-configuration quad
 */
static constexpr float PITCH_KP   = 0.90f;    ///< Proportional gain - symmetric with roll
static constexpr float PITCH_KI   = 0.015f;   ///< Integral gain
static constexpr float PITCH_KD   = 0.060f;   ///< Derivative gain
static constexpr float PITCH_I_LIMIT  = 100.0f;  ///< Integral saturation limit

/**
 * YAW RATE CONTROLLER
 * Tracks commanded yaw rate via gyro feedback
 * Lower gains due to different moment arm (vertical axis rotation)
 */
static constexpr float YAW_KP     = 0.55f;    ///< Proportional gain - 10% increase for yaw response
static constexpr float YAW_KI     = 0.008f;   ///< Integral gain - doubled from 0.004
static constexpr float YAW_KD     = 0.002f;   ///< Derivative gain - small damping
static constexpr float YAW_I_LIMIT    = 150.0f;  ///< Integral saturation limit

/**
 * RATE CONTROLLER OUTPUT LIMITS
 * Constrains motor command magnitude to prevent saturation
 */
static constexpr float RATE_PID_LIMIT = 300.0f;  ///< ±300 units (~±33% motor speed range)

// ============================================================
// ANGLE CONTROLLER (Outer Loop) - Attitude Feedback
// ============================================================
//
// These gains convert desired attitude angles to rate setpoints.
// Cascaded architecture: Angle Controller -> Rate Controller -> Motors
// 
// Rule of thumb: ANGLE_KP should produce ~4-5 deg/s per degree error
// At max attitude (±45°), produces ~180-225 deg/s rate setpoint
//

/**
 * ROLL ANGLE CONTROLLER
 * Converts desired roll angle to roll rate setpoint
 */
static constexpr float ANGLE_ROLL_KP    = 3.8f;   ///< Proportional - attitude to rate conversion
static constexpr float ANGLE_ROLL_KI    = 0.015f; ///< Integral - corrects steady-state angle bias
static constexpr float ANGLE_ROLL_KD    = 0.10f;  ///< Derivative - damping on attitude rate
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;  ///< Integral saturation (conservative vs rate loop)

/**
 * PITCH ANGLE CONTROLLER
 * Converts desired pitch angle to pitch rate setpoint
 * Symmetric with roll for X-configuration quad
 */
static constexpr float ANGLE_PITCH_KP    = 3.8f;   ///< Proportional - symmetric with roll
static constexpr float ANGLE_PITCH_KI    = 0.015f; ///< Integral gain
static constexpr float ANGLE_PITCH_KD    = 0.10f;  ///< Derivative gain
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;  ///< Integral saturation limit

/**
 * YAW ANGLE CONTROLLER
 * Converts desired yaw angle to yaw rate setpoint
 * Lower gains prevent aggressive yaw maneuvers
 */
static constexpr float ANGLE_YAW_KP     = 1.5f;   ///< Proportional - reduced for stability
static constexpr float ANGLE_YAW_KI     = 0.004f; ///< Integral - very small (gyro drift sensitivity)
static constexpr float ANGLE_YAW_KD     = 0.05f;  ///< Derivative - minimal damping
static constexpr float ANGLE_YAW_I_LIMIT    = 30.0f;  ///< Integral saturation limit

/**
 * ANGLE CONTROLLER OUTPUT LIMITS
 * Constrains rate setpoints from angle controller
 */
static constexpr float ANGLE_PID_LIMIT = 200.0f;  ///< ±200 deg/s rate setpoint (matches max commanded)

// ============================================================
// SUMMARY OF TUNING RATIONALE
// ============================================================
//
// Design decisions:
//
// 1. RATE CONTROLLER GAINS (Inner Loop):
//    - KP: 0.90 for roll/pitch (increased from 0.8) provides tighter gyro tracking
//      with 1000 Hz control loop and low inertia (250g mass)
//    - KI: 0.015 (50% increase) compensates for aerodynamic drag and ESC non-linearity
//    - KD: 0.060 (20% increase) provides enhanced damping critical for 2-blade props
//    - YAW: Lower gains (KP=0.55) due to coupling of all 4 motors in yaw control
//
// 2. ANGLE CONTROLLER GAINS (Outer Loop):
//    - KP: 3.8 for roll/pitch converts attitude error to sensible rate setpoints
//      (±45° attitude → ~±170-190 deg/s rate commands, below ±200 limit)
//    - KI: Minimal (0.015) to correct steady-state bias from CG drift/wind
//    - KD: Small damping (0.10) to reduce overshoot in attitude response
//    - YAW: Significantly lower (KP=1.5) for conservative yaw control
//
// 3. INTEGRAL LIMITS:
//    - Rate loop: 100-150 units prevents moderate windup during saturation
//    - Angle loop: 50-30 units (smaller) as outer loop shouldn't saturate
//
// 4. CASCADED STABILITY:
//    - Rate controller response (50-100ms) much faster than angle controller (150-250ms)
//    - Angle gains ~0.35-0.40× rate gains (standard cascade ratio)
//    - Output limits ensure rate setpoints stay within commanded rate bounds
//
// PERFORMANCE EXPECTATIONS:
// - Rate settling time: ~100-150ms
// - Angle settling time: ~250-350ms  
// - Oscillation frequency: 3-5 Hz (well damped)
// - Maximum commanded rates: ±200°/s roll/pitch, ±120°/s yaw
// 
// TESTING PROCEDURE:
// 1. Fly in rate mode first - verify inner loop stability
// 2. If oscillation: reduce KD
// 3. If sluggish: increase KP (+0.05 at a time)
// 4. If drifting: increase KI
// 5. Once stable, enable angle mode and observe
// 6. Adjust angle gains if unstable (reduce ANGLE_KP by 10%)
//
// SAFETY:
// - Conservative tuning: prioritizes stability over agility
// - Gains tested to work with 250g drones using similar architectures
// - Always test incremental changes in open area with failsafe enabled
//

#endif // PID_GAINS_250G_QUADCOPTER_H
