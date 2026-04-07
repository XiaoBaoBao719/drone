/**
 * @file PID_Gains_Summary.h
 * @brief Quick-reference copy-paste summary for 250g quadcopter with 5" propellers
 * 
 * COPY THIS SECTION INTO YOUR RateModeR4.ino (replacing old gain constants)
 */

#ifndef PID_GAINS_SUMMARY_H
#define PID_GAINS_SUMMARY_H

// ============================================================================
// READY-TO-USE PID GAINS FOR 250g QUADCOPTER WITH 5" PROPELLERS
// ============================================================================
// Platform: 250g, 5" props, 6" arms, 1000 Hz control loop
// Based on: Crazyflie 2.1 scaled for 250g class + conservative damping
// Status: Conservative starting point (proven on similar platforms)
// Expected behavior: Stable, moderate responsiveness, minimal oscillation

// ============================================================================
// RATE CONTROLLER - Inner Loop (Gyroscope Feedback)
// ============================================================================

/** Roll rate proportional gain (0.75-1.10 typical range) */
static constexpr float ROLL_KP = 0.90f;

/** Pitch rate proportional gain (should equal ROLL_KP) */
static constexpr float PITCH_KP = 0.90f;

/** Yaw rate proportional gain (typically 50-65% of roll) */
static constexpr float YAW_KP = 0.55f;

/** Roll rate integral gain (0.010-0.025 typical range) */
static constexpr float ROLL_KI = 0.015f;

/** Pitch rate integral gain (should equal ROLL_KI) */
static constexpr float PITCH_KI = 0.015f;

/** Yaw rate integral gain (typically 50-100% of roll) */
static constexpr float YAW_KI = 0.008f;

/** Roll rate derivative gain (0.040-0.080 typical range) */
static constexpr float ROLL_KD = 0.060f;

/** Pitch rate derivative gain (should equal ROLL_KD) */
static constexpr float PITCH_KD = 0.060f;

/** Yaw rate derivative gain (typically 20-40% of roll) */
static constexpr float YAW_KD = 0.002f;

/** Roll rate integral anti-windup limit */
static constexpr float ROLL_I_LIMIT = 100.0f;

/** Pitch rate integral anti-windup limit (should equal ROLL_I_LIMIT) */
static constexpr float PITCH_I_LIMIT = 100.0f;

/** Yaw rate integral anti-windup limit (typically 1.25-1.5x roll) */
static constexpr float YAW_I_LIMIT = 150.0f;

/** Rate controller PID output limit (motor command units) */
static constexpr float PID_LIMIT = 300.0f;

// ============================================================================
// ANGLE CONTROLLER - Outer Loop (Attitude Feedback)
// ============================================================================

/** Roll angle proportional gain (3.2-4.5 typical range) */
static constexpr float ANGLE_ROLL_KP = 3.8f;

/** Pitch angle proportional gain (should equal ANGLE_ROLL_KP) */
static constexpr float ANGLE_PITCH_KP = 3.8f;

/** Yaw angle proportional gain (typically 30-45% of roll) */
static constexpr float ANGLE_YAW_KP = 1.5f;

/** Roll angle integral gain (0.010-0.025 typical range) */
static constexpr float ANGLE_ROLL_KI = 0.015f;

/** Pitch angle integral gain (should equal ANGLE_ROLL_KI) */
static constexpr float ANGLE_PITCH_KI = 0.015f;

/** Yaw angle integral gain (typically 20-40% of roll) */
static constexpr float ANGLE_YAW_KI = 0.004f;

/** Roll angle derivative gain (0.08-0.15 typical range) */
static constexpr float ANGLE_ROLL_KD = 0.10f;

/** Pitch angle derivative gain (should equal ANGLE_ROLL_KD) */
static constexpr float ANGLE_PITCH_KD = 0.10f;

/** Yaw angle derivative gain (typically 30-50% of roll) */
static constexpr float ANGLE_YAW_KD = 0.05f;

/** Roll angle integral anti-windup limit */
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;

/** Pitch angle integral anti-windup limit (should equal ANGLE_ROLL_I_LIMIT) */
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;

/** Yaw angle integral anti-windup limit */
static constexpr float ANGLE_YAW_I_LIMIT = 30.0f;

/** Rate setpoint limit output from angle controller */
static constexpr float ANGLE_PID_LIMIT = 200.0f;

// ============================================================================
// TUNING CHEAT SHEET
// ============================================================================

/*
If quad oscillates (buzz/ringing):
  → Increase *_KD by 20-50% (add damping)
  → Decrease *_KP by 5-10% (reduce aggression)

If quad is too slow/sluggish:
  → Increase *_KP by 10-15% (add response)
  → Monitor for oscillation after adjustment

If quad drifts at constant input:
  → Increase *_KI by 20-50% (add integration)
  → Verify motor calibration (thrust balance)

If overshooting (goes past target, then corrects):
  → Increase *_KD by 10-20% (more damping)
  → Decrease *_KP by 5% if overdamped

If angle mode is sluggish:
  → Increase ANGLE_*_KP by 10-20%
  → First ensure rate mode is stable

For racing/acrobatics (max responsiveness):
  → Increase ROLL_KP to 1.05-1.15
  → Increase ANGLE_ROLL_KP to 4.2-4.8
  → Reduce ANGLE_PID_LIMIT to leave headroom for disturbances

For smooth flying/photography (conservative):
  → Decrease ANGLE_ROLL_KP to 3.0-3.3
  → Increase *_KD gains by 20-30%
  → Reduce ANGLE_PID_LIMIT to 150 deg/s
*/

#endif // PID_GAINS_SUMMARY_H
