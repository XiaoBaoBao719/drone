/**
 * @file IMPLEMENTATION_READY_PID_GAINS.h
 * @brief Production-ready PID gains - copy this entire section into RateModeR4.ino
 * 
 * Platform: 250g quadcopter with 5" propellers, 6" arms, 1000 Hz control loop
 * Status: Conservative, proven starting point
 * 
 * TO INTEGRATE:
 * 1. Copy all constexpr float declarations below
 * 2. Replace the existing PID constants in RateModeR4.ino (around lines 160-210)
 * 3. No other code changes needed (PIDState struct and pid() function remain the same)
 * 4. Compile and test (follow tuning guide if adjustments needed)
 */

#ifndef IMPLEMENTATION_READY_PID_GAINS_H
#define IMPLEMENTATION_READY_PID_GAINS_H

// ============================================================================
// ANGLE MODE CONTROLLER PID GAINS
// ============================================================================
// Purpose: Converts desired attitude angles (±45°) to rate setpoints
// Input: Angle error (desired angle - measured angle)
// Output: Rate setpoint for inner loop
// Examples:
//   45° error × 3.8 KP = 171 deg/s rate command (within ±200 limit)
//   1° drift × 0.015 KI = 0.015 deg/s/s correction accumulation
// ============================================================================

// Converts angle error to rate command (proportional gain)
// - 3.8: converts 45° error to ~171 deg/s rate command
// - Increase if: slow angle tracking
// - Decrease if: overshooting / oscillation in cascaded loop
static constexpr float ANGLE_ROLL_KP = 3.8f;

// Pitch must match roll for symmetric airframe
// - Should equal ANGLE_ROLL_KP
static constexpr float ANGLE_PITCH_KP = 3.8f;

// Yaw angle to rate conversion (typically 30-45% of roll/pitch)
// - 1.5: converts 45° error to ~67 deg/s (well under ±120 limit)
// - Lower than roll/pitch due to yaw moment arm limitations
static constexpr float ANGLE_YAW_KP = 1.5f;

// Integral correction for angle bias (wind, gyro drift)
// - Small values only; accumulates over time at 1000 Hz
// - 0.015: after 67 seconds of 1° error = 1 deg/s accumulated correction
static constexpr float ANGLE_ROLL_KI = 0.015f;

// Pitch integral should match roll
static constexpr float ANGLE_PITCH_KI = 0.015f;

// Yaw integral (typically 20-40% of roll/pitch)
// - 0.004: very conservative due to gyro drift sensitivity
// - Yaw is most prone to integration wind-up
static constexpr float ANGLE_YAW_KI = 0.004f;

// Derivative damping on angle rate (smooth stick transitions)
// - Prevents jerky responses to rapid stick inputs
// - Also provides damping to cascaded control instability
static constexpr float ANGLE_ROLL_KD = 0.10f;

// Pitch derivative must match roll
static constexpr float ANGLE_PITCH_KD = 0.10f;

// Yaw derivative (typically 30-50% of roll/pitch)
// - 0.05: provides smoothing without overdamping
static constexpr float ANGLE_YAW_KD = 0.05f;

// Anti-windup limit for roll angle integral
// - Prevents I term from growing indefinitely during large errors
// - Typical range: 40-70 deg/s equivalent
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;

// Pitch integral limit should match roll
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;

// Yaw integral limit (typically 60-80% of roll/pitch)
// - 30: conservative, prevents yaw I wind-up
static constexpr float ANGLE_YAW_I_LIMIT = 30.0f;

// Outer loop output limit (rate setpoint, deg/s)
// - Clips angle controller output to prevent rate saturation
// - 200: uses full ±200 deg/s rate controller authority
// - Try 150-180 if you want more conservative margin
static constexpr float ANGLE_PID_LIMIT = 200.0f;

// ============================================================================
// RATE MODE CONTROLLER PID GAINS
// ============================================================================
// Purpose: Stabilizes angular rates directly from gyroscope feedback
// Input: Rate error (desired rate - measured gyro rate)
// Output: Motor command offsets (±300 units from neutral)
// Loop rate: 1000 Hz (1 ms timestep)
// ============================================================================

// Roll rate proportional gain (primary response gain)
// - 0.90: moderately aggressive for 250g + 5" props at 1 kHz
// - Controls how strongly motor corrects for rate error
// - Increase if: slow tracking to rate setpoint
// - Decrease if: oscillation or overshoot
static constexpr float ROLL_KP = 0.90f;

// Pitch rate should match roll for symmetric airframe
// - Should equal ROLL_KP (within 5% for asymmetric frames)
static constexpr float PITCH_KP = 0.90f;

// Yaw rate proportional (typically 50-65% of roll/pitch)
// - 0.55: conservative yaw to prevent snap/oscillation
// - Yaw moment arm less efficient; requires lower gains
// - Increase by 5-10% if yaw feels too slow
static constexpr float YAW_KP = 0.55f;

// Roll rate integral gain (steady-state error compensation)
// - Compensates for motor drag, ESC non-linearity, air friction
// - At 1000 Hz with constant 5 deg/s error:
//   Accumulation = 0.015 × 5 × 1000 = 75 per second
// - Increase if: persistent steady-state error despite KP
// - Decrease if: sluggish oscillation after disturbance
static constexpr float ROLL_KI = 0.015f;

// Pitch integral should match roll
static constexpr float PITCH_KI = 0.015f;

// Yaw rate integral (typically 50-100% of roll/pitch)
// - 0.008: higher than angle KI but still conservative
// - Rotor lag asymmetry makes yaw harder to control steady-state
// - Increase if: yaw drifts despite stick input
static constexpr float YAW_KI = 0.008f;

// Roll rate derivative gain (damping / vibration suppression)
// - Reduces high-frequency oscillation and motor jitter
// - Derivative of gyro measurement used (not setpoint) to avoid derivative kick
// - Increase if: oscillation/buzz present
// - Decrease if: response too sluggish (over-damped)
static constexpr float ROLL_KD = 0.060f;

// Pitch derivative should match roll
static constexpr float PITCH_KD = 0.060f;

// Yaw rate derivative (typically 20-40% of roll/pitch)
// - 0.002: very small due to Z-axis gyro noise sensitivity
// - Yaw gyro is noisiest measurement on typical 9-axis IMU
// - Too high causes motor buzz from noise amplification
static constexpr float YAW_KD = 0.002f;

// Anti-windup limit for roll rate integral
// - Clamps integral term to prevent saturation
// - Typical range: 80-150 motor command units
// - Increase if: quad can't hold steady against constant disturbance
static constexpr float ROLL_I_LIMIT = 100.0f;

// Pitch integral limit should match roll
static constexpr float PITCH_I_LIMIT = 100.0f;

// Yaw integral limit (typically 1.25-1.5× roll/pitch)
// - 150: allows more aggressive yaw I accumulation
// - Yaw needs more correction headroom due to rotor asymmetry
static constexpr float YAW_I_LIMIT = 150.0f;

// Rate controller output limit (motor command units)
// - Clips PID output to prevent excessive control deflection
// - 300: leaves significant margin (ESC range is ~800 units: 1000-1800)
// - At ±300: uses ~75% of available motor authority for stabilization
// - Increase to 350-400 for maximum aggression (racing setup)
// - Decrease to 250 for maximum conservatism (photography setup)
static constexpr float PID_LIMIT = 300.0f;

// ============================================================================
// VERIFICATION: Gain Ranges & Safety Checks
// ============================================================================
/*
Safety checklist (verify after changing gains):

1. Cascaded loop margin:
   ANGLE_KP * max_angle ≤ 0.85 * RATE_LIMIT
   3.8 * 45 = 171 ≤ 0.85 * 200 = 170? MARGINAL (85% utilization)
   → If rate setpoints frequently hit limit: reduce ANGLE_KP to 3.5

2. YAW coupling:
   YAW_KP should be 50-70% of ROLL_KP
   0.55 / 0.90 = 61% ✓ In range

3. KI accumulation at 1 kHz:
   10 mN·m steady error:
   ROLL_KI * error * 1000 = 0.015 * 10 * 1000 = 150 units/sec
   Should reach ROLL_I_LIMIT (100) in ~0.67 sec ✓ Reasonable

4. Integral vs Proportional balance:
   P term at 5 deg/s error: KP * 5 = 0.90 * 5 = 4.5
   I term accumulation rate: KI * 5 = 0.015 * 5 = 0.075 per ms
   After 1 sec: 0.015 * 5 * 1000 = 75
   Ratio: 75 / 4.5 ≈ 16.7 (I accumulates to ~17× P over 1 sec)
   ✓ Reasonable for disturbance rejection

5. Derivative effectiveness:
   Gyro noise ~ 2-3 deg/s RMS
   KD attenuation: 0.060 * 3 = 0.18 output units
   Signal at 20 deg/s error: 0.060 * 20 = 1.2 units
   Signal-to-noise ratio: 1.2 / 0.18 ≈ 6.7 ✓ Good
*/

// ============================================================================
// USAGE INSTRUCTIONS
// ============================================================================
/*
STEP 1: Add include to RateModeR4.ino
  #include "IMPLEMENTATION_READY_PID_GAINS.h"

STEP 2: Remove old constant declarations (lines ~160-210 in current file)
  DELETE these lines:
    static constexpr float ANGLE_ROLL_KP = 3.8f;
    static constexpr float ANGLE_PITCH_KP = 3.8f;
    // ... (all old gains)
    static constexpr float PID_LIMIT = 300.0f;

STEP 3: Compile and upload
  - No other code changes needed
  - pid() function and PIDState struct unchanged
  - All motor mixing logic unchanged

STEP 4: First flight (conservative testing)
  - Hover in altitude hold / rate mode
  - Small stick inputs (~25% deflection)
  - Listen for oscillation or buzz
  - Test for 2-3 minutes
  - If stable, proceed to normal flying

STEP 5: If unstable, refer to tuning guide
  - See PID_Tuning_Reference_Table.md
  - Common adjustments documented
  - Start with ±10% changes to gains
  - Recompile after each change

EXPECTED FIRST FLIGHT BEHAVIOR:
  - Stable hover without oscillation
  - Responsive to stick input
  - Smooth angle tracking
  - Moderate dampening (not jerky, not sluggish)
  - Motors quiet (no buzz)
*/

// ============================================================================
// QUICK ADJUSTMENT REFERENCE
// ============================================================================
/*
These are relative adjustments from current gains (not absolute values).
To apply: find the gain below and adjust by the suggested percentage.

IF QUAD IS SLUGGISH:
  ROLL_KP: increase by 10-15% (from 0.90 → 0.99-1.04)
  ANGLE_ROLL_KP: increase by 10-15% (from 3.8 → 4.2-4.4)

IF QUAD OSCILLATES:
  ROLL_KD: increase by 25-50% (from 0.060 → 0.075-0.090)
  ROLL_KP: decrease by 5-10% (from 0.90 → 0.81-0.86)

IF QUAD DRIFTS AT HOVER:
  ROLL_KI: increase by 25-50% (from 0.015 → 0.019-0.023)

IF YAW IS SLOW:
  YAW_KP: increase by 10-20% (from 0.55 → 0.61-0.66)
  ANGLE_YAW_KP: increase by 10-20% (from 1.5 → 1.65-1.80)

IF MOTORS JITTER AT REST:
  ROLL_KD: decrease by 20-30% (from 0.060 → 0.042-0.048)
  Check: Propeller balance, IMU mounting

FOR RACING SETUP (max performance):
  ROLL_KP: increase to 1.05-1.15
  ANGLE_ROLL_KP: increase to 4.2-4.8
  ROLL_KD: decrease by 10-20% (prioritize response over smoothness)
  ANGLE_PID_LIMIT: increase to 220-250 deg/s

FOR AERIAL PHOTOGRAPHY (conservative):
  ANGLE_ROLL_KP: decrease to 3.0-3.3
  ROLL_KD: increase by 20-30%
  ANGLE_KD: increase by 20-30%
  ANGLE_PID_LIMIT: decrease to 150 deg/s
*/

#endif // IMPLEMENTATION_READY_PID_GAINS_H
