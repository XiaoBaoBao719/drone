/**
 * @file PID_Gains_Integration_Guide.cpp
 * @brief Integration instructions for 250g quadcopter PID gains
 * 
 * This file shows how to incorporate the new PID gains into RateModeR4.ino
 * and provides a summary table of all gains with their meanings.
 */

// ============================================================================
// STEP 1: Include the new gains header in RateModeR4.ino
// ============================================================================

// Add this line to the top of RateModeR4.ino with other includes:
// #include "PID_Gains_250g_5inch.h"

// Then replace the old constant declarations with the new ones.
// The following section shows what the constants should look like:

// ============================================================================
// STEP 2: Updated constants section for RateModeR4.ino
// ============================================================================
/*

// Angle Mode Controller PID Gains
static constexpr float ANGLE_ROLL_KP   = 3.8f;   // Converts ±45° angle error to rate command
static constexpr float ANGLE_PITCH_KP  = 3.8f;   // Symmetric with roll
static constexpr float ANGLE_YAW_KP    = 1.5f;   // Lower for conservative yaw control

static constexpr float ANGLE_ROLL_KI   = 0.015f; // Corrects steady-state angle bias (drift)
static constexpr float ANGLE_PITCH_KI  = 0.015f;
static constexpr float ANGLE_YAW_KI    = 0.004f; // Very small (gyro drift sensitivity)

static constexpr float ANGLE_ROLL_KD   = 0.10f;  // Damping on attitude rate
static constexpr float ANGLE_PITCH_KD  = 0.10f;
static constexpr float ANGLE_YAW_KD    = 0.05f;  // Minimal damping for yaw

static constexpr float ANGLE_ROLL_I_LIMIT   = 50.0f;   // Conservative vs rate loop
static constexpr float ANGLE_PITCH_I_LIMIT  = 50.0f;
static constexpr float ANGLE_YAW_I_LIMIT    = 30.0f;

static constexpr float ANGLE_RATE_SETPOINT_LIMIT = 200.0f; // Rate setpoint limit from angle controller

// Rate Mode Controller PID Gains
static constexpr float ROLL_KP    = 0.90f;   // Primary response gain for roll stabilization
static constexpr float PITCH_KP   = 0.90f;   // Symmetric with roll
static constexpr float YAW_KP     = 0.55f;   // Lower to prevent yaw oscillation

static constexpr float ROLL_KI    = 0.015f;  // Compensates motor drag & ESC non-linearity
static constexpr float PITCH_KI   = 0.015f;
static constexpr float YAW_KI     = 0.008f;  // Handles rotor asymmetry

static constexpr float ROLL_KD    = 0.060f;  // Damping for smooth response
static constexpr float PITCH_KD   = 0.060f;
static constexpr float YAW_KD     = 0.002f;  // Minimal damping for yaw

static constexpr float ROLL_I_LIMIT   = 100.0f;  // Integral saturation
static constexpr float PITCH_I_LIMIT  = 100.0f;
static constexpr float YAW_I_LIMIT    = 150.0f;  // Higher for yaw authority

static constexpr float RATE_PID_LIMIT = 300.0f;  // Motor command saturation (±300 units)

*/

// ============================================================================
// STEP 3: Quick Reference - Gain Meanings & Tuning Direction
// ============================================================================

/*
┌──────────────────────────────────────────────────────────────────────────────┐
│ RATE CONTROLLER GAINS (Inner Loop - Gyroscope Feedback)                     │
├──────────────┬──────────┬─────────────────────────────────────────────────────┤
│ Gain         │ Current  │ Meaning / Tuning Direction                         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ROLL_KP      │ 0.90     │ Roll response stiffness                            │
│              │          │ ↑ Faster response (risk: oscillation)             │
│              │          │ ↓ Smoother response (risk: lag)                   │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ROLL_KI      │ 0.015    │ Roll steady-state error correction                 │
│              │          │ ↑ Corrects drift faster (risk: slow oscillation)  │
│              │          │ ↓ Avoids wind-up (risk: persistent error)         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ROLL_KD      │ 0.060    │ Roll damping / vibration suppression               │
│              │          │ ↑ Smoother, less jittery (risk: sluggish)         │
│              │          │ ↓ Crisper response (risk: noise amplification)    │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ PITCH_*      │ Same as  │ Pitch follows roll due to airframe symmetry        │
│              │ ROLL_*   │ (small differences are OK for asymmetric airframes)│
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ YAW_KP       │ 0.55     │ Yaw response (typically 50-65% of roll/pitch)      │
│              │          │ ↑ Snappier yaw (risk: oscillation)                │
│              │          │ ↓ Softer yaw (risk: lag in yaw control)           │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ YAW_KI       │ 0.008    │ Yaw steady-state (less than roll due to rotor lag) │
│              │          │ ↑ Better steady-state (risk: wind-up)             │
│              │          │ ↓ Avoid asymmetric rotor effects (risk: drift)    │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ YAW_KD       │ 0.002    │ Yaw damping (very small; noise-prone)              │
│              │          │ ↑ Smooth yaw (risk: sluggish)                     │
│              │          │ ↓ Crisp response (risk: ringing)                  │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ROLL_I_LIMIT │ 100      │ Max integral wind-up for roll                      │
│ PITCH_I_LIMIT│ 100      │ (prevents integral from saturating)                │
│ YAW_I_LIMIT  │ 150      │ Yaw typically needs more integral room             │
│              │          │ ↑ More aggressive integral correction              │
│              │          │ ↓ More conservative (less overshoot risk)         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ RATE_PID_LIM │ 300      │ Motor command limit from rate controller           │
│              │          │ ↑ Uses more of motor range (aggressive)           │
│              │          │ ↓ Leaves more margin (conservative)               │
└──────────────┴──────────┴─────────────────────────────────────────────────────┘


┌──────────────────────────────────────────────────────────────────────────────┐
│ ANGLE CONTROLLER GAINS (Outer Loop - Attitude Feedback)                      │
├──────────────┬──────────┬─────────────────────────────────────────────────────┤
│ Gain         │ Current  │ Meaning / Tuning Direction                         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_ROLL_KP   │ 3.8   │ Angle error to rate command conversion             │
│                 │       │ Example: 3.8 * 45° error = 171 deg/s rate cmd    │
│                 │       │ ↑ Faster angle tracking (risk: overshoot)         │
│                 │       │ ↓ Smoother angles (risk: sluggish)                │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_PITCH_KP  │ 3.8   │ Same as roll (symmetric airframes)                 │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_YAW_KP    │ 1.5   │ Yaw angle to rate conversion (typically 40% of P) │
│                 │       │ Example: 1.5 * 45° = 67.5 deg/s (well under 120)│
│                 │       │ ↑ Faster yaw heading tracking                     │
│                 │       │ ↓ More deliberate yaw movement                    │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_ROLL_KI   │ 0.015 │ Angle steady-state correction (slow)               │
│ ANGLE_PITCH_KI  │ 0.015 │ ↑ Corrects drift faster                           │
│                 │       │ ↓ Avoids wind-up (typically 25-50% of ROLL_KI)    │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_YAW_KI    │ 0.004 │ Yaw steady-state (small; gyro drift sensitive)    │
│                 │       │ ↑ Better hold (risk: wind-up)                     │
│                 │       │ ↓ Minimal correction (risk: yaw drift)            │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_ROLL_KD   │ 0.10  │ Angle rate damping (smooth stick transitions)      │
│ ANGLE_PITCH_KD  │ 0.10  │ ↑ Smoother angles (less harsh response)           │
│                 │       │ ↓ More direct (faster stick feel)                 │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_YAW_KD    │ 0.05  │ Yaw rate damping (typically 30-50% of P/R)        │
│                 │       │ ↑ Smooth yaw (risk: sluggish)                     │
│                 │       │ ↓ Crisp yaw (risk: jerky)                         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_*_I_LIM   │ 50/50 │ Integral saturation (typically <rate_limit / KP)  │
│ ANGLE_YAW_I_LIM │ 30    │ Prevents I from dominating; guards against wind   │
│                 │       │ ↑ Allows more aggressive integration              │
│                 │       │ ↓ Conservative (leaves margin for P term)         │
├──────────────┼──────────┼─────────────────────────────────────────────────────┤
│ ANGLE_RATE_SETPOINT_LIMIT │ 200 │ Max rate demand from angle controller    │
│                           │     │ Should not exceed rate loop limits       │
│                           │     │ ↑ More aggressive angle tracking         │
│                           │     │ ↓ Conservative (leaves margin)           │
└──────────────┴──────────┴─────────────────────────────────────────────────────┘
*/

// ============================================================================
// STEP 4: Testing Procedure
// ============================================================================

/*
PREFLIGHT CHECKLIST:
  ☐ Motor mapping verified (M1-M4 in correct positions)
  ☐ ESC calibration complete (min/max PWM ranges set)
  ☐ Propellers balanced
  ☐ IMU gyro calibration performed (zero rate at rest)
  ☐ Airframe is level and symmetric
  ☐ Batteries fully charged

INDOOR FLIGHT TEST (Rate Mode first):
  1. Arm on level ground with no throttle
  2. Slowly increase throttle to hover (neutral stick)
  3. Small roll input (~10 deg/s demand):
     - Should respond smoothly within ~100ms
     - Should stop immediately on stick neutral
     - No overshoot or oscillation
  4. Verify no motor oscillation/buzz at rest
  5. Test max rate (full deflection):
     - Should reach ±200 deg/s smoothly
     - No chattering or jitter

OUTDOOR FLIGHT TEST (if Rate Mode stable):
  1. Switch to Angle Mode
  2. Small angle inputs (~10° demand):
     - Should hold angle when stick centered
     - Should track stick smoothly
  3. Rapid angle changes (max stick):
     - Should reach 45° with <5% overshoot
     - Should settle within 1-2 seconds
  4. Disturbance test:
     - Hand push while holding angle
     - Should recover automatically
  5. GPS hold / position hold (if available):
     - Should maintain position with slight drift correction

TUNING ITERATION:
  If oscillation appears:
    1. Note the frequency (look at motors/props)
    2. Increase KD by 20-30%
    3. Reduce KP by 5-10%
    4. Retest
    5. If persists, check IMU sample rate and noise
  
  If lag appears:
    1. Increase KP by 10-15%
    2. Retest
    3. Watch for oscillation after pushing limits
  
  If steady-state error (e.g., always slightly tilted):
    1. Increase KI by 20-30%
    2. Retest
    3. Watch for slow oscillation
*/

// ============================================================================
// STEP 5: Performance Monitoring
// ============================================================================

/*
Key metrics to log during testing:

Serial output should include (example format):
  "Time(ms), Roll(°), Pitch(°), Yaw(°), RollRate(°/s), PitchRate(°/s), YawRate(°/s), 
   RollCmd, PitchCmd, YawCmd, M1(PWM), M2(PWM), M3(PWM), M4(PWM)"

Healthy behavior looks like:
  - Angles: Smooth curve, settle within 1-2s
  - Rates: Peak undershoot <10%, no ringing
  - Motors: Smooth PWM ramps, no sudden jumps
  - Latency: <10ms from stick input to motor response

Problem indicators:
  - Rates oscillating at fixed frequency → tune KD
  - Motors chattering (100+ Hz ripple) → reduce KD, check IMU noise
  - Angles unstable (continuously drifting) → rate loop issue
  - Asymmetric roll vs pitch response → check motor mapping or balance
  - Yaw lag compared to pitch/roll → increase ANGLE_YAW_KP
*/

// ============================================================================
// STEP 6: Platform-Specific Notes for 250g + 5" Props
// ============================================================================

/*
Why these gains for YOUR platform:

MASS (250g):
  - Heavier than FPV racing drones (~100-150g)
  - Lighter than traditional 500g+ drones
  - Requires moderate damping (too much → sluggish, too little → oscillation)
  - Result: KP at 0.90 is aggressive but reasonable

PROPELLERS (5 inch):
  - Moderate blade inertia
  - Good power-to-weight ratio
  - Responsive to throttle changes
  - Result: Support tight feedback gains without excessive vibration

ARM LENGTH (6 inches / ~152mm):
  - Standard for this class
  - Moment arm sufficient for moment generation
  - Not extreme (no special scaling needed)
  - Result: Standard scaling from Crazyflie (27g) applies with modest adjustment

CONTROL LOOP (1000 Hz):
  - Very fast updates (1ms timestep)
  - Allows aggressive feedback gains
  - KI values accumulate quickly (∑ = 0.015 * 1000 = 15 per second per degree)
  - Result: Smaller KI values OK; responsive to tuning

RATE LIMITS (±200°/s roll/pitch, ±120°/s yaw):
  - Roll/pitch: Leaves ~30-50% safety margin below mechanical limits
  - Yaw: More conservative due to moment arm asymmetry
  - Cascaded control prevents saturation under normal conditions
  - Result: Good authority for acrobatics without being reckless

DESIGN PHILOSOPHY:
  - Conservative tuning prioritizes stability over aggressiveness
  - Easier to add responsiveness (↑ KP) than remove oscillation (↓ KD + KP)
  - 5" props have low vibration baseline; can push gains higher
  - 250g mass provides good control authority per pound
  - Result: Starting gains are near "tuned" state; minor adjustments expected
*/

// ============================================================================
// REFERENCE IMPLEMENTATIONS (Other Platforms)
// ============================================================================

/*
If you later build frames with different specs, here's how to scale:

CRAZYFLIE 2.1 (Reference, 27g @ 500 Hz):
  Rate:  KP=1.7, KI=0.04, KD=0.07 (in native units)
  Angle: KP=3.0, KI=1.0, KD=0.0

100g Racing Frame (450 Hz):
  Rate:  KP=0.75, KI=0.012, KD=0.050
  Angle: KP=3.5, KI=0.015, KD=0.08

250g Standard (1000 Hz):
  Rate:  KP=0.90, KI=0.015, KD=0.060  ← YOUR FRAME
  Angle: KP=3.8, KI=0.015, KD=0.10

500g Photography Drone (500 Hz):
  Rate:  KP=0.55, KI=0.010, KD=0.040
  Angle: KP=2.5, KI=0.010, KD=0.05

1000g Heavy Lift (200 Hz):
  Rate:  KP=0.30, KI=0.005, KD=0.020
  Angle: KP=1.5, KI=0.005, KD=0.02

Scaling factors:
  - KP scales roughly as: sqrt(mass_ref / mass_new)
  - KI scales as: loop_freq_new / loop_freq_ref
  - KD scales as: loop_freq_new / loop_freq_ref
  - Always verify via testing; no formula is perfect
*/

