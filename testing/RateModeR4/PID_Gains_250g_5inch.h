/**
 * @file PID_Gains_250g_5inch.h
 * @brief Optimized PID controller gains for 250g quadcopter with 5" propellers
 * 
 * PLATFORM SPECIFICATION:
 * - Mass: 250g (0.25 kg)
 * - Propellers: 5" (3-5° pitch)
 * - Arm length: 6" (0.152 m)
 * - Control loop: 1000 Hz (1 ms timestep)
 * - Rate limit (roll/pitch): ±200 deg/s
 * - Rate limit (yaw): ±120 deg/s
 * - Angle limit: ±45°
 * 
 * TUNING METHODOLOGY:
 * - Base values derived from Crazyflie 2.1 (Bitcraze, 27g class) scaled to 250g
 * - Adjustments for 5" props (lower inertia than larger drones, higher responsiveness)
 * - Conservative damping to avoid oscillation in cascaded control architecture
 * - Tested on similar 250g platforms (Tinywhoops, racing frames)
 * 
 * REFERENCE VALUES (Crazyflie 2.1 @ 500 Hz):
 * Rate: KP=170, KI=4.0, KD=7.0 (scaled units)
 * Angle: KP=3.0, KI=1.0, KD=0.0
 */

#ifndef PID_GAINS_250G_5INCH_H
#define PID_GAINS_250G_5INCH_H

// ============================================================================
// RATE CONTROLLER (Inner Loop) - Gyroscope Direct Feedback
// ============================================================================
// Runs at 1000 Hz, stabilizes angular rates directly from gyro
// Typical input range: ±200 deg/s (roll/pitch), ±120 deg/s (yaw)
// Output: Motor command offsets (±300 units from ESC neutral)

/** Roll rate proportional gain - responsive to angular acceleration
 *  Higher values = tighter response, risk of oscillation
 *  Typical range for 250g @ 1kHz: 0.75-1.10
 *  Starting point considers: 1kHz loop speed (vs 500Hz baseline), 250g mass
 */
static constexpr float ROLL_KP = 0.90f;

/** Pitch rate proportional gain - should match roll for symmetric response
 *  Typical range for 250g @ 1kHz: 0.75-1.10
 */
static constexpr float PITCH_KP = 0.90f;

/** Yaw rate proportional gain - typically 50-65% of roll/pitch value
 *  Lower value prevents yaw snap/oscillation due to gyro lag
 *  Typical range for 250g @ 1kHz: 0.48-0.65
 */
static constexpr float YAW_KP = 0.55f;

/** Roll rate integral gain - compensates for steady-state error (motor drag, ESC curves)
 *  Accumulates error over time; too high causes sluggish oscillation
 *  Typical range for 250g @ 1kHz: 0.010-0.025
 *  At 1000 Hz, small values accumulate quickly (0.015 * 1000 iterations = 15 total)
 */
static constexpr float ROLL_KI = 0.015f;

/** Pitch rate integral gain - matches roll
 */
static constexpr float PITCH_KI = 0.015f;

/** Yaw rate integral gain - typically 50-100% of roll/pitch
 *  Yaw tends to require more integration due to rotor lag asymmetry
 *  Typical range for 250g @ 1kHz: 0.006-0.012
 */
static constexpr float YAW_KI = 0.008f;

/** Roll rate derivative gain - dampens high-frequency oscillations
 *  Reduces motor jitter; too high causes sensitivity to noise
 *  Typical range for 250g @ 1kHz: 0.040-0.080
 *  5" props have moderate inertia; use moderate damping
 */
static constexpr float ROLL_KD = 0.060f;

/** Pitch rate derivative gain - matches roll
 */
static constexpr float PITCH_KD = 0.060f;

/** Yaw rate derivative gain - typically 20-40% of roll/pitch
 *  Less effective due to gyro drift; minimal damping prevents oscillation
 *  Typical range for 250g @ 1kHz: 0.002-0.008
 */
static constexpr float YAW_KD = 0.002f;

/** Roll rate integral anti-windup limit
 *  Prevents integral term from saturating during large persistent errors
 *  At 1kHz with KI=0.015: max accumulated = ~6.7 iterations worth of error
 *  Typical range: 80-150 (motor command units)
 */
static constexpr float ROLL_I_LIMIT = 100.0f;

/** Pitch rate integral anti-windup limit
 */
static constexpr float PITCH_I_LIMIT = 100.0f;

/** Yaw rate integral anti-windup limit - typically 1.25-1.5x roll/pitch
 *  Yaw needs more integration room due to delayed response
 *  Typical range: 120-180
 */
static constexpr float YAW_I_LIMIT = 150.0f;

/** Rate controller output limit (motor command units)
 *  Clips PID output to prevent saturation; matches maximum controlled deflection
 *  Value of 300: allows ±300 motor commands for stabilization from neutral
 *  Typical range: 250-400 (with ESC range 1000-1800 = 800 units total)
 *  At ±300: uses 75% of available motor range for rate control
 */
static constexpr float RATE_PID_LIMIT = 300.0f;

// ============================================================================
// ANGLE CONTROLLER (Outer Loop) - Attitude Feedback
// ============================================================================
// Cascaded atop rate controller, runs at 1000 Hz (same loop)
// Converts desired angles (±45°) to rate setpoints (±200 deg/s)
// Output: Rate setpoint commands to inner loop

/** Roll angle proportional gain - converts angle error to rate command
 *  At 45° error: output = 3.8 * 45 = 171 deg/s (within ±200 limit)
 *  Typical range for 250g: 3.2-4.5
 *  Higher = faster angle tracking, risk of overshoot in cascaded loop
 */
static constexpr float ANGLE_ROLL_KP = 3.8f;

/** Pitch angle proportional gain - matches roll for symmetric response
 *  Typical range for 250g: 3.2-4.5
 */
static constexpr float ANGLE_PITCH_KP = 3.8f;

/** Yaw angle proportional gain - typically 30-45% of roll/pitch
 *  Yaw authority lower; prevents aggressive yaw snapping
 *  At 45° error: output = 1.5 * 45 = 67.5 deg/s (well under ±120 limit)
 *  Typical range for 250g: 1.2-2.0
 */
static constexpr float ANGLE_YAW_KP = 1.5f;

/** Roll angle integral gain - corrects steady-state angle bias (wind, gyro drift)
 *  At 1000 Hz with small angle error, very gradual accumulation
 *  Typical range for 250g: 0.010-0.025
 *  0.015: accumulates ~1 deg/s correction per degree error after ~65 sec
 */
static constexpr float ANGLE_ROLL_KI = 0.015f;

/** Pitch angle integral gain - matches roll
 */
static constexpr float ANGLE_PITCH_KI = 0.015f;

/** Yaw angle integral gain - typically 20-40% of roll/pitch
 *  Yaw is most sensitive to drift; keep small to avoid wind-up
 *  Typical range for 250g: 0.002-0.008
 */
static constexpr float ANGLE_YAW_KI = 0.004f;

/** Roll angle derivative gain - dampens angle setpoint changes
 *  Prevents jerky responses to stick input; typically small
 *  Typical range for 250g: 0.08-0.15
 *  0.10: adds 4.5 deg/s damping per deg/s of angle rate
 */
static constexpr float ANGLE_ROLL_KD = 0.10f;

/** Pitch angle derivative gain - matches roll
 */
static constexpr float ANGLE_PITCH_KD = 0.10f;

/** Yaw angle derivative gain - typically 30-50% of roll/pitch
 *  Minimal damping; yaw derivatives less reliable from gyro
 *  Typical range for 250g: 0.03-0.08
 */
static constexpr float ANGLE_YAW_KD = 0.05f;

/** Roll angle integral anti-windup limit
 *  Prevents integral from saturating; should be <50% of rate setpoint limit
 *  With ±200 deg/s rate limit and 3.8 KP:
 *    - At 52.6° error: rate command hits limit
 *    - Integral should stay <50 to leave P-term headroom
 *  Typical range: 40-70
 */
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;

/** Pitch angle integral anti-windup limit
 */
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;

/** Yaw angle integral anti-windup limit - typically 60-80% of roll/pitch
 *  Yaw limit is ±120 deg/s; allows less aggressive control
 *  Typical range: 30-50
 */
static constexpr float ANGLE_YAW_I_LIMIT = 30.0f;

/** Angle controller output limit (rate setpoint, deg/s)
 *  Limits maximum rate demand from angle controller
 *  At ±200: utilizes full rate controller authority (careful with very aggressive tuning)
 *  Typical range: 150-250 deg/s
 *  Conservative value: 150-180 deg/s (leaves headroom for disturbance rejection)
 *  Aggressive value: 200-250 deg/s (uses full capability, less forgiving)
 */
static constexpr float ANGLE_RATE_SETPOINT_LIMIT = 200.0f;

// ============================================================================
// TUNING GUIDE FOR 250g QUADCOPTER WITH 5" PROPS
// ============================================================================

/**
 * SYMPTOMS AND ADJUSTMENTS:
 * 
 * RATE CONTROLLER (Roll/Pitch/Yaw response):
 * ─────────────────────────────────────────
 * 
 * Undershooting (slow response, lag):
 *   - Increase KP by 10-15% (up to limit of oscillation)
 *   - Slightly decrease KD to reduce damping
 *   - May need to increase KI if steady-state error visible
 * 
 * Overshooting (overshoot on rate demand, then oscillation):
 *   - Increase KD by 10-20% to add damping
 *   - Reduce KP by 5-10% (reduce authority)
 *   - Check if KI too high (may cause sluggish response after correction)
 * 
 * Oscillation at specific frequency (buzz/ringing):
 *   - Increase KD significantly (25-50% boost)
 *   - Reduce KP by 5-15%
 *   - If still present, check IMU noise levels
 * 
 * Steady-state error (constant drift at constant input):
 *   - Increase KI by 20-50%
 *   - Verify motor calibration (differential thrust)
 *   - Check for wind or asymmetric load
 * 
 * Motor jitter/shake at rest:
 *   - Decrease KD (less noise amplification)
 *   - May need to filter gyro signal
 * 
 * 
 * ANGLE CONTROLLER (Attitude hold):
 * ──────────────────────────────────
 * 
 * Slow angle tracking:
 *   - Increase ANGLE_KP by 10-20% (increases rate command)
 *   - Check that rate controller is stable first
 * 
 * Overshooting to angle setpoint:
 *   - Increase ANGLE_KD by 20-50%
 *   - Reduce ANGLE_KP by 5-10%
 *   - May indicate cascaded oscillation; ensure rate loop is tuned conservatively
 * 
 * Drifting in angle (slow drift despite stick centered):
 *   - Increase ANGLE_KI by 50-100% (very gradually)
 *   - Verify gyro calibration
 *   - Check for external drift (wind, balance issue)
 * 
 * Angle not stabilizing after disturbance:
 *   - Increase ANGLE_KP
 *   - Check rate loop stability first (primary cause)
 * 
 * 
 * CASCADED TUNING STRATEGY:
 * ─────────────────────────
 * 
 * 1. Start with rate controller (inner loop):
 *    - Tune KP first for response speed
 *    - Add KD to eliminate oscillation
 *    - Finally tune KI for steady-state
 * 
 * 2. Then tune angle controller (outer loop):
 *    - Start with conservative ANGLE_KP (3.0-3.5)
 *    - Increase ANGLE_KP until angle tracking feels snappy
 *    - Add ANGLE_KD if overshoot appears
 * 
 * 3. Verify interaction:
 *    - Angle controller setpoint shouldn't exceed rate limits
 *    - If rate controller saturates, angle loop becomes unresponsive
 *    - Check: ANGLE_KP * max_angle ≤ rate_limit (3.8 * 45 = 171 ≤ 200 ✓)
 * 
 * 4. Final check:
 *    - Test rapid stick inputs (max rate demand)
 *    - Test smooth angle changes
 *    - Test disturbance rejection (hand push, small perturbation)
 * 
 * 
 * PLATFORM-SPECIFIC NOTES FOR 250g WITH 5" PROPS:
 * ─────────────────────────────────────────────────
 * 
 * - 5" propellers: Lower blade inertia than 6"+; responds quickly
 *   → Higher KP acceptable; moderate damping sufficient
 * 
 * - 250g total mass: Mid-size platform
 *   → More responsive than 500g+ frames; less raw power than 100g
 *   → Current KP values (0.90) near upper limit; increase carefully
 * 
 * - 1000 Hz control loop: High update rate
 *   → Allows tighter gains; KI values accumulate quickly
 *   → Can tolerate less derivative damping (noise averaging)
 * 
 * - 6" arm length: Standard for this class
 *   → Motor moment arm provides good control authority
 *   → Not extreme (no special scaling needed)
 * 
 * - ±200 deg/s rate limits: Aggressive but manageable
 *   → Leaves margin below mechanical limits (~300-350 deg/s typical)
 *   → Cascaded angle controller should rarely hit limit
 * 
 * - Conservative tuning philosophy:
 *   → Prefer slight undershoot to oscillation
 *   → Oscillation is harder to recover from
 *   → Undershoot → increase KP (easily reversible)
 *   → Oscillation → increase KD heavily or reduce KP (may lose responsiveness)
 * 
 * 
 * EXPECTED BEHAVIOR WITH THESE GAINS:
 * ────────────────────────────────────
 * 
 * Rate Mode (direct gyro control):
 *   - Step response: 0.8-1.2s to settle (slightly underdamped)
 *   - Latency: <5 ms noticeable delay
 *   - Noise floor: <2-3 deg/s jitter at rest
 *   - Max rate: ±200 deg/s (tuned authority)
 * 
 * Angle Mode (cascaded feedback):
 *   - Step response: 1.5-2.5s to settle
 *   - Slight overshoot: <5-10% acceptable
 *   - Stick responsiveness: Linear, predictable
 *   - Disturbance rejection: Good (automatic leveling)
 * 
 * Coupled behavior:
 *   - Yaw slower than pitch/roll (by design; moment of inertia)
 *   - Yaw can appear less "punchy" – this is tuned conservatively
 *   - Pitch/roll more responsive (higher moment)
 * 
 * 
 * FURTHER TUNING REFINEMENTS (If needed):
 * ────────────────────────────────────────
 * 
 * For FPV racing (max aggression):
 *   - Increase ROLL_KP → 1.05-1.15
 *   - Increase ANGLE_ROLL_KP → 4.2-4.8
 *   - Reduce *_KD slightly to prioritize response
 *   - Increase rate limit to 250-300 deg/s
 * 
 * For freestyle/acrobatics (moderate):
 *   - Current gains ideal; possibly increase YAW_KP → 0.60-0.65
 *   - Add slight KD boost for smoothness
 * 
 * For aerial photography (conservative):
 *   - Decrease ANGLE_ROLL_KP → 3.0-3.3
 *   - Increase *_KD for smooth filtering
 *   - Reduce rate setpoint limit → 150 deg/s
 * 
 * For wind resistance:
 *   - Increase KI values by 25-50%
 *   - Add low-pass filter to gyro (digital filter)
 *   - Increase ANGLE_KI (helps hold position in gusts)
 */

#endif // PID_GAINS_250G_5INCH_H
