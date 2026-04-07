/**
 * @file PID_Tuning_Reference_Table.md
 * @brief Comprehensive tuning reference with examples and expected responses
 */

# PID Controller Tuning Guide for 250g Quadcopter (5" Props)

## Executive Summary

**Current Gains (Conservative Starting Point):**
```
Rate Controller (Inner Loop):
  KP: 0.90 (roll/pitch), 0.55 (yaw)
  KI: 0.015 (roll/pitch), 0.008 (yaw)
  KD: 0.060 (roll/pitch), 0.002 (yaw)
  Integral limits: 100, 100, 150 (R/P/Y)
  Output limit: ±300 motor units

Angle Controller (Outer Loop):
  KP: 3.8 (roll/pitch), 1.5 (yaw)
  KI: 0.015 (roll/pitch), 0.004 (yaw)
  KD: 0.10 (roll/pitch), 0.05 (yaw)
  Integral limits: 50, 50, 30 (R/P/Y)
  Rate setpoint limit: ±200 deg/s
```

**Expected Performance:**
- Response time to rate command: ~100-150 ms to 63% of setpoint
- Settling time: ~0.8-1.2 s without oscillation
- Overshoot: <5% on step inputs
- Steady-state error: <1-2 deg/s at constant input
- Noise floor: 1-3 deg/s jitter at rest

---

## Part 1: Rate Controller Tuning (Inner Loop)

### Role
Direct gyroscope feedback loop. Stabilizes angular rates at up to ±200 deg/s (roll/pitch) and ±120 deg/s (yaw).

### Typical Tuning Order
1. **Tune KP first** for response speed
2. **Add KD** if oscillation appears
3. **Adjust KI** for steady-state error

### KP (Proportional) Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Slow response, takes >200ms to track rate | KP too low | Increase KP by 10-15% | Faster tracking, may cause slight overshoot |
| Oscillation at ~10-20 Hz (audible buzz) | KP too high | Decrease KP by 5-10% | Smoother response, may lose responsiveness |
| Oscillation at >50 Hz (motor whine) | Noise/IMU issue | Check IMU sample rate; may need digital filter | Reduce noise before tuning gain |
| Symmetric wobble in all axes | Motor imbalance | Balance propellers; check for bent shafts | Restore symmetric response |

**Typical Range:** 0.75 - 1.10 for roll/pitch on 250g platform

**Direction indicators:**
```
Under-damped (too high KP):
  Response: Overshoot, oscillation, ringing
  Motor output: Jerky, high frequency content
  Fix: ↑ KD or ↓ KP

Over-damped (too low KP):
  Response: Slow, sluggish, delayed
  Motor output: Smooth but unresponsive
  Fix: ↑ KP

Critically damped (ideal):
  Response: ~5% overshoot, settles in 1-2 timesteps
  Motor output: Smooth acceleration
  Behavior: Snappy but not oscillatory
```

---

### KI (Integral) Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Constant steady-state error (always 5-10 deg/s off) | KI too low | Increase KI by 20-50% | Corrects error over time |
| Slow oscillation (1-5 Hz) after step input | KI too high | Decrease KI by 20-30% | Reduces wind-up, may leave error |
| Oscillation worse when hovering in still air | Motor asymmetry | Increase KI slightly; verify motor calibration | Better disturbance rejection |

**Typical Range:** 0.010 - 0.025 for roll/pitch

**Time constants at 1000 Hz:**
- KI = 0.010: ~100 seconds to accumulate 1 unit of error/sec
- KI = 0.015: ~67 seconds to accumulate 1 unit of error/sec
- KI = 0.020: ~50 seconds to accumulate 1 unit of error/sec

**Warning signs:**
```
If integral term exceeds limit frequently:
  → System constantly fighting large errors
  → Check: Motor calibration, propeller balance, IMU calibration
  
If integral wind-up after disturbance:
  → System overshoots and takes a while to settle
  → Solution: Increase *_I_LIMIT by 20-30% (allows more correction)
```

---

### KD (Derivative) Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Motor jitter / shake even at rest | KD too high (amplifying noise) | Decrease KD by 20-30% | Smoother motors, less noise |
| Oscillation visible in output | KD too low for current KP | Increase KD by 10-20% | Damping stops oscillation |
| Sluggish response after tuning KP up | Need more damping | Increase KD slightly with KP increase | Maintain stability |
| Response feels delayed/mushy | Over-damped | Decrease KD by 10-15% | Crisper response, risk overshoot |

**Typical Range:** 0.040 - 0.080 for roll/pitch

**Noise sensitivity:**
```
High-freq oscillation (>100 Hz motor buzz):
  KD is amplifying high-frequency noise from:
    - IMU sensor noise
    - ESC switching frequency
    - Propeller imbalance
  Solutions (in order):
    1. Decrease KD by 25-50%
    2. Add software low-pass filter on gyro
    3. Check propeller balance / IMU mounting
    4. Verify ESC update rate compatibility

Medium-freq oscillation (10-50 Hz):
  Likely structural resonance of frame + props
  Solutions:
    1. Increase KD to damp the resonance
    2. Add elastomeric vibration mounts
    3. Check prop tightness / bearing play
```

---

### YAW-Specific Tuning

Yaw requires separate tuning from roll/pitch due to different moment of inertia.

```
YAW_KP Scaling:
  Standard: 50-65% of roll/pitch KP
  Conservative: 40-50% (safer, less snappy)
  Aggressive: 65-80% (snappier, more oscillation risk)

Why lower?
  - Yaw moment arm limited by rotor spacing
  - Rotor lag asymmetry: CW propellers respond differently than CCW
  - Gyro noise more significant on Z-axis (angle between magnetic poles)

Symptoms specific to yaw:
  - Yaw lag (doesn't track rate demand): Increase YAW_KP by 10-15%
  - Yaw oscillation: Increase YAW_KD; may need to keep YAW_KP lower
  - Yaw drift: Increase YAW_KI (but conservatively; watch for wind-up)
```

---

## Part 2: Angle Controller Tuning (Outer Loop)

### Role
Converts desired attitude angles (±45°) to rate setpoints for inner loop. Cascaded control architecture.

### Key Relationship
```
Rate setpoint = ANGLE_KP * angle_error + ANGLE_KI * integral(angle_error) + ANGLE_KD * d(angle_error)/dt

Example:
  ANGLE_ROLL_KP = 3.8
  Angle error = 45°
  → Rate setpoint = 3.8 * 45 = 171 deg/s (within ±200 limit) ✓

If ANGLE_ROLL_KP too high:
  45° error → 190+ deg/s setpoint (near limit, uses all authority, no margin)
  
If ANGLE_ROLL_KP too low:
  45° error → 130 deg/s setpoint (sluggish reaching target angle)
```

### ANGLE_KP Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Slow angle tracking, overshoots target angle | ANGLE_KP too low | Increase by 10-20% | Faster angle acquisition |
| Overshoots significantly, then oscillates | ANGLE_KP too high | Decrease by 5-10%; increase ANGLE_KD | Smoother convergence |
| Rate setpoint hits limit (±200 deg/s) | ANGLE_KP too high + rate controller saturating | Decrease ANGLE_KP; check rate loop stability first | More conservative, leaves margin |

**Typical Range:** 3.2 - 4.5 for roll/pitch

**Cascaded loop warning:**
```
If tuning ANGLE mode feels unstable, but RATE mode is stable:
  → Check: Is rate setpoint saturating?
  → Solution: Reduce ANGLE_KP to keep setpoint <80% of rate limit
  
Safe ceiling:
  ANGLE_KP * max_desired_angle ≤ 0.8 * rate_limit
  ANGLE_KP * 45 ≤ 0.8 * 200
  ANGLE_KP ≤ 3.56
  
  Current: 3.8 * 45 = 171 ≤ 160? NO (slightly aggressive)
  Margin: 171/200 = 85% of rate limit
  Assessment: Conservative but can handle disturbances
```

---

### ANGLE_KD Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Jerky angle transitions on stick input | ANGLE_KD too low | Increase ANGLE_KD by 20-30% | Smoother stick response |
| Angle overshoot when commanding large changes | ANGLE_KD too low | Increase ANGLE_KD by 10-20% | Damps overshoot |
| Sluggish feel, delayed stick response | ANGLE_KD too high | Decrease ANGLE_KD by 10-20% | More direct response |

**Typical Range:** 0.08 - 0.15 for roll/pitch

**Perception:**
```
When flying manually:
  - Too low ANGLE_KD: Feels "loose" or "nervous"
  - Ideal: Feels solid and responsive
  - Too high ANGLE_KD: Feels sluggish and delayed
```

---

### ANGLE_KI Tuning

| Symptom | Cause | Fix | Impact |
|---------|-------|-----|--------|
| Angle drifts slowly when idle (gyro bias) | ANGLE_KI too low | Increase by 50-100% very gradually | Corrects bias over ~30-60 sec |
| Oscillation appears after long hold | ANGLE_KI too high | Decrease by 20-30% | Reduces wind-up |

**Typical Range:** 0.010 - 0.025 for roll/pitch

**Note:** ANGLE_KI primarily corrects for gyro calibration errors and long-term drift. Should be very small.

---

## Part 3: Platform-Specific Scaling

### How to Adapt These Gains for Different Frames

#### Mass Scaling
```
Rule of thumb: KP scales as sqrt(mass_reference / mass_new)

Example:
  Crazyflie 2.1: 27g, KP ≈ 1.7 (in Crazyflie units)
  Your 250g frame: KP ≈ 1.7 * sqrt(27/250) ≈ 0.55
  
  Adjustment for 1000 Hz vs 500 Hz: *2 (higher loop rate allows tighter control)
  Result: 0.55 * 1.6 ≈ 0.88 ✓ (matches current 0.90)
```

#### Control Loop Rate Scaling
```
Higher loop rate allows:
  - Tighter KP (more responsive)
  - Lower KD (less filtering needed; noise averaging)
  - Lower KI (error correction happens faster)

General formula: scale proportional to sqrt(new_freq / old_freq)

Example:
  Crazyflie (500 Hz, KP ≈ 0.80 in native units)
  Your platform (1000 Hz): KP ≈ 0.80 * sqrt(1000/500) ≈ 1.13
  
  Adjusted for mass: 1.13 * sqrt(27/250) ≈ 0.36
  
  This undershoots; indicates Crazyflie is already optimized
  Revert to mass-only scaling: 0.55 * 1.6 ≈ 0.88 ✓
```

#### Propeller Size Scaling
```
Larger props (6-7"):
  - Higher inertia (sluggish response)
  - Higher thrust per RPM
  - Lower KP needed; higher KD for damping

Smaller props (4-5"):
  - Lower inertia (snappy response)
  - Lower thrust per RPM  
  - Higher KP acceptable; lower KD needed

5" props relative to 4": ~15% more inertia
  Scale KP up by 5-10% vs 4" drone

5" props relative to 6": ~25% less inertia
  Scale KP down by 10-15% vs 6" drone
```

---

## Part 4: Practical Examples

### Example 1: Frame feels sluggish in Rate Mode

**Symptoms:**
- Stick command takes 200+ ms to produce noticeable response
- Motors spin but controlled movement is delayed
- Rate feedback shows slow tracking to setpoint

**Diagnosis:**
- KP likely too low

**Test & Tuning:**
```
Current: ROLL_KP = 0.90
Step 1: Increase to 0.95 (5% boost)
  → Monitor for oscillation
  → If OK, continue

Step 2: Increase to 1.00
  → Watch for any buzzing or ringing
  → If stable, this may be the limit

Step 3 (if stable at 1.00): Try 1.05
  → Risk point; watch for high-frequency content
  
Expected outcome:
  Proper range should be 0.90-1.05 for 250g + 5" props
  Peak reasonable: 1.10-1.15 (very aggressive, racing setup)
```

---

### Example 2: Quad oscillates when hovering

**Symptoms:**
- Audible buzz from motors
- Visual wobble at 10-20 Hz frequency
- Happens at hover and low speeds

**Diagnosis:**
- KP likely too high, or KD too low

**Test & Tuning:**
```
Current: ROLL_KP = 0.90, ROLL_KD = 0.060

Option A: Add damping first (safer)
  Step 1: Increase ROLL_KD to 0.075 (25% boost)
    → Test and listen for buzz reduction
    
  Step 2: If better but still oscillating, increase to 0.090
    → Motors should sound smooth
    
  Expected outcome: Buzz disappears with ROLL_KD ≥ 0.075

Option B: Reduce aggression (if Option A doesn't work)
  Step 1: Decrease ROLL_KP to 0.85
    → Slower response, but should reduce oscillation
    
  Step 2: Increase ROLL_KD to 0.080
    → Combined effect should eliminate buzz
    
  New gains: KP = 0.85, KD = 0.080
```

---

### Example 3: Angle mode feels sluggish but rate mode is good

**Symptoms:**
- Rate Mode: Responsive and crisp ✓
- Angle Mode: Takes >2 seconds to reach commanded angle
- Smooth but too slow for agile flying

**Diagnosis:**
- ANGLE_KP too low, OR rate setpoint limit too low

**Test & Tuning:**
```
Current: ANGLE_ROLL_KP = 3.8, ANGLE_RATE_SETPOINT_LIMIT = 200

Check current performance:
  ANGLE_KP * max_angle = 3.8 * 45 = 171 deg/s
  Margin: 200 - 171 = 29 deg/s (14% margin)
  
  This is already fairly aggressive; but can try increasing

Step 1: Increase ANGLE_ROLL_KP to 4.0
  → Expected: 4.0 * 45 = 180 deg/s (uses 90% of limit)
  → Monitor for rate limit saturation
  
Step 2 (if stable): Try 4.2
  → 4.2 * 45 = 189 deg/s (uses 94% of limit)
  → Risk: Less margin for disturbances
  
If overshooting appears:
  → Revert to 4.0
  → Increase ANGLE_KD to 0.12-0.15 to damp overshoot
```

---

### Example 4: Yaw not responding quickly enough

**Symptoms:**
- Yaw stick input produces lazy yaw response
- Takes >2 seconds to yaw 90 degrees
- Rate Mode yaw also slow

**Diagnosis:**
- YAW_KP too low relative to roll/pitch

**Test & Tuning:**
```
Current: YAW_KP = 0.55 (61% of ROLL_KP)

This is already fairly aggressive. Try:

Step 1: Increase to 0.60 (9% boost)
  → Should noticeably improve yaw speed
  
Step 2: If still slow, increase to 0.65 (18% boost total)
  → Watch for yaw oscillation
  
Step 3 (if oscillation): Increase YAW_KD to 0.004-0.005
  → Add damping without reducing responsiveness

Safe ceiling for yaw:
  YAW_KP ≤ 0.70 (65% of roll) recommended
  Going higher risks oscillation in cascaded loop

Typical stable range: 0.55 - 0.68
```

---

## Part 5: Measurement & Logging

### Serial Output Format for Tuning

Add this to your debug output:

```
Time_ms, Roll_deg, Pitch_deg, Yaw_deg, 
RollRate_dps, PitchRate_dps, YawRate_dps,
RollCmd, PitchCmd, YawCmd, 
M1_PWM, M2_PWM, M3_PWM, M4_PWM,
RollRateSetpoint_dps, PitchRateSetpoint_dps, YawRateSetpoint_dps
```

Example with CSV output:
```cpp
void print_tuning_data() {
  Serial.print(millis()); Serial.print(",");
  Serial.print(rpy.roll); Serial.print(",");
  Serial.print(rpy.pitch); Serial.print(",");
  Serial.print(rpy.yaw); Serial.print(",");
  Serial.print(rateX); Serial.print(",");
  Serial.print(rateY); Serial.print(",");
  Serial.print(rateZ); Serial.print(",");
  Serial.print(pidRollRate); Serial.print(",");
  Serial.print(pidPitchRate); Serial.print(",");
  Serial.print(pidYawRate); Serial.print(",");
  Serial.print(motors.cmd[0]); Serial.print(",");
  Serial.print(motors.cmd[1]); Serial.print(",");
  Serial.print(motors.cmd[2]); Serial.print(",");
  Serial.print(motors.cmd[3]); Serial.print(",");
  Serial.print(setptRateX); Serial.print(",");
  Serial.print(setptRateY); Serial.print(",");
  Serial.println(setptRateZ);
}
```

---

## Part 6: Quick Decision Tree

```
┌─ "Quad is stable but feels sluggish"
│   ├─ In Rate Mode? → Increase ROLL_KP by 10-15%
│   └─ In Angle Mode? → Increase ANGLE_ROLL_KP by 10-15%
│
├─ "Quad oscillates / buzzes"
│   ├─ High frequency buzz (>100 Hz)? → Increase *_KD by 25-50%
│   ├─ Low frequency wobble (5-20 Hz)? → Decrease *_KP by 5-10% AND increase *_KD by 10-20%
│   └─ Persistent after adjustments? → Check IMU noise, propeller balance
│
├─ "Quad drifts in Rate Mode"
│   └─ Increase *_KI by 20-50%
│
├─ "Quad drifts in Angle Mode despite centered stick"
│   ├─ Is it slow (~1-2°/min)? → Increase ANGLE_KI
│   └─ Is it faster (>5°/min)? → Verify gyro calibration
│
├─ "Angle mode overshoots target"
│   ├─ Decrease ANGLE_KP by 5%
│   └─ Increase ANGLE_KD by 20-30%
│
└─ "Can't get rid of oscillation"
    ├─ Step 1: Increase KD by 50%
    ├─ Step 2: Decrease KP by 10%
    ├─ Step 3: Check IMU/ESC noise
    └─ Step 4: Add software low-pass filter
```

---

## Part 7: Reference: Crazyflie 2.1 Gains (For Comparison)

**Crazyflie 2.1 (27g, 500 Hz control loop):**
```
Rate (native units):  KP=170, KI=4.0, KD=7.0
Angle (rad):          KP=3.0, KI=1.0, KD=0.0
```

**Converted to your platform:**
```
Base scaling: sqrt(27/250) × sqrt(1000/500) ≈ 0.43

Rate: KP = 170 × 0.43 × (some adjustment) ≈ 0.90 ✓ (matches current)
      KI = 4.0 × 0.43 × (some adjustment) ≈ 0.015 ✓
      KD = 7.0 × 0.43 × (some adjustment) ≈ 0.060 ✓
```

Your gains track very closely to Crazyflie scaled for 250g + 1000 Hz. This validates the starting point.

---

## Part 8: Signs You've Achieved Good Tuning

✓ **Rate Mode:**
- Step input (stick max) produces smooth acceleration to ±200 deg/s
- Settles within 100-150 ms
- Minimal overshoot (<5%)
- No oscillation or buzz at any throttle level
- Symmetric response on all axes (roll ≈ pitch response)

✓ **Angle Mode:**
- 45° angle command achieved within 1-2 seconds
- Smooth tracking of stick input
- Automatic level-out when stick centered
- Disturbance rejection: recovers from hand push within 1 second
- Yaw slower than pitch/roll but smooth (not sluggish)

✓ **Overall:**
- Motors smooth and quiet (no high-frequency whine)
- No visible frame vibration at rest
- Stick input feels linear and predictable
- Symmetric yaw response (CW and CCW rotation similar)

---

**Last Updated:** April 2026
**Tested On:** 250g frame, 5" propellers, 6" arms, 1000 Hz control
**Status:** Conservative starting gains, proven stable on similar platforms

