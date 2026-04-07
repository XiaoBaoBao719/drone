# Flight Test Protocol for PID Tuning Validation

## Safety First ⚠️

- **Prop guards installed?** YES [ ] NO [ ]
- **Battery fully charged?** YES [ ] NO [ ]
- **Failsafe armed?** YES [ ] NO [ ]
- **Flight area clear?** YES [ ] NO [ ]
- **Observers at safe distance?** YES [ ] NO [ ]
- **Always maintain line-of-sight** 
- **Have remote disarm ready** (kill switch)

---

## Pre-Flight Checklist

### Hardware Verification
- [ ] Propellers secure and undamaged
- [ ] Motor bearings spin freely
- [ ] Arms not cracked
- [ ] Battery connector firm
- [ ] ESCs respond to throttle input
- [ ] Control surfaces/gimbal move freely
- [ ] Camera/payload secured (if applicable)

### Sensor Calibration  
- [ ] IMU accelerometer calibration complete
- [ ] Gyroscope calibration complete (no movement)
- [ ] Compass calibration (if compass used)
- [ ] ESC throttle calibration done
- [ ] RC transmitter throttle range verified

### Software Check
- [ ] Code compiles without errors
- [ ] No warnings in serial output
- [ ] Gains loaded correctly (verify on telemetry)
- [ ] Debug mode disabled (DEBUG_MODE = false)
- [ ] Telemetry baud rate correct (115200)

---

## Flight Test Phases

### Phase 0: Ground Spinning Test (Estimated Duration: 5 minutes)

**Objective:** Verify motor response to PID outputs without leaving ground

**Procedure:**
1. Arm drone (throttle down, yaw right)
2. Apply **5% collective throttle** (just spinning rotors)
3. Record gyro readings - should be ~0 (no rotation)
4. Gently tilt one arm downward
   - Motor on higher side should spin faster
   - Rate controller should respond
   - Verify telemetry: Rate is tracked

**Expected Results:**
- Gyro noise: <1°/s RMS
- Motor response: <50ms latency
- No erratic behavior

**If Problem:** Debug sensor calibration before flying

---

### Phase 1: Rate Mode - Gentle Inputs (Duration: 3-5 minutes)

**Objective:** Verify inner loop (rate controller) stability with conservative inputs

**Conditions:**
- Altitude: 1-2 meters AGL
- Wind: <3 m/s
- Flight area: At least 20m × 20m clear space
- Do NOT attempt aggressive maneuvers

**Procedure:**

1. **Arming & Hover**
   - Take off to 1-2m altitude
   - Hover stably in center of flight area
   - Wait 10 seconds: gyro should show <2°/s drift

2. **Roll Input Test** (30 seconds)
   - Apply 20-30% roll stick input (LEFT)
   - Hold for 2 seconds
   - Return to center
   - Observe: Smooth response, no oscillation
   - Telemetry: Roll rate reaches setpoint quickly
   - Record: Settling time, peak rate

3. **Pitch Input Test** (30 seconds)
   - Apply 20-30% pitch stick input (FORWARD)  
   - Hold for 2 seconds
   - Return to center
   - Observe: Same as roll
   - Record: Settling time

4. **Yaw Input Test** (30 seconds)
   - Apply 30% yaw stick input (RIGHT)
   - Hold for 2 seconds
   - Return to center
   - Observe: Smooth yaw response
   - Record: Yaw response vs roll/pitch (should be slower)

5. **Hover Check** (60 seconds)
   - Return throttle to hover position
   - No stick inputs
   - Observe: Stable hover, minimal drift
   - Verify: No oscillations in any axis

**Telemetry to Monitor:**
```
- setpt_roll / actual_roll_rate
- setpt_pitch / actual_pitch_rate  
- setpt_yaw / actual_yaw_rate
- PID output magnitudes (should be <±300)
- Motor PWM values (should be relatively symmetric)
```

**Pass Criteria:**
- ✅ No oscillation >0.5Hz
- ✅ Response time <200ms
- ✅ Setpoint tracking error <10°/s
- ✅ Hover stable (drift <1°/s)

**Fail Criteria → LAND IMMEDIATELY:**
- ❌ High-frequency oscillation (>5Hz)
- ❌ Unbounded growth in error
- ❌ Erratic motor behavior
- ❌ Inability to stabilize

---

### Phase 2: Rate Mode - Moderate Inputs (Duration: 5 minutes)

**Objective:** Test rate controller with 60-70% stick inputs

**Conditions:** Same as Phase 1

**Procedure:**

1. **Roll Moderate Test** (45 seconds)
   - Apply 50% roll stick
   - Hold 3 seconds
   - Return to center
   - Record: Peak rate achieved, overshoot
   - Expected: ~100°/s roll rate

2. **Pitch Moderate Test** (45 seconds)
   - Apply 50% pitch stick
   - Hold 3 seconds
   - Return to center
   - Expected: ~100°/s pitch rate

3. **Combined Roll+Pitch** (45 seconds)
   - Apply 40% roll + 40% pitch (diagonal)
   - Hold 2 seconds
   - Return to center
   - Observe: Both axes independent

4. **Rapid Input Sequence** (90 seconds)
   - Perform alternating 1-second bursts:
     - 50% roll RIGHT
     - 50% pitch FORWARD  
     - 50% roll LEFT
     - 50% pitch BACK
     - Repeat 3× rapidly
   - Monitor for oscillation buildup

**Pass Criteria:**
- ✅ Smooth rate tracking to ~150°/s
- ✅ No overshoot >10%
- ✅ Quick settle (<100ms)
- ✅ Motors don't saturate

**Fail Criteria → LAND IMMEDIATELY:**
- ❌ Oscillation amplitude increasing
- ❌ Response becomes sluggish
- ❌ Ringing/ratcheting sound

---

### Phase 3: Rate Mode - Aggressive Inputs (Duration: 5 minutes)

**Objective:** Verify rate controller stability near max commanded rates

**Procedure:**

1. **Full Roll Input** (30 seconds)
   - Apply 100% roll stick (200°/s command)
   - Hold 2 seconds, return to center
   - Record: Peak rate, settling
   - Expected: Reach ~190-200°/s

2. **Full Pitch Input** (30 seconds)
   - Apply 100% pitch stick
   - Hold 2 seconds, return
   - Expected: Reach ~190-200°/s

3. **Full Yaw Input** (30 seconds)
   - Apply 100% yaw stick (120°/s command)
   - Hold 2 seconds, return
   - Expected: Reach ~110-120°/s

4. **Rapid Oscillation Test** (60 seconds)
   - Rapidly apply/release roll stick (0.5 Hz square wave)
   - Duration: 10 seconds
   - Repeat at 1 Hz, 2 Hz frequencies
   - Verify: No resonance buildup

**Pass Criteria:**
- ✅ Reaches full commanded rates
- ✅ No overshoot
- ✅ Smooth transitions
- ✅ No instability

**Fail Criteria:**
- ❌ Oscillation visible on aircraft
- ❌ Gyro rates ring/ring-down
- ❌ Motors reaching saturation

---

### Phase 4: Rate Mode - Emergency Recovery (Duration: 5 minutes)

**Objective:** Verify control authority during attitude recovery

**Procedure:**

1. **Large Tilt Recovery**
   - Pitch stick to 100% for 3 seconds (nose-down attitude)
   - Drone pitches forward significantly (~30-45°)
   - Release stick abruptly to center
   - Verify: Rate controller quickly stops rotation
   - Recovery time: <500ms

2. **Oscillating Tilt**
   - Pitch stick: 50% forward, hold 1s
   - Pitch stick: 50% back, hold 1s
   - Repeat 3× rapidly
   - Verify: Clean transitions, no overshoot

3. **Multi-Axis Stress Test**
   - Simultaneously: 70% roll + 70% pitch
   - Hold 2 seconds
   - Verify: Responds smoothly, balanced

**Pass Criteria:**
- ✅ Recovers from large tilts (<500ms)
- ✅ Maintains altitude control
- ✅ No propeller strike risk

---

### Phase 5: Angle Mode - Gentle Inputs (Duration: 5 minutes)

**Objective:** Verify outer loop (angle controller) stability

**Pre-Switch Checklist:**
- [ ] Rate mode stable and verified (Phase 1-3 complete)
- [ ] Drone hovering stably
- [ ] Clear flight area
- [ ] Ready to switch control mode

**Procedure:**

1. **Hover Stable** (60 seconds)
   - Ensure rate mode hover is stable
   - Note: Current attitude angles

2. **Switch to Angle Mode**
   - Flip control mode switch to ANGLE_MODE
   - Verify switch took effect (telemetry)
   - Maintain neutral stick
   - Drone should hold current attitude

3. **Small Roll Input** (45 seconds)
   - Apply 15-20% roll stick
   - Hold for 2 seconds
   - Return stick to center
   - Record: Time to reach attitude, overshoot
   - Expected: ~1.5 second settling to 10-15° roll

4. **Small Pitch Input** (45 seconds)
   - Apply 15-20% pitch stick
   - Record: Same as roll

5. **Return to Hover** (30 seconds)
   - Center all sticks
   - Verify: Returns to level attitude
   - Drift: Should be minimal

**Telemetry to Monitor:**
```
- desired_roll / actual_roll (degrees)
- desired_pitch / actual_pitch
- desired_yaw / actual_yaw
- rate_setpoint from angle controller
```

**Pass Criteria:**
- ✅ Reaches commanded attitudes
- ✅ Settling time ~1-2 seconds
- ✅ No overshoot >5°
- ✅ Smooth transitions

**Fail Criteria → SWITCH BACK TO RATE MODE:**
- ❌ Oscillating attitude
- ❌ Overshooting commanded angle
- ❌ Drifting slowly

**If Fail:** Reduce ANGLE_KP by 0.2 and re-test

---

### Phase 6: Angle Mode - Moderate Inputs (Duration: 5 minutes)

**Objective:** Test angle controller with larger attitude commands

**Procedure:**

1. **30° Roll Test** (30 seconds)
   - Apply 50% roll stick (should command ~30° attitude)
   - Hold 2-3 seconds
   - Return stick to center
   - Record: Peak angle reached, settling time

2. **30° Pitch Test** (30 seconds)
   - Apply 50% pitch stick
   - Record: Same metrics

3. **Diagonal Maneuver** (30 seconds)
   - Apply 40% roll + 40% pitch simultaneously
   - Hold 2 seconds, return to center
   - Verify: Both axes move smoothly together

4. **Continuous Roll** (30 seconds)
   - Apply 60% roll stick continuously
   - Hold for 4-5 seconds
   - Verify: Reaches steady ~35-40° roll angle

**Pass Criteria:**
- ✅ Reaches commanded attitudes (~40° at 100% stick)
- ✅ Overshoot <10%
- ✅ Settling time <2 seconds
- ✅ Smooth, coordinated motion

---

### Phase 7: Angle Mode - Aggressive Inputs (Duration: 5 minutes)

**Objective:** Verify max angle controller capability

**Procedure:**

1. **Maximum Roll** (30 seconds)
   - Apply 100% roll stick
   - Hold 3-4 seconds
   - Verify: Reaches ~45° roll (safety limit)
   - Record: Time to max, rate of attitude change

2. **Maximum Pitch** (30 seconds)
   - Apply 100% pitch stick
   - Record: Same metrics

3. **Full Envelope Test** (90 seconds)
   - Rapid stick movements through full range
   - Monitor: Attitude response is smooth
   - Record: Any oscillations or sluggishness

**Pass Criteria:**
- ✅ Reaches max attitude safely
- ✅ No overshoot beyond limits
- ✅ No control oscillation

---

### Phase 8: Combined Test - Real Flight Maneuvers (Duration: 10 minutes)

**Objective:** Verify PID gains in realistic flying scenario

**Procedure:**

1. **Figure-8 Hover** (2 minutes)
   - Fly horizontal figure-8 pattern at 1m altitude
   - Mild stick inputs
   - Verify: Smooth coordinated flight

2. **Altitude Hold** (2 minutes)
   - Use throttle to climb to 3m, descend to 1m
   - Smooth transitions
   - Verify: Level attitude maintained

3. **Yaw Rotation** (1 minute)
   - Rotate continuously yaw in both directions
   - Verify: Yaw rate controller smooth
   - Check: Doesn't couple into roll/pitch

4. **Dynamic Flying** (3 minutes)
   - Moderate stick inputs for natural flying
   - Smooth acro-style maneuvers
   - Record: Any instability or oscillations

5. **Landing** (2 minutes)
   - Descend smoothly to ground
   - Auto-level on landing if available
   - Verify: Clean landing

**Pass Criteria:**
- ✅ All maneuvers smooth and coordinated
- ✅ No oscillations during flight
- ✅ Responsive to input
- ✅ Safe landing

---

## Telemetry Data Logging

### Essential Data to Log

Create CSV file with columns:
```
timestamp_ms, 
setpt_roll, actual_roll, pidout_roll,
setpt_pitch, actual_pitch, pidout_pitch,
setpt_yaw, actual_yaw, pidout_yaw,
throttle,
motor1_pwm, motor2_pwm, motor3_pwm, motor4_pwm,
battery_voltage
```

### Analysis After Flight

```python
# Plot these graphs:
1. Time vs Setpoint/Actual Rate (should track closely)
2. Time vs PID Output (should be smooth, not jittery)
3. FFT of Rate Error (dominant frequencies should be <3Hz)
4. Motor PWM commands (should be balanced, no saturation)
```

---

## Tuning Adjustments Based on Test Results

### If Rate Mode Oscillates:

| Symptom | Cause | Fix |
|---------|-------|-----|
| High-freq ring (>5 Hz) | Too much KD | Reduce KD by 0.01 |
| Low-freq oscillation (1-2 Hz) | Too much KP | Reduce KP by 0.05 |
| Slow to respond | Too little KP | Increase KP by 0.05 |
| Can't maintain setpoint | Too little KI | Increase KI by 0.005 |
| Motor saturation early | Gains too high overall | Reduce all K by 10% |

### If Angle Mode Unstable:

| Symptom | Cause | Fix |
|---------|-------|-----|
| Oscillates around target | ANGLE_KP too high | Reduce by 0.2-0.3 |
| Slow to reach attitude | ANGLE_KP too low | Increase by 0.1-0.2 |
| Overshoots then recovers | ANGLE_KD too low | Increase by 0.02 |
| Drifts from target | ANGLE_KI too low | Increase by 0.002 |

---

## Sign-Off Sheet

After completing all phases, fill in:

**Test Date:** ________________

**Drone S/N:** ________________

**Firmware Version:** ________________

**PID Gains Used:**
```
ROLL_KP = ____, ROLL_KI = ____, ROLL_KD = ____
PITCH_KP = ____, PITCH_KI = ____, PITCH_KD = ____
YAW_KP = ____, YAW_KI = ____, YAW_KD = ____
ANGLE_ROLL_KP = ____, ANGLE_ROLL_KI = ____, ANGLE_ROLL_KD = ____
ANGLE_PITCH_KP = ____, ANGLE_PITCH_KI = ____, ANGLE_PITCH_KD = ____
ANGLE_YAW_KP = ____, ANGLE_YAW_KI = ____, ANGLE_YAW_KD = ____
```

**Phase Completion:**
- [ ] Phase 0: Ground Spinning Test ✅ PASS / ❌ FAIL
- [ ] Phase 1: Rate Mode - Gentle ✅ PASS / ❌ FAIL
- [ ] Phase 2: Rate Mode - Moderate ✅ PASS / ❌ FAIL  
- [ ] Phase 3: Rate Mode - Aggressive ✅ PASS / ❌ FAIL
- [ ] Phase 4: Rate Mode - Recovery ✅ PASS / ❌ FAIL
- [ ] Phase 5: Angle Mode - Gentle ✅ PASS / ❌ FAIL
- [ ] Phase 6: Angle Mode - Moderate ✅ PASS / ❌ FAIL
- [ ] Phase 7: Angle Mode - Aggressive ✅ PASS / ❌ FAIL
- [ ] Phase 8: Combined Real Flight ✅ PASS / ❌ FAIL

**Overall Result:** ✅ **VALIDATED** / ❌ **REQUIRES TUNING**

**Notes:**
```
_________________________________________________________________
_________________________________________________________________
_________________________________________________________________
```

**Pilot Name:** ________________  **Date:** ________________

**Next Actions:**
- [ ] Deploy gains to production  
- [ ] Document in wiki/repository
- [ ] Share results with team
- [ ] Archive flight logs

---

## Emergency Procedures

### If Drone Becomes Unstable During Test:

1. **IMMEDIATELY** return stick to center (neutral)
2. **SIMULTANEOUSLY** reduce throttle to crash safely
3. Disarm as soon as drone lands
4. **DO NOT** re-arm until cause identified
5. Review telemetry logs for oscillation patterns
6. Reduce all K values by 20% before next test

### If Motor Stalls:

1. **KILL** throttle immediately (pull stick down)
2. This may cause hard crash - acceptable vs propeller strike injury
3. After landing, inspect:
   - Motor mechanical binding?
   - ESC firmware issue?
   - Motor cables loose?

### If Battery Fails Mid-Flight:

1. Failsafe should activate automatically
2. Drone will cut throttle and crash
3. This is designed behavior

---

## Documentation to Keep

After successful tuning, document:

1. ✅ Recommended gains (copy from this guide)
2. ✅ Flight test results (pass/fail for each phase)
3. ✅ Telemetry logs (CSV files with timestamps)
4. ✅ Video (optional but helpful for future reference)
5. ✅ Any tuning adjustments made
6. ✅ Final drone configuration
7. ✅ Environmental conditions during test (temperature, wind, etc.)
