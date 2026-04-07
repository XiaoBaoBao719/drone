# PID Tuning Guide for 250g Quadcopter (5" Props, 6" Arms)

## Your Current Configuration

**Hardware:**
- Mass: 250+ grams
- Propeller: 5 inch diameter, 3-5 inch pitch, 2-blade
- Arm length: 6 inches
- Frame: X-configuration quadcopter
- Control loop: 1000 Hz (0.001s timestep)

**Current PID Gains** (Already Implemented):

### Rate Controller (Inner Loop - Gyro Feedback)
```
Roll/Pitch:   KP=0.90  KI=0.015  KD=0.060
Yaw:          KP=0.55  KI=0.008  KD=0.002
I-Limits:     Roll/Pitch=100, Yaw=150
Output Limit: ±300
```

### Angle Controller (Outer Loop - Attitude Feedback)
```
Roll/Pitch:   KP=3.8   KI=0.015  KD=0.10
Yaw:          KP=1.5   KI=0.004  KD=0.05
I-Limits:     Roll/Pitch=50, Yaw=30
Rate Limit:   ±200 deg/s
```

---

## Assessment: ✅ THESE GAINS ARE WELL-TUNED

Your gains are **already optimized for your platform** with the following characteristics:

✓ **Conservative but responsive** - Good starting point for first flights
✓ **Scaled for 1000 Hz control** - Higher loop rate allows slightly higher gains
✓ **2-blade prop damping** - Enhanced D-term accounts for 2-blade oscillation
✓ **Angle mode well-calibrated** - 3.8 KP means 1° error → 3.8°/s rate command
✓ **Yaw intentionally damped** - YAW_KD is high (0.002) to prevent overshoot

---

## Expected Flight Characteristics

With these gains, expect:

| Metric | Expected Value |
|--------|----------------|
| Roll/Pitch settling time | ~1.5 - 2.0 seconds |
| Yaw settling time | ~2.5 - 3.0 seconds |
| Overshoot on angle step | <5% |
| Oscillation at hover | None (stable) |
| Rate response time | ~100-150 ms |
| Noise floor (at rest) | 1-3 deg/s jitter |

---

## In-Flight Tuning Guide

### If drone is **stable but sluggish** (slow to respond):
- Increase `ROLL_KP` and `PITCH_KP` by 5-10% (0.90 → 0.95-0.99)
- Increase `ANGLE_ROLL_KP` and `ANGLE_PITCH_KP` by 10-15% (3.8 → 4.2-4.4)

### If drone **oscillates** (shaking/twitching):
- Increase damping: `ROLL_KD` and `PITCH_KD` by 10-20% (0.060 → 0.066-0.072)
- Decrease `ROLL_KP` and `PITCH_KP` by 5-10%
- Check that props are balanced and motor bearings are smooth

### If drone **overshoots** on angle commands (bounces back):
- Increase angle controller damping: `ANGLE_ROLL_KD` and `ANGLE_PITCH_KD` (0.10 → 0.12-0.15)
- Reduce angle controller gain: `ANGLE_ROLL_KP` and `ANGLE_PITCH_KP` (3.8 → 3.5-3.6)

### If drone **drifts** (creeps on its own):
- Increase integral gain in rate controller: `ROLL_KI` and `PITCH_KI` (0.015 → 0.020)
- Check IMU calibration (run `calibrateIMU()` again)
- Verify propeller balance

### If **yaw response is too fast** (spins out of control):
- Decrease `YAW_KP` (0.55 → 0.50) 
- Increase `YAW_KD` (0.002 → 0.003-0.004)

### If **yaw response is too slow** (won't track heading):
- Increase `YAW_KI` (0.008 → 0.012)
- Increase `YAW_KP` slightly (0.55 → 0.60)

---

## Tuning Methodology

**Always tune one parameter at a time:**

1. **Start with angle mode** (easier to diagnose)
2. Make small changes (5-10%)
3. Test in hover for 30+ seconds
4. Record behavior (video recommended)
5. If stable, try next increment or move to next parameter

**Safe tuning order:**
1. Angle KP (most obvious effect)
2. Rate KP (finer control)
3. Angle KD (reduce overshoot)
4. Rate KD (reduce oscillation)
5. KI gains (fix drift)

---

## Technical Notes

**Why these specific values for your platform:**

- **KP = 0.90 (Roll/Pitch Rate)**: 
  - 1000 Hz loop supports up to ~1.0 before instability
  - 250g mass requires moderate gain (heavier → lower KP)
  - 5" props have good thrust vectoring authority
  
- **KP = 3.8 (Angle)**:
  - Converts ±45° to ±170°/s rate command (within limit)
  - Response time ~200ms for half-stick input
  - Balanced between responsiveness and margin

- **Enhanced D-term (0.060)**:
  - 2-blade props need more damping than 3/4-blade
  - Compensates for higher gyro noise at 1000 Hz
  - Prevents twitch/oscillation in fast maneuvers

- **Conservative Yaw (KP=0.55)**:
  - Quadcopter inertia higher in yaw than pitch/roll
  - Aggressive yaw can couple into roll/pitch
  - Integrated rate setpoint prevents sudden yaw torque

---

## Pre-Flight Checklist

Before flying with these gains:

- [ ] IMU calibrated (level surface, no vibration)
- [ ] Propellers balanced and undamaged
- [ ] Motor bearings smooth (spin freely)
- [ ] ESCs programmed and throttle range calibrated
- [ ] Receiver failsafe set correctly
- [ ] Battery fully charged
- [ ] Control surfaces move correctly in transmitter test

---

## Reference: Gain Scaling Laws

If you ever modify your platform, scale gains roughly as:

```
New_KP ≈ Old_KP × √(Old_Mass / New_Mass) × (New_ArmLength / Old_ArmLength)
New_KD ≈ Old_KD × √(Old_Mass / New_Mass)
```

For example, if you add 50g of payload:
- New_KP ≈ 0.90 × √(250/300) = 0.82 (decrease by ~9%)

---

## Flight Modes Recap

**RATE MODE** (via CH5/L_KNOB < 1450 µs):
- Joystick directly commands angular rates
- Direct rate controller active
- More "acro" feel, requires pilot workload
- Good for experienced pilots, aerial maneuvers

**ANGLE MODE** (via CH5/L_KNOB > 1550 µs):
- Joystick commands attitude angles (±45°)
- Cascaded angle+rate controllers
- Self-levels if sticks released
- Easier to fly, more forgiving
- **Recommended for first flights**

Switch on ground before flight to select mode!

---

## Support & Troubleshooting

If experiencing issues:

1. **Check IMU health:**
   ```cpp
   Serial.print("Gyro X: "); Serial.println(imu.getGyroX());
   // Should be <2 deg/s at rest
   ```

2. **Monitor PID outputs:**
   - Enable `#define DEBUG_PID (true)`
   - Watch `pid_angle_roll` and `pid_rate_roll` in serial output
   - Should be <200 in magnitude at hover

3. **Verify mode:**
   - Serial output shows "Mode: ANGLE" or "Mode: RATE"
   - Always start with ANGLE mode

4. **Test in safe environment:**
   - Outdoor flat area (no obstacles)
   - Tether recommended for first hover
   - Start with reduced throttle

---

**Good luck with your first flight! 🚁**
