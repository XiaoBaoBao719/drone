# PID Gains Quick Reference - 250g Quadcopter

## At a Glance

```cpp
// RATE CONTROLLER (Inner Loop) - Direct Gyro Feedback
// Used in RATE_MODE - produces motor commands
ROLL_KP  = 0.90    ROLL_KI  = 0.015   ROLL_KD  = 0.060   ROLL_I_LIMIT  = 100
PITCH_KP = 0.90    PITCH_KI = 0.015   PITCH_KD = 0.060   PITCH_I_LIMIT = 100
YAW_KP   = 0.55    YAW_KI   = 0.008   YAW_KD   = 0.002   YAW_I_LIMIT   = 150
RATE_PID_LIMIT = 300 (±300 motor command units)

// ANGLE CONTROLLER (Outer Loop) - Attitude Feedback  
// Used in ANGLE_MODE - produces rate setpoints
ANGLE_ROLL_KP  = 3.8    ANGLE_ROLL_KI  = 0.015  ANGLE_ROLL_KD  = 0.10  ANGLE_ROLL_I_LIMIT  = 50
ANGLE_PITCH_KP = 3.8    ANGLE_PITCH_KI = 0.015  ANGLE_PITCH_KD = 0.10  ANGLE_PITCH_I_LIMIT = 50
ANGLE_YAW_KP   = 1.5    ANGLE_YAW_KI   = 0.004  ANGLE_YAW_KD   = 0.05  ANGLE_YAW_I_LIMIT   = 30
ANGLE_PID_LIMIT = 200 (±200 deg/s rate setpoint)
```

---

## Change Summary

| Axis | Current | Recommended | Change |
|------|---------|-------------|--------|
| **RATE ROLL** | 0.80/0.01/0.05 | 0.90/0.015/0.060 | +12.5% KP, +50% KI, +20% KD |
| **RATE PITCH** | 0.80/0.01/0.05 | 0.90/0.015/0.060 | +12.5% KP, +50% KI, +20% KD |
| **RATE YAW** | 0.50/0.004/0.001 | 0.55/0.008/0.002 | +10% KP, +100% KI, +100% KD |
| **ANGLE ROLL** | Not defined | 3.8/0.015/0.10 | NEW |
| **ANGLE PITCH** | Not defined | 3.8/0.015/0.10 | NEW |
| **ANGLE YAW** | Not defined | 1.5/0.004/0.05 | NEW |

---

## Why These Values

### Rate Controller (Why Higher Than Current)

✅ **1000 Hz control loop** = Can handle higher gains (less aliasing)
✅ **250g mass** = Low inertia, responds quickly  
✅ **127mm props** = Higher RPM, faster dynamics
✅ **2-blade props** = Need damping (higher KD)
✅ **1104-1105 motors** = Good torque response at 1000 Hz

### Angle Controller (Why These Specific Values)

✅ **KP = 3.8** → Converts 45° error to ~190 deg/s rate command
✅ **KI = 0.015** → Corrects steady-state angle drift
✅ **KD = 0.10** → Gentle damping on attitude rate
✅ **Cascaded** → Inner loop stabilizes before outer loop acts

---

## Performance Expectations

| Metric | Value | Notes |
|--------|-------|-------|
| **Rate Settling Time** | 100-150 ms | Time to reach commanded gyro rate |
| **Angle Settling Time** | 250-350 ms | Time to reach commanded attitude |
| **Oscillation Frequency** | 3-5 Hz | Low frequency = well damped |
| **Overshoot** | <20% | Conservative tuning |
| **Steady-State Error** | <1% | Good error rejection |
| **Response to Max Rate** (200°/s) | <0.3 deg/s error | Tight tracking |

---

## How to Use (3 Steps)

### Step 1: Copy to Your Code
Replace lines 161-204 in `RateModeR4.ino` with recommended values
(OR copy from `pid_gains_250g_quadcopter.h`)

### Step 2: Test Rate Mode First  
- Arm drone in open space
- Switch to RATE_MODE
- Give small stick inputs (30-50%)
- Listen for: Smooth response, no oscillation
- Watch telemetry: No drift, stable rates

### Step 3: Test Angle Mode Second
- Switch to ANGLE_MODE  
- Give small stick inputs (20-30%)
- Observe: ~1 second to reach new attitude
- If oscillates: Reduce ANGLE_KP by 0.2
- If drifts: Increase ANGLE_KI by 0.002

---

## Troubleshooting

### If oscillating in rate mode:
1. **High frequency (~5-10 Hz)** → Reduce KD by 0.01
2. **Low frequency (~1-3 Hz)** → Reduce KP by 0.05
3. **Ringing after step** → Increase KI by 0.005

### If drifting in angle mode:
1. **Rolls to one side** → Increase ANGLE_KI by 0.005
2. **Overshoots attitude** → Reduce ANGLE_KP by 0.2
3. **Slow to respond** → Increase ANGLE_KP by 0.1

### If unstable (diverging):
1. **STOP IMMEDIATELY** - Disarm and debug
2. Reduce all KP values by 20%
3. Increase all KD values by 20%
4. Test again in open space

---

## Tuning Order of Importance

1. **KP** (proportional) - Primary stability
2. **KD** (derivative) - Damping/oscillation
3. **KI** (integral) - Steady-state accuracy
4. Limits - Output saturation protection

*Tune in this order for best results*

---

## Reference Data

**Quadcopter Specifications:**
- Mass: 250g (0.25 kg)
- Arm: 152.4mm (0.1524m)
- Propeller: 127mm diameter, 3-5" pitch
- Motors: ~1104-1105 class brushless
- ESC: Typical PWM range 1000-2000 µs
- Control: 1000 Hz (1ms timestep)
- Max rates: ±200°/s roll/pitch, ±120°/s yaw
- Max attitude: ±45°

**Estimated Inertia:**
- Roll/Pitch: ~7.2 × 10⁻⁴ kg·m²
- Yaw: ~1.44 × 10⁻³ kg·m²

**Motor Torque (Estimated):**
- At 50% throttle: ~0.05-0.08 N·m per motor
- Differential control torque: ~0.05-0.1 N·m

---

## Implementation Checklist

- [ ] Copy recommended gains to code
- [ ] Compile and upload to ESP32
- [ ] Test rate mode with gentle inputs
- [ ] Test rate mode with aggressive inputs  
- [ ] Test angle mode with gentle inputs
- [ ] Test angle mode with aggressive inputs
- [ ] Monitor telemetry for oscillation
- [ ] Log flight data for analysis
- [ ] Fine-tune ±5-10% if needed
- [ ] Document final values used

---

## Further Reading

For detailed math and physics:
- See `PID_TUNING_MATHEMATICS.md`
- See `PID_TUNING_GUIDE.md`

For standard references:
- Beard & McLain - "Small Unmanned Aircraft: Theory and Practice"
- Ellington - "The Aerodynamics of Hovering Insect Flight"
- Beard et al. - "Cooperative Control of Multi-Agent Systems"

---

## Pro Tips

1. **Always test in Rate Mode first** - Inner loop must be stable
2. **Use telemetry logs** - Track KP effect vs KD effect
3. **Incremental changes** - Never jump gains by >10% at once
4. **Cold weather** → Air denser → Motors have more torque → Can slightly reduce KP
5. **Hot weather** → Air thinner → Motors have less torque → Might need higher KP
6. **Loaded drone** → Heavier than 250g → Reduce all KP by 5-10%
7. **Propeller wear** → Worn props less efficient → Might need higher KI

---

## Support

If you experience issues:

1. **Check physical aircraft first**
   - Are props balanced?
   - Are bearings smooth?
   - Are motors spinning free?

2. **Check sensors**
   - Gyro calibrated?
   - Accelerometer level?
   - No vibration?

3. **Incremental tuning**
   - Reduce all K values by 20%
   - Test again
   - Slowly increase back

4. **Log data**
   - Record gyro rates, setpoints, PID outputs
   - Look for patterns in oscillation
   - Identify which axis is problematic

5. **Community**
   - Post logs + video to forums
   - Include drone specs
   - Share troubleshooting steps taken
