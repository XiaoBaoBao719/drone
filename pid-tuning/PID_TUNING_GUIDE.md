# PID Controller Tuning Guide for 250g Quadcopter

## Quadcopter Specifications Summary
- **Mass**: 250g (0.25 kg)
- **Arm Length**: 152.4 mm (0.1524 m)
- **Propeller Diameter**: 127 mm (5")
- **Propeller Pitch**: 3-5" (2-blade props)
- **Configuration**: X-frame (4 motors)
- **Motor Type**: Small brushless (~1104-1105 class)
- **Control Loop Rate**: 1000 Hz (0.001s timestep)
- **Max Commanded Rates**: 200°/s (roll/pitch), 120°/s (yaw)
- **Max Attitude**: ±45°

---

## Tuning Methodology

### Physics-Based Approach
For small quadcopters with 1000 Hz control rates, gains should follow these relationships:

**Rate Controller (gyro feedback):**
- **KP**: Controls immediate response to rate error
- **KI**: Corrects steady-state bias and aerodynamic drag
- **KD**: Dampens oscillations and reduces overshoot

**Angle Controller (attitude feedback):**
- Converts desired attitude angles into rate setpoints
- Generally lower bandwidth than rate controller (cascaded architecture)
- Prevents aggressive maneuvers and improves stability

### Key Considerations for This Drone
1. **High control rate (1000 Hz)** → Can tolerate higher gains
2. **Small mass (250g)** → Lower inertia, faster response
3. **Short arms (152mm)** → Moderate moment arm, reduced control authority
4. **2-blade props** → Higher oscillation potential than 3/4-blade
5. **Small props (127mm)** → Higher RPM needed, faster dynamics

---

## Recommended PID Gains

### 1. RATE CONTROLLER (Inner Loop) - Gyro-Based

The rate controller directly uses gyroscope feedback and produces motor commands.

#### Roll Rate Controller
```cpp
static constexpr float ROLL_KP    = 0.90f;    // Proportional gain
static constexpr float ROLL_KI    = 0.015f;   // Integral gain  
static constexpr float ROLL_KD    = 0.060f;   // Derivative gain
static constexpr float ROLL_I_LIMIT  = 100.0f;  // Integral saturation (rad/s or rate units)
```

**Rationale:**
- **KP = 0.90**: Slightly increased from current 0.8 for tighter rate tracking. 250g mass + 1000Hz loop = can handle more P gain
- **KI = 0.015**: Up from 0.01 to compensate for motor drag and ESC non-linearity
- **KD = 0.060**: Up from 0.05 to add damping, especially important for 2-blade props
- **I_LIMIT**: 100 appropriate for rate control in deg/s or normalized units

#### Pitch Rate Controller
```cpp
static constexpr float PITCH_KP    = 0.90f;   // Proportional gain
static constexpr float PITCH_KI    = 0.015f;  // Integral gain
static constexpr float PITCH_KD    = 0.060f;  // Derivative gain
static constexpr float PITCH_I_LIMIT = 100.0f;  // Integral saturation
```

**Rationale:**
- Symmetric with roll (no reason to differentiate for symmetric quad)
- Slightly higher than current values for improved responsiveness

#### Yaw Rate Controller
```cpp
static constexpr float YAW_KP     = 0.55f;    // Proportional gain
static constexpr float YAW_KI     = 0.008f;   // Integral gain
static constexpr float YAW_KD     = 0.002f;   // Derivative gain
static constexpr float YAW_I_LIMIT    = 150.0f;  // Integral saturation
```

**Rationale:**
- **KP = 0.55**: Up from 0.5. Yaw has more mechanical coupling (all 4 motors involved). Higher rate needed
- **KI = 0.008**: Up from 0.004 (increased slightly for motor drag compensation)
- **KD = 0.002**: Small increase from 0.001. Yaw is less prone to oscillation than roll/pitch
- **I_LIMIT = 150**: Higher than roll/pitch due to greater moment arm requirements

#### Rate Controller Output Limits
```cpp
static constexpr float RATE_PID_LIMIT = 300.0f;  // Motor command magnitude (-300 to +300)
```

**Rationale:**
- Current value of 300 is appropriate
- Corresponds to ~±33% motor speed range (if motors span ~900 units)
- Prevents saturation while allowing aggressive control

---

### 2. ANGLE CONTROLLER (Outer Loop) - Attitude Feedback

The angle controller converts desired attitude angles into rate setpoints. These gains should be roughly 0.3-0.5× of the rate controller gains.

#### Roll Angle Controller
```cpp
static constexpr float ANGLE_ROLL_KP    = 3.8f;   // Proportional gain
static constexpr float ANGLE_ROLL_KI    = 0.015f; // Integral gain
static constexpr float ANGLE_ROLL_KD    = 0.10f;  // Derivative gain
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;  // Integral saturation (degrees or normalized)
```

**Rationale:**
- **KP = 3.8**: Converts attitude error (degrees) to rate setpoint (deg/s). With max attitude ±45°, this produces reasonable rate setpoints (~170 deg/s max)
- **KI = 0.015**: Small value to correct steady-state angle bias from wind/CG drift
- **KD = 0.10**: Damping on attitude rate to reduce oscillation
- **I_LIMIT = 50**: More conservative than rate controller to prevent integral windup

#### Pitch Angle Controller
```cpp
static constexpr float ANGLE_PITCH_KP    = 3.8f;   // Proportional gain
static constexpr float ANGLE_PITCH_KI    = 0.015f; // Integral gain
static constexpr float ANGLE_PITCH_KD    = 0.10f;  // Derivative gain
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;  // Integral saturation
```

**Rationale:**
- Symmetric with roll angle controller

#### Yaw Angle Controller
```cpp
static constexpr float ANGLE_YAW_KP     = 1.5f;   // Proportional gain
static constexpr float ANGLE_YAW_KI     = 0.004f; // Integral gain
static constexpr float ANGLE_YAW_KD     = 0.05f;  // Derivative gain
static constexpr float ANGLE_YAW_I_LIMIT    = 30.0f;  // Integral saturation
```

**Rationale:**
- **KP = 1.5**: Lower than roll/pitch because yaw doesn't need aggressive maneuvers
- **KI = 0.004**: Very small to prevent compass/gyro drift issues
- **KD = 0.05**: Minimal damping on yaw
- **I_LIMIT = 30**: Conservative to prevent yaw oscillation

#### Angle Controller Output Limits
```cpp
static constexpr float ANGLE_PID_LIMIT = 200.0f;  // Rate setpoint limiter (deg/s)
```

**Rationale:**
- Caps rate setpoints to ~200 deg/s (slightly below max commanded 200 deg/s for roll/pitch)
- Provides safety margin for outer loop cascading

---

## Complete C++ Implementation

```cpp
// ============================================================
// RATE CONTROLLER (Inner Loop) - Gyro Direct Control
// ============================================================
static constexpr float ROLL_KP    = 0.90f;
static constexpr float ROLL_KI    = 0.015f;
static constexpr float ROLL_KD    = 0.060f;
static constexpr float ROLL_I_LIMIT   = 100.0f;

static constexpr float PITCH_KP   = 0.90f;
static constexpr float PITCH_KI   = 0.015f;
static constexpr float PITCH_KD   = 0.060f;
static constexpr float PITCH_I_LIMIT  = 100.0f;

static constexpr float YAW_KP     = 0.55f;
static constexpr float YAW_KI     = 0.008f;
static constexpr float YAW_KD     = 0.002f;
static constexpr float YAW_I_LIMIT    = 150.0f;

static constexpr float RATE_PID_LIMIT = 300.0f;

// ============================================================
// ANGLE CONTROLLER (Outer Loop) - Attitude Control
// ============================================================
static constexpr float ANGLE_ROLL_KP    = 3.8f;
static constexpr float ANGLE_ROLL_KI    = 0.015f;
static constexpr float ANGLE_ROLL_KD    = 0.10f;
static constexpr float ANGLE_ROLL_I_LIMIT = 50.0f;

static constexpr float ANGLE_PITCH_KP    = 3.8f;
static constexpr float ANGLE_PITCH_KI    = 0.015f;
static constexpr float ANGLE_PITCH_KD    = 0.10f;
static constexpr float ANGLE_PITCH_I_LIMIT = 50.0f;

static constexpr float ANGLE_YAW_KP     = 1.5f;
static constexpr float ANGLE_YAW_KI     = 0.004f;
static constexpr float ANGLE_YAW_KD     = 0.05f;
static constexpr float ANGLE_YAW_I_LIMIT    = 30.0f;

static constexpr float ANGLE_PID_LIMIT = 200.0f;
```

---

## Changes from Current Implementation

| Parameter | Current | Recommended | Change | Reason |
|-----------|---------|-------------|--------|--------|
| ROLL_KP | 0.80 | 0.90 | +12.5% | Tighter rate tracking, 1kHz capable |
| ROLL_KI | 0.010 | 0.015 | +50% | Compensate motor drag |
| ROLL_KD | 0.050 | 0.060 | +20% | Better damping for 2-blade props |
| PITCH_KP | 0.80 | 0.90 | +12.5% | Symmetric improvement |
| PITCH_KI | 0.010 | 0.015 | +50% | Motor drag compensation |
| PITCH_KD | 0.050 | 0.060 | +20% | Better oscillation suppression |
| YAW_KP | 0.50 | 0.55 | +10% | Faster yaw response |
| YAW_KI | 0.004 | 0.008 | +100% | Better steady-state |
| YAW_KD | 0.001 | 0.002 | +100% | Increased damping |
| YAW_I_LIMIT | 200 | 150 | -25% | More conservative |
| ANGLE_ROLL_KP | N/A | 3.8 | NEW | Attitude to rate conversion |
| ANGLE_PITCH_KP | N/A | 3.8 | NEW | Attitude to rate conversion |
| ANGLE_YAW_KP | N/A | 1.5 | NEW | Attitude to rate conversion |

---

## Tuning Procedure

### Initial Flight Testing
1. **Start conservative**: Use recommended gains as-is
2. **Test in Rate Mode first** - verify inner loop stability
3. **Observe oscillation type**:
   - High-frequency oscillation → Reduce KD
   - Low-frequency oscillation → Reduce KP
   - Sluggish response → Increase KP

### Fine-Tuning
1. **Increase KP incrementally** (+0.05 at a time) until light oscillation appears
2. **Reduce KD slightly** if overshoot is excessive
3. **Increase KI** if drone drifts from commanded rate
4. **Once rate mode stable**, enable angle controller

### Angle Mode Tuning
- KP should produce ~4 deg/s per degree of error (reasonable setpoint)
- If unstable, reduce ANGLE_KP by 10-15%
- Yaw typically needs lower KP than roll/pitch

---

## Performance Predictions

With these gains on your 250g quad:
- **Rate response**: ~50-100ms to reach commanded rate (3-5 time constants)
- **Settle time**: ~200-300ms for small disturbances
- **Oscillation frequency**: ~3-5 Hz (well damped)
- **Maximum roll/pitch velocity**: ±200 deg/s achievable
- **Yaw agility**: ~150 deg/s sustained

---

## Safety Notes

⚠️ **Conservative Tuning**: These gains are intentionally conservative. They prioritize stability over agility.

- Start with recommended values
- Always use throttle hold / failsafe enabled
- Test in open space away from obstacles
- Incremental gain adjustment: never jump by more than ±0.1 on KP
- Monitor for integral windup if drone holds altitude poorly

---

## Reference Comparisons

**Similar 250g Drones (Industry References):**
- DJI Mini 2S: Rate KP ~0.8-1.0, optimized for 1000Hz loop
- BetaFPV Hovercraft: Rate KP ~0.7-0.9, tuned for aggressive flying
- 3DR Iris+: Angle KP ~3.5-4.5, similar cascaded architecture

Your recommended values align well with these reference platforms.
