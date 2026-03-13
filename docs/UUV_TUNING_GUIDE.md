# UUV Controller Tuning Guide

This guide explains how the Reconbot UUV control system works and how to tune it. Read this before changing parameters.

---

## How the Vehicle Moves

The vehicle has two controllers running in series:

```
Joystick / Mission
       |
       v
 Position Controller    "Where should I be? How fast should I move?"
       |                 Outputs: thrust in body frame [surge, sway, heave]
       v
 Attitude Controller    "Which way should I point?"
       |                 Outputs: torque [roll, pitch, yaw] + thrust passthrough
       v
 Control Allocator      "Which motors do I spin?"
       |                 Maps torque + thrust to 8 motor commands
       v
    Thrusters
```

**Position controller** handles translation (forward/back, left/right, up/down).
**Attitude controller** handles rotation (roll, pitch, yaw).

They are independent — position computes thrust, attitude computes torque. The control allocator combines both and sends commands to motors.

---

## The Control Law

### Position Controller

For each axis (X, Y, Z):

```
thrust = P * position_error - D * velocity_error + I * accumulated_error + buoyancy
```

| Term | What it does | Too low | Too high |
|------|-------------|---------|----------|
| **P** (proportional) | Drives toward target | Sluggish, large steady-state error | Overshoot, oscillation |
| **D** (derivative) | Brakes before reaching target | Overshoot | Jerky response, amplifies sensor noise |
| **I** (integral) | Eliminates steady-state error | Drifts in current | Overshoot after disturbance, slow to recover |
| **Buoyancy** | Constant offset for neutral buoyancy | Vehicle sinks | Vehicle rises |

**Key relationship: P:D ratio**

| Ratio | Behavior |
|-------|----------|
| 10:1 | Underdamped — drives hard, brakes late, overshoots |
| 4:1 | Well-damped — responsive with controlled stopping |
| 2:1 | Overdamped — slow, no overshoot, sluggish feel |

Current tuning targets 4:1 for all axes.

### Attitude Controller

For each axis (roll, pitch, yaw):

```
torque = -P * attitude_error - D * angular_velocity + yaw_integral + feedforward
```

Same P/D logic as position, but operating on angles instead of distances.

The **yaw integrator** eliminates steady-state heading error from thruster asymmetry and reaction torques.

The **feedforward** terms compensate for known coupling:
- Surge thrust causes pitch (thrusters not at CG)
- Surge thrust causes yaw (thruster asymmetry)
- Heave thrust causes yaw (canted thruster reaction torque)

---

## Parameter Reference

### Position Controller — Primary Tuning

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_GAIN_X_P` | 2.0 | Surge (forward/back) position stiffness |
| `UUV_GAIN_X_D` | 0.5 | Surge velocity damping |
| `UUV_GAIN_Y_P` | 1.5 | Sway (left/right) position stiffness |
| `UUV_GAIN_Y_D` | 0.4 | Sway velocity damping |
| `UUV_GAIN_Z_P` | 2.0 | Heave (up/down) position stiffness |
| `UUV_GAIN_Z_D` | 0.5 | Heave velocity damping |

### Position Controller — Integrators

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `POSE_XY_I_EN` | 1 | Enable XY integrator (0=off, 1=on) |
| `POSE_KI_XY` | 0.3 | XY integrator gain — how fast it builds correction |
| `POSE_I_MAX_XY` | 0.2 | XY integrator max output — limits correction magnitude |
| `POSE_Z_I_ENABLE` | 1 | Enable Z integrator |
| `POSE_KI_Z` | 0.3 | Z integrator gain |
| `POSE_I_MAX_Z` | 0.2 | Z integrator max output |

### Position Controller — Other

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_BUOY_COMP` | -0.075 | Buoyancy feedforward (negative = upward in NED) |
| `DVL_ALT_MAX_AGE` | 1.0 | Max age (seconds) of DVL data before fallback to EKF |

### Attitude Controller — Primary Tuning

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_ROLL_P` | 6.0 | Roll stiffness |
| `UUV_ROLL_D` | 1.0 | Roll damping |
| `UUV_PITCH_P` | 8.0 | Pitch stiffness |
| `UUV_PITCH_D` | 2.5 | Pitch damping |
| `UUV_YAW_P` | 8.0 | Yaw stiffness |
| `UUV_YAW_D` | 2.5 | Yaw damping |

### Attitude Controller — Yaw Integrator

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_YAW_I` | 0.5 | Yaw integrator gain |
| `UUV_YAW_I_MAX` | 0.2 | Yaw integrator max output |

### Attitude Controller — Feedforward

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_SURGE_PFF` | 0.0 | Surge-to-pitch coupling compensation |
| `UUV_SURGE_YFF` | 0.0 | Surge-to-yaw coupling compensation |
| `UUV_HEAVE_YFF` | 0.1 | Heave-to-yaw coupling compensation |

### Saturation Limits

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `UUV_THRUST_SAT` | 1.0 | Max thrust output per axis (normalized) |
| `UUV_TORQUE_SAT` | 1.0 | Max torque output per axis (normalized) |

---

## Tuning Procedures

### Step 1: Get Buoyancy Right First

Before tuning anything else:

1. Put the vehicle in the water in manual mode
2. Command zero thrust on all axes
3. Observe if the vehicle sinks or rises
4. Adjust `UUV_BUOY_COMP` until the vehicle holds depth with zero input
   - Vehicle sinks → make more negative (more upward thrust)
   - Vehicle rises → make more positive (less upward thrust)

**Why first:** If buoyancy comp is wrong, the Z integrator wastes its budget fighting a constant force. Every other Z tuning will be wrong.

### Step 2: Tune Attitude Controller

Attitude must be stable before position control works. Test in attitude/stabilized mode (no position hold):

1. Start with current defaults
2. Command a yaw step. Watch for:
   - **Overshoot** → increase `UUV_YAW_D` or decrease `UUV_YAW_P`
   - **Sluggish** → increase `UUV_YAW_P`
   - **Oscillation** → decrease `UUV_YAW_P` and increase `UUV_YAW_D`
   - **Steady-state error** → increase `UUV_YAW_I` (small increments of 0.1)
3. Repeat for pitch and roll

### Step 3: Tune Position Controller

Test in position hold mode:

1. **Start with D gains only** — temporarily set all P gains to 0 and integrators off. The vehicle should resist being pushed (damping only). If it oscillates, D is too high or velocity data is noisy.

2. **Add P gains** — increase from 0 in steps of 0.5. The vehicle should return to position when pushed. Find the point where it returns without overshooting.

3. **Check P:D ratio** — aim for 4:1. If overshooting, increase D. If sluggish, increase P.

4. **Enable integrators** — turn on `POSE_XY_I_EN` and `POSE_Z_I_ENABLE`. Start with low gains (`KI = 0.1`). The vehicle should eliminate drift in current. If it overshoots after holding against current then releasing, reduce `I_MAX`.

### Step 4: Tune Feedforward (if needed)

Only tune these if you see coupling:

1. **Surge-yaw coupling:** Command forward thrust with heading hold. If the vehicle yaws, adjust `UUV_SURGE_YFF`. Positive or negative depends on direction — flip sign if yaw gets worse.

2. **Heave-yaw coupling:** Command vertical thrust with heading hold. If the vehicle yaws, adjust `UUV_HEAVE_YFF`.

3. **Surge-pitch coupling:** Command forward thrust with pitch hold. If the vehicle pitches, adjust `UUV_SURGE_PFF`. Currently disabled (0.0) because the control allocator handles this through the effectiveness matrix.

---

## Troubleshooting

### Vehicle overshoots target position
- **Most likely:** D gain too low (underdamped). Increase D to get P:D ratio closer to 4:1
- **Also check:** Integrator windup — reduce `I_MAX` values
- **Less likely:** Thruster response lag — thrusters can't decelerate fast enough

### Vehicle drifts in current
- **Most likely:** Integrators disabled or gains too low. Enable and increase `KI`
- **Also check:** Is the position estimate drifting? Check DVL lock and EKF health

### Vehicle oscillates in position hold
- **Most likely:** P gain too high relative to D, or D gain amplifying noisy velocity data
- **Also check:** DVL velocity noise — if DVL has poor beam lock, D-term becomes noisy
- **Try:** Reduce P, or reduce D if you suspect sensor noise

### Vehicle oscillates in attitude
- **Most likely:** Attitude P gain too high relative to D
- **Also check:** Gyro noise, external angular velocity source quality

### Depth drifts slowly
- **Most likely:** `UUV_BUOY_COMP` is wrong — redo buoyancy calibration
- **Also check:** Z integrator enabled? `POSE_Z_I_ENABLE = 1`
- **Also check:** Depth sensor working? Check pressure sensor data

### Yaw drifts slowly
- **Most likely:** `UUV_YAW_I` too low to overcome persistent torque bias
- **Also check:** Heading source quality — is INS heading stable?

### Vehicle jerks or vibrates
- **Most likely:** D gains too high — amplifying sensor noise
- **Also check:** Thruster dead zone — small commands produce no thrust, then controller ramps up and thrust kicks in suddenly

### Vehicle can't hold position in strong current
- **Not a tuning problem** — thrusters aren't strong enough. The position controller is already at max thrust (saturated). Options:
  - Accept the drift
  - Reduce operating area to calmer water
  - Use more powerful thrusters

---

## Sensor Dependencies

| Sensor | What uses it | If it fails |
|--------|-------------|-------------|
| **DVL velocity** | Position controller D-term, velocity feedforward | Falls back to EKF velocity (drifts fast) |
| **DVL altitude** | Position controller Z measurement | Falls back to EKF Z (pressure/depth sensor) |
| **INS quaternion** | Attitude controller setpoint, body/world rotation | No heading — position control breaks |
| **INS angular velocity** | Attitude controller D-term | Falls back to PX4 gyro |
| **Depth sensor** | EKF2 height reference | Z estimate drifts |
| **PX4 IMU** | EKF2 prediction step | Everything breaks |

**Rule of thumb:** If the vehicle behaves badly, check sensors before changing gains. Bad sensor data makes good gains look wrong.

---

## Quick Reference Card

```
POSITION OVERSHOOTS     → increase D gain (or decrease P)
POSITION SLUGGISH       → increase P gain
POSITION DRIFTS         → enable/increase integrator
DEPTH DRIFTS            → fix UUV_BUOY_COMP first, then Z integrator
ATTITUDE OSCILLATES     → decrease attitude P or increase attitude D
YAW DRIFTS              → increase UUV_YAW_I
SURGE CAUSES YAW        → tune UUV_SURGE_YFF
HEAVE CAUSES YAW        → tune UUV_HEAVE_YFF
JERKY MOTION            → D gains too high, or thruster dead zone
CAN'T HOLD IN CURRENT   → thruster limit, not a tuning problem
```
