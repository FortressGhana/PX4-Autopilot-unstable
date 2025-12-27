# PX4-Autopilot UUV (Underwater Vehicle) Architecture

A comprehensive guide to the PX4 UUV control flow, modules, and architecture for the **Reconbot** underwater vehicle.

---

## Table of Contents

1. [UUV Overview](#1-uuv-overview)
2. [Reconbot Configuration](#2-reconbot-configuration)
3. [Directory Structure](#3-directory-structure)
4. [Control Flow Architecture](#4-control-flow-architecture)
5. [Core UUV Modules](#5-core-uuv-modules)
6. [Actuator Effectiveness & Allocation](#6-actuator-effectiveness--allocation)
7. [Sensor Integration (DVL)](#7-sensor-integration-dvl)
8. [Initialization Sequence](#8-initialization-sequence)
9. [Parameters Reference](#9-parameters-reference)
10. [UUV vs Other Vehicle Types](#10-uuv-vs-other-vehicle-types)
11. [Key File Paths](#11-key-file-paths)

---

## 1. UUV Overview

PX4 provides full 6-DOF (Degrees of Freedom) control for Unmanned Underwater Vehicles.

**Reconbot** is an 8-thruster vectored UUV with:
- **Full 6-DOF control** (X, Y, Z translation + Roll, Pitch, Yaw rotation)
- **Diagonal vectored thrusters** for efficient multi-axis thrust
- **Depth/altitude hold** via DVL or pressure sensors
- **Position control** in world or body frame

**Airframe ID:** `60003`
**MAVLink Vehicle Type:** `12` (SUBMARINE)
**Maintainer:** Nathaniel Asiak

---

## 2. Reconbot Configuration

### Motor Layout

The Reconbot uses 8 thrusters in a vectored configuration for full 6-DOF control:

```
                    FRONT (BOW)
                        ↑
              ┌─────────────────────┐
              │                     │
        M2 ↗  │   M5 ←      → M6   │  ↖ M1
      (diagonal)  (forward)  (forward)  (diagonal)
              │                     │
              │       M7  M8        │
              │        ↑   ↑        │
              │      (lateral)      │
              │                     │
        M4 ↘  │                     │  ↙ M3
      (diagonal)                      (diagonal)
              └─────────────────────┘
                    REAR (STERN)

Motor Functions:
  M1-M4: Diagonal thrusters (Y-Z vectored at 45°) - Heave, Roll, Pitch
  M5-M6: Forward thrusters (X-axis) - Surge
  M7-M8: Lateral thrusters (Y-axis) - Sway
```

### Motor Specifications

| Motor | Position (X,Y,Z) | Thrust Axis (AX,AY,AZ) | Function |
|-------|------------------|------------------------|----------|
| M1 | (-0.424, 0.134, -0.117) | (0, -0.707, 0.707) | Bow starboard diagonal |
| M2 | (-0.424, -0.134, -0.117) | (0, 0.707, 0.707) | Bow port diagonal |
| M3 | (0.462, -0.134, -0.117) | (0, 0.707, -0.707) | Stern port diagonal |
| M4 | (0.462, 0.134, -0.117) | (0, -0.707, -0.707) | Stern starboard diagonal |
| M5 | (-0.169, -0.139, -0.099) | (1, 0, 0) | Port forward |
| M6 | (-0.169, 0.139, 0.099) | (1, 0, 0) | Starboard forward |
| M7 | (0.306, 0.088, 0.160) | (0, 1, 0) | Stern lateral 1 |
| M8 | (0.270, 0.088, 0.160) | (0, 1, 0) | Stern lateral 2 |

### PWM Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| PWM_MAIN_MIN | 253 | Minimum PWM (full reverse) |
| PWM_MAIN_MAX | 2275 | Maximum PWM (full forward) |
| PWM_MAIN_DIS | 1250 | Disarm/neutral (zero thrust) |
| CA_R_REV | 255 | All motors reversible |

### Battery Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| BAT1_N_CELLS | 4 | 4S LiPo |
| BAT1_CAPACITY | 18000 | 18 Ah capacity |
| BAT1_A_PER_V | 37.8798 | Current sensor scaling |
| BAT1_V_DIV | 11 | Voltage divider ratio |

### AUX Channels

| Channel | Function | Use |
|---------|----------|-----|
| AUX1 | Servo 1 (301) | Camera tilt |
| AUX2 | Servo 2 (302) | Lights/gripper |
| AUX3-8 | Disabled | Available for expansion |

---

## 3. Directory Structure

```
PX4-Autopilot/
├── src/modules/
│   ├── uuv_att_control/          # UUV attitude control
│   │   ├── uuv_att_control.cpp
│   │   ├── uuv_att_control.hpp
│   │   └── uuv_att_control_params.c
│   │
│   ├── uuv_pos_control/          # UUV position control
│   │   ├── uuv_pos_control.cpp
│   │   ├── uuv_pos_control.hpp
│   │   └── uuv_pos_control_params.c
│   │
│   └── control_allocator/
│       └── ActuatorEffectivenessUUV.cpp  # 6-DOF motor mixing
│
├── ROMFS/px4fmu_common/init.d/
│   ├── rc.uuv_defaults           # UUV default parameters
│   ├── rc.uuv_apps               # UUV module startup
│   └── airframes/
│       └── 60003_uuv_reconbot    # ← Reconbot airframe
│
├── msg/versioned/
│   └── DopplerVelocityLog.msg    # DVL sensor message
│
└── boards/px4/fmu-v6x/
    └── uuv.px4board              # UUV-specific board config
```

---

## 4. Control Flow Architecture

### Complete UUV Control Pipeline (Reconbot)

```
┌─────────────────────────────────────────────────────────────────┐
│                        INPUT SOURCES                             │
├─────────────────────────────────────────────────────────────────┤
│  RC Controller ──► manual_control_setpoint                      │
│  Offboard Mode ──► trajectory_setpoint6dof                      │
│  Navigator     ──► position_setpoint_triplet                    │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    UUV POSITION CONTROL                          │
│                   (uuv_pos_control)                              │
├─────────────────────────────────────────────────────────────────┤
│  Inputs:                                                         │
│    • trajectory_setpoint6dof (desired X,Y,Z,Roll,Pitch,Yaw)     │
│    • vehicle_local_position (current position from EKF2)        │
│    • vehicle_attitude (current orientation)                      │
│    • doppler_velocity_log (DVL altitude & velocity)             │
│    • manual_control_setpoint (RC sticks)                        │
│                                                                  │
│  Processing:                                                     │
│    • Position error calculation (world or body frame)           │
│    • P-D control with optional Z-integral                       │
│    • Altitude source selection (DVL vs barometer)               │
│    • Velocity feedforward                                        │
│                                                                  │
│  Output:                                                         │
│    • vehicle_attitude_setpoint (roll, pitch, yaw, thrust)       │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                   UUV ATTITUDE CONTROL                           │
│                   (uuv_att_control)                              │
├─────────────────────────────────────────────────────────────────┤
│  Inputs:                                                         │
│    • vehicle_attitude_setpoint (from position controller)       │
│    • vehicle_rates_setpoint (direct rate commands)              │
│    • vehicle_attitude (current orientation from EKF2)           │
│    • vehicle_angular_velocity (gyro rates)                      │
│    • manual_control_setpoint (RC override)                      │
│    • vehicle_control_mode (mode flags)                          │
│                                                                  │
│  Control Modes:                                                  │
│    • SGM (Stabilization) - Attitude hold                        │
│    • RGM (Rate) - Angular rate control                          │
│    • MGM (Manual) - Direct stick mapping                        │
│                                                                  │
│  Processing:                                                     │
│    • Attitude error → desired angular rates                     │
│    • PD control on angular rates                                │
│    • Torque saturation limiting                                 │
│                                                                  │
│  Outputs:                                                        │
│    • vehicle_thrust_setpoint (X, Y, Z thrust commands)          │
│    • vehicle_torque_setpoint (Roll, Pitch, Yaw torques)         │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                    CONTROL ALLOCATOR                             │
│              (ActuatorEffectivenessUUV)                         │
├─────────────────────────────────────────────────────────────────┤
│  Inputs:                                                         │
│    • vehicle_thrust_setpoint [X, Y, Z]                          │
│    • vehicle_torque_setpoint [Roll, Pitch, Yaw]                 │
│                                                                  │
│  Processing:                                                     │
│    • 6-DOF → Motor thrust mapping                               │
│    • Effectiveness matrix multiplication                        │
│    • Sequential desaturation algorithm                          │
│    • RPY command normalization                                  │
│                                                                  │
│  Output:                                                         │
│    • actuator_motors (individual motor commands 0-1)            │
└───────────────────────────┬─────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────────┐
│                      PWM OUTPUT                                  │
├─────────────────────────────────────────────────────────────────┤
│  • Motor 1-8 PWM signals                                        │
│  • Range: 253 - 2275 µs                                         │
│  • Neutral (disarmed): 1250 µs                                  │
│  • Bidirectional thrust support                                 │
└─────────────────────────────────────────────────────────────────┘
```

### Feedback Loop

```
                    ┌──────────────────┐
                    │   THRUSTERS      │
                    │  (ESCs/Motors)   │
                    └────────┬─────────┘
                             │
                             ▼
                    ┌──────────────────┐
                    │  VEHICLE MOTION  │
                    └────────┬─────────┘
                             │
        ┌────────────────────┼────────────────────┐
        │                    │                    │
        ▼                    ▼                    ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│     IMU       │   │     DVL       │   │   PRESSURE    │
│ (Gyro/Accel)  │   │ (Velocity/Alt)│   │   (Depth)     │
└───────┬───────┘   └───────┬───────┘   └───────┬───────┘
        │                   │                   │
        └───────────────────┼───────────────────┘
                            │
                            ▼
                    ┌──────────────────┐
                    │      EKF2        │
                    │ (State Estimator)│
                    └────────┬─────────┘
                             │
            ┌────────────────┼────────────────┐
            ▼                ▼                ▼
   vehicle_attitude  vehicle_local_pos  vehicle_angular_vel
            │                │                │
            └────────────────┴────────────────┘
                             │
                             ▼
                   Back to Controllers
```

---

## 5. Core UUV Modules

### 5.1 UUV Attitude Control

**Location:** `src/modules/uuv_att_control/`

**Purpose:** Convert attitude/rate setpoints to thrust and torque commands.

```cpp
class UUVAttitudeControl : public ModuleBase<UUVAttitudeControl>, public ModuleParams
{
    // Subscriptions
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Subscription _angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

    // Publications
    uORB::Publication<vehicle_thrust_setpoint_s> _vehicle_thrust_setpoint_pub;
    uORB::Publication<vehicle_torque_setpoint_s> _vehicle_torque_setpoint_pub;
};
```

**Control Modes:**

| Mode | Description | Use Case |
|------|-------------|----------|
| **SGM** (Stabilization) | Holds attitude, RC controls angles | Normal operation |
| **RGM** (Rate) | RC controls angular rates | Acrobatic maneuvers |
| **MGM** (Manual) | Direct passthrough | Emergency/testing |

**Control Algorithm:**
```
attitude_error = desired_attitude - current_attitude
rate_setpoint = attitude_error * P_gain
rate_error = rate_setpoint - current_rate
torque_output = rate_error * D_gain
```

### 5.2 UUV Position Control

**Location:** `src/modules/uuv_pos_control/`

**Purpose:** Convert position setpoints to attitude setpoints (outer loop).

```cpp
class UUVPositionControl : public ModuleBase<UUVPositionControl>, public ModuleParams
{
    // Subscriptions
    uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint6dof)};
    uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription _dvl_sub{ORB_ID(doppler_velocity_log)};

    // Publications
    uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub;
};
```

**Position Control Modes:**

| Parameter | Mode | Description |
|-----------|------|-------------|
| `UUV_POS_MODE=0` | World Frame | Setpoints in NED coordinates |
| `UUV_POS_MODE=1` | Body Frame | Setpoints relative to vehicle |
| `UUV_STAB_MODE=1` | Stabilization | Hold current position |
| `UUV_STAB_MODE=0` | Tracking | Follow trajectory |

**Depth Control Logic:**
```cpp
// Altitude source selection (DVL preferred)
if (dvl_data_fresh && dvl.altitude > 0) {
    current_altitude = -dvl.altitude;  // Convert to NED
    vertical_velocity = dvl.velocity[2];
} else {
    current_altitude = local_position.z;
    vertical_velocity = local_position.vz;
}

// P-D control with optional integral
z_error = z_setpoint - current_altitude;
thrust_z = z_error * P_z + velocity_error * D_z + integral_z * I_z;
```

---

## 6. Actuator Effectiveness & Allocation

### ActuatorEffectivenessUUV

**Location:** `src/modules/control_allocator/ActuatorEffectivenessUUV.cpp`

**Purpose:** Map 6-DOF commands to individual motor thrusts.

**Selection:** Activated when `CA_AIRFRAME = 7` (6DOF Motors)

### Effectiveness Matrix (Reconbot)

The Reconbot uses a unique vectored thrust configuration:

```
Motor Layout (Top View):
                  FRONT
            ┌───────────────┐
      M2 ↗  │  M5 ←   → M6 │  ↖ M1     (M1-M4: 45° diagonal Y-Z)
            │               │          (M5-M6: Forward X-axis)
            │    M7↑  M8↑   │          (M7-M8: Lateral Y-axis)
      M4 ↘  │               │  ↙ M3
            └───────────────┘
                  REAR

Effectiveness Matrix [6 x 8]:
              M1      M2      M3      M4      M5    M6    M7    M8
Thrust X   [  0       0       0       0       1     1     0     0   ]
Thrust Y   [-0.707   0.707   0.707  -0.707   0     0     1     1   ]
Thrust Z   [ 0.707   0.707  -0.707  -0.707   0     0     0     0   ]
Torque Roll[ Pz*Ay  Pz*Ay   Pz*Ay   Pz*Ay    0     0     0     0   ]
Torque Pitch[Pz*Ax  Pz*Ax   Pz*Ax   Pz*Ax    Pz    Pz    0     0   ]
Torque Yaw [ Py*Ax  Py*Ax   Py*Ax   Py*Ax    Py    Py    Px    Px  ]

Where: P = position, A = axis (thrust direction)
```

**Key Insight:** Motors M1-M4 provide coupled Y-Z thrust (diagonal), enabling simultaneous heave and sway from the same motors, while M5-M6 handle surge and M7-M8 handle additional sway control.

### Allocation Algorithm

```
1. Receive thrust/torque setpoints [Fx, Fy, Fz, Tx, Ty, Tz]
2. Normalize RPY commands
3. Multiply by inverse effectiveness matrix
4. Apply sequential desaturation:
   - If any motor saturates, reduce affected axis
   - Prioritize: Z thrust > Attitude > XY thrust
5. Output motor commands [0.0 - 1.0]
```

---

## 7. Sensor Integration (DVL)

### Doppler Velocity Log Message

**Location:** `msg/versioned/DopplerVelocityLog.msg`

```
uint64 timestamp          # Time of measurement

float32 altitude          # Range to seafloor (m)
float32[3] velocity       # Velocity X,Y,Z (m/s)
float32 smg               # Speed made good

bool velocity_valid
uint8 fom                 # Figure of merit (accuracy)
bool water_mode_supported
bool is_water_tracking_enabled

bool[4] transducer_beam_validity
bool is_dvl_beam_valid    # At least 3 of 4 beams valid
bool is_velocity_realistic

float32[3] covariance     # Measurement uncertainty
```

### DVL Usage in Position Control

```cpp
// Check DVL data freshness
bool dvl_fresh = (hrt_absolute_time() - _dvl.timestamp) < DVL_ALT_MAX_AGE;

if (dvl_fresh && _dvl.altitude > 0.0f && _dvl.is_dvl_beam_valid) {
    // Use DVL altitude (convert to NED: positive down)
    _current_altitude = -_dvl.altitude;

    // Use DVL velocity for derivative term
    _vertical_velocity = _dvl.velocity[2];
} else {
    // Fallback to EKF local position
    _current_altitude = _local_pos.z;
    _vertical_velocity = _local_pos.vz;
}
```

---

## 8. Initialization Sequence

### Startup Flow (Reconbot)

```
Board Power-On
      │
      ▼
┌───────────────────────────────────────┐
│  rc.uuv_defaults                      │
│  • Set MAV_TYPE=12 (Submarine)        │
│  • Configure EKF2 for vision          │
│  • Disable GPS, enable DVL            │
│  • Set failsafe parameters            │
└──────────────────┬────────────────────┘
                   │
                   ▼
┌───────────────────────────────────────┐
│  60003_uuv_reconbot                   │
│  • CA_ROTOR_COUNT=8                   │
│  • CA_R_REV=255 (all reversible)      │
│  • Configure 8 motor positions        │
│  • Set vectored thrust axes           │
│  • PWM: 253-2275, neutral 1250        │
│  • Battery: 4S 18Ah                   │
│  • Failsafe actions configured        │
│  • AUX1-2 for servo/lights            │
└──────────────────┬────────────────────┘
                   │
                   ▼
┌───────────────────────────────────────┐
│  rc.uuv_apps                          │
│  • control_allocator start            │
│  • uuv_att_control start              │
│  • uuv_pos_control start              │
│  • land_detector start rover          │
└──────────────────┬────────────────────┘
                   │
                   ▼
┌───────────────────────────────────────┐
│  System Ready                         │
│  (Awaiting arm command)               │
└───────────────────────────────────────┘
```

### rc.uuv_defaults Key Settings

```bash
# Vehicle type
param set-default MAV_TYPE 12

# EKF2 - Vision/Odometry mode (no GPS underwater)
param set-default EKF2_GPS_CTRL 0      # Disable GPS
param set-default EKF2_EV_CTRL 15      # Full vision fusion
param set-default EKF2_EV_DELAY 60     # Vision delay (ms)
param set-default EKF2_EVP_NOISE 0.01  # Position noise
param set-default EKF2_EVV_NOISE 0.01  # Velocity noise
param set-default EKF2_EVA_NOISE 0.05  # Attitude noise

# Safety
param set-default CBRK_SUPPLY_CHK 894281  # Disable supply check
param set-default COM_DISARM_PRFLT 0      # No preflight disarm
```

### Reconbot Failsafe Settings

```bash
# Failsafe actions (from 60003_uuv_reconbot)
param set-default COM_ACT_FAIL_ACT 0   # Actuator failure: disabled
param set-default COM_LOW_BAT_ACT 0    # Low battery: disabled (underwater)
param set-default NAV_DLL_ACT 0        # Data link loss: disabled
param set-default GF_ACTION 1          # Geofence: warning only
param set-default NAV_RCL_ACT 1        # RC loss: hold position
param set-default COM_POSCTL_NAVL 2    # Position control navigation loss

# Disable attitude failure detection (underwater maneuvers)
param set-default FD_FAIL_P 0          # Pitch failure: disabled
param set-default FD_FAIL_R 0          # Roll failure: disabled
```

### rc.uuv_apps Module Startup

```bash
#!/bin/sh
# UUV apps startup

control_allocator start
uuv_att_control start
uuv_pos_control start
land_detector start rover
```

---

## 9. Parameters Reference

### Attitude Control Parameters (UUV_*)

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `UUV_ROLL_P` | 4.0 | 0-10 | Roll proportional gain |
| `UUV_ROLL_D` | 1.5 | 0-10 | Roll derivative gain |
| `UUV_PITCH_P` | 4.0 | 0-10 | Pitch proportional gain |
| `UUV_PITCH_D` | 2.0 | 0-10 | Pitch derivative gain |
| `UUV_YAW_P` | 4.0 | 0-10 | Yaw proportional gain |
| `UUV_YAW_D` | 2.0 | 0-10 | Yaw derivative gain |
| `UUV_TORQUE_SAT` | 0.3 | 0-1 | Torque saturation limit |
| `UUV_THRUST_SAT` | 0.1 | 0-1 | Thrust saturation limit |

### Stabilization Mode Gains (SGM)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `UUV_SGM_ROLL` | 0.5 | Roll angle gain (rad) |
| `UUV_SGM_PITCH` | 0.5 | Pitch angle gain (rad) |
| `UUV_SGM_YAW` | 0.5 | Yaw angle gain (rad) |
| `UUV_SGM_THRTL` | 0.1 | Throttle gain |

### Position Control Parameters

| Parameter | Default | Range | Description |
|-----------|---------|-------|-------------|
| `UUV_GAIN_X_P` | 1.0 | 0-10 | X position P gain |
| `UUV_GAIN_X_D` | 0.2 | 0-10 | X position D gain |
| `UUV_GAIN_Y_P` | 1.0 | 0-10 | Y position P gain |
| `UUV_GAIN_Y_D` | 0.2 | 0-10 | Y position D gain |
| `UUV_GAIN_Z_P` | 1.0 | 0-10 | Z position P gain |
| `UUV_GAIN_Z_D` | 0.2 | 0-10 | Z position D gain |
| `UUV_STAB_MODE` | 1 | 0-1 | 1=Stabilization, 0=Tracking |
| `UUV_POS_MODE` | 0 | 0-1 | 0=World frame, 1=Body frame |

### DVL Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `DVL_ALT_MAX_AGE` | 1.0 | Max age before fallback (s) |
| `POSE_KI_Z` | 0.0 | Z-axis integral gain |
| `POSE_I_MAX_Z` | 1.0 | Max integral value |
| `POSE_Z_I_ENABLE` | 0 | Enable Z integral |

---

## 10. Reconbot vs Other Vehicle Types

| Aspect | Reconbot (UUV) | Multicopter | Fixed-Wing |
|--------|----------------|-------------|------------|
| **DOF** | 6 (X,Y,Z,R,P,Y) | 4 (Z,R,P,Y) | 4 (R,P,Y,Throttle) |
| **Thrust** | 3-axis vectored (8 motors) | Z-axis only | Forward only |
| **Attitude** | Full 3-axis control | Limited pitch/roll | Coupled to flight path |
| **Altitude Sensor** | DVL / Pressure | Barometer | Barometer / GPS |
| **Position Sensor** | DVL / Vision | GPS / Vision | GPS |
| **Airframe ID** | 60003 | 4xxx | 2xxx |
| **MAV_TYPE** | 12 | 2 | 1 |
| **Control Allocator** | MOTORS_6DOF | MULTIROTOR | STANDARD_VTOL |
| **PWM Range** | 253-2275 | 1000-2000 | 1000-2000 |
| **Neutral PWM** | 1250 (bidirectional) | 1000 (min thrust) | 1500 |
| **Reversible** | All (CA_R_REV=255) | None | N/A |

### Control Architecture Comparison

```
MULTICOPTER:                         RECONBOT (UUV):
Position → Attitude → Rate           Position → Attitude → Rate
    ↓          ↓         ↓               ↓          ↓         ↓
  Thrust    Torque    Torque          Thrust     Torque    Torque
   (Z)     (R,P,Y)   (R,P,Y)         (X,Y,Z)    (R,P,Y)   (R,P,Y)
    ↓          ↓                         ↓          ↓
    └────┬─────┘                         └────┬─────┘
         ↓                                    ↓
    Motor Mixer                         6-DOF Allocator
    (4-8 motors)                        (8 vectored thrusters)
         ↓                                    ↓
    Unidirectional                      Bidirectional PWM
    (1000-2000)                         (253-1250-2275)
```

### Reconbot Thrust Capabilities

| Motion | Primary Motors | Backup/Assist |
|--------|----------------|---------------|
| **Surge (X)** | M5, M6 | - |
| **Sway (Y)** | M7, M8 | M1-M4 (diagonal component) |
| **Heave (Z)** | M1, M2, M3, M4 | - |
| **Roll** | M1-M4 differential | - |
| **Pitch** | M1-M4 differential | M5, M6 |
| **Yaw** | M1-M4 differential | M5-M8 |

---

## 11. Key File Paths

### Core Modules

```
src/modules/uuv_att_control/
├── uuv_att_control.cpp         # Main attitude control logic
├── uuv_att_control.hpp         # Class definition
├── uuv_att_control_params.c    # Parameter definitions
└── CMakeLists.txt

src/modules/uuv_pos_control/
├── uuv_pos_control.cpp         # Main position control logic
├── uuv_pos_control.hpp         # Class definition
├── uuv_pos_control_params.c    # Parameter definitions
└── CMakeLists.txt
```

### Control Allocator

```
src/modules/control_allocator/
├── ControlAllocator.cpp
└── ActuatorEffectiveness/
    ├── ActuatorEffectivenessUUV.cpp
    └── ActuatorEffectivenessUUV.hpp
```

### Configuration Files

```
ROMFS/px4fmu_common/init.d/
├── rc.uuv_defaults             # Default UUV parameters
├── rc.uuv_apps                 # Module startup script
└── airframes/
    └── 60003_uuv_reconbot      # ← Reconbot airframe config
```

### Message Definitions

```
msg/versioned/
├── DopplerVelocityLog.msg      # DVL sensor data
├── TrajectorySetpoint6dof.msg  # 6-DOF trajectory
├── VehicleThrustSetpoint.msg   # Thrust commands
└── VehicleTorqueSetpoint.msg   # Torque commands
```

### Board Configuration

```
boards/px4/fmu-v6x/uuv.px4board   # UUV-specific build config
```

---

## Quick Reference: Reconbot Commands

### Build & Flash

```bash
# Build for your board
make px4_fmu-v6x_uuv

# Upload firmware
make px4_fmu-v6x_uuv upload

# Set airframe (if not auto-detected)
param set SYS_AUTOSTART 60003
reboot
```

### Module Status

```bash
# Check UUV modules
uuv_att_control status
uuv_pos_control status
control_allocator status

# Check sensor status
sensors status
ekf2 status
```

### Topic Monitoring

```bash
# Control outputs
listener vehicle_thrust_setpoint
listener vehicle_torque_setpoint
listener actuator_motors

# Sensor inputs
listener doppler_velocity_log
listener vehicle_local_position
listener vehicle_attitude
```

### Parameter Commands

```bash
# Show all UUV parameters
param show UUV_*

# Show motor configuration
param show CA_ROTOR*

# Show PWM settings
param show PWM_MAIN*

# Save parameters
param save
```

### Motor Testing

```bash
# Test individual motor (M1 at 10%)
actuator_test set -m 1 -v 0.1

# Test all motors sequentially
actuator_test iterate-motors 0.1

# Stop all motors
actuator_test set -m -1 -v 0
```

### Reconbot Motor Test Sequence

```bash
# Test diagonal thrusters (heave/roll/pitch)
actuator_test set -m 1 -v 0.1   # M1: Bow starboard diagonal
actuator_test set -m 2 -v 0.1   # M2: Bow port diagonal
actuator_test set -m 3 -v 0.1   # M3: Stern port diagonal
actuator_test set -m 4 -v 0.1   # M4: Stern starboard diagonal

# Test forward thrusters (surge)
actuator_test set -m 5 -v 0.1   # M5: Port forward
actuator_test set -m 6 -v 0.1   # M6: Starboard forward

# Test lateral thrusters (sway)
actuator_test set -m 7 -v 0.1   # M7: Stern lateral 1
actuator_test set -m 8 -v 0.1   # M8: Stern lateral 2
```

---

## Troubleshooting

### Common Issues

| Issue | Check | Solution |
|-------|-------|----------|
| Motors not spinning | `actuator_motors` topic | Verify arming, check PWM output |
| Wrong motor direction | CA_ROTOR*_AX/AY/AZ | Flip sign of thrust axis |
| No depth hold | DVL data freshness | Check `doppler_velocity_log` |
| Attitude drift | EKF2 health | Verify IMU calibration |
| RC not responding | RC input | Check `manual_control_setpoint` |

### Log Analysis

```bash
# Download log
# (Use QGroundControl or SCP)

# Key topics to analyze:
# - actuator_motors: Motor commands
# - vehicle_thrust_setpoint: Control output
# - vehicle_local_position: Position estimate
# - doppler_velocity_log: DVL data
```

---

*Documentation for PX4 Reconbot UUV (Airframe 60003)*
