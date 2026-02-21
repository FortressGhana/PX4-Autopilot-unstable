# Reconbot UUV — Thruster Allocation Analysis

## Effectiveness Matrix (current config)

```
             R0       R1       R2       R3       R4       R5       R6       R7
Roll:    -0.065   +0.065   -0.065   +0.065    0        0        0        0
Pitch:   -0.384   -0.384   +0.384   +0.384   -0.113   -0.113    0        0
Yaw:     -0.222   +0.222   +0.221   -0.221   -0.139   +0.139  -0.289   +0.289
FX:       0        0        0        0       +1       +1        0        0
FY:      -0.500   +0.500   -0.499   +0.499    0        0       -1       -1
FZ:      +0.866   +0.866   +0.867   +0.867    0        0        0        0
```

### Thruster Map

| Rotor | Motor | Description | Thrust Axis | Position (config) | Position (physical) |
|-------|-------|-------------|-------------|-------------------|---------------------|
| R0 | Motor 1 | Bow port canted (60 deg) | (0, -0.50, +0.87) | (+0.443, -0.134, +0.103) | same |
| R1 | Motor 2 | Bow starboard canted (60 deg) | (0, +0.50, +0.87) | (+0.443, +0.134, +0.103) | same |
| R2 | Motor 3 | Stern port canted (60 deg) | (0, -0.50, +0.87) | (-0.443, -0.134, +0.103) | same |
| R3 | Motor 4 | Stern starboard canted (60 deg) | (0, +0.50, +0.87) | (-0.443, +0.134, +0.103) | same |
| R4 | Motor 5 | Mid starboard surge | (1, 0, 0) | (+0.088, +0.139, -0.113) | same |
| R5 | Motor 6 | Mid port surge | (1, 0, 0) | (+0.088, -0.139, -0.113) | same |
| R6 | Motor 7 | Forward lateral sway | (0, -1, 0) | (+0.2885, -0.088, 0) | (+0.2885, -0.088, -0.1734) |
| R7 | Motor 8 | Aft lateral sway | (0, -1, 0) | (-0.2885, -0.088, 0) | (-0.2885, -0.088, -0.1734) |

### Thruster Positions (FRD body frame, relative to CG)

- **R0–R3 (canted):** Below CG (PZ=+0.103). R0/R2 on port side (PY=-0.134), R1/R3 on starboard side (PY=+0.134).
- **R4/R5 (surge):** Above CG (PZ=-0.113). R4 starboard (PY=+0.139), R5 port (PY=-0.139). Slightly forward of CG (PX=+0.088).
- **R6/R7 (sway):** PZ zeroed in config. R6 forward (PX=+0.289), R7 aft (PX=-0.289). Both on port side (PY=-0.088).

### Config Decisions

- **R0–R5 positions set to re-measured physical values** — All positions reflect actual thruster locations measured relative to CG in FRD frame.
- **R6/R7 PX set to physical values, PZ zeroed** — PX gives the allocator yaw authority from differential sway (±0.289 per thruster). PZ is zeroed to prevent the allocator from recruiting canted thrusters to cancel roll coupling during pure sway. The physical roll coupling (thrusters 0.1734m above CG) is handled by the attitude PD controller as a disturbance.
- **R6/R7 same direction (AY=-1)** — both push port at positive command. Positive command = port sway, negative command = starboard sway.
- **KM=0 for all rotors** — propeller reaction torques not modeled. Compensated by feedforwards where possible.
- **UUV_SURGE_PFF disabled** — Surge-pitch coupling is now handled by the allocator directly (R4/R5 pitch=-0.113 in the effectiveness matrix). The feedforward was designed for the old config where thrusters were below CG; with thrusters above CG the feedforward would double the nose-down coupling.

---

## Motion-by-Motion Analysis

### 1. SURGE (+FX)

| Thruster | Role | Direction |
|----------|------|-----------|
| R4, R5 | **Primary** | Forward |
| R0, R1 | Secondary (pitch cancel) | Forward (small) |
| R2, R3 | Secondary (pitch cancel) | Reverse (small) |
| R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Pitch coupling (thrusters above CG → nose down) | Pitch | Medium | Allocator compensates via R0–R3 pitch authority |
| Reverse asymmetry on R2/R3 | Pitch, Heave | Low | Residual ~1% pitch, ~2.5% heave up |
| Reaction torque (KM=0) | Roll | Low | Attitude PD only |

**Note:** Surge thrusters are above CG (PZ=-0.113), producing nose-down pitch coupling. The allocator sees this in the effectiveness matrix (R4/R5 pitch=-0.113) and compensates by slightly adjusting canted thruster commands. `UUV_SURGE_PFF` feedforward is disabled to avoid double compensation.

---

### 2. SWAY PORT (-FY)

| Thruster | Role | Direction |
|----------|------|-----------|
| R6, R7 | **Primary** | Both forward |
| R0–R3 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Roll coupling (physical PZ offset, unmodeled) | Roll | Low | Attitude PD handles as disturbance |
| Reaction torque (KM=0) | Pitch | Low | Attitude PD only |

**Note:** Port sway is the strong direction (both thrusters forward, full authority). Pure sway activates only R6/R7 — no canted thruster recruitment.

---

### 3. SWAY STARBOARD (+FY)

| Thruster | Role | Direction |
|----------|------|-----------|
| R6, R7 | **Primary** | Both **reverse** |
| R0–R3 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| **20% sway authority loss** | FY | **High** | None — fundamental reverse thrust asymmetry |
| Roll coupling (physical PZ offset, unmodeled) | Roll | Low | Attitude PD handles as disturbance |
| Reaction torque (KM=0) | Pitch | Low | Attitude PD only |

**Note:** Starboard sway is weaker because both thrusters run reverse. The allocator does not know this and will under-deliver starboard sway force. Pure sway activates only R6/R7.

---

### 4. HEAVE (+FZ)

| Thruster | Role | Direction |
|----------|------|-----------|
| R0, R1, R2, R3 | **Primary** | All forward |
| R4, R5, R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| **Reaction torque (KM=0)** | **Yaw** | **High** | `UUV_HEAVE_YFF` feedforward (tune in water) |
| FY coupling | FY | None | Port/starboard pairs cancel |

**Note:** This is the most significant unmodeled effect. All 4 canted props spin the same direction for heave; reaction torques all add in yaw. Every heave command produces a persistent, uncompensated yaw disturbance proportional to heave demand.

---

### 5. ROLL

| Thruster | Role | Direction |
|----------|------|-----------|
| R1, R3 (starboard) | **Primary** | Forward |
| R0, R2 (port) | **Primary** | Reverse |
| R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| **Massive FY coupling** | FY | **High** | Uncompensated (R6/R7 have zero roll effectiveness, allocator cannot use them to cancel FY) |
| **Saturation** | Roll | **High** | Max pure roll ~26% of unit demand |
| Reverse asymmetry on R0/R2 | Roll, Heave | Medium | Roll under-delivered ~10%, upward heave bias |

**Note:** Positive roll (starboard down) is produced by R1/R3 forward + R0/R2 reverse. The FY coupling during roll pushes the vehicle starboard (all four FY contributions add to +2.0). With R6/R7 PZ zeroed, the allocator cannot cancel this FY and must be handled by the position controller.

---

### 6. PITCH

| Thruster | Role | Direction |
|----------|------|-----------|
| R0, R1 (bow) | **Primary** | Reverse (bow down) |
| R2, R3 (stern) | **Primary** | Forward (stern up) |
| R4, R5 | **Secondary** (pitch authority) | Small adjustments |
| R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Reverse asymmetry on R0/R1 | Pitch, Heave | Medium | Pitch under-delivered ~10%, upward heave bias |
| Reaction torque (KM=0) | Yaw | None | Net canted cmd ~0, reaction torques cancel |

**Note:** R4/R5 now contribute pitch authority (±0.113 each) since they are above CG. The allocator can use them for pitch corrections, supplementing the canted thrusters.

---

### 7. YAW

| Thruster | Role | Direction |
|----------|------|-----------|
| R1, R2 (diagonal) | **Primary** | Forward |
| R0, R3 (diagonal) | **Primary** | Reverse |
| R4 | **Primary** (differential surge) | Reverse |
| R5 | **Primary** (differential surge) | Forward |
| R6 | **Primary** (differential sway) | Reverse (starboard) |
| R7 | **Primary** (differential sway) | Forward (port) |

**Yaw authority breakdown:**

| Source | Moment per thruster | Total contribution |
|--------|--------------------|--------------------|
| R0–R3 canted differential | ±0.222 | 0.886 (4 thrusters) |
| R4–R5 surge differential | ±0.139 | 0.278 (2 thrusters) |
| R6–R7 sway differential | ±0.289 | 0.577 (2 thrusters) |
| **Total yaw authority** | | **1.741** |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Reverse asymmetry on R0/R3/R4/R6 | Yaw, Heave, FX | Medium | Yaw -10%, heave bias, small surge |
| Reaction torque (KM=0) | — | None | Net commands ~0, reaction torques cancel |

**Note:** R6/R7 contribute 33% of total yaw authority via differential sway. All three yaw sources (canted diagonal, surge differential, sway differential) produce zero coupling on every other axis.

---

## Summary: Unmodeled Effects by Severity

| Priority | Effect | Affected Motions | Residual |
|----------|--------|-----------------|----------|
| **P0** | KM=0 reaction torque in yaw | Heave | Persistent yaw proportional to heave demand |
| **P1** | Starboard sway 20% weaker | Sway starboard | Asymmetric lateral authority, under-delivery |
| **P1** | Roll FY coupling (uncompensated) | Roll | Lateral drift during roll corrections |
| **P1** | Roll saturation | Roll | Max ~26% authority |
| **P2** | Sway-roll coupling (physical PZ, unmodeled) | Sway | ~0.173 roll per sway thruster, attitude PD handles |
| **P2** | Reverse asymmetry in heave bias | Pitch, Yaw, Roll | Upward drift during attitude corrections |
| **P2** | KM=0 reaction torque in roll | Surge | Small roll disturbance |
| **P3** | KM=0 reaction torque in pitch | Sway | Small pitch disturbance |

---

## Current Compensations

| Compensation | Parameter | Status | Addresses |
|-------------|-----------|--------|-----------|
| Surge-pitch feedforward | `UUV_SURGE_PFF` | **Disabled (0)** — allocator handles directly | Pitch coupling from surge |
| Heave-yaw feedforward | `UUV_HEAVE_YFF` | Active (0.1, needs tuning) | Yaw from heave reaction torque |
| Surge-yaw feedforward | `UUV_SURGE_YFF` | Available (default 0.0, needs tuning) | Yaw from surge reaction torque |
| Yaw integrator | `UUV_YAW_I` | Active (0.5, max 0.2) | Steady-state yaw drift |
| XY position integrator | `POSE_XY_I_EN` | Active (gain 0.3, max 0.2) | Steady-state lateral drift |
| Z position integrator | `POSE_Z_I_ENABLE` | Active (gain 0.3, max 0.2) | Steady-state depth drift |
| Buoyancy compensation | `UUV_BUOY_COMP` | Active (-0.075) | Net buoyancy offset in heave |

## Remaining Gaps

| Gap | Proposed Fix | Complexity |
|-----|-------------|------------|
| Heave-yaw (KM=0) | Tune `UUV_HEAVE_YFF` in water | Low — parameter tuning only |
| Surge-yaw coupling | Tune `UUV_SURGE_YFF` in water | Low — parameter tuning only |
| Starboard sway asymmetry | Accept asymmetry (port is now the strong direction) | N/A |
| Roll FY coupling (uncompensated) | Position controller XY integrator handles drift; accept tradeoff for clean sway | N/A |
| Reverse asymmetry in heave bias | Z integrator handles steady-state; feedforward for transients | Medium — new feedforward parameter |
| Roll authority limit (~26%) | Geometric constraint — no software fix possible | N/A |
