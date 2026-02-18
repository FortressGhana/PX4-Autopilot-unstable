# Reconbot UUV — Thruster Allocation Analysis

## Effectiveness Matrix (current config)

```
             R0       R1       R2       R3       R4       R5       R6    R7
Roll:    +0.065   -0.065   +0.065   -0.065    0        0        0      0
Pitch:   -0.384   -0.384   +0.384   +0.384   +0.113   +0.113    0      0
Yaw:     -0.222   +0.222   +0.221   -0.221   -0.139   +0.140    0      0
FX:       0        0        0        0       +1       +1         0      0
FY:      -0.500   +0.500   -0.499   +0.499    0        0       +1     +1
FZ:      +0.866   +0.866   +0.867   +0.867    0        0        0      0
```

### Thruster Map

| Rotor | Motor | Description | Thrust Axis | Position |
|-------|-------|-------------|-------------|----------|
| R0 | Motor 1 | Bow starboard canted (60 deg) | (0, -0.50, +0.87) | (+0.443, +0.134, -0.103) |
| R1 | Motor 2 | Bow port canted (60 deg) | (0, +0.50, +0.87) | (+0.443, -0.134, -0.103) |
| R2 | Motor 3 | Stern starboard canted (60 deg) | (0, -0.50, +0.87) | (-0.443, +0.134, -0.103) |
| R3 | Motor 4 | Stern port canted (60 deg) | (0, +0.50, +0.87) | (-0.443, -0.134, -0.103) |
| R4 | Motor 5 | Mid starboard surge | (1, 0, 0) | (+0.088, +0.139, +0.113) |
| R5 | Motor 6 | Mid port surge | (1, 0, 0) | (+0.088, -0.140, +0.113) |
| R6 | Motor 7 | Forward lateral sway | (0, +1, 0) | config: (0, 0, 0), physical: (+0.289, +0.088, +0.173) |
| R7 | Motor 8 | Aft lateral sway | (0, +1, 0) | config: (0, 0, 0), physical: (-0.288, +0.088, +0.174) |

### Config Decisions

- **R6/R7 position zeroed** — PX, PY, PZ set to 0 to remove roll and yaw authority from the allocator. Prevents the pseudo-inverse from recruiting sway thrusters for attitude corrections.
- **R6/R7 same direction (AY=+1)** — both push starboard at positive command. Eliminates yaw bias from reverse thrust asymmetry. Requires Motor 8 phase wires swapped to match.
- **KM=0 for all rotors** — propeller reaction torques not modeled. Compensated by feedforwards where possible.

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
| Pitch coupling (thrusters below CG) | Pitch | Medium | `UUV_SURGE_PFF` feedforward + PD |
| Reverse asymmetry on R2/R3 | Pitch, Heave | Low | Residual ~1% pitch, ~2.5% heave up |
| Reaction torque (KM=0) | Roll | Low | Attitude PD only |

---

### 2. SWAY STARBOARD (+FY)

| Thruster | Role | Direction |
|----------|------|-----------|
| R6, R7 | **Primary** | Both forward |
| R0–R3 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Physical yaw (PX offset, unmodeled) | Yaw | None | Cancels exactly (both forward, equal force) |
| Reaction torque (KM=0) | Pitch | Low | Attitude PD only |

---

### 3. SWAY PORT (-FY)

| Thruster | Role | Direction |
|----------|------|-----------|
| R6, R7 | **Primary** | Both **reverse** |
| R0–R3 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| **20% sway authority loss** | FY | **High** | None — fundamental design asymmetry |
| Physical yaw (PX offset, unmodeled) | Yaw | None | Cancels (both reverse at equal reduced force) |
| Reaction torque (KM=0) | Pitch | Low | Attitude PD only |

**Note:** Port sway is fundamentally weaker than starboard because both thrusters run reverse. The allocator does not know this and will under-deliver port sway force.

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
| R0, R2 (starboard) | **Primary** | Forward |
| R1, R3 (port) | **Primary** | Reverse |
| R6, R7 | **Secondary** (cancel FY coupling) | Forward |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| **Massive FY coupling** | FY | **High** | R6/R7 compensate, but saturate at high roll |
| **Saturation** | Roll | **High** | Max pure roll ~26% of unit demand |
| Reverse asymmetry on R1/R3 | Roll, Heave | Medium | Roll under-delivered ~10%, upward heave bias |

**Note:** Roll effectiveness is the weakest axis (0.065 per thruster). Commands reach 3.87x per unit roll demand. At saturation, R6/R7 cannot fully cancel FY → residual lateral drift during roll corrections. This is a geometric constraint of the 60-degree cant angle and small port/starboard moment arm.

---

### 6. PITCH

| Thruster | Role | Direction |
|----------|------|-----------|
| R0, R1 (bow) | **Primary** | Reverse (bow down) |
| R2, R3 (stern) | **Primary** | Forward (stern up) |
| R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Reverse asymmetry on R0/R1 | Pitch, Heave | Medium | Pitch under-delivered ~10%, upward heave bias |
| Reaction torque (KM=0) | Yaw | None | Net canted cmd ~0, reaction torques cancel |

---

### 7. YAW

| Thruster | Role | Direction |
|----------|------|-----------|
| R1, R2 (diagonal) | **Primary** | Forward |
| R0, R3 (diagonal) | **Primary** | Reverse |
| R4 | **Primary** (differential) | Reverse |
| R5 | **Primary** (differential) | Forward |
| R6, R7 | Inactive | — |

**Unwanted effects:**

| Effect | Axis | Severity | Compensation |
|--------|------|----------|--------------|
| Reverse asymmetry on R0/R3/R4 | Yaw, Heave, FX | Medium | Yaw -10%, heave +33%, surge +12% |
| Reaction torque (KM=0) | — | None | Net commands ~0, reaction torques cancel |

---

## Summary: Unmodeled Effects by Severity

| Priority | Effect | Affected Motions | Residual |
|----------|--------|-----------------|----------|
| **P0** | KM=0 reaction torque → yaw | Heave | Persistent yaw proportional to heave demand |
| **P1** | Port sway 20% weaker | Sway port | Asymmetric lateral authority, under-delivery |
| **P1** | Roll saturation + FY coupling | Roll | Max 26% authority, lateral drift at limits |
| **P2** | Reverse asymmetry → heave bias | Pitch, Yaw, Roll | Upward drift during attitude corrections |
| **P2** | KM=0 reaction torque → roll | Surge | Small roll disturbance |
| **P3** | KM=0 reaction torque → pitch | Sway | Small pitch disturbance |
| **P3** | R6/R7 physical mismatch → yaw | Any sway | Only if thrusters differ physically (~1.5% yaw per 5% mismatch) |

---

## Current Compensations

| Compensation | Parameter | Status | Addresses |
|-------------|-----------|--------|-----------|
| Surge-pitch feedforward | `UUV_SURGE_PFF` | Active (0.113) | Pitch coupling from surge |
| Heave-yaw feedforward | `UUV_HEAVE_YFF` | Active (0.1, needs tuning) | Yaw from heave reaction torque |
| XY position integrator | `POSE_XY_I_EN` | Active (gain 0.3, max 0.2) | Steady-state lateral drift |
| Z position integrator | `POSE_Z_I_ENABLE` | Active (gain 0.3, max 0.2) | Steady-state depth drift |
| Buoyancy compensation | `UUV_BUOY_COMP` | Active (-0.075) | Net buoyancy offset in heave |

## Remaining Gaps

| Gap | Proposed Fix | Complexity |
|-----|-------------|------------|
| Heave-yaw (KM=0) | Tune `UUV_HEAVE_YFF` in water | Low — parameter tuning only |
| Port sway asymmetry | Swap Motor 8 wires so AY=-1 works, or accept asymmetry | Hardware change or accept trade-off |
| Reverse asymmetry → heave bias | Z integrator handles steady-state; feedforward for transients | Medium — new feedforward parameter |
| Roll authority limit (26%) | Geometric constraint — no software fix possible | N/A |
| Yaw integrator (attitude) | Add yaw I-term to eliminate steady-state yaw drift | Medium — code change |
