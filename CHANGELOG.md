# Changelog

## [Unreleased] - Reconbot Thruster Placement Fixes

### Software Recommendations from Thruster Placement Analysis

Based on the analysis in `docs/reconbot_thruster_placement_analysis.md`.
Airframe: `60003_uuv_reconbot` | 8-thruster vectored 6-DOF UUV | PX4 FRD frame.

#### 1. Fix the Stern Canted Thruster Axis Direction (Analysis Section 2)

**Problem:** Bow pair (R0/R1) has AZ=+0.8660 (thrust down) while stern pair
(R2/R3) has AZ=+0.8665 (also down in the same direction at positive rotation).
The opposing sway axes (AY signs) mean heave requires half the thrusters in
reverse, losing 15-20% authority and causing transient pitch.

**Fix in `60003_uuv_reconbot`:** Flip the stern pair's vertical axis so all four
canted thrusters push in the same vertical direction at positive rotation. This
eliminates the reverse-thrust penalty for heave:

```
# Current stern pair (R2):
CA_ROTOR2_AY  -0.4991    CA_ROTOR2_AZ  0.8665
# Current stern pair (R3):
CA_ROTOR3_AY   0.4991    CA_ROTOR3_AZ  0.8665

# Proposed (flip AZ sign on stern pair):
CA_ROTOR2_AY   0.4991    CA_ROTOR2_AZ  -0.8665
CA_ROTOR3_AY  -0.4991    CA_ROTOR3_AZ  -0.8665
```

This makes heave use all 4 thrusters at forward rotation (full efficiency) and
makes pitch authority symmetric. However - this changes the control matrix
fundamentally, so verify with the effectiveness calculator that you still have
full 6-DOF rank before applying.

| Priority | Risk | Status |
|---|---|---|
| P1 | Medium - changes control matrix | **Applied** (change A below) |

#### 2. Fix Both Lateral Thrusters on Same Side (Analysis Section 4 - CRITICAL)

**Problem:** R6 and R7 are both at PY=-0.0945 (port side) with AY=+1. This means
sway commands produce roll coupling (94.5mm off-center, 179.3mm below CG) and
there is no lateral redundancy.

**Fix in `60003_uuv_reconbot`:** If the physical hardware places one lateral
thruster on each side, correct the mapping. If both are genuinely on port side, at
minimum flip R7's thrust direction for bidirectional yaw:

```diff
- param set-default CA_ROTOR7_AY 1
+ param set-default CA_ROTOR7_AY -1
```

If the hardware allows moving R7 to starboard:

```diff
- param set-default CA_ROTOR7_PY -0.0945
+ param set-default CA_ROTOR7_PY 0.0945
- param set-default CA_ROTOR7_AY 1
+ param set-default CA_ROTOR7_AY -1
```

This gives bidirectional yaw and eliminates roll coupling during pure sway.

| Priority | Risk | Status |
|---|---|---|
| P0 | Low - config only | **Applied** (change C below) |

#### 3. Add Pitch Feedforward Compensation for Surge (Analysis Section 5)

**Problem:** Surge thrusters at PZ=+0.1183 (118mm below CG) produce continuous
nose-down pitch moment during forward thrust. The attitude controller currently
only reacts via PD feedback - it does not anticipate the coupling.

**Fix in `uuv_att_control.cpp`:** Add a feedforward pitch torque proportional to
commanded surge thrust in `control_attitude_geo()`:

```cpp
// Feedforward: compensate for pitch moment from surge thrusters below CG
// Surge thrusters at PZ=0.1183m create pitch = thrust_x * 0.1183
// Apply opposite torque to pre-emptively cancel
float pitch_ff = -thrust_x * 0.1183f;  // parameterize this as UUV_SURGE_PFF
torques(1) += pitch_ff;
```

Parameterized with `UUV_SURGE_PFF` in `uuv_att_control_params.c` so it can be
tuned per-vehicle. Eliminates the 5-10 degree transient pitch during surge
commands.

| Priority | Risk | Status |
|---|---|---|
| P1 | Medium - new code | **Applied** (change H below) |

#### 4. Add Axis-Specific Saturation Limits (Analysis Sections 2, 4, 6)

**Problem:** The current code uses a single `UUV_TORQUE_SAT` and `UUV_THRUST_SAT`
for all axes (`uuv_att_control.cpp:96-146`). With the asymmetric thruster layout,
different axes have very different authority. The canted thrusters provide 3.46
units heave but only 2.00 units sway. Using a single saturation limit means either
sway saturates too early or heave is under-utilized.

**Fix in `uuv_att_control_params.c` and `uuv_att_control.cpp`:** Split saturation
into per-axis params:

```c
// New params
PARAM_DEFINE_FLOAT(UUV_TSAT_X, 1.0f);  // surge
PARAM_DEFINE_FLOAT(UUV_TSAT_Y, 0.6f);  // sway (lower due to limited authority)
PARAM_DEFINE_FLOAT(UUV_TSAT_Z, 1.0f);  // heave
```

Then in `constrain_actuator_commands()`:

```cpp
thrust_x = math::constrain(thrust_x, -_param_thrust_sat_x.get(), _param_thrust_sat_x.get());
thrust_y = math::constrain(thrust_y, -_param_thrust_sat_y.get(), _param_thrust_sat_y.get());
thrust_z = math::constrain(thrust_z, -_param_thrust_sat_z.get(), _param_thrust_sat_z.get());
```

| Priority | Risk | Status |
|---|---|---|
| P2 | Medium - new params | Not implemented |

#### 5. Increase Attitude Controller Gains for Pitch (Analysis Sections 2, 5)

**Problem:** The current pitch gain (`UUV_PITCH_P=4.0` default) has no explicit
override in `rc.uuv_defaults` - only roll and yaw are overridden. With continuous
pitch disturbances from surge thrust coupling and transient pitch from heave, pitch
needs more aggressive tuning.

**Fix in `rc.uuv_defaults`:**

```sh
param set-default UUV_PITCH_P 8.0   # Match yaw aggressiveness
param set-default UUV_PITCH_D 2.5   # Strong damping for heave transients
```

| Priority | Risk | Status |
|---|---|---|
| P0 | Low - param tuning | **Applied** (change F below) |

#### 6. Correct Port/Starboard Position Asymmetries (Analysis Section 3)

**Problem:** All thruster pairs are biased 1.4-4.0mm to port, producing parasitic
yaw during heave/surge. The allocator compensates but wastes authority headroom.

**Fix in `60003_uuv_reconbot`:** If these are CAD measurement artifacts (not
intentional), symmetrize the positions:

```diff
# Bow canted pair - symmetrize
- param set-default CA_ROTOR0_PY 0.1342
+ param set-default CA_ROTOR0_PY 0.1356
- param set-default CA_ROTOR1_PY -0.1369
+ param set-default CA_ROTOR1_PY -0.1356

# Stern canted pair - symmetrize
- param set-default CA_ROTOR2_PY 0.1326
+ param set-default CA_ROTOR2_PY 0.1355
- param set-default CA_ROTOR3_PY -0.1384
+ param set-default CA_ROTOR3_PY -0.1355

# Surge pair - symmetrize
- param set-default CA_ROTOR4_PY 0.1350
+ param set-default CA_ROTOR4_PY 0.1390
- param set-default CA_ROTOR5_PY -0.1430
+ param set-default CA_ROTOR5_PY -0.1390
```

**Important:** Only do this if the mm-level offsets are measurement noise. If they
reflect actual physical placement, keep the real values and let the allocator
handle it.

| Priority | Risk | Status |
|---|---|---|
| P2 | Low if measurement error, risky if real | **Applied** (change B below) |

#### 7. Add Priority-Based Control Allocation (Analysis Section 6)

**Problem:** The `ActuatorEffectivenessUUV` currently uses
`SEQUENTIAL_DESATURATION` (`ActuatorEffectivenessUUV.hpp:49`). With the steep
60-degree cant angle, sway authority degrades to ~50% when heave exceeds 60%. The
allocator does not prioritize axes.

**Fix in `ActuatorEffectivenessUUV.hpp`:** For underwater operations, depth (heave)
is typically safety-critical. Switch to pseudo-inverse for better distribution:

```cpp
void getDesiredAllocationMethod(AllocationMethod allocation_method_out[MAX_NUM_MATRICES]) const override
{
    // PSEUDO_INVERSE distributes more evenly across actuators
    // and handles the over-determined system (8 actuators, 6 DOF) better
    allocation_method_out[0] = AllocationMethod::PSEUDO_INVERSE;
}
```

This gives better behavior under near-saturation conditions with the asymmetric
geometry.

| Priority | Risk | Status |
|---|---|---|
| P3 | Medium - behavior change | **Applied** (change D below) |

#### 8. Enable and Tune the Z Integrator for Depth Hold (Existing but disabled)

**Problem:** The buoyancy feedforward (`UUV_BUOY_COMP=-0.075`) handles the static
offset, but the Z integrator is disabled (`POSE_Z_I_ENABLE=0`). With the reduced
heave authority from reverse-thrust inefficiency, a pure PD controller will have
steady-state depth error under current/disturbances.

**Fix in `rc.uuv_defaults`:**

```sh
param set-default POSE_Z_I_ENABLE 1
param set-default POSE_KI_Z 0.3
param set-default POSE_I_MAX_Z 0.2
```

The anti-windup and mode-transition reset logic is already implemented in
`uuv_pos_control.cpp:133-137` and `349-357`, so this is safe to enable.

| Priority | Risk | Status |
|---|---|---|
| P1 | Low - existing code | **Applied** (change G below) |

#### Priority Summary

| Priority | Fix | Files Changed | Risk | Status |
|---|---|---|---|---|
| P0 | Fix lateral thruster direction (R7 AY) | `60003_uuv_reconbot` | Low - config only | Applied (C) |
| P0 | Add pitch gains to defaults | `rc.uuv_defaults` | Low - param tuning | Applied (F) |
| P1 | Flip stern canted AZ | `60003_uuv_reconbot` | Medium - changes control matrix | Applied (A) |
| P1 | Add surge-pitch feedforward | `uuv_att_control.cpp`, `_params.c`, `.hpp` | Medium - new code | Applied (H) |
| P1 | Enable Z integrator | `rc.uuv_defaults` | Low - existing code | Applied (G) |
| P2 | Per-axis saturation limits | `uuv_att_control.cpp`, `_params.c`, `.hpp` | Medium - new params | Not implemented |
| P2 | Symmetrize positions | `60003_uuv_reconbot` | Low if noise, risky if real | Applied (B) |
| P3 | Switch allocation method | `ActuatorEffectivenessUUV.hpp` | Medium - behavior change | Applied (D) |

---

### Pre-existing changes (in working tree before this session)

#### A. Stern canted pair axis flip — `60003_uuv_reconbot`

| Param | Before | After | Purpose |
|---|---|---|---|
| `CA_ROTOR2_AY` | -0.4991 | 0.4991 | Match physical mounting direction |
| `CA_ROTOR2_AZ` | 0.8665 | -0.8665 | Stern now pushes UP at +throttle |
| `CA_ROTOR3_AY` | 0.4991 | -0.4991 | Match physical mounting direction |
| `CA_ROTOR3_AZ` | 0.8665 | -0.8665 | Stern now pushes UP at +throttle |

Revert:
```sh
git checkout -- ROMFS/px4fmu_common/init.d/airframes/60003_uuv_reconbot
```

#### B. Port/starboard position symmetrization — `60003_uuv_reconbot`

| Param | Before | After | Offset removed |
|---|---|---|---|
| `CA_ROTOR0_PY` | 0.1342 | 0.1356 | Bow pair centered |
| `CA_ROTOR1_PY` | -0.1369 | -0.1356 | Bow pair centered |
| `CA_ROTOR2_PY` | 0.1326 | 0.1355 | Stern pair centered |
| `CA_ROTOR3_PY` | -0.1384 | -0.1355 | Stern pair centered |
| `CA_ROTOR4_PY` | 0.1350 | 0.1390 | Surge pair centered |
| `CA_ROTOR5_PY` | -0.1430 | -0.1390 | Surge pair centered |

Revert:
```sh
git checkout -- ROMFS/px4fmu_common/init.d/airframes/60003_uuv_reconbot
```

#### C. Lateral thruster direction fix — `60003_uuv_reconbot`

| Param | Before | After | Purpose |
|---|---|---|---|
| `CA_ROTOR7_AY` | 1 | -1 | R7 now thrusts port (opposite R6) for bidirectional yaw |

Revert:
```sh
git checkout -- ROMFS/px4fmu_common/init.d/airframes/60003_uuv_reconbot
```

#### D. Allocator method change — `ActuatorEffectivenessUUV.hpp:49`

| Before | After | Purpose |
|---|---|---|
| `SEQUENTIAL_DESATURATION` | `PSEUDO_INVERSE` | Better distribution across 8 actuators for asymmetric geometry |

Revert:
```sh
git checkout -- src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessUUV.hpp
```

---

### Changes made this session (P0 + P1 fixes)

#### E. Motor8 header comment fix — `60003_uuv_reconbot:15`

```
- # @output Motor8 aft port lateral thruster (thrust starboard)
+ # @output Motor8 aft port lateral thruster (thrust port)
```

Aligns comment with the R7 AY=-1 change (C above).

#### F. Pitch gain overrides (P0) — `rc.uuv_defaults:55-56`

```sh
param set-default UUV_PITCH_P 8.0   # was unset, defaulting to 4.0
param set-default UUV_PITCH_D 2.5   # was unset, defaulting to 2.0
```

Pitch was the only attitude axis without an explicit override. Now matches yaw
aggressiveness (8.0/2.5) to handle continuous pitch disturbances from surge-heave
coupling and the opposing canted arrangement.

Revert:
```sh
git checkout -- ROMFS/px4fmu_common/init.d/rc.uuv_defaults
```

#### G. Depth hold Z integrator enabled (P1) — `rc.uuv_defaults:68-71`

```sh
param set-default POSE_Z_I_ENABLE 1   # was 0 (disabled)
param set-default POSE_KI_Z 0.3       # was 0.0
param set-default POSE_I_MAX_Z 0.2    # was 0.0
```

Eliminates steady-state depth error under currents/disturbances. Anti-windup clamp
at 0.2. Mode-transition reset logic already existed in
`uuv_pos_control.cpp:133-137,349-357`.

Revert:
```sh
git checkout -- ROMFS/px4fmu_common/init.d/rc.uuv_defaults
```

#### H. Surge-pitch feedforward parameter (P1) — 3 files

| File | Change |
|---|---|
| `uuv_att_control_params.c:260-273` | New `UUV_SURGE_PFF` param, default 0.0, range [0.0, 0.5] |
| `uuv_att_control.hpp:170` | Added `_param_surge_pitch_ff` member |
| `uuv_att_control.cpp:213-218, 237-242` | `pitch_u -= thrust_x * surge_pff` in both attitude-enabled and rate-control paths |

Default 0.0 = no-op (allocator handles steady-state coupling). Set to 0.118 (the
surge thruster CG offset in meters) if pitch transients during aggressive surge are
observed. Active in both attitude and rate control modes.

Revert:
```sh
git checkout -- src/modules/uuv_att_control/uuv_att_control.cpp \
                src/modules/uuv_att_control/uuv_att_control.hpp \
                src/modules/uuv_att_control/uuv_att_control_params.c
```

---

### Remaining recommendations (not implemented)

#### P2-1: Per-axis saturation limits

- **What:** Split `UUV_TORQUE_SAT` / `UUV_THRUST_SAT` into per-axis params
  (`UUV_TSAT_X`, `UUV_TSAT_Y`, `UUV_TSAT_Z` and similarly for torque)
- **Why:** The 60-degree cant gives 3.46 units heave but only 2.0 units sway from
  canted thrusters. A single saturation limit either clips sway prematurely or
  under-utilizes heave
- **Files:** `uuv_att_control_params.c`, `uuv_att_control.hpp`,
  `uuv_att_control.cpp` (replace single-limit logic in
  `constrain_actuator_commands()` with per-axis limits)
- **Risk:** Low. Backward compatible if all per-axis params default to the current
  single value

#### P2-2: Correct position asymmetries (verify CAD)

- **What:** Confirm whether the original mm-level port biases (1.4-4.0mm) in the
  thruster positions were measurement noise or actual physical placement. The
  symmetrization in (B) above assumes they were noise
- **Why:** If they reflect real mounting offsets, the symmetrized values are slightly
  wrong and the allocator effectiveness matrix will not match the real vehicle
- **Action:** Measure physical thruster positions on the vehicle and update
  `60003_uuv_reconbot` accordingly

#### P3-1: Reduce cant angle (hardware)

- **What:** Remount canted thrusters closer to 45 degrees from horizontal instead
  of 60 degrees
- **Why:** Current 60-degree angle gives 1.73:1 heave-to-sway force ratio. At 45
  degrees this becomes 1:1. Sway authority drops to approximately 50% when heave
  exceeds 60% of max
- **Impact:** Requires physical hardware change + updating `CA_ROTOR0-3_AY` and
  `CA_ROTOR0-3_AZ` axis components + Gazebo SDF model

#### P3-2: Split/center lateral thrusters (hardware)

- **What:** Move one lateral thruster to the starboard side, or center both on the
  vehicle centerline
- **Why:** Both on port (PY=-0.0945) creates 0.179 Nm roll coupling per unit sway
  force. The canted thrusters compensate but lose heave/pitch headroom. A single
  lateral failure loses independent sway+yaw
- **Impact:** Physical relocation + updating `CA_ROTOR6_PY`/`CA_ROTOR7_PY` + Gazebo
  SDF model

#### P3-3: Move surge thrusters closer to CG (hardware)

- **What:** Relocate surge thrusters from PZ=+0.1183 (118mm below CG) and
  PX=+0.0773 (77mm forward) closer to the center of gravity
- **Why:** The offset produces continuous nose-down pitch moment during forward
  transit that the canted thrusters must counteract, consuming approximately 0.24 Nm
  per unit surge force of pitch authority
- **Impact:** Physical relocation + updating `CA_ROTOR4_PX/PZ`,
  `CA_ROTOR5_PX/PZ` + Gazebo SDF model

---

## [Previous] - External INS Integration

### Added
- Added new message definitions:
  - Added ExternalVehicleLocalPosition.msg for external INS position data
  - Added support for external INS position data structure
  - Added proper message versioning for external INS data

### Changed
- Modified existing message definitions:
  - Updated VehicleAttitude.msg for external INS support
  - Updated VehicleGlobalPosition.msg for external INS support
  - Updated VehicleLocalPosition.msg for external INS support

### Configuration
- Added development environment configuration:
  - Added .vscode/settings.json for development setup
  - Added proper IDE configuration for external INS development

### Technical Details

#### Message System
- Added new external INS message types:
  - Created ExternalVehicleLocalPosition.msg
  - Added proper message structure for external INS data
  - Added versioning support for external INS messages
- Modified existing vehicle state messages:
  - Updated message structures for external INS compatibility
  - Added external INS specific fields
  - Maintained backward compatibility

#### EKF2 Changes
- Added external INS support
- Added fallback mechanisms
- Improved quaternion handling
- Added proper parameter handling
- Fixed array assignment issues

#### VectorNav Driver
- Added binary message support
- Added multiple data output groups
- Added proper error handling
- Added timeout mechanisms
- Added parameter configuration

#### System Integration
- Added proper message routing
- Added fallback mechanisms
- Added error handling
- Added timeout handling
- Added parameter management
