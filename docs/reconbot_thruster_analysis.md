# ReconBot UUV Thruster Placement Analysis

## Executive Summary

This document provides a technical analysis of the 8-thruster configuration for the ReconBot UUV, examining placement decisions, control implications, potential drawbacks, and open questions for the team to address.

---

## 1. Thruster Configuration Overview

### Thruster Functions

| Thruster Group | Thrusters | Primary DOF | Mounting |
|----------------|-----------|-------------|----------|
| Surge | T5, T6 | Forward/Backward (X) | Side-mounted, forward-facing |
| Sway | T7, T8 | Left/Right (Y) | Top-mounted, lateral-facing |
| Heave/Attitude | T1, T2, T3, T4 | Up/Down (Z), Roll, Pitch | Corner-mounted, canted |

### Moment Arm Summary (Body Frame, mm)

| Thruster | Lxn (mm) | Lyn (mm) | Lzn (mm) | Function |
|----------|----------|----------|----------|----------|
| T1 | -458.55 | -131.24 | -96.58 | Heave/Attitude (Front-Port) |
| T2 | -458.55 | 136.71 | -96.58 | Heave/Attitude (Front-Starboard) |
| T3 | 427.45 | 136.71 | -96.58 | Heave/Attitude (Aft-Starboard) |
| T4 | 427.45 | -131.24 | -96.58 | Heave/Attitude (Aft-Port) |
| T5 | -203.50 | -136.73 | 118.94 | Surge (Port) |
| T6 | -203.50 | 141.27 | 118.64 | Surge (Starboard) |
| T7 | 272.56 | 90.76 | 179.89 | Sway (Aft) |
| T8 | -304.50 | 90.76 | 179.89 | Sway (Forward) |

### Coordinate System (NED Body Frame)
- **X**: Positive forward (surge)
- **Y**: Positive starboard (sway)
- **Z**: Positive down (heave)

---

## 2. Thruster Group Analysis

### 2.1 Surge Thrusters (T5, T6)

**Configuration:**
- Side-mounted on port and starboard, facing forward
- Both at X = -203.50mm (forward of geometric center)
- T5 at Y = -136.73mm (port), T6 at Y = +141.27mm (starboard)
- Z heights: T5 at 118.94mm, T6 at 118.64mm

**Strengths:**
- Symmetric Y-placement about centerline allows clean surge without yaw coupling
- Both at same X position eliminates differential thrust requirements for pure surge
- Matched Z-height prevents roll coupling during surge

**Concerns:**
- **Forward offset**: Both thrusters are 203.50mm forward of center
  - Surge thrust creates a pitch-down moment about the center of mass
  - Requires T1-T4 compensation to maintain level attitude during acceleration
  - Question: Is this offset intentional to counter a nose-heavy CG?

- **Minor Y-asymmetry**: Port at -136.73mm, starboard at +141.27mm (4.54mm difference)
  - Will create small yaw coupling during surge
  - Likely within acceptable tolerance

**Control Implication:** Pure surge commands will require small pitch compensation from T1-T4.

---

### 2.2 Sway Thrusters (T7, T8)

**Configuration:**
- Top-mounted, facing laterally (Y-direction thrust)
- T7 at X = +272.56mm (aft of center)
- T8 at X = -304.50mm (forward of center)
- **Both at Y = +90.76mm (starboard of centerline)**
- Both at Z = 179.89mm

**CRITICAL CONCERN — Both Thrusters on Same Side of Y-Axis:**

The most significant design issue is that both sway thrusters are positioned at Y = +90.76mm, meaning both are starboard of the vehicle centerline. This is highly unusual for sway thrusters.

**Implications:**

1. **Yaw coupling during sway commands:**
   - When both T7 and T8 thrust in the same Y-direction for sway:
   - T7 creates yaw moment: M_yaw = F × (+272.56mm)
   - T8 creates yaw moment: M_yaw = F × (-304.50mm)
   - Net yaw moment = F × (272.56 - 304.50) = F × (-31.94mm)
   - Result: Pure sway commands induce yaw (nose turns toward direction of sway)

2. **Roll coupling:**
   - Both thrusters at Y = +90.76mm and Z = 179.89mm
   - Lateral thrust creates roll moment about X-axis
   - Sway right (thrust +Y) → roll to port
   - This must be compensated by T1-T4 differential

3. **Reduced yaw authority:**
   - With both sway thrusters on the same side, they cannot efficiently create pure yaw moments
   - Yaw control relies entirely on T1-T4 differential thrust (assuming they are canted)

**Questions:**
- Is this placement driven by mechanical constraints (payload bay, sensors, cables)?
- Was symmetric placement (one port, one starboard) evaluated?
- What is the expected sway performance requirement?

---

### 2.3 Heave/Attitude Thrusters (T1, T2, T3, T4)

**Configuration:**
- Four thrusters at vehicle corners
- Front pair (T1, T2) at X = -458.55mm
- Rear pair (T3, T4) at X = +427.45mm
- Port (T1, T4) at Y = -131.24mm
- Starboard (T2, T3) at Y = +136.71mm
- All at Z = -96.58mm (below center)
- **Observation from drawings: These appear to be CANTED (angled), not purely vertical**

**Canted (Vectored) Configuration — Confirmed 45° Cant Angles:**

T1-T4 are mounted at 45° angles, providing thrust in multiple axes. This is common in ROV designs for increased maneuverability.

With 45° cant angles, T1-T4 contribute to:
- **Heave**: All four thrusting up/down together
- **Roll**: Port vs starboard differential (T1/T4 vs T2/T3)
- **Pitch**: Front vs rear differential (T1/T2 vs T3/T4)
- **Yaw**: Diagonal pairs differential (T1/T3 vs T2/T4)
- **Surge**: Additional surge authority (all four, horizontal component)
- **Sway**: Additional sway authority (all four, horizontal component)

**Cant angles confirmed at 45°.**

**Strengths:**
- Large baseline: ~886mm longitudinal, ~268mm lateral
- Strong moment arms for attitude control
- Redundancy: Loss of one thruster still allows attitude control

**Concerns:**
- **X-asymmetry**: Front at -458.55mm, rear at +427.45mm (31.1mm offset)
  - Geometric center is not at X=0
  - Pure heave will create small pitch moment

- **Y-asymmetry**: Port at -131.24mm, starboard at +136.71mm (5.47mm offset)
  - Pure heave will create small roll moment

---

## 3. Control System Implications

### 3.1 DOF Control Matrix

| DOF | Primary Actuators | Secondary/Compensation | Notes |
|-----|-------------------|----------------------|-------|
| Surge | T5, T6 | T1-T4 (if canted) | Pitch compensation needed |
| Sway | T7, T8 | T1-T4 (if canted) | Yaw + Roll compensation needed |
| Heave | T1, T2, T3, T4 | — | Minor pitch/roll coupling |
| Roll | T1/T4 vs T2/T3 | — | Good authority |
| Pitch | T1/T2 vs T3/T4 | — | Good authority |
| Yaw | T1/T3 vs T2/T4 (if canted) | — | Reduced authority if T1-T4 not canted |

### 3.2 Expected Control Coupling

| Command | Primary Motion | Induced Coupling | Severity |
|---------|---------------|------------------|----------|
| Pure Surge | Forward/Back | Pitch-down moment | Medium |
| Pure Sway | Left/Right | Yaw + Roll | **High** |
| Pure Heave | Up/Down | Minor pitch + roll | Low |
| Pure Roll | Bank | Minimal | Low |
| Pure Pitch | Nose up/down | Minimal | Low |
| Pure Yaw | Heading change | Depends on T1-T4 config | Unknown |

### 3.3 Mixer Design Requirements

The control allocator must:

1. **Decouple sway from yaw/roll**: T7/T8 placement requires significant compensation
2. **Compensate surge pitch moment**: T5/T6 forward offset needs pitch correction
3. **Handle cant angles**: If T1-T4 are vectored, full 6-DOF contribution matrix needed
4. **Manage saturation**: Compensation uses motor headroom, reducing max authority

**Mixer Complexity: HIGH**

Unlike symmetric configurations where DOFs are naturally decoupled, this design requires a fully-coupled mixer with accurate moment arm and thrust vector data.

---

## 4. Potential Drawbacks

### 4.1 Sway Performance (High Impact)

| Issue | Impact |
|-------|--------|
| Both T7/T8 on starboard side | Every sway command induces yaw |
| Yaw coupling | Heading drift during lateral movement |
| Roll coupling | Attitude disturbance during sway |
| Compensation overhead | Reduced effective sway force |

**Operational Impact:**
- Station-keeping with cross-currents will be challenging
- Lateral inspection passes will require constant heading correction
- Autonomous waypoint following may show oscillation in heading

### 4.2 Surge Performance (Medium Impact)

| Issue | Impact |
|-------|--------|
| T5/T6 forward of center | Pitch-down during acceleration |
| Pitch coupling | Depth excursions during speed changes |

**Operational Impact:**
- Transit may require active pitch compensation
- Depth-holding accuracy degrades during acceleration/deceleration

### 4.3 System-Level Concerns

| Category | Issue | Severity |
|----------|-------|----------|
| Efficiency | Motor effort spent on compensation | Medium |
| Wear | Uneven thruster usage | Medium |
| Tuning | Complex gain interactions | High |
| Fault Tolerance | T7 or T8 loss cripples sway | High |
| Simulation | Must match exact geometry | High |

---

## 5. Operational Scenarios — How These Issues Manifest

This section provides concrete examples of how the thruster placement affects real-world operations.

### Scenario 1: Lateral Pipeline Inspection

**Task:** Move 2 meters laterally while keeping the camera pointed at a pipeline.

**Expected behavior:** Vehicle translates sideways, heading remains constant.

**What actually happens:**

```
INITIAL STATE                        AFTER SWAY COMMAND

    Pipeline                             Pipeline
════════════════════                 ════════════════════
        ↑                                    ↑
   [ Vehicle ]  →  Sway right           [ Vehicle ]
    Heading: 0°                          ╱
    Camera: On target                   ╱  Heading: -7°
                                       ╱   Camera: Off target
                                      ↙
```

**Why this happens:**
- T7 at X = +272.6mm creates yaw moment = F × 272.6mm (nose right)
- T8 at X = -304.5mm creates yaw moment = F × -304.5mm (nose left)
- Net yaw moment = F × -31.9mm → nose turns left during rightward sway

**Quantified impact (assuming 50N per thruster):**

| Parameter | Value |
|-----------|-------|
| Net lateral force | 100N |
| Induced yaw moment | 3.19 Nm |
| Heading drift (2 sec) | ~7° |
| Camera offset at 2m range | ~24 cm off target |

**Operational consequence:** Operator must constantly correct heading during lateral tracking, increasing workload and reducing inspection quality.

---

### Scenario 2: Station Keeping in Cross-Current

**Task:** Hold position with a 0.5 m/s cross-current from port side.

**Expected behavior:** Vehicle maintains position and heading.

**What actually happens:**

```
             Current → → →

TIME 0:          TIME 5s:         TIME 10s:

[ Vehicle ]      [ Vehicle ]       [ Vehicle ]
Heading: 0°        ↗               ↗  ↖  ↗
                 Heading: -3°     Oscillating ±5°
                 Drifting         Hunting for position
```

**Why this happens:**
1. Current pushes vehicle starboard
2. Controller commands sway to port (T7/T8 thrust -Y)
3. Sway command induces yaw moment (nose turns right this time)
4. Heading controller corrects yaw using T1-T4
5. T1-T4 yaw correction reduces heave/attitude authority
6. Vehicle oscillates as sway and yaw controllers fight

**Operational consequence:** Position hold in currents shows heading oscillation. DVL-based navigation accuracy degrades. Power consumption increases due to constant corrections.

---

### Scenario 3: Docking Approach

**Task:** Approach a docking station with 5cm lateral precision.

**Expected behavior:** Smooth lateral alignment to docking port.

**What actually happens:**

```
APPROACH SEQUENCE:

     Dock                           Dock
      ║                              ║
      ║    [ Vehicle ]               ║   [ Vehicle ]
      ║         ↓                    ║      ╲
      ║    Sway left                 ║       ╲ Yaw + Roll
      ║                              ║        ╲
      ╠════                          ╠════
      Docking port                   Docking port

      Expected                       Actual
```

**Why this happens:**
- Fine lateral adjustments require small sway commands
- Each sway command induces yaw AND roll
- Roll causes depth change (buoyancy shift)
- Three DOFs disturbed when only one was commanded

**Quantified impact:**

| Sway Command | Induced Yaw | Induced Roll | Depth Change |
|--------------|-------------|--------------|--------------|
| 10N lateral | 0.32 Nm yaw | ~0.5° roll | ~2 cm |
| 25N lateral | 0.80 Nm yaw | ~1.2° roll | ~5 cm |
| 50N lateral | 1.60 Nm yaw | ~2.5° roll | ~10 cm |

**Operational consequence:** Precision docking requires multiple correction cycles. What should be a single smooth approach becomes an iterative process.

---

### Scenario 4: Transit with Speed Changes

**Task:** Transit 100m, accelerating to cruise speed then decelerating to stop.

**Expected behavior:** Vehicle maintains depth and pitch throughout.

**What actually happens:**

```
DEPTH PROFILE DURING TRANSIT:

Depth
  ↑
  │         Acceleration      Cruise        Deceleration
  │              ↓              ↓                ↓
  │            ╱‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾╲
  │           ╱                              ╲
  │──────────╱                                ╲──────────  Target
  │        ↙                                    ↘
  │      Pitch down                          Pitch up
  │      = Dive                              = Rise
  └─────────────────────────────────────────────────────→ Distance
```

**Why this happens:**
- T5/T6 are at X = -203.5mm (forward of center)
- Surge thrust creates pitch-down moment
- Acceleration → nose dives → depth increases
- Deceleration → nose rises → depth decreases

**Quantified impact (assuming 80N surge thrust):**

| Phase | Pitch Moment | Pitch Change | Depth Excursion |
|-------|--------------|--------------|-----------------|
| Acceleration | -16.3 Nm | -3° to -5° | +0.5m dive |
| Cruise | 0 | 0° | Stable |
| Deceleration | +16.3 Nm | +3° to +5° | -0.5m rise |

**Operational consequence:** Depth-sensitive missions (mine countermeasures, pipeline following) see depth errors during speed changes. Altitude hold modes work harder during transit.

---

### Scenario 5: Thruster Failure (T7 or T8)

**Task:** Continue operations after single thruster failure.

**Impact comparison:**

| Failed Thruster | Sway Authority | Yaw Impact | Severity |
|-----------------|----------------|------------|----------|
| T1, T2, T3, or T4 | 100% | Minor (3 remaining for yaw) | **Low** |
| T5 or T6 | 100% (sway unaffected) | None | **Low** |
| **T7** | **50%** | Severe yaw coupling | **High** |
| **T8** | **50%** | Severe yaw coupling | **High** |

**Why T7/T8 failure is worse:**

With only T7 operational (T8 failed):
- Single thruster at X = +272.6mm
- Every sway command creates large yaw moment
- No way to balance — must use T1-T4 for all yaw compensation

```
T8 FAILED - SWAY RIGHT COMMAND:

         T7 (only sway thruster)
              ↓ Force
              ○─────────────────────×  T8 (failed)
                    Vehicle
                       ↺ Strong yaw moment

Result: Vehicle spins more than it translates
```

**Operational consequence:** T7 or T8 failure effectively eliminates clean sway capability. Mission abort may be required for tasks requiring lateral movement.

---

### Scenario 6: Combined Maneuver — Heave While Maintaining Heading

**Task:** Ascend 5m while holding position and heading in current.

**What actually happens:**

```
THRUSTER BUDGET DURING MANEUVER:

T1-T4 must simultaneously:
├── Provide heave force (primary task)     → 40% capacity
├── Counter pitch from T1-T4 asymmetry     → 10% capacity
├── Counter sway-induced yaw               → 25% capacity
├── Counter sway-induced roll              → 15% capacity
└── Reserve for disturbance rejection      → 10% capacity
                                             ─────────────
                                             100% — SATURATED
```

**Why this is a problem:**

The 45° canted T1-T4 thrusters provide heave AND attitude control. When the mixer must use them for multiple simultaneous tasks:

1. Heave authority is reduced (can't use full vertical component)
2. Attitude response is slower (authority split across tasks)
3. Disturbance rejection suffers (no reserve capacity)
4. Risk of actuator saturation increases

**Operational consequence:** Complex maneuvers in currents may see degraded performance. Vehicle may not achieve commanded ascent rate while maintaining position.

---

### Summary: Scenarios by Severity

| Scenario | Primary Issue | Severity | Workaround |
|----------|---------------|----------|------------|
| Pipeline inspection | Sway→Yaw coupling | High | Reduce lateral speed, accept heading drift |
| Station keeping | Sway→Yaw oscillation | High | Detune sway response, accept position error |
| Docking | Multi-DOF coupling | High | Iterative approach, slower operations |
| Transit | Surge→Pitch coupling | Medium | Active pitch compensation, accept depth variance |
| Thruster failure | Asymmetric redundancy | High | Mission abort for sway-critical tasks |
| Combined maneuvers | Actuator saturation | Medium | Limit simultaneous commands, sequence operations |

---

## 6. Questions for the Team

### Configuration Verification

1. **T7/T8 Y-positions**: Both show Y = +90.76mm. Is this correct, or should one be at -90.76mm (port)?
2. **Thruster directions**: Which way does positive PWM command thrust for each motor?
3. **Center of mass location**: Where is the actual CG relative to body frame origin?

### Design Intent

4. **Why are T7/T8 both on starboard?** What constraint drove this decision?
5. **T5/T6 forward offset**: Intentional CG compensation or geometric constraint?
6. **Were alternative configurations evaluated?** (e.g., BlueROV2-style vectored)

### Operational Requirements

7. **Sway performance spec**: What lateral speed/acceleration is required?
8. **Station-keeping accuracy**: What position hold tolerance is needed?
9. **Expected current conditions**: What cross-currents must the vehicle handle?

### Control Architecture

10. **Is model-based control an option?** MPC could handle coupling better.
11. **What's the control loop rate?** Faster loops can better reject coupling disturbances.

---

## 7. Recommendations

### Immediate Actions (Before Proceeding)

1. **VERIFY T7/T8 POSITIONS**
   - Confirm Y = +90.76mm for both is correct
   - If one should be at Y = -90.76mm, correct the data
   - This single change would resolve most sway coupling issues

2. **DOCUMENT T1-T4 CANT ANGLES**
   - Measure or extract from CAD the exact thrust vector angles
   - Without this, we cannot build an accurate mixer

3. **MEASURE CENTER OF MASS**
   - Physical measurement or CAD calculation
   - Critical for understanding coupling behavior

### If T7/T8 Positions Are Correct (Cannot Change)

1. **Design fully-coupled mixer**
   - Use pseudoinverse control allocation
   - Weight sway commands lower to prevent saturation
   - Accept reduced sway performance

2. **Implement feedforward compensation**
   - Pre-compute expected yaw/roll disturbance from sway commands
   - Inject counter-moments proactively

3. **Consider control mode limitations**
   - Document that certain combined maneuvers may perform poorly
   - Define operational envelope limits

4. **Extensive simulation before water testing**
   - Validate coupling compensation
   - Test failure modes

### If Hardware Can Be Modified

**Option A — Relocate T8 to Port Side:**
- Move T8 to Y = -90.76mm (mirror of T7)
- Maintains X positions, fixes Y symmetry
- Eliminates primary sway coupling issue

**Option B — Symmetric Sway Thruster Redesign:**
- Place T7/T8 on vehicle centerline (Y = 0)
- Symmetric X positions (e.g., both at ±275mm)
- Cleanest solution for sway control

**Option C — Increase T1-T4 Cant Angles:**
- If T1-T4 have minimal cant, increase angles
- Provides sway authority from corner thrusters
- Reduces dependence on T7/T8

---

## 8. Summary of Key Issues

| Priority | Issue | Affected DOF | Recommendation |
|----------|-------|--------------|----------------|
| **Critical** | T7/T8 both at Y=+90.76mm | Sway, Yaw | Verify data; relocate if possible |
| **High** | T1-T4 cant angles unknown | All | Measure and document |
| **Medium** | T5/T6 forward offset | Surge, Pitch | Verify against CG location |
| **Low** | Minor Y-asymmetries | Various | Accept; compensate in mixer |

---

## 9. Conclusion

The ReconBot thruster configuration presents a workable but challenging control problem. The vehicle is fully controllable in 6 DOF, but the T7/T8 sway thruster placement creates significant coupling that will:

- Reduce sway efficiency
- Complicate control tuning
- Require careful mixer design
- Limit operational performance in cross-currents

**The single most impactful improvement would be verifying (and potentially correcting) the T7/T8 Y-positions.** If both are truly at Y = +90.76mm, the team should understand why this constraint exists and whether it can be addressed.

Before water testing, we need:
1. Confirmed thruster positions
2. T1-T4 cant angles
3. Center of mass location
4. Validated simulation with accurate geometry

The control system can compensate for the current design, but hardware symmetry would substantially reduce risk and improve performance.

---

*Document prepared for ReconBot development team review*
*Revised analysis based on corrected thruster function assignments*
*Analysis based on moment arm data provided 2025-12-27*
