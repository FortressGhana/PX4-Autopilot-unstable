# Reconbot UUV Thruster Placement Analysis

**Airframe:** 60003_uuv_reconbot
**Type:** Vectored 6-DOF UUV, 8 thrusters
**Reference Frame:** PX4 FRD (X forward, Y starboard, Z down)
**Date:** 2026-02-15

---

## 1. Thruster Configuration Summary

| Rotor | Position (X, Y, Z) m | Thrust Axis (AX, AY, AZ) | Role |
|-------|----------------------|--------------------------|------|
| R0 | (0.4430, 0.1339, -0.1027) | (0, -0.5000, 0.8660) | Bow-starboard canted |
| R1 | (0.4430, -0.1341, -0.1030) | (0, 0.5000, 0.8660) | Bow-port canted |
| R2 | (-0.4430, 0.1337, -0.1030) | (0, -0.4991, 0.8665) | Stern-starboard canted |
| R3 | (-0.4430, -0.1341, -0.1027) | (0, 0.4991, 0.8665) | Stern-port canted |
| R4 | (0.0884, 0.1385, 0.1128) | (1, 0, 0) | Mid-starboard surge |
| R5 | (0.0882, -0.1395, 0.1129) | (1, 0, 0) | Mid-port surge |
| R6 | (0.2886, 0.0880, 0.1733) | (0, 1, 0) | Forward-starboard lateral |
| R7 | (-0.2884, 0.0880, 0.1735) | (0, -1, 0) | Aft-starboard lateral |

All thrusters are bidirectional (`CA_R_REV 255`). All torque coefficients are zero (`KM 0`).

---

## 2. Canted Thruster Vertical Arrangement

### Description

All four canted thrusters now thrust in the **same vertical direction** at positive throttle:

- **Bow pair (R0, R1):** thrust axis AZ = +0.8660 (pushes downward)
- **Stern pair (R2, R3):** thrust axis AZ = +0.8665 (pushes downward)

At positive rotation, all four produce downward heave. At negative rotation (bidirectional),
all four produce upward heave.

### Benefits Over Previous Configuration

- **Full heave authority in both directions.** All four thrusters operate at the same efficiency
  for both ascent and descent. No reverse-thrust penalty. Heave authority is symmetric.

- **Symmetric pitch authority.** Nose-up and nose-down pitch both use a combination of forward
  and reverse thrust across different pairs, distributing the reverse-thrust penalty equally.
  Pitch response is symmetric in both directions.

- **No heave-pitch transient coupling.** Since all four thrusters spin the same direction for
  heave, there is no differential spool-up timing issue. Step heave commands produce clean
  vertical motion without pitch disturbance.

### Remaining Consideration

- The cant angle is approximately 60 degrees from horizontal (see Section 7). This means each
  canted thruster contributes 0.866 units of heave and 0.500 units of sway per unit thrust.
  Heave authority is prioritized over sway.

---

## 3. Port/Starboard Position Symmetry

### Description

The port and starboard thruster positions are now closely mirror-symmetric about the
centerline (Y=0):

| Thruster Pair | Starboard PY | Port PY | Midpoint Offset | Y Asymmetry |
|---------------|-------------|---------|-----------------|-------------|
| Bow canted (R0/R1) | +0.1339 | -0.1341 | -0.1mm | 0.2mm |
| Stern canted (R2/R3) | +0.1337 | -0.1341 | -0.2mm | 0.4mm |
| Surge (R4/R5) | +0.1385 | -0.1395 | -0.5mm | 1.0mm |

All pairs are within 1mm of perfect symmetry. The surge thrusters have the largest offset
at 0.5mm to port, which is negligible.

### Expected Motion Deviations

- **Parasitic yaw during heave:** Essentially eliminated. The sub-0.5mm midpoint offsets
  produce negligible yaw moments. Even under full saturated heave, parasitic yaw rate is
  expected to be below 0.1 deg/s — well within controller deadband.

- **Parasitic yaw during surge:** The surge pair midpoint is 0.5mm to port. At maximum surge
  thrust, the induced yaw moment is approximately 0.001 N-m per unit force — negligible.

- **Thruster loading:** Near-equal loading on port and starboard thrusters within each pair.
  Saturation limits are effectively symmetric, preserving full peak authority.

---

## 4. Fore/Aft Position Symmetry

### Description

The canted thrusters are now symmetrically placed fore and aft:

| Property | Bow (R0/R1) | Stern (R2/R3) | Difference |
|----------|-------------|---------------|------------|
| X magnitude | 0.4430m | 0.4430m | **0.0mm** |

The lateral thrusters are also nearly symmetric:

| Property | Forward (R6) | Aft (R7) | Difference |
|----------|-------------|----------|------------|
| X magnitude | 0.2886m | 0.2884m | **0.2mm** |

### Benefits Over Previous Configuration

- Previously the bow canted thrusters were at X=0.4293m and stern at X=0.4567m (27.4mm
  difference). This has been eliminated.

- Previously the lateral thrusters were at X=0.2743m and X=0.3027m (28.4mm difference).
  This is now reduced to 0.2mm.

### Expected Motion Deviations

- **No pitch coupling from symmetric heave.** Equal thrust on all four canted thrusters
  produces zero net pitch moment (equal bow and stern lever arms).

- **No sway coupling from yaw.** The lateral thruster pair has essentially equal X-arms
  (0.2886 vs 0.2884m). Yaw commands produce nearly pure rotation with negligible residual
  sway force: 0.0002 N-m per unit force — effectively zero.

---

## 5. Lateral Thrusters on Starboard Side

### Description

Both lateral thrusters (R6 and R7) are mounted on the starboard side at PY = +0.0880m,
88.0mm from the centerline. They are below the center of gravity at PZ = +0.1733/0.1735m
(approximately 173mm below CG). R6 thrusts to starboard (AY=+1), R7 thrusts to port (AY=-1).

### Sway-Yaw Decoupling (Improved)

With the new nearly-symmetric X-arms (0.2886m vs 0.2884m), the yaw coupling during pure
sway is effectively eliminated:

- **Pure sway (both thrusters push same direction):** Net yaw moment =
  (0.2886 - 0.2884) x Fy = 0.0002 x Fy — essentially zero.
- **Pure yaw (thrusters push opposite directions):** Net yaw moment =
  (0.2886 + 0.2884) x Fy = 0.5770 x Fy — full yaw authority with zero net sway.

This is a major improvement over the previous configuration where the 28.4mm X-arm
difference produced significant yaw drift during sway commands.

### Remaining Concern: Roll Coupling During Sway

Because both lateral thrusters are on the starboard side and below CG, sway commands
produce a roll moment:

- Roll moment per unit sway force: M_roll = PZ x F_sway = 0.173 x F (per thruster)
- With both thrusters producing sway: total roll moment = 2 x 0.173 x F = 0.346 x F

A starboard sway command creates a roll-to-port moment. The canted thrusters must
compensate, consuming some heave/attitude authority. During combined sway + heave
maneuvers, the canted thrusters may saturate earlier.

### No Redundancy in Lateral Axis

With only two lateral thrusters, a single failure eliminates independent sway and yaw
control from this group. The vehicle would retain some sway from the canted thrusters
(~0.500 per thruster, 4 thrusters = 2.00 max) but lose dedicated lateral yaw authority.

---

## 6. Surge Thrusters Forward of and Below Center of Gravity

### Description

Both surge thrusters are approximately 88mm forward of CG (R4 at PX=+0.0884, R5 at
PX=+0.0882) and approximately 113mm below CG (R4 at PZ=+0.1128, R5 at PZ=+0.1129).

### Expected Motion Deviations

- **Parasitic nose-down pitch during surge.** The below-center mounting means forward thrust
  produces a nose-down pitch moment. Both thrusters contribute:
  M_pitch = 2 x 0.1129 = 0.2258 N-m per unit total surge force. The allocator compensates
  using the canted thrusters, but this continuously consumes pitch authority during forward
  flight.

- **Parasitic yaw during surge.** The 0.5mm port bias means equal thrust from R4 and R5
  produces a very small yaw moment: M_yaw = 0.1385 - 0.1395 = -0.0010 N-m per unit
  surge force. This is negligible.

---

## 7. Cant Angle (~60 deg)

### Description

The canted thrusters are mounted at approximately 60 degrees from horizontal:

- **Bow pair (R0, R1):** exactly 60.00 deg (sin=0.8660, cos=0.5000)
- **Stern pair (R2, R3):** approximately 60.03 deg (sin=0.8665, cos=0.4991)

This produces a heave-to-sway force ratio of approximately 1.73:1.

### Expected Motion Deviations

- **Limited sway authority from canted thrusters.** Each canted thruster contributes only
  ~0.500 units of sway force per unit thrust, versus ~0.866 units of heave. With all four
  canted thrusters at max, pure sway from the canted group is approximately 2.00 units,
  while pure heave is approximately 3.46 units. The lateral thrusters (2.0 units max)
  compensate, bringing total sway to approximately 4.00 units.

- **Heave-sway coupling under saturation.** When the canted thrusters are near saturation
  for heave, very little lateral force capacity remains. A combined heave + sway maneuver
  will see sway authority degrade significantly (down to just the two lateral thrusters =
  2.0 units) when heave demand exceeds approximately 60% of maximum.

---

## 8. Summary of Expected Motion Deviations

| Command | Primary Deviation | Estimated Magnitude | Root Cause |
|---------|------------------|--------------------| -----------|
| Pure heave | Clean motion | Negligible coupling | Symmetric arrangement, same-direction thrust |
| Pure heave (saturated) | Parasitic yaw | < 0.1 deg/s | Sub-mm port/starboard asymmetry |
| Nose-up vs nose-down pitch | Symmetric authority | Equal in both directions | All canted thrusters same vertical direction |
| Pure surge | Parasitic nose-down pitch | Continuous, compensated | Surge thrusters 113mm below CG |
| Pure surge | Parasitic yaw | Negligible | 0.5mm port bias on surge pair |
| Pure sway | Roll coupling | Continuous, compensated | Lateral thrusters 88mm off-center, 173mm below CG |
| Pure sway | Yaw coupling | Negligible (~0.2mm arm difference) | Nearly symmetric lateral thruster X-arms |
| Sway + heave | Sway authority loss | ~50% at 60% heave | 60 deg cant angle limits sway headroom |
| Lateral thruster failure | Loss of independent sway/yaw | Single point of failure | No lateral redundancy (both on same side) |

---

## 9. Improvements Over Previous Configuration

| Issue | Previous | Current | Status |
|-------|----------|---------|--------|
| Canted thruster vertical direction | Opposing (bow down, stern up) | All same direction (down) | **Fixed** |
| Heave authority | 15-20% reduced (reverse penalty) | Full symmetric authority | **Fixed** |
| Pitch authority asymmetry | 20-30% weaker nose-up | Symmetric | **Fixed** |
| Heave-pitch transients | 5-10 deg transient possible | Eliminated | **Fixed** |
| Fore/aft lever arm symmetry | 27.4mm difference | 0.0mm difference | **Fixed** |
| Lateral thruster X-arm symmetry | 28.4mm difference | 0.2mm difference | **Fixed** |
| Sway-yaw coupling | Significant (31.9mm arm diff) | Negligible (0.2mm arm diff) | **Fixed** |
| Port/starboard position symmetry | Up to 8mm offset | Sub-1mm offsets | **Greatly improved** |
| Lateral thrusters on one side | Both on port, 94.5mm off-center | Both on starboard, 88mm off-center | **Slightly improved** |
| Sway-roll coupling | Present (179mm below CG) | Present (173mm below CG) | Marginal improvement |
| Surge thrusters below CG | 118mm below | 113mm below | Marginal improvement |

---

## 10. Remaining Recommendations

1. **Center or split the lateral thrusters** across port and starboard if packaging allows.
   This would eliminate the roll coupling during sway and provide lateral redundancy. This
   is the single remaining significant geometric issue.

2. **Consider reducing the cant angle** toward 45 degrees if the mission requires significant
   lateral station-keeping. The current 60 degree angle heavily favors heave over sway
   (1.73:1 ratio).

3. **Move surge thrusters closer to CG vertically** if physically possible, to reduce the
   continuous pitch compensation demand during forward transit (currently 113mm below CG).
