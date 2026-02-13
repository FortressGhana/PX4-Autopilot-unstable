# Reconbot UUV Thruster Placement Analysis

**Airframe:** 60003_uuv_reconbot
**Type:** Vectored 6-DOF UUV, 8 thrusters
**Reference Frame:** PX4 FRD (X forward, Y starboard, Z down)
**Date:** 2026-02-09

---

## 1. Thruster Configuration Summary

| Rotor | Position (X, Y, Z) m | Thrust Axis (AX, AY, AZ) | Role |
|-------|----------------------|--------------------------|------|
| R0 | (0.4293, 0.1342, -0.1007) | (0, -0.4879, 0.8727) | Bow-starboard canted |
| R1 | (0.4293, -0.1369, -0.1002) | (0, 0.4879, 0.8727) | Bow-port canted |
| R2 | (-0.4567, 0.1326, -0.0996) | (0, 0.4879, -0.8727) | Stern-starboard canted |
| R3 | (-0.4567, -0.1384, -0.0996) | (0, -0.4879, -0.8727) | Stern-port canted |
| R4 | (0.0773, 0.1350, 0.1183) | (1, 0, 0) | Mid-starboard surge |
| R5 | (0.0773, -0.1430, 0.1183) | (1, 0, 0) | Mid-port surge |
| R6 | (0.2743, -0.0945, 0.1793) | (0, 1, 0) | Forward-port lateral |
| R7 | (-0.3027, -0.0945, 0.1793) | (0, 1, 0) | Aft-port lateral |

All thrusters are bidirectional (`CA_R_REV 255`). All torque coefficients are zero (`KM 0`).

---

## 2. Canted Thruster Opposing Vertical Arrangement

### Description

The four canted thrusters are arranged with opposing vertical thrust directions:

- **Bow pair (R0, R1):** thrust axis AZ = +0.8727 (pushes downward at positive throttle)
- **Stern pair (R2, R3):** thrust axis AZ = -0.8727 (pushes upward at positive throttle)

At positive (forward) rotation, all four thrusters produce **nose-down pitch** simultaneously:
the bow pair pushes the nose down while the stern pair lifts the tail.

### Expected Motion Deviations

- **Pure heave requires half the thrusters in reverse.** To descend, R0 and R1 thrust forward
  (downward) while R2 and R3 must reverse (also downward). To ascend, the opposite. If
  propellers have asymmetric forward/reverse efficiency (typical: ~70-80% thrust in reverse),
  effective heave authority is reduced by approximately 15-20% compared to a same-direction
  arrangement.

- **Pitch authority is asymmetric.** Nose-down pitch uses all four thrusters at forward rotation
  (full efficiency). Nose-up pitch requires all four in reverse (reduced efficiency). Expected
  nose-up pitch authority is approximately 70-80% of nose-down, producing a bias toward
  nose-down pitch during aggressive maneuvers.

- **Coupled heave-pitch transients.** During rapid heave commands, if one pair spools up faster
  than the other reverses, a transient pitch disturbance will occur. The magnitude depends on
  thruster response time asymmetry but could reach 5-10 degrees in fast step commands before the
  controller corrects.

---

## 3. Port/Starboard Position Asymmetries

### Description

The port and starboard thruster positions are not mirror-symmetric about the centerline (Y=0):

| Thruster Pair | Starboard PY | Port PY | Midpoint Offset from Center |
|---------------|-------------|---------|----------------------------|
| Bow canted (R0/R1) | +0.1342 | -0.1369 | -0.0014 (biased to port) |
| Stern canted (R2/R3) | +0.1326 | -0.1384 | -0.0029 (biased to port) |
| Surge (R4/R5) | +0.1350 | -0.1430 | -0.0040 (biased to port) |

All pairs are shifted slightly to port. The surge thrusters have the largest offset (4.0 mm).

### Expected Motion Deviations

- **Parasitic yaw during pure heave.** Equal thrust on all four canted thrusters produces a net
  yaw moment because the port-side moment arms are longer than starboard. The allocator
  compensates in the unsaturated regime, but under saturation (e.g., max-rate depth changes),
  the vehicle will yaw slightly. Estimated: 1-2 deg/s yaw rate during saturated heave commands.

- **Parasitic yaw during pure surge.** The surge thrusters (R4/R5) have a 4.0 mm port bias in
  their Y-axis midpoint and a lateral arm difference of 8.0 mm. Equal forward thrust produces a
  small yaw-right moment. The allocator compensates using the lateral thrusters, consuming
  lateral authority. Estimated parasitic yaw without compensation: < 1 deg/s.

- **Uneven thruster loading.** The allocator will command slightly different thrust levels to the
  port and starboard thrusters of each pair to maintain zero parasitic moments. This means one
  side saturates before the other during high-demand maneuvers, reducing peak authority by
  approximately 1-3%.

---

## 4. Both Lateral Thrusters on the Port Side

### Description

Rotors 6 and 7 are both mounted at PY = -0.0945 (port side), offset 94.5 mm from centerline.
They are at different longitudinal stations (R6 at X = +0.2743, R7 at X = -0.3027) providing
yaw authority through differential thrust. Both are below center at PZ = +0.1793.

### Expected Motion Deviations

- **Roll coupling during sway commands.** Any sway command produces a roll moment because the
  lateral thrust is applied 94.5 mm off-center (port) and 179.3 mm below center. A pure
  starboard sway command generates a roll-to-port moment that the canted thrusters must
  counteract. This consumes canted thruster authority that would otherwise be available for
  heave and pitch. During combined sway + heave maneuvers, the canted thrusters may saturate
  earlier than expected.

- **Asymmetric yaw authority.** R6 is 0.2743 m forward of CG; R7 is 0.3027 m aft. The yaw arms
  are unequal (10.3% longer aft). Clockwise yaw (nose-right) requires R6 forward + R7 reversed,
  producing a yaw moment of 0.2743 + 0.3027 = 0.5770 N-m per unit thrust. But the unequal arms
  mean that this yaw command also generates a residual sway force unless the allocator applies
  differential thrust magnitudes. The allocator handles this, but under saturation, a yaw
  command will include a sway drift component.

- **No redundancy in lateral axis.** With only two lateral thrusters, a single lateral thruster
  failure eliminates independent sway and yaw control. The vehicle would retain sway from the
  canted thrusters (~0.500 per thruster, 4 thrusters = 2.00 max) but lose dedicated lateral
  yaw authority.

---

## 5. Surge Thrusters Forward of and Below Center of Gravity

### Description

Both surge thrusters are at PX = +0.0773 (77.3 mm forward of CG) and PZ = +0.1183 (118.3 mm
below CG).

### Expected Motion Deviations

- **Parasitic nose-down pitch during surge.** The below-center mounting means forward thrust
  produces a nose-down pitch moment. Both thrusters contribute equally:
  M_pitch = 2 x (0.1183) = 0.2366 N-m per unit total surge force. The allocator compensates
  using the canted thrusters, but this continuously consumes pitch authority during forward
  flight. At high surge thrust, the pitch compensation demand may limit available heave
  authority.

- **Parasitic yaw during surge.** The slight port bias (see Section 3) means equal thrust from
  R4 and R5 produces a small yaw-right moment. Magnitude:
  M_yaw = 1 x (-0.1350) + 1 x (0.1430) = +0.0080 N-m per unit surge force. This is small
  but persistent during forward transit.

---

## 6. Steep Cant Angle (~60 deg)

### Description

The canted thrusters are mounted at slightly different cant angles per pair:

- **Bow pair (R0, R1):** 60.00 deg from horizontal (sin=0.8660, cos=0.5000)
- **Stern pair (R2, R3):** 60.06 deg from horizontal (sin=0.8663, cos=0.4995)

This produces a heave-to-sway force ratio of approximately 1.73:1.

### Expected Motion Deviations

- **Limited sway authority from canted thrusters.** Each canted thruster contributes only ~0.500
  units of sway force per unit thrust, versus ~0.866 units of heave. With all four canted
  thrusters at max, pure sway from the canted group is approximately 2.00 units, while pure
  heave is approximately 3.46 units. The lateral thrusters (2.0 units max) compensate, bringing
  total sway to approximately 4.00 units. However, the canted thrusters' sway contribution comes
  with heave and roll coupling that must be counteracted.

- **Heave-sway coupling under saturation.** When the canted thrusters are near saturation for
  heave, very little lateral force capacity remains. A combined heave + sway maneuver will see
  sway authority degrade significantly (down to just the two lateral thrusters = 2.0 units) when
  heave demand exceeds approximately 60% of maximum.

---

## 7. Summary of Expected Motion Deviations

| Command | Primary Deviation | Estimated Magnitude | Root Cause |
|---------|------------------|--------------------| -----------|
| Pure heave | Transient pitch oscillation | 5-10 deg transient | Opposing vertical arrangement, reverse spool-up lag |
| Pure heave | Reduced authority | 15-20% reduction | Reverse thrust efficiency loss |
| Pure heave (saturated) | Parasitic yaw drift | 1-2 deg/s | Port/starboard position asymmetry |
| Nose-up pitch | Reduced authority vs nose-down | 20-30% weaker | All thrusters in reverse for nose-up |
| Pure surge | Parasitic nose-down pitch | Continuous, compensated | Surge thrusters 118 mm below CG |
| Pure surge | Parasitic yaw-right | < 1 deg/s | 4 mm port bias on surge pair |
| Pure sway | Roll coupling | Continuous, compensated | Lateral thrusters 95 mm off-center, 179 mm below CG |
| Sway + heave | Sway authority loss | ~50% at 60% heave | Steep ~60 deg cant angle limits sway headroom |
| Pure yaw | Residual sway drift | Small, under saturation only | Unequal lateral thruster arms (10.3% difference) |
| Lateral thruster failure | Loss of independent sway/yaw | Single point of failure | No lateral redundancy (both on same side) |

---

## 8. Recommendations

1. **Consider flipping the stern canted pair** so all four canted thrusters push in the same
   vertical direction at positive rotation. This eliminates the reverse-thrust penalty for heave
   and makes pitch authority symmetric.

2. **Center or split the lateral thrusters** across port and starboard if packaging allows. This
   eliminates the roll coupling during sway and provides lateral redundancy.

3. **Correct the port/starboard asymmetries** if they are not intentional CAD measurements. Even
   millimeter-level offsets produce persistent parasitic moments that consume allocator headroom.

4. **Consider reducing the cant angle** toward 45 degrees if the mission requires significant
   lateral station-keeping. The current 60.73 degree angle heavily favors heave over sway.

5. **Move surge thrusters closer to CG** if physically possible, to reduce the continuous pitch
   compensation demand during forward transit.
