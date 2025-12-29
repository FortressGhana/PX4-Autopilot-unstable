# UUV Reconbot Motor Mapping Issues

## Current Issues in `60003_uuv_reconbot`

### Issue 1: Rotors 6 & 7 on Same Side (CRITICAL)

**Problem:** Both lateral thrusters are on starboard side with same thrust direction.

```
Current config:
  Rotor 6: PY = +0.088 (starboard), AY = +1 (thrust right)
  Rotor 7: PY = +0.088 (starboard), AY = +1 (thrust right)

Result: Can only yaw LEFT, cannot yaw RIGHT
```

**Effectiveness Matrix (Yaw Row):**
```
Rotor 6: +0.306  (positive yaw)
Rotor 7: +0.270  (positive yaw)
         ^^^^^^
         Both same sign = unidirectional yaw
```

**Fix:**
```bash
# Move rotor 7 to port side with opposite thrust
param set-default CA_ROTOR7_PY -0.08809   # Port side
param set-default CA_ROTOR7_AY -1         # Thrust left
```

**After fix:**
```
Rotor 6: +0.306  (positive yaw - turn left)
Rotor 7: -0.270  (negative yaw - turn right)
         ^^^^^^
         Opposite signs = bidirectional yaw
```

---

### Issue 2: Rotors 4 & 5 Asymmetric Z Position

**Problem:** Forward thrusters at different heights creates roll coupling with surge.

```
Current config:
  Rotor 4: PZ = -0.09882 (below center)
  Rotor 5: PZ = +0.09882 (above center)

Result: Commanding forward thrust also creates roll torque
```

**Effectiveness Matrix (Pitch Row):**
```
Rotor 4: -0.0988  (pitch down)
Rotor 5: +0.0988  (pitch up)
         ^^^^^^
         Opposite signs = pitch coupling when surging
```

**Fix:**
```bash
# Make both at same height
param set-default CA_ROTOR5_PZ -0.09882   # Same as Rotor 4
```

---

### Issue 3: Comments Don't Match Configuration

**Problem:** Header comments say rotors 5-8 are "vertical" but actual config is:

| Rotor | Comment Says | Actual Thrust Direction |
|-------|--------------|------------------------|
| 4 | "bow starboard vertical" | Forward (AX=1) |
| 5 | "bow port vertical" | Forward (AX=1) |
| 6 | "stern starboard vertical" | Lateral (AY=1) |
| 7 | "stern port vertical" | Lateral (AY=1) |

**Fix:** Update comments in airframe file header.

---

## Common Motor Mapping Problems

| Issue | Symptom | How to Check |
|-------|---------|--------------|
| Same-side thrusters | Can only move/rotate one direction | Check PY signs for lateral thrusters |
| Wrong thrust direction | Vehicle moves opposite to command | Check AX/AY/AZ signs |
| Asymmetric positions | Unwanted coupling (e.g., roll when surging) | Compare PZ values for paired thrusters |
| Missing DOF | Can't control one axis | Run effectiveness calculator, check rank |
| Zero thrust axis | No movement in one direction | Check effectiveness matrix row for all zeros |
| High condition number | Poor control authority on some axes | Check singular values |

---

## Verification Tool

Run the effectiveness calculator to verify configuration:

```bash
# Check current config
python3 Tools/uuv_effectiveness_calculator/effectiveness_calculator.py \
    --airframe ROMFS/px4fmu_common/init.d/airframes/60003_uuv_reconbot

# Check fixed config
python3 Tools/uuv_effectiveness_calculator/effectiveness_calculator.py --fixed
```

**What to look for:**

| Check | Good | Bad |
|-------|------|-----|
| Rank | 6 | < 6 (missing DOF) |
| Yaw row | Has + and - values | All same sign |
| Thrust rows | Non-zero where expected | All zeros |
| Condition number | < 10 | > 100 (poor authority) |
| Issues detected | None | Any warnings |

---

## Math Reference

### Effectiveness Matrix Formula

For each rotor:
```
Thrust = CT × [AX, AY, AZ]
Torque = CT × (Position × Axis) - CT × KM × Axis
```

### Cross Product (Position × Axis = Torque)

```
Roll  = PY × AZ - PZ × AY
Pitch = PZ × AX - PX × AZ
Yaw   = PX × AY - PY × AX
```

### Yaw Rules for Lateral Thrusters

| Position | Thrust Direction | Yaw Result |
|----------|------------------|------------|
| Stern (+X) | Right (+Y) | + (turn left) |
| Stern (+X) | Left (-Y) | - (turn right) |
| Bow (-X) | Right (+Y) | - (turn right) |
| Bow (-X) | Left (-Y) | + (turn left) |

---

## Fixed Configuration

Apply these changes to fix all issues:

```bash
# Fix Rotor 5: Same Z as Rotor 4
param set-default CA_ROTOR5_PZ -0.09882

# Fix Rotor 7: Port side with opposite thrust
param set-default CA_ROTOR7_PY -0.08809
param set-default CA_ROTOR7_AY -1
```

Or generate full fixed config:
```bash
python3 Tools/uuv_effectiveness_calculator/effectiveness_calculator.py --fixed --generate-params
```
