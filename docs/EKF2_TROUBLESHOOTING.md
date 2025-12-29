# EKF2 Troubleshooting Guide for UUV with External INS

This guide covers EKF2 diagnostics, troubleshooting, and tuning for underwater vehicles using the Boreas D90 INS as external vision input.

## Current Configuration

```bash
param set-default EKF2_GPS_CTRL 0       # GPS disabled (underwater)
param set-default EKF2_EV_CTRL 15       # Full vision fusion (pos + vel + yaw)
param set-default EKF2_EV_DELAY 60      # Vision delay (ms)
param set-default EKF2_EVP_NOISE 0.01   # Position noise (matches D90 spec)
param set-default EKF2_EVV_NOISE 0.01   # Velocity noise
param set-default EKF2_EVA_NOISE 0.05   # Attitude noise
param set-default EKF2_HGT_REF 0        # Depth sensor as height reference
param set-default EKF2_BARO_NOISE 0.1   # Depth sensor noise
```

---

## Quick Health Check

### 1. Monitor These Topics

```bash
# Primary status
listener estimator_status

# Innovation test ratios
listener estimator_status_flags

# Sensor-specific diagnostics
listener estimator_aid_src_ev_pos
listener estimator_aid_src_ev_vel
listener estimator_aid_src_ev_yaw
listener estimator_aid_src_baro_hgt
```

### 2. Key Health Indicators

| Metric | Good | Warning | Bad |
|--------|------|---------|-----|
| `pos_test_ratio` | < 0.5 | 0.5 - 1.0 | > 1.0 (rejected) |
| `vel_test_ratio` | < 0.5 | 0.5 - 1.0 | > 1.0 (rejected) |
| `hgt_test_ratio` | < 0.5 | 0.5 - 1.0 | > 1.0 (rejected) |
| `hdg_test_ratio` | < 0.5 | 0.5 - 1.0 | > 1.0 (rejected) |

### 3. Preflight Check (test_ratio < 0.5 required)

If preflight fails, check `estimator_status` for:
- `pre_flt_fail_innov_heading`
- `pre_flt_fail_innov_height`
- `pre_flt_fail_innov_pos_horiz`
- `pre_flt_fail_innov_vel_horiz`
- `pre_flt_fail_innov_vel_vert`

---

## Common Problems and Solutions

### Problem 1: Position Jumps or Oscillations

**Symptoms:**
- Vehicle position estimate jumps suddenly
- Oscillation in position hold

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| Wrong delay | Compare D90 timestamp vs PX4 receive | Adjust `EKF2_EV_DELAY` |
| Noise too low | `pos_test_ratio` frequently > 1.0 | Increase `EKF2_EVP_NOISE` |
| D90 glitches | Monitor D90 output directly | Add filtering or increase noise |

```bash
# If position jumps on motion, delay is likely wrong
param set EKF2_EV_DELAY 80   # Try increasing
```

### Problem 2: Height Estimate Drifts

**Symptoms:**
- Depth doesn't match actual depth
- Slow drift over time

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| D90 vertical dominating | Check `EKF2_EV_CTRL` | Set to 14 (disable vertical) |
| Depth sensor noise too high | Check `EKF2_BARO_NOISE` | Lower to 0.05-0.1 |
| Depth sensor not primary | Check `EKF2_HGT_REF` | Set to 0 |

```bash
# Trust depth sensor for height
param set EKF2_HGT_REF 0
param set EKF2_BARO_NOISE 0.1
param set EKF2_EV_CTRL 14     # Remove vertical from D90
```

### Problem 3: Heading Drift or Jumps

**Symptoms:**
- Yaw estimate doesn't match actual heading
- Sudden heading changes

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| D90 heading delay | Heading lags motion | Increase `EKF2_EV_DELAY` |
| Attitude noise too low | `hdg_test_ratio` > 1.0 | Increase `EKF2_EVA_NOISE` |
| Magnetometer interfering | Check `EKF2_MAG_TYPE` | Set to 5 (no mag fusion) |

```bash
# Disable magnetometer (trust D90 for heading)
param set EKF2_MAG_TYPE 5
```

### Problem 4: Velocity Estimate Noisy

**Symptoms:**
- Jerky velocity estimates
- Control oscillations

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| D90 velocity noise | Monitor raw D90 data | Increase `EKF2_EVV_NOISE` |
| IMU conflict | Check attitude alignment | Align D90 and PX4 frames |
| Noise too tight | `vel_test_ratio` spikes | Increase `EKF2_EVV_NOISE` |

```bash
# Relax velocity trust slightly
param set EKF2_EVV_NOISE 0.02
```

### Problem 5: Innovations Frequently Rejected

**Symptoms:**
- `innovation_rejected` flags true
- `test_ratio` > 1.0 consistently

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| Noise parameters too tight | All test ratios | Increase noise values |
| Sensor mismatch | Compare D90 vs PX4 estimates | Check frame alignment |
| Latency wrong | Jumps on motion | Adjust `EKF2_EV_DELAY` |

```bash
# Loosen all noise parameters
param set EKF2_EVP_NOISE 0.05
param set EKF2_EVV_NOISE 0.05
param set EKF2_EVA_NOISE 0.1
```

### Problem 6: Filter Divergence

**Symptoms:**
- Position/velocity estimates become unrealistic
- Fault flags set

**Causes & Fixes:**

| Cause | Check | Fix |
|-------|-------|-----|
| D90 dropout | Monitor D90 data rate | Ensure consistent 1000Hz output |
| Bad IMU | `fs_bad_acc_vertical` flag | Check IMU mounting/vibration |
| Initialization failed | Alignment flags | Allow proper warmup time |

```bash
# Check fault status
listener estimator_status_flags
```

---

## Innovation Gates Reference

These define how many standard deviations a measurement can differ before rejection:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `EKF2_EVP_GATE` | 5.0 SD | External vision position |
| `EKF2_EVV_GATE` | 3.0 SD | External vision velocity |
| `EKF2_HDG_GATE` | 2.6 SD | Heading |
| `EKF2_BARO_GATE` | 5.0 SD | Barometer/depth |

**Interpretation:**
- `test_ratio = (innovation²) / (gate² × variance)`
- `test_ratio > 1.0` → measurement rejected
- Larger gate = more tolerant of outliers

---

## EKF2_EV_CTRL Bitmask Reference

| Bit | Value | Function |
|-----|-------|----------|
| 0 | 1 | Vertical position |
| 1 | 2 | Horizontal position |
| 2 | 4 | 3D velocity |
| 3 | 8 | Yaw |

**Common Configurations:**

| Value | Fusion | Use Case |
|-------|--------|----------|
| 15 | All (pos + vel + yaw) | Full D90 trust |
| 14 | Horizontal pos + vel + yaw | Depth sensor for vertical |
| 7 | Position + velocity (no yaw) | Use PX4 mag for heading |
| 6 | Velocity + yaw only | Position from other source |

---

## Diagnostic Commands

### Real-time Monitoring

```bash
# Overall EKF status
listener estimator_status -n 1

# Check what's being fused
listener estimator_status_flags -n 1

# External vision innovations
listener estimator_aid_src_ev_pos -n 1
listener estimator_aid_src_ev_vel -n 1

# Height innovations
listener estimator_aid_src_baro_hgt -n 1
```

### Log Analysis

Key log topics for post-flight analysis:
- `estimator_status`
- `estimator_innovations`
- `estimator_status_flags`
- `vehicle_local_position`
- `vehicle_attitude`

---

## Recommended Tuning Process

### Step 1: Start Conservative

```bash
param set EKF2_EVP_NOISE 0.1    # Start loose
param set EKF2_EVV_NOISE 0.1
param set EKF2_EVA_NOISE 0.1
```

### Step 2: Monitor Test Ratios

Run the system and check:
- If `test_ratio` consistently < 0.3 → tighten noise
- If `test_ratio` frequently > 0.8 → loosen noise
- If `test_ratio` > 1.0 → measurement rejected (too tight)

### Step 3: Tighten Gradually

```bash
# If stable, tighten toward D90 specs
param set EKF2_EVP_NOISE 0.05
param set EKF2_EVV_NOISE 0.05
param set EKF2_EVA_NOISE 0.05

# Monitor, then tighten more if stable
param set EKF2_EVP_NOISE 0.01   # D90 spec
param set EKF2_EVV_NOISE 0.01
```

### Step 4: Verify Delay

1. Command a step motion
2. Watch for position overshoot/lag
3. Adjust `EKF2_EV_DELAY` until response is crisp

---

## D90 INS Specifications Reference

| Parameter | D90 Spec | Recommended EKF Setting |
|-----------|----------|-------------------------|
| Position (RTK) | 0.01 m | `EKF2_EVP_NOISE` = 0.01-0.05 |
| Velocity | 0.005 m/s | `EKF2_EVV_NOISE` = 0.01-0.02 |
| Roll/Pitch | 0.005° | N/A (within attitude) |
| Heading | 0.01° | `EKF2_EVA_NOISE` = 0.01-0.05 |

**Note:** Underwater without GNSS, position accuracy degrades (INS drift). Consider using looser position noise for extended dives.

---

## File References

| File | Description |
|------|-------------|
| `src/modules/ekf2/EKF2.cpp` | Main EKF2 module |
| `src/modules/ekf2/EKF/ekf.h` | Core EKF interface |
| `src/modules/ekf2/EKF/control.cpp` | Fusion mode control |
| `msg/EstimatorStatus.msg` | Status message definition |
| `msg/EstimatorInnovations.msg` | Innovations message |
| `msg/EstimatorAidSource*.msg` | Per-sensor diagnostics |

---

## Quick Reference Card

```
┌─────────────────────────────────────────────────────────────┐
│                    EKF2 QUICK DIAGNOSTICS                   │
├─────────────────────────────────────────────────────────────┤
│ GOOD:     test_ratio < 0.5     → Sensor matches prediction  │
│ WARNING:  test_ratio 0.5-1.0   → Marginal fit               │
│ BAD:      test_ratio > 1.0     → REJECTED                   │
├─────────────────────────────────────────────────────────────┤
│ Position jumps?  → Check EKF2_EV_DELAY                      │
│ Height drift?    → Set EKF2_HGT_REF=0, EKF2_EV_CTRL=14      │
│ Heading wrong?   → Set EKF2_MAG_TYPE=5                      │
│ Oscillations?    → Increase noise parameters                │
│ Rejections?      → Loosen noise or check sensor data        │
└─────────────────────────────────────────────────────────────┘
```
