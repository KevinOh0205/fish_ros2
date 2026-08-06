# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

ROS2 Jazzy control software for an underwater fish robot, running on a Raspberry Pi 5 (Ubuntu Noble).
Single package `fish_robot_pkg`, 7 C++ nodes, no custom messages. The Pi does sensor fusion, PID
control, and logging; an **nRF52840 MCU** (firmware lives outside this repo) owns the RC link, the
IMU, and the servo/ESC PWM outputs.

## Commands

```bash
# Build (from workspace root)
colcon build --packages-select fish_robot_pkg
source install/setup.bash

# Run everything
ros2 launch fish_robot_pkg fish_robot.launch.py

# Run one node in isolation
ros2 run fish_robot_pkg state_estimation_node

# The launch normally runs as a systemd service (enabled + active on this machine).
sudo systemctl {status,restart,stop} fish-robot
journalctl -u fish-robot -f              # node stdout goes to journald, not a terminal
```

**Stop the service before launching manually** — otherwise two copies of every node fight over
`/dev/ttyAMA0`, the I2C bus, and GPIO 21 (`libgpiod` line request will throw and kill
`rpm_driver_node`).

**There are no tests.** `package.xml` declares `ament_lint_auto`/`ament_lint_common` as test deps,
but `CMakeLists.txt` has no `BUILD_TESTING` block, so `colcon test` runs nothing. Verification is
done on hardware by reading topics (`ros2 topic echo`) and the CSV logs in `log_csv/`.

## Architecture

Everything is a **100 Hz (10 ms) loop**. Data flows one direction with no feedback except the motor
command returning to the MCU:

```
nRF52840 ──UART 46B──▶ uart_bridge_node ──▶ /raw/imu_6dof ──┐
                              ▲                /rc/command   │
                              │                /rc/status ───┤
                              │                              ▼
I2C sensors ──▶ i2c_driver_node ──▶ /raw/magnetometer ──▶ state_estimation_node
                                    /sensor/pressure_raw          │
                                                                  ▼
                                                        /filtered/attitude
                                                          │       │       │
                              ┌───────────────────────────┘       │       └──▶ data_logger_node
                              ▼                                   ▼                  │
                     auto_scenario_node ──/auto/command──▶ pid_control_node           ▼
                                                                  │            log_csv/*.csv
                              └──────────/motor/output────────────┘
                                             │
                                   uart_bridge_node ──UART 10B──▶ nRF52840
```

`rpm_driver_node` is independent: GPIO 21 rising-edge interrupts → `/sensor/tail_rpm`.

### Node roles

| Node | Owns |
|---|---|
| `uart_bridge_node` | `/dev/ttyAMA0` 115200 8N1. Byte-level packet state machine. **Pure pass-through — no scaling, no sign flips.** |
| `i2c_driver_node` | `/dev/i2c-1`: AK8963 magnetometer (via MPU9250 bypass) + 3× MS5837 behind a TCA9548A mux |
| `state_estimation_node` | Mahony filter, gyro/pressure/mag zeroing, button UI. The "brain". |
| `pid_control_node` | PID + motor mixing. Output `[0]=left servo, [1]=right servo, [2]=yaw servo, [3]=tail BLDC` |
| `auto_scenario_node` | Time-based trajectory generator for AUTO mode |
| `rpm_driver_node` | Hall-sensor pulse counting in a worker thread |
| `data_logger_node` | 23-column CSV snapshot at 100 Hz |

## Conventions you must know before editing

These are load-bearing and span multiple files. Breaking one silently corrupts behavior.

### 1. Mode is smuggled inside the attitude message

There is no mode topic. `state_estimation_node` adds **±5000 to `attitude.x` (roll)** when in AUTO
mode. Three nodes decode it independently with the same `|x| > 2500` test:

- `pid_control_node.cpp` `attitude_callback`
- `auto_scenario_node.cpp` `attitude_callback`
- `data_logger_node.cpp` `attitude_callback`

Change the encoding in one place and you must change all four.

### 2. IMU units are **g** and **deg/s**, not REP-103

`/raw/imu_6dof` is a `sensor_msgs/Imu`, but the nRF firmware sends **g** and **deg/s**, and
`uart_bridge_node` publishes them unconverted. Confirmed by measurement (|a| ≈ 1.03 at rest, not
9.81) and by `state_estimation_node.cpp:523/525` multiplying the gyro by `M_PI/180`.

Accel is fed to Mahony **unconverted** — the filter normalizes it, so scale doesn't matter, only
direction. Any new consumer of this topic must apply the conversion itself.

### 3. Message types are reused as plain containers

- `/rc/command` is a `geometry_msgs/Quaternion` holding **4 raw stick ints**, not a rotation:
  `x=roll, y=pitch, z=yaw, w=throttle`. Same for `/auto/command` (`x/y/z` in **0.1°** units, `w` in
  **PWM µs**).
- `/rc/status` is `Int32MultiArray[5]` = `[btn1, btn2, vbat1×100, vbat2×100, rssi]`. Voltages are
  centivolts — divide by 100.
- `/sensor/pressure_raw` is `Float32MultiArray[6]`: `[0..2]` pressure (mbar), `[3..5]` temperature (°C).

### 4. `-9999` is the RC-link-lost sentinel

The nRF sends `throttle = -9999` when the transmitter link drops. `pid_control_node` treats it two
ways — the explicit sentinel, and a 5 s timeout on `last_valid_rc_time_` (for when the nRF itself
dies). `data_logger_node` deliberately records it unfiltered so dropouts are visible in the CSV.

**Note:** the failsafe is skipped entirely in AUTO mode (`pid_control_node.cpp` — `!is_auto_mode_ &&`),
by design, since autonomous running without a transmitter is the normal case.

### 5. The control loop is event-driven, not timed

`pid_control_node` has **no timer**. PID runs inside `attitude_callback`, so the control rate is
whatever `/filtered/attitude` publishes at. The PID also **omits `dt`** — gains absorb it, assuming
a fixed 100 Hz. If the attitude rate ever changes, the gains are wrong.

`state_estimation_node` **no longer** shares that assumption: `dt_` is measured per sample from
`header.stamp`, clamped to `[0.001, 0.1]` s, and a clamp event is logged. The clamp exists because
Mahony's gain is fixed — a large `dt` becomes a large correction step (`Kp·dt`), so it must be
bounded. `state_estimation_ekf_node` deliberately has **no** upper clamp; its covariance adjusts the
trust automatically, so integrating the true `dt` is strictly better there.

### 6. Startup is not instantaneous

- **Gyro calibration blocks for ~10 s** (1000 samples). `imu_callback` returns early and publishes
  **nothing** during this window — so no `/filtered/attitude`, so no motor output. The robot must be
  completely still. Handling it during this window poisons the bias for the whole session.
- **Pressure zeroing waits 300 s** (`PRESSURE_WARMUP_SEC`) before capturing baseline, because warm-up
  drift correlates with elapsed time (R²≈0.95), not temperature. Re-zero afterwards with a btn1 long press.

### 7. Hardware quirks

- **`gpiochip4`**, not `gpiochip0` — the Pi 5 moved the 40-pin header onto the RP1 chip. Changing
  boards means changing `rpm_driver_node.cpp`.
- RPM resolution is coarse: 1 pulse per 100 ms window = **200 RPM**.
- The three MS5837 sensors all share address `0x76`, hence the mux. Pressure updates at **~11 Hz**
  despite the 100 Hz timer — a 3-state machine spreads the 17 ms ADC conversions across ticks.
- A dead pressure channel is marked by `prom_C_[i][1] == 0` and reported downstream as `0.0`;
  `state_estimation_node` treats `< 100 mbar` as invalid.
- `SERVO_MIN_US`/`SERVO_MAX_US` (1250/1750) in `pid_control_node.cpp` **must match the nRF firmware's
  `MotorControl.cpp`**. This repo cannot enforce that.

### 8. Magnetometer failure is a supported state

If AK8963 doesn't answer for 500 ms, `state_estimation_node` falls back from 9-DOF to 6-DOF Mahony
automatically. **As of 2026-07-31 the AK8963 IS responding (~94 Hz), so the system runs 9-DOF.**
Measured yaw drift in that state is **+0.04°/min**. The −8°/min figure quoted previously applies only
to the 6-DOF case (mag dead), where yaw has no absolute reference; do not use it as a 9-DOF baseline.

**A bad magnetic vector corrupts roll/pitch, not just yaw** — `update_9dof_mahony` adds the magnetic
error into the *same* correction vector as gravity. Measured at rest 2026-07-31 with the old
calibration: `gravity → roll −1.19 / pitch 3.75` vs `Mahony → roll −6.16 / pitch 2.10` (**5.0° / 1.7°
off**). This is why the calibration below matters to the *control* loop and not just to heading. The
EKF does not share the flaw: its magnetometer update is a 1-D horizontal yaw projection
(`H_m = [ĝ_bᵀ 0]`), so it is structurally incapable of moving roll/pitch.

#### The axis remap is CORRECT — settled 2026-08-06 by a tumble, do not reopen it

`(my, mx, −mz)` at `state_estimation_node.cpp:495-521` is **verified**. A 1164-point tumble (all 8
octants covered) fitted an axis-aligned ellipsoid; after correction the **dip reads 144.3° ± 3.2°
against Korea's 143°**. Dip is the angle between the field and gravity — it can only come out right if
the hard iron *and* all three magnetometer axes are right, so this closes both questions at once.

That the remap's stated *derivation* is bogus remains true and does not matter: the comment derives the
Z negation from the MPU9250 datasheet's AK8963-vs-MPU6500 die alignment, but accel/gyro come from the
**nRF52840's own IMU**, not the MPU6500 (which `i2c_driver_node` never wakes; the MPU9250 is only an
I²C bypass to reach the AK8963). Right answer, wrong reason. Measurement settled it.

**This result is permanent** — chip axes do not change when the robot is assembled or moved. The
*numbers* below are not.

#### What the tumble measured

All four candidate calibrations, scored on the same tumble dataset (dip over stationary samples only):

| | \|m\| | scatter | dip (143° = correct) |
|---|---|---|---|
| no calibration | 94.4 ± 20.9 µT | 22.1 % | 94.7 ± 51.0° |
| previous session's file | 90.7 ± 19.2 µT | 21.1 % | 101.9 ± 39.3° |
| **btn2 min/max, taken the same day in the same configuration** | 45.9 ± 17.5 µT | **38.1 %** | 124.7 ± 24.1° |
| **tumble + ellipsoid fit** | 34.7 ± 1.5 µT | **4.3 %** | **144.5 ± 2.9°** |

Hard iron is **93.6 µT total, dominated by chip Z (−89.8)** — nearly three times Earth's field here.
A flat spin can only see the horizontal part (37.8 µT) and is therefore **structurally unable** to
find this; the earlier "37.8 µT" figure was never the whole offset. Never calibrate from a flat spin.

Two method traps, both hit on 2026-08-06:

- **`min/max` — what btn2 and `/calibrate_mag` do — is worse than no calibration at all.** Not a
  hypothesis: btn2 was run on 2026-08-06 18:29 in this exact configuration and scored **38.1 %
  scatter against 22.1 % for doing nothing** (row 3 above). It takes two extremes per axis, so any
  hand tumble that misses a true extreme puts the midpoint in the wrong place, and the resulting
  soft-iron scales (0.953/0.953/1.108) then actively distort a field that was at least self-consistent
  before. Use an ellipsoid fit over all points — `tumble.py` in the session scratchpad.
  **Every btn2 long press silently overwrites `mag_calib_params.txt`**, so a stray 3 s press can undo
  a good calibration without any obvious symptom until yaw misbehaves.
- **Center the data before fitting the quadric.** `a·x²+…=1` on raw values fails outright (returns
  negative radii) because the cloud sits ~65 µT off the origin, so the squared terms are nearly
  collinear. Subtract the mean, fit, add it back.

#### What is still open

- **Field magnitude is 30 % low**: 34.7 µT after correction against Korea's ~50 µT. Direction is
  right, so heading and attitude are unaffected — but a real scale error would also mean the AK8963
  ASA sensitivity ROM may not be applied anywhere in `i2c_driver_node`. Unverified.
- ~~End-to-end heading~~ — **CLOSED 2026-08-06 by the 360° spin.** See the sub-section below.
- **Thermal drift, unresolved.** After a 4-core CPU load the field moved 3 µT and its noise tripled
  (σ 0.68 → 2.64) *in the phase after the load* — consistent with heat reaching the sensor rather than
  current. 3 µT ≈ 8° of heading, which may cap achievable accuracy. See the load test below.

#### End-to-end heading — PASSED 2026-08-06 (360° spin, `scripts/verify_spin.py`)

One 66 s hand-held level turn, gyro-about-gravity as truth (that axis was itself verified to +359.96°).

| | sweep | ratio | backtracking (0.5 s smoothed) |
|---|---|---|---|
| physical (gyro) | −357.74° | — | — |
| **calibrated mag** | **−361.79°** | **1.01** | **7.7°** |
| uncalibrated mag | −1.55° | 0.00 | 102.0° |
| Mahony yaw | −359.54° | 1.01 | 0.0° |

Total sweep alone proves nothing — check the **per-45° segments**. Calibrated: 38–52°, all near 45.
Uncalibrated: 12–19° per segment, then **−45.1°** in 270–315° — it runs slow and then reverses. That
reversal, not the total, is what made the old heading unusable.

**Score backtracking on smoothed data.** At 100 Hz and 33 °/s the true step is 0.33°/sample, well under
the mag noise, so per-sample sign flips are noise: raw scoring returns >3500° for *every* series
including the good one.

Residual against the gyro: **median 2.22°, 95 %ile 5.87°, σ 3.10°** — and it is a **single sine cycle
per revolution, amplitude 3.58°**, which is the signature of leftover hard iron. Back-solving gives
**1.34 µT horizontal**, matching the ellipsoid fit's own 1.5 µT residual. So the remaining error is the
known calibration residual showing up exactly as predicted, not a new fault; removing that one cycle
leaves 2.08°. The lone 18.1° excursion is a momentary spike at −3 °/s (10 of 6601 samples exceed 10°).

`|m|` scatter while turning was **2.5 %**, better than the tumble's 4.3 % — a level spin is the easy
case, but it confirms the calibration does not fall apart with attitude.

**Subtract the gyro bias before using the gyro as truth.** It was **−2.2 °/s** in this session (see the
gyro-bias gap below), i.e. 66° of phantom rotation per 30 s. `verify_spin.py` measures it from a 6 s
stationary window first, the same thing `state_estimation_node` does at boot.

#### The Pi is not the cause (measured 2026-08-06)

40 s idle → 40 s 4-core 100 % → 40 s idle, robot stationary to 0.07°. Load confirmed (1647 → 2400 MHz,
65.3 → 75.9 °C). Field moved **2.15 µT, only 1.05 µT of it horizontal (≈2.7° of heading)** — about 5 %
of the contamination. **The offset is essentially constant, which is why calibrating it works at all.**

#### Calibration is tied to the ASSEMBLY, not the place

Hard/soft iron are body-fixed, so a calibration travels to the pool unchanged. An environment-fixed
external field does **not** shift the fitted center — in the body frame it rotates with the robot like
Earth's own field, changing only the radius. So:

- **Adding or removing any part invalidates the numbers.** The current file was taken with the robot
  **stripped to the IMU alone** and must be redone after final assembly.
- Tumble in one spot, ≥1 m from steel furniture. Carrying the robot around while tumbling walks it
  through a changing external field and corrupts the fit.

Coefficients load from `~/ros2_ws/log_csv/mag_calib_params.txt` — **read once at node startup**, so
editing it requires a restart. (The `mag_calib.txt` at the workspace root is **not** read by any code.)
Current contents, written 2026-08-06 from the tumble; the previous file is kept as `.bak_20260806_194340`:

```
-12.2555 23.5024 -89.7587      # hard iron, chip axes
1.046 0.983502 0.973518        # soft iron scale
```

The built-in triggers still use the inferior min/max method — prefer `tumble.py`:

```bash
ros2 service call /calibrate_mag std_srvs/srv/Trigger    # call twice: start, then finish
```

### 9. Button UI (`/rc/status`, 1 tick = 10 ms)

Short press = 30 ms–1 s; long press = exactly 3 s (`== 300`, fires once).

| | Short | Long |
|---|---|---|
| btn1 | AUTO/MANUAL toggle | re-zero atmospheric pressure |
| btn2 | set current heading as yaw 0 | start magnetometer calibration |

When the RC link drops the nRF sends `btn = 255`; both `state_estimation_node` and `data_logger_node`
explicitly reject non-0/1 values so the `1 → 255` transition isn't misread as a release.

## Known gaps

- **Magnetometer — the axis question is CLOSED, the calibration is provisional** (2026-08-06, §8).
  The tumble confirmed `(my, mx, −mz)` (dip 144.3° vs 143°) and cut sphericity 22.1 % → 4.3 %; that
  axis result is permanent. But the coefficients now on disk were taken with the robot **stripped to
  the IMU alone** and must be redone after final assembly. Until then, restart
  `state_estimation_node` to pick up the new file (it reads it once at startup) — the old file was
  worse than none and is what put the 5° error into Mahony's roll. **End-to-end heading now passes**
  (360° spin, ratio 1.01, residual median 2.2°, §8) — so the coefficients are good *for this
  configuration*; what stays provisional is only that the configuration will change on assembly.
- **The IMU axis transform is verified end to end** (`state_estimation_node.cpp:575-580`,
  `R = [0 0 −1; −1 0 0; 0 1 0]`, `det = +1`) — accelerometer and gyro, all three axes, signs included.
  Accel, tilt cross-axis test 2026-08-04 (two runs): the physical rotation axis sits **0.01°** from
  where the non-level baseline predicts; nose up → pitch negative, port side up → roll positive.
  Gyro, 2026-08-06: integrate the gyro as a quaternion and compare its predicted gravity against the
  (already verified) accel. Residual **at rest** after a 61 s run was 1.79° worst case. A flat 360°
  spin closed to **+359.96°** — magnitude and sign both right (counterclockwise = `yaw+` = 좌선회) —
  and left only 0.53° of tilt residual, which is what rules out gyro Z leaking into X/Y.
  Judge that test on the **at-rest** residual, never on the error during motion: sliding the robot by
  hand pushed the in-motion error to 7.7°, because the accel reads specific force (gravity minus
  linear acceleration), not gravity. It falls back under 1° the moment the robot stops.
  Two things to know before reading a tilt result: (a) the mounting baseline is **pitch ≈ 7.9°**, not
  level, and rotating about a world-horizontal axis from a tilted start moves the Euler *other* angle by
  ~2° at 45° — a geometric artifact, not an axis error; (b) a wrong axis transform is a rotation, and
  rotations preserve angles, so it can **never** change the measured sweep angle. A sweep that reads
  short means the sensor is not turning with the hull (mounting), or the applied angle was misjudged.
  **The sensor does track the hull** — 90° test 2026-08-06: laid on its side → 87.1°, stood on its tail
  → 94.8°, against 3.3° repeatability in re-placing it level. Any scale factor able to turn 45° into 29°
  would have read 90° as 58°. Hand-judged tilt angles are worth ±15°; trust the sweep number instead.
  Note the Euler readout **is** degenerate near pitch ±90 — during that nose-up hold the reported roll
  wandered 20° while the true attitude moved 0.9°. Judge stillness on the gravity vector, never on
  Euler angles.
- `state_estimation_ekf_node`: the accelerometer update **must** project the world-vertical direction
  (`g_b`) out of the Kalman gain — `update()` takes a `null_dir` for this and `update_accel` passes
  `&g_b`. Without it the accel update leaked into yaw: shaking the robot in yaw cost **−33°/−37°** per
  event in 6-DOF (Mahony, whose cross-product correction is structurally yaw-free, lost 0.7°/2.7°).
  `H_a`'s null space is `g_b` in theory, but `K = P Hᵀ S⁻¹` re-introduces yaw through any
  yaw↔tilt correlation in `P`, amplified by the 60° initial yaw variance. Project the **gain**, not the
  state correction — projecting `dx` alone leaves `P` still counting yaw information, so the leak's
  cause survives while its symptom hides. Diagnostic: in 6-DOF the yaw 1σ (`ekf_status[5]`) must only
  ever **grow** — nothing observes yaw, so shrinking is proof of a leak (measured 40°→7° before the fix,
  60°→61° after). Do **not** pass `null_dir` to the magnetometer update (yaw is the only thing it
  observes) or to the bias pseudo-measurement (a stationary average genuinely measures all three axes).
- **`update_9dof_mahony` had a wrong rotation-matrix element — fixed 2026-08-06**
  (`state_estimation_node.cpp:199`). `halfwy`'s `bz` coefficient was `0.5 − q1q1 − q3q3` (that is
  `R22`); the correct term is `q0q1 + q2q3` (`R21`, and identically `halfvy`). Check it at identity:
  the predicted field's y component must be 0, but the old expression returned `0.5·bz`.
  **The symptom scaled with how *good* the magnetometer calibration was** — with the dip wrong at
  ~100° `bz` was small and tilt error sat at 5.0°; once the tumble put the dip at 144° `bz` grew ~5×
  and the error jumped to **14.0°**. A latent bug that gets worse as you fix something else. After the
  fix, 6.8°. Never read "recalibration made attitude worse" as "the calibration is bad".
- **The "9-DOF Mahony has a structural ~6.8° tilt error" claim is RETRACTED (2026-08-06).** Re-measured
  at rest on a clean topic: **Mahony 0.134° mean, EKF 0.117°** over 76 s (max gyro rate 0.26 °/s),
  roll/pitch agreeing with gravity to 0.04°/0.02°, and the two filters' yaw separating by only 0.40°
  over that span. There is no measurable structural penalty at rest, so **the proposed fix — projecting
  the magnetic error onto `halfv` — has no demonstrated problem to solve. Do not apply it on theory alone.**
  The mechanism it targets is real (`ex/ey/ez` add the magnetic cross product to the gravity cross
  product, so magnetic direction error *can* rotate tilt); it simply is not costing anything measurable
  once `halfwy` is right and the calibration is good.
  **Why the 6.8° was wrong, and the general lesson:** `/filtered/attitude` had *two* publishers — an
  orphaned `state_estimation_node` (ppid 1) running the now-*deleted* binary alongside the current one —
  so the topic interleaved two filters' output and the subscriber saw a blend. It survived a restart and
  was invisible in `ros2 node list` beyond a duplicated name; `pgrep -x` cannot find it either (`comm`
  truncates at 15 chars). **Before any comparison run:** `ros2 topic info /filtered/attitude --verbose |
  grep -c "Endpoint type: PUBLISHER"` must return **1**, and enumerate `/proc/*/exe` looking for
  `(deleted)`. Note the bad number *looked* trustworthy — stable at 6.7–7.0° across six 30 s windows.
  Stability across windows is not evidence of validity when the contaminant is also stationary.
  Caveat on the new number too: it is **one attitude** (roll −2.7°, pitch 7.3°), stationary. Magnetic
  contamination of tilt is attitude-dependent by nature, so this is not yet a general result.
- **Mahony vs EKF, measured 2026-08-06 (post-fix baseline).** Run in parallel, both 6-DOF, compared by
  subtraction after decoding Mahony's ±5000 and `yaw_offset_`. Static roll/pitch agree to
  **0.030° / 0.070°**; under hand motion they diverge by up to **4.93°**, which is the accel reading
  specific force, not either filter being wrong. Yaw error vs integrated gyro after the `null_dir` fix
  is **−0.80° / −0.20°** per shake event (was −33°/−37°). The two are directly comparable — byte-identical
  axis transforms, units, mag calibration order, Euler extraction and 500 ms mag timeout — so any
  difference is filter behaviour, not setup. **These numbers predate the duplicate-publisher discovery
  above** and were not taken with the publisher count verified; re-take them opportunistically before
  relying on any of them.
- **Gyro z bias is not stable between sessions**: measured **−0.044 °/s** during the gyro axis test and
  **−2.2 °/s** during the comparison runs hours later — a 50× spread. Any 6-DOF yaw-drift number is
  therefore only valid for the session that produced it; do not carry one forward as a spec. The EKF
  estimates bias online and so absorbs this; Mahony's `Ki = 0.005` integral does too, but slowly.
- **`gyro_noise_sigma` is set ~10× above measurement**: configured `0.05`, measured stationary
  **0.0028–0.0069 °/s/√Hz**. Deliberate headroom for vibration and arrival-timestamp jitter — stationary
  noise underestimates in-motion noise — but it has never been checked against an Allan variance, so
  the margin is a guess rather than a bound.
- **Positive pitch = nose DOWN** in this code's Euler convention (opposite of aerospace convention).
  Confirmed by measurement 2026-08-04. Confirm sign expectations before touching the pitch PID or the
  AUTO scenario.
- **Accelerometer needs no calibration — CLOSED 2026-08-06.** Solved from the same tumble as the
  magnetometer (552 stationary points of 1164). Bias `(−0.0043, +0.0010, −0.0028) g`, the three axis
  radii agree to **0.3 %**, and applying the fit moves the gravity direction by only **0.26° mean /
  0.37° max** while barely touching the scatter (1.11 % → 1.07 %). The earlier "~2 % asymmetry, ≤0.6°"
  claim was an artifact of fitting 6 parameters to 3 attitudes — underdetermined, so it reported the
  posture spread as sensor error. Nothing to implement; the ≤0.4 % scale error does not meaningfully
  shift the EKF's `a_ref_` gate either.
- **Roll/pitch have no offset calibration** analogous to `yaw_offset_`, so a non-level mounting shows
  up as permanent nonzero attitude.
- `state_estimation_node.cpp`: `btn1_long_processed_` was missing from the constructor initializer
  list (indeterminate until the first `/rc/status` with `btn1 == 0` cleared it) — **fixed 2026-07-31**.
  Several members (`right_btn_pressed_`, `left_btn_pressed_`, `button_press_start_time_`,
  `left_btn_press_start_time_`, `last_progress_`, `dynamic_pressures_`) are initialized or assigned
  but never read — leftovers from an earlier button implementation.
- `data_logger_node` writes ~40 MB/hour with **no rotation**. `log_csv/` grows without bound.

## Language

Code comments, log messages, and commit history are in **Korean**. Match that when editing — do not
convert existing comments to English.
