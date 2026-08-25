# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

Full measurement history — every experiment, retraction, and trap — lives in **`docs/imu_detailed.md`**
(Korean); "§N" below points into it. Its results are settled by measurement — do not reopen them without a new one.

## What this is

ROS2 Jazzy control software for an underwater fish robot, running on a Raspberry Pi 5 (Ubuntu Noble).
Single package `fish_robot_pkg`, 7 C++ nodes, no custom messages. The Pi does sensor fusion, PID
control, and logging; an **nRF52840 MCU** (firmware lives outside this repo) owns the RC link, the
IMU, and the servo/ESC PWM outputs.

## Commands

```bash
colcon build --packages-select fish_robot_pkg       # from workspace root
source install/setup.bash
ros2 launch fish_robot_pkg fish_robot.launch.py     # run everything
ros2 run fish_robot_pkg state_estimation_ekf_node   # run one node in isolation

# The launch normally runs as a systemd service (enabled + active on this machine).
sudo systemctl {status,restart,stop} fish-robot
journalctl -u fish-robot -f              # node stdout goes to journald, not a terminal
```

**Stop the service before launching manually** — otherwise two copies of every node fight over
`/dev/ttyAMA0`, the I2C bus, and GPIO 21 (`libgpiod` line request will throw and kill `rpm_driver_node`).

**There are no tests.** `package.xml` declares `ament_lint_auto`/`ament_lint_common` as test deps,
but `CMakeLists.txt` has no `BUILD_TESTING` block, so `colcon test` runs nothing. Verification is
done on hardware by reading topics (`ros2 topic echo`) and the CSV logs in `log_csv/`.

## Architecture

Everything is a **100 Hz (10 ms) loop**. Data flows one direction with no feedback except the motor
command returning to the MCU:

```
nRF52840 ──UART 46B──▶ uart_bridge_node ──▶ /raw/imu_6dof ─┐
                              ▲                /rc/command  │
                              │                /rc/status ──┤
                              │                             ▼
I2C sensors ──▶ i2c_driver_node ──▶ /raw/magnetometer ──▶ state_estimation_ekf_node
                        │                                            │
                        │                                            ▼
                        │                                  /filtered/attitude
                        │                                    │      │       │
                        │        ┌───────────────────────────┘      │       └──┐
                        │        ▼                                  ▼          │
                        │ auto_scenario_node ─/auto/command─▶ pid_control_node  │
                        │                                           │          │
                        │        └─────────/motor/output────────────┘          │
                        │                       │                              │
                        │             uart_bridge_node ─UART 10B─▶ nRF52840     │
                        │                                                      ▼
                        └─ /sensor/pressure_raw ──▶ hydro_estimator_node ─▶ data_logger_node
                                    (+ /rc/status)      /filtered/hydro            │
                                                        /sensor/pressure_calibrated   ▼
                                                                            log_csv/*.csv
```

`rpm_driver_node` is independent: GPIO 21 rising-edge interrupts → `/sensor/tail_rpm`.

### Node roles

| Node | Owns |
|---|---|
| `uart_bridge_node` | `/dev/ttyAMA0` 115200 8N1. Byte-level packet state machine. **Pure pass-through — no scaling, no sign flips.** |
| `i2c_driver_node` | `/dev/i2c-1`: AK8963 magnetometer (via MPU9250 bypass) + 3× MS5837 behind a TCA9548A mux |
| `state_estimation_ekf_node` | MEKF filter, button UI, yaw offset, ±5000 mode encoding, mag calibration via `magneto_cal.py`. The "brain" — sole publisher of `/filtered/attitude`. **Pressure left this node on 2026-08-21** — it never entered the filter state; it was inherited furniture from the Mahony migration. |
| `state_estimation_node` | **Retired 2026-08-17** — kept in the repo (still built) as the rollback path, removed from the launch. Mahony reference; never run it alongside the EKF node. |
| `pid_control_node` | PID + motor mixing. Output `[0]=left servo, [1]=right servo, [2]=yaw servo, [3]=tail BLDC` |
| `auto_scenario_node` | Time-based trajectory generator for AUTO mode |
| `rpm_driver_node` | Hall-sensor pulse counting in a worker thread |
| `hydro_estimator_node` | **Owns pressure end to end** (new 2026-08-21): atmospheric zeroing, `/sensor/pressure_calibrated`, btn1 long-press re-zero, and depth in metres on `/filtered/hydro`. Speed (pitot) is the next stage — those array slots publish NaN today. Observation-only; no control loop consumes it. |
| `data_logger_node` | 39-column CSV snapshot at 100 Hz — fused attitude **plus raw IMU/mag/pressure and EKF gyro bias + flags** (post-hoc diagnosis; raw mag enables offline recalibration, raw pressure enables offline depth/pitot recomputation). 200 MB per-file rotation, `Time(s)` continuous across files. Schema: `docs/csv_format.md`. |

## Conventions you must know before editing

These are load-bearing and span multiple files. Breaking one silently corrupts behavior.

### 1. Mode is smuggled inside the attitude message

There is no mode topic. `state_estimation_ekf_node` adds **±5000 to `attitude.x` (roll)** in AUTO
mode. Three nodes decode it independently with the same `|x| > 2500` test — `attitude_callback` in
`pid_control_node.cpp`, `auto_scenario_node.cpp`, `data_logger_node.cpp`. Change the encoding in
one place and you must change all four.

### 2. IMU units are **g** and **deg/s**, not REP-103

`/raw/imu_6dof` is a `sensor_msgs/Imu`, but the nRF firmware sends **g** and **deg/s**, published
unconverted (measured |a| ≈ 1.03 at rest, not 9.81; the estimators multiply the gyro by `M_PI/180`
themselves — `state_estimation_node.cpp:523/525`). The filters normalize accel, so scale doesn't
matter to them, only direction. **Any new consumer of this topic must convert.**

### 3. Message types are reused as plain containers

- `/rc/command` is a `geometry_msgs/Quaternion` holding **4 raw stick ints**, not a rotation:
  `x=roll, y=pitch, z=yaw, w=throttle`. Same for `/auto/command` (`x/y/z` in **0.1°**, `w` in **PWM µs**).
- `/rc/status` is `Int32MultiArray[5]` = `[btn1, btn2, vbat1×100, vbat2×100, rssi]`. Voltages are
  centivolts — divide by 100.
- `/sensor/pressure_raw` is `Float32MultiArray[6]`: `[0..2]` pressure (mbar), `[3..5]` temperature (°C).
  **A dead channel is `NaN`, never `0.0`** (changed 2026-08-20, same on `/sensor/pressure_calibrated`).
  On the zeroed topic `0.0` is a legitimate reading — *at the surface* — so the old `0.0` sentinel made
  "bus is dead" and "floating at the surface" the same value, and a depth loop would keep diving on a
  locked bus. NaN collides with nothing, still fails the existing `>= 100 mbar` validity test (every
  comparison with NaN is false, so downstream needed no change), and propagates through arithmetic so
  a consumer that forgets to check breaks loudly instead of quietly. **Test with `std::isnan(x)` —
  `x == NAN` is always false.** `i2c_driver_node` also logs an ERROR after 30 consecutive all-channel
  failures (~3 s), because the mux-select and ADC-read failure paths used to be silent.

### 4. `-9999` is the RC-link-lost sentinel

The nRF sends `throttle = -9999` when the transmitter link drops. `pid_control_node` treats it two
ways — the explicit sentinel, and a 5 s timeout on `last_valid_rc_time_` (for when the nRF itself
dies). `data_logger_node` records it unfiltered so dropouts stay visible in the CSV. **The failsafe
is skipped entirely in AUTO mode** (`!is_auto_mode_ &&`) by design — autonomous running without a
transmitter is the normal case.

### 5. The control loop is event-driven, not timed

`pid_control_node` has **no timer** — PID runs inside `attitude_callback`, and it **omits `dt`**
(gains absorb it, assuming a fixed 100 Hz). `/filtered/attitude` must only ever have ONE publisher
(ghost-publisher check below) — but be precise about *why*, because the obvious reason is currently
false: **`ki` and `kd` are all `0.0f`** (`pid_control_node.cpp:39-41`), so P-only control is
rate-independent and doubling the attitude rate does **not** change the servo command. The real
damage from two publishers is (1) two different estimators' attitudes arriving alternately, so the
output chatters between two values, and (2) `/motor/output` doubling to 200 Hz, which doubles the
UART packet rate to the nRF. The `dt`-omission gain problem becomes real **the moment I or D is
enabled** — which is exactly when someone will least expect it. Telling an operator "the gains are
wrong" today sends them to re-tune gains that are fine.
`state_estimation_ekf_node` measures `dt` per sample from `header.stamp` and integrates it with no
upper clamp (covariance adjusts trust); the retired Mahony node clamps to `[0.001, 0.1]` s and logs
clamp events, because its fixed `Kp·dt` correction step must stay bounded. §1.3.

### 6. Startup is not instantaneous

- **The gyro-bias stillness window is 1000 samples (~10 s), but it does NOT block.** The EKF
  initialises on 50 samples (~0.5 s) and publishes attitude from then on; the 1000-sample window runs
  **in parallel** and joins as a pseudo-measurement when it closes (`state_estimation_ekf_node.cpp:1245`).
  The robot must still be completely still for those 10 s — motion trips the stillness test
  (`자이로 std < 1.0 °/s` and `|a|` deviation `< 0.05 g`) and restarts the window, up to 3 times.
  The old "blocks for 10 s, nothing publishes" behaviour was the **retired Mahony node's**.
- **Pressure zeroing waits 180 s** (`atm_warmup_sec` in `hydro_estimator_node`, cut from 300 s on
  2026-08-20) before capturing baseline. It delays **depth only** — pitot speed needs no atmospheric
  zero (the per-channel offsets enter the differential as a constant and are absorbed by the in-water
  q zero), so it will publish from t=0 once implemented. The 300 s figure sized a 30BA-era steep initial drop (~250 s); with 02BA
  sensors **that transient is absent** — slopes sit inside the estimator's own noise floor from
  t=30 s, and 180 s vs 300 s changes the resulting depth error by 0.03–0.17 mm, below sensor noise.
  Re-verify after final assembly (`press_char.py --s1`); a sealed housing may bring the transient back.
  Re-zero any time with a btn1 long press.

### 7. Hardware quirks

- **`gpiochip4`**, not `gpiochip0` — the Pi 5 moved the 40-pin header onto the RP1 chip. Changing
  boards means changing `rpm_driver_node.cpp`.
- RPM resolution is coarse: 1 pulse per 100 ms window = **200 RPM**.
- The three pressure sensors are **MS5837-02BA** (swapped from 30BA on 2026-08-20). The two models
  need **different compensation constants** — using 30BA math on an 02BA reads exactly **20× high**
  (measured: 20132.60 vs 1006.63 mbar) and sails past the estimator's `>= 100 mbar` validity test,
  silently scaling depth by 20. `i2c_driver_node.cpp` warns when pressure leaves 300–1200 mbar to
  catch this. 02BA vs 30BA on paper: resolution 0.016 vs 0.20 mbar (0.16 mm vs 2 mm), full-accuracy
  depth **~1.9 m** vs ~300 m, and the 02BA datasheet expects drying roughly once a day.
- **The 02BA's 0.016 mbar resolution does NOT hold on this hardware** (measured 2026-08-20, 60 min,
  filter bypassed): σ = 0.13 / 0.13 / 0.24 mbar (**1.3 / 1.3 / 2.4 mm**) on ch0/ch1/ch2 — **8–15×
  the datasheet**. The three channels' fluctuations are uncorrelated (r ≈ −0.005, and averaging the
  three drops σ by exactly √3), so it is **per-sensor electrical noise**, not real pressure, supply
  ripple, or EMI — those would be common-mode. Still 5–10× finer than the cm-level a depth loop
  needs, but it caps differential-pressure attitude: at 30 cm spacing 1° of pitch is only 0.51 mbar,
  so a single sample resolves ~0.3–0.5°. Quote the measured figure, never the datasheet one.
- The three MS5837 sensors all share address `0x76`, hence the mux. Pressure updates at **~11 Hz**
  despite the 100 Hz timer — a 3-state machine spreads the ADC conversions across ticks. **~11 Hz is
  not a hardware floor**: the two waits are 40 ms each against an 18 ms datasheet max (OSR 8192), and
  the three sensors convert concurrently, so the rate can roughly double with no resolution loss if
  depth control ever needs it. The mux stagger between the three (~0.8 ms at 100 kHz) is the smallest
  delay in the chain by 100×.
- **`/sensor/pressure_raw` is normally not raw** — "raw" means *before zeroing*. `i2c_driver_node`
  applies a 1st-order IIR (α 0.1, τ ≈ 0.86 s) before publishing, and the unfiltered signal is kept
  nowhere — which is exactly why the 8–15× noise above went unnoticed until the filter was bypassed.
  **That bypass is currently still in place** (temporary, marked `##### [임시 2026-08-19]`; the node
  logs `※ 압력 IIR 필터 비활성` at startup as the reminder), so pressure is being published unfiltered
  right now. Decide the coefficient when depth control lands: measured noise 1.3–2.4 mm is already
  5–10× finer than cm-level needs, while the filter's 0.19 Hz corner sits inside a depth loop's
  bandwidth and costs 0.86 s of lag (25 cm at 0.3 m/s descent).
- A dead pressure channel is marked by `prom_C_[i][1] == 0` and reported downstream as `0.0`; the
  estimator treats `< 100 mbar` as invalid.
- `SERVO_MIN_US`/`SERVO_MAX_US` (1250/1750) in `pid_control_node.cpp` **must match the nRF firmware's
  `MotorControl.cpp`**. This repo cannot enforce that.

### 8. Magnetometer: fallback, axes, calibration

- **Failure is a supported state**: 500 ms of AK8963 silence → automatic 9-DOF → 6-DOF fallback. It
  is alive (~94 Hz since 2026-07-31), so the system runs 9-DOF; yaw drift there is +0.04°/min. The
  −8°/min figure is 6-DOF-only, and no 6-DOF drift number survives a session (§3.2, §6.3).
- **Axis remap `(my, mx, −mz)` is verified and PERMANENT** (tumble 2026-08-06, dip 144.3° vs Korea's
  143°); the code comment's derivation is bogus but the answer is measured — do not reopen (§4.2).
- **Ellipsoid fit only; min/max retired 2026-08-17** — it measured *worse than no calibration*
  (38.1 % vs 22.1 % scatter). btn2 long press and `/calibrate_mag` (`ros2 service call
  /calibrate_mag std_srvs/srv/Trigger` — call twice: start, then finish early) both run
  `magneto_cal.py`: refuses to save above 8 % scatter, backs up the old file, and the EKF node
  auto-reloads on success, no restart (§4.3, §9).
- **Calibration is tied to the ASSEMBLY, not the place** — any part change invalidates it.
  Recalibrated assembled 2026-08-17: scatter 6.1 %, dip 137.2°, 360° spin ratio 1.00; hard iron
  changed completely, (−12.3, +23.5, −89.8) → (+9.9, +61.6, −16.1) µT (§9). Tumble in one spot,
  ≥1 m from steel; **never calibrate from a flat spin** (it cannot see the vertical offset). File:
  `~/ros2_ws/log_csv/mag_calib_params.txt`. (A stale `mag_calib.txt` sat at the workspace root,
  read by nothing — deleted 2026-08-25. It held June-9 **min/max** values, the method retired for
  measuring *worse than no calibration*, so it looked restorable but would have made things worse.)
- **A bad mag vector corrupted Mahony's roll/pitch** (5.0°/1.7° measured), not just yaw; the EKF's
  1-D horizontal yaw update is structurally immune (§5.1, §5.2).
- **A hand-held spin cannot validate heading better than ~3°** — the residual sine is the ROOM
  (spatial field gradient), not the robot, and the calibration model is at its limit: do not re-fit
  it over a ~3° residual (§4.4, §4.6).

### 9. Button UI (`/rc/status`, 1 tick = 10 ms)

Short press = 30 ms–1 s; long press = exactly 3 s (`== 300`, fires once).

| | Short | Long |
|---|---|---|
| btn1 | AUTO/MANUAL toggle | re-zero atmospheric pressure (**handled by `hydro_estimator_node`** since 2026-08-21; the EKF still detects the 3 s hold, but only to suppress the release from counting as a short press) |
| btn2 | set current heading as yaw 0 | run `magneto_cal.py` (ellipsoid); long-press again = finish collection early (SIGINT) |

When the RC link drops the nRF sends `btn = 255`; both `state_estimation_ekf_node` and
`data_logger_node` explicitly reject non-0/1 values so the `1 → 255` transition isn't misread as a release.

## Before any measurement: the ghost-publisher check

The costliest trap so far (§7.1): an orphaned estimator — ppid 1, running a *deleted* binary —
survived restarts, hid from `ros2 node list` and `pgrep -x` (comm truncates at 15 chars), and
blended two filters on one topic into a stable-looking fake 6.8° error. Before trusting any run:

```bash
ros2 topic info /filtered/attitude --verbose | grep -c "Endpoint type: PUBLISHER"   # must print 1
ls -l /proc/*/exe 2>/dev/null | grep '(deleted)'                                    # must be empty
```

## Closed questions — do not re-litigate (details in docs/imu_detailed.md)

- **IMU axis transform verified end to end** (`R = [0 0 −1; −1 0 0; 0 1 0]`, `det = +1`; accel
  2026-08-04, gyro 2026-08-06). Judge axis tests on at-rest residuals only, and stillness on the
  gravity vector, never on Euler angles (degenerate near pitch ±90) — §2.
- **halfwy bug** (`R22` where `R21` belongs, 9-DOF Mahony) fixed 2026-08-06. Its symptom grew as the
  calibration improved — never read "recalibration made attitude worse" as "the calibration is bad" (§5.1).
- **"9-DOF Mahony has a structural ~6.8° tilt error" RETRACTED** — a ghost-publisher artifact; clean
  re-measure 0.134°/0.117°. Do not apply the proposed `halfv` projection fix on theory alone (§7.1).
- **EKF accel update must project world-vertical out of the Kalman GAIN** (`null_dir = &g_b`) or yaw
  leaks −33°/−37° per shake; never pass `null_dir` to the mag or bias updates. Diagnostic: 6-DOF yaw
  1σ may only grow — score any decrease cumulatively, never per-sample (§5.2).
- **EKF vs Mahony**: tied at rest; EKF 2.8× better under rotate+shake, and passed the 6-DOF
  yaw-drift gate at +0.155°/min (≤ 2 °/min required) — §6.
- **Accelerometer needs no calibration** — axis radii agree to 0.3 %; the earlier "~2 % asymmetry"
  claim was an underdetermined-fit artifact (§7.3).

## What to do next

Working checklist, ordered so each item unblocks the next: **`docs/todo.md`**.
It carries the water-vs-bench split, the "must be done before water" items, and the open decisions.

## Known gaps (open)

| Gap | Notes |
|---|---|
| AK8963 ASA ROM (0x10–0x12) never read | \|m\| reads 30 % low (34.7 vs ~50 µT). Magnitude-only — heading/attitude unaffected; fixing it forces recalibration. §4.1 |
| Thermal drift near the sensor | 3 µT (≈8° heading) after CPU load, noise ~×3 — may cap accuracy. §4.7 |
| Gyro z bias session-unstable | −0.044 vs −2.2 °/s hours apart (50×); the EKF absorbs it online. §3.2 |
| `gyro_noise_sigma` validated 2026-08-17 | Allan (2 h, 711k samples): ARW 0.0060–0.0078 °/s/√Hz, bias instability 0.0011–0.0024 °/s at τ 69–434 s. The 0.05 setting is a measured 6.4–8.3× headroom, no longer a guess; `bias_tau` 1000 s and `gyro_bias_rw_sigma` 0.002 are consistent. §3.3 |
| **Positive pitch = nose DOWN** | opposite of aerospace convention (measured 2026-08-04). Confirm signs before touching the pitch PID or AUTO scenario. §2.1 |
| Roll/pitch have no offset calibration | **Roll** shows a stable +6.4° over a 3.87-day bench record — that looks like a genuine mounting offset. **Pitch does not**: the "≈ 7.9°" from 2026-08-04 (§2.1) did not reproduce — the same 3.87 days averaged **−0.45°** (range −1.03 … +0.38) and another day read +5.83°. Resting pitch depends on how the robot is set down, so **do not treat 7.9° as a constant**. `hydro_estimator_node` used it as an attitude fallback until 2026-08-25; that fallback is gone (speed is NaN without attitude). |
| Post-assembly tests pending | tail-beat pitch bias, PID sign bench check **before water**, multi-attitude tilt accuracy. §8.2 |
| Retired Mahony node dead members | six write-only members left from an old button implementation. §5.1 |
| `data_logger_node` total log volume unbounded | ~84 MB/hour since raw pressure columns (2026-08-21; ~80 MB since 2026-08-18). Per-file 200 MB rotation exists, but old files are never deleted (deliberate — `log_csv/` also holds experiment CSVs and `mag_calib_params.txt`). Disk space is managed by hand. |

## Language

Code comments, log messages, and commit history are in **Korean**. Match that when editing — do not
convert existing comments to English.
