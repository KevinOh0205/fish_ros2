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

Same assumption in `state_estimation_node`: `dt_` is a hard-coded `0.01f`.

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
automatically. **On this machine the AK8963 is currently not responding, so it always runs 6-DOF.**
Consequence: roll/pitch are absolutely referenced by gravity and stable, but **yaw has no absolute
reference and drifts** — measured at roughly −8°/min. Yaw PID chases that drift.

Calibration coefficients load from `~/ros2_ws/log_csv/mag_calib_params.txt` (note: the `mag_calib.txt`
at the workspace root is **not** read by any code). Trigger calibration with a btn2 long press or:

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

- **No IMU axis transform.** The sensor frame and the filter's expected frame are not aligned, and
  `state_estimation_node` passes accel/gyro straight through. Correcting this means inserting a
  rotation right after `imu_callback` reads the message (`state_estimation_node.cpp:484`), applied to
  **both** accel and gyro. Verify any candidate rotation with `det(R) = +1` and `R·Rᵀ = I`.
- **Positive pitch = nose DOWN** in this code's Euler convention (opposite of aerospace convention).
  Confirm sign expectations before touching the pitch PID or the AUTO scenario.
- **Roll/pitch have no offset calibration** analogous to `yaw_offset_`, so a non-level mounting shows
  up as permanent nonzero attitude.
- `state_estimation_node.cpp`: `btn1_long_processed_` is declared and used but missing from the
  constructor initializer list — indeterminate until the first `/rc/status` with `btn1 == 0` clears it.
  Several members (`right_btn_pressed_`, `left_btn_pressed_`, `button_press_start_time_`,
  `left_btn_press_start_time_`, `last_progress_`, `dynamic_pressures_`) are initialized or assigned
  but never read — leftovers from an earlier button implementation.
- `data_logger_node` writes ~40 MB/hour with **no rotation**. `log_csv/` grows without bound.

## Language

Code comments, log messages, and commit history are in **Korean**. Match that when editing — do not
convert existing comments to English.
