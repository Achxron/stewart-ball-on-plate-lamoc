# Technical manual — Stewart Ball-on-Plate

Internal firmware reference: what each module does, how to maintain it, how to
extend it, and which divergences between comments and code exist today. For
operation (commands, telemetry, capture), see `USER_GUIDE.md`.

Versão em português: `MANUAL_TECNICO.md`.

A note on the title: this file serves as the reference and maintenance manual. A
short `README.md` at the root, pointing to this manual and to the user guide, is
optional and complementary.

The descriptions below follow what the code does in the supplied files. Where a
code comment says one thing and the code does another, the manual follows the code
and records the divergence in section 8.

---

## 1. Folder structure

```
StewartBallOnPlate/
├── StewartBallOnPlate.ino     entry point (setup/loop)
├── config.h                   pins, geometry, gains, timing
├── types.h                    shared structs and the mode enum
├── kinematics.{h,cpp}         inverse kinematics (pose → 6 servos)
├── imu_sensor.{h,cpp}         BNO085 over SPI1 + recovery
├── touch_sensor.{h,cpp}       resistive touchscreen → position/velocity
├── servo_driver.{h,cpp}       PCA9685 output
├── attitude_ctrl.{h,cpp}      inner loop (attitude)
├── ball_ctrl.{h,cpp}          outer loop (ball position)
├── supervisor.{h,cpp}         orchestration, modes, telemetry, serial
├── tools/
│   ├── telemetry_logger.py    capture/command/plot (runs on the PC)
│   └── scripts/               timed sequences (.txt)
└── logs/                      capture output (created at runtime)
```

Arduino compiles only the sketch root and the `src/` subfolder. The `tools/`
folder is not compiled and can coexist with the sketch without affecting the
build.

---

## 2. Architecture

Data flow in `BALL_BALANCE` (full cascade):

```
Touch ─► BallCtrl ─► desired tilt (tilt_ref)
                          │
              + trim      ▼
        AttitudeCtrl(tilt_ref, IMU) ─► velocity command (deg/s)
                          │
              integrate (∫dt) ─► absolute pose (roll, pitch)
                          │
                  Kinematics ─► 6 servo angles ─► Servos (PCA9685)
```

Each peripheral has a single module that controls it; no module knows another's
hardware. The supervisor is the sole owner of the state: it holds the structs and
passes them by reference to the modules.

Timing (from `config.h`):

- Main loop target: `LOOP_PERIOD_US` = 10000 us (100 Hz). `tick()` rate-limits to
  this period.
- Outer loop (ball): runs once every `BALL_LOOP_DIV` ticks. With
  `BALL_LOOP_DIV = 3`, that is about 33 Hz. The code comments say 20 Hz (divisor
  5); see section 8.
- Telemetry: `SERIAL_PERIOD_MS` = 20 ms (up to 50 Hz).
- BNO085 reports: `BNO_REPORT_US` = 10000 us (100 Hz).

---

## 3. Control principle (velocity form)

The attitude controller does not emit an absolute angle. It emits a pose rate of
change (deg/s), which the supervisor integrates over time (`integratePose`) into
an absolute pose. At steady state, equilibrium requires zero pose derivative,
which only happens with zero error. That is why the steady-state error tends to
zero without needing a large integral term.

Integration only runs while the IMU is fresh; with a stalled IMU the pose freezes
at the last valid value (protection against windup over a dead measurement). The
integrated pose is limited to the workspace (`POSE_TILT_MAX`), not to the velocity
limit (`ATT_CMD_MAX`).

State of this integration: `pose_cmd_roll` and `pose_cmd_pitch`, in
`supervisor.cpp`. They are zeroed on every mode change.

---

## 4. Project conventions

- No magic numbers outside `config.h`. Pins, geometry, gains, limits and timing
  all live there. If you tune something, tune it in `config.h`.
- Plain (POD) structs in `types.h`, passed by reference. No module heap-allocates
  for them.
- One module per peripheral. `servo_driver` is the only one that talks to the
  PCA9685; `imu_sensor`, to the BNO085; `touch_sensor`, to the touchscreen.
- One namespace per module (`Kinematics`, `IMU`, `Touch`, `Servos`,
  `AttitudeCtrl`, `BallCtrl`, `Supervisor`), each with `init()` and its work
  function.

---

## 5. Modules

### 5.1 `config.h`
Single source of pins, geometry, gains and timing. Points to know:

- Effective pins: PCA9685 on I2C (SDA 18, SCL 19, address 0x40); BNO085 on SPI1
  (SCK 14, MOSI 15, MISO 12, CS 13, INT 11, RST 10); touch on D2–D7 (GP2–GP7) and
  ADC GP26/GP27.
- Geometry: `L1` (horn, 24.95 mm), `L2` (rod, 160 mm), `Z_HOME` (149.33 mm), the
  `BASE_PX/PY`, `PLAT_PX/PY`, `DIR_X/Y` vectors, and the per-servo trims/signs.
- Limits: `SERVO_MIN_DEG`/`MAX_DEG`, `POSE_TILT_MAX`, `ATT_CMD_MAX`,
  `BALL_OUT_MAX`, `BALL_I_MAX`.
- Per-axis inner-loop gains (`ATT_KP/KI/KD_ROLL` and `_PITCH`) and ball-loop gains
  (`BALL_KP/KI/KD_CLOSE` and `_FAR`, with `BALL_TRANSITION_DIST`).
- There are commented `OLD` blocks (old geometry and gains) and several
  end-of-line alternatives. They do not affect the build; clean them up after
  consolidating.

### 5.2 `types.h`
Defines the structs exchanged between modules and the `enum class OpMode`:

- `Pose` (x, y, z, roll, pitch, yaw): 6-DOF pose. Translation in mm, rotation in
  degrees.
- `Attitude` (roll, pitch, yaw and their rates, `valid`): IMU attitude, referenced
  to gravity; rates come from the calibrated gyroscope.
- `BallState` (x, y, vx, vy, `detected`, `timestamp_ms`): touch output.
- `ServoCmd` (`angle[6]`): the six servo command, in degrees.
- `TiltCmd` (roll, pitch): desired tilt; it is the ball-loop output and the
  attitude-loop reference.
- `OpMode`: the eight modes. The order defines the integer reported in telemetry.
  Append new modes at the end so the existing integers do not shift.

### 5.3 `kinematics.{h,cpp}`
Inverse kinematics: given a `Pose`, it solves the six servo angles. It is the only
module that knows the geometry.

- `solve(pose, cmd)`: for each leg it builds the vector from the base to the
  platform joint and solves `A·cosθ + B·sinθ = C`. If the discriminant is negative
  (pose outside that leg's workspace), it returns `false` and the servo keeps its
  previous value. The angle is converted to servo degrees with `SERVO_SIGN` and
  `SERVO_TRIM` and clamped to the envelope.
- `makeTiltPose(roll, pitch)`: tilt-only pose, at height `Z_HOME`.
- `computeJoints(cmd, out)`: forward, computes each horn tip (debugging).

### 5.4 `imu_sensor.{h,cpp}`
Reads the BNO085 over SPI1 and keeps the non-blocking recovery.

- Reads two reports: the rotation vector (becomes roll/pitch/yaw, with mounting
  offset) and the calibrated gyroscope (rates, no numerical differentiation).
  Using the gyroscope directly avoids the derivative spike and the
  quantization-noise amplification of numerical differentiation.
- `update(att)` is non-blocking: it drains pending events, with a per-call cap
  (`IMU_DRAIN_MAX`), and only calls `getSensorEvent()` when the INT pin signals
  data ready (avoids the wait-for-INT spin when the FIFO is empty).
- Glitch rejection: drops single-frame jumps above `ATT_JUMP_MAX`; drops gyroscope
  above `GYRO_MAX_VALID`.
- Recovery via state machine (`HEALTHY → SW_REENABLE → HW_RESET_PULSE →
  HW_RESET_BOOT`), all timed with `millis()`. No stage blocks, so open-loop modes
  keep running during recovery.
- `isFresh()` reports whether a valid rotation vector arrived within
  `IMU_TIMEOUT_MS`. The loops use this to decide whether to trust the attitude.

Current constants: `IMU_COUPLING_ANGLE = 0` (Z-misalignment correction off) and
`GYRO_ALPHA = 0` (no gyroscope low-pass filter, pass-through).

### 5.5 `touch_sensor.{h,cpp}`
Reads the resistive touchscreen and estimates position and velocity.

- `readAxis()`: takes `TS_SAMPLES` readings, discards the two smallest and the two
  largest, and checks the remaining range against `TS_MAX_RANGE_ADC`; returns the
  trimmed mean (`TS_SAMPLES` must be ≥ 6, since it discards 4).
- `update(ball)`: reads both axes, detects no-touch (deadzone), converts ADC → mm
  with the affine transform (`TS_CAL_*`), runs a range check and threshold-based
  outlier rejection (`TS_THRESHOLD_MM`), and estimates velocity by finite
  difference.
- Raw output for calibration: there are commented lines at the end of `update()`
  that would write `raw_xp`/`raw_yp` instead of the mm value. To calibrate the
  affine transform, uncomment that block temporarily to capture the raw ADC,
  obtain the coefficients, and revert.

### 5.6 `servo_driver.{h,cpp}`
Output through the PCA9685, channels 1 to 6.

- `write(cmd)`: clamps each angle to the envelope and maps 0–180 degrees to
  `SERVO_PWM_MIN`–`SERVO_PWM_MAX`.
- `writeNeutral()`: sets all to 90 degrees plus trim. It is the safe startup pose,
  called in `init()`.

### 5.7 `attitude_ctrl.{h,cpp}`
Inner loop (2 DOF). Structure in use:

```
cmd = Kp·e + Ki·∫e·dt − Kd·ω_gyro
```

with `e = ref − measured` and the derivative coming from the gyroscope (not from
the differentiated measurement), which eliminates the derivative spike on a
reference step. The output is clamped to `ATT_CMD_MAX` and interpreted as a
velocity (deg/s); the integration into absolute pose is done in the supervisor.

Notes on the current configuration: `ATT_KI = 0` on both axes, so the loop runs as
PD. The `deadband_punch` block is zeroed (dead code). `ATT_DEADBAND = 0` (no
deadband). There is a commented feedforward path, unused because it is not robust
to base tilt.

### 5.8 `ball_ctrl.{h,cpp}`
Outer loop (per-axis PID, with gain scheduling).

- `update(ball, dt, out)`: computes the error to the target, applies a deadband
  (`BALL_DEADBAND`), picks `CLOSE`/`FAR` gains by distance
  (`BALL_TRANSITION_DIST`), runs per-axis PID (derivative on the raw error,
  anti-windup at `BALL_I_MAX`, output at `BALL_OUT_MAX`), and maps the ball error
  to world-frame tilt (X → pitch, Y → roll, with sign). The mapping signs are
  marked in the code and must be adjusted if the physical axes respond inverted.
- `setTarget(x, y)`: sets the target (mm, plate frame).
- Current configuration: `BALL_KI = 0`, so it also runs as PD.

### 5.9 `supervisor.{h,cpp}`
Orchestrates everything. Responsibilities:

- Holds the global state (attitude, ball, references, pose, servo command,
  telemetry) and the trim offsets and the integrated pose.
- `tick()`: IMU poll (always), loop rate-limit, outer-loop division, the per-mode
  `switch`, and telemetry. Each mode decides whether it needs a fresh IMU.
- `setMode(mode)`: resets the controllers and the cascade state on any mode
  change, and re-emits the telemetry header.
- `handleSerial()` + `processCommand()`: non-blocking command parser, accumulates
  characters and only parses at end of line.
- Telemetry: `sendTelemetry()` (data) and `sendTelemetryHeader()` (header).

The three-phase machine of `BALL_BALANCE` (`CLOSED`/`OPEN`/`RELEVELING`) lives
here, as do the pose integration, the relevel ramp (`slewToZero`) and the level
attitude criterion (`attitudeSettledLevel`).

### 5.10 `StewartBallOnPlate.ino`
Entry point. `setup()` calls `Supervisor::init()`; `loop()` calls `handleSerial()`
and `tick()`. The comment header of this file lists outdated pins; see section 8.

### 5.11 `tools/telemetry_logger.py`
Runs on the PC, not on the Pico. Captures telemetry, optionally sends commands
(interactive or scripted), saves CSV and produces plots and statistics. Usage
details in `USER_GUIDE.md`, section 8. The expected columns are in
`EXPECTED_COLUMNS`; if the firmware telemetry changes, that list must follow (see
section 7.5).

---

## 6. Maintenance

### 6.1 Tune gains
Edit the values in `config.h` and recompile. There is no runtime tuning: the
`GAINS` command is not implemented (section 8). Roll and pitch have separate
gains, because the rectangular acrylic plate has different inertias per axis.

### 6.2 Adjust geometry
`L1`, `L2`, `Z_HOME` and the `BASE_*`, `PLAT_*`, `DIR_*` vectors in `config.h`.
After any change, validate the kinematics numerically (e.g. check that at level all
servos land near 90 degrees) before powering up.

### 6.3 Servo trim and sign
`SERVO_TRIM[6]` zeroes the level of each horn; `SERVO_SIGN[6]` corrects each
servo's rotation direction. `SERVO_MIN_DEG`/`MAX_DEG` define the envelope.

### 6.4 Calibrations
- IMU: `IMU_OFFSET_ROLL`/`PITCH` (procedure in `USER_GUIDE.md`, 9.2).
- Touch: `TS_CAL_AX/BX/DX/AY/BY/DY` (procedure in 9.3).

### 6.5 Timing
`LOOP_PERIOD_US`, `BALL_LOOP_DIV`, `SERIAL_PERIOD_MS`, `BNO_REPORT_US`. Remember
that `BALL_LOOP_DIV` is a tick divisor, not a direct frequency; the outer-loop
frequency is 100 Hz / `BALL_LOOP_DIV`.

---

## 7. How to extend

### 7.1 Add an operating mode
1. `types.h`: add the value at the end of `enum OpMode` (at the end, so the
   integers already used in telemetry do not shift).
2. `supervisor.cpp`, `processCommand()`: add a branch under `MODE`. Mind the
   ordering: the parser matches by substring with `indexOf`, so more specific
   names must be tested first (e.g. `BALL_OPEN` before `OPEN` and before `BALL`).
3. `supervisor.cpp`, `tick()`: add a `case` in the `switch` with the mode's
   pipeline. Use `IMU::isFresh()` if the mode depends on attitude.
4. `setMode()` already does the generic reset; add a mode-specific reset if needed.
5. Telemetry: the mode integer is already streamed. If the mode exposes new state,
   see 7.5.

### 7.2 Add a serial command
In `processCommand()`, add `else if (line.startsWith("X"))`, parse with
`indexOf`/`substring`/`toFloat`. Unrecognized commands fall into the final branch
that prints `ERR: unknown command`. Keep the pattern of the existing commands
(`MODE`/`POSE`/`TARGET`/`TRIM`).

### 7.3 Change a data struct
Edit `types.h`, then update the module that produces the field and those that
consume it. If the field is telemetered, update the three points described in 7.5.
Since the structs are passed by reference and have default initializers, adding a
field does not break callers that do not use it.

### 7.4 Add a sensor or actuator
Create a `my_module.{h,cpp}` pair with its own namespace, `init()` and the work
function. Put the pins and constants in `config.h`. Call `init()` in
`Supervisor::init()` and use the module in `tick()`. Do not touch the hardware
outside its module.

### 7.5 Change the telemetry
Three points must stay in sync, in the same column order:
1. `supervisor.cpp`, `sendTelemetry()`: the `Serial.print` calls that emit the
   data.
2. `supervisor.cpp`, `sendTelemetryHeader()`: the column-names line.
3. `tools/telemetry_logger.py`, `EXPECTED_COLUMNS`: the count and names the logger
   uses (it validates by count and maps by order).

If the count changes and `EXPECTED_COLUMNS` does not follow, the logger discards
the lines as malformed.

---

## 8. Known inconsistencies

Real divergences between comments and code in the current files. They do not
necessarily affect operation, but they confuse the reader. Recorded so they can be
reconciled.

1. **The `.ino` header pins diverge from `config.h`.** The comment in
   `StewartBallOnPlate.ino` lists the PCA on I2C0 (SDA GP8, SCL GP9) and the BNO
   with SCK GP10, MOSI GP11, INT GP14, RST GP15. The values compiled in `config.h`
   are the PCA on SDA 18/SCL 19 and the BNO with SCK 14, MOSI 15, INT 11, RST 10.
   `config.h` is what holds. Update the `.ino` header.

2. **`BALL_LOOP_DIV` vs comments.** `BALL_LOOP_DIV = 3` in `config.h`, which gives
   an outer loop at about 33 Hz. Several comments (in `config.h` and in
   `supervisor.cpp`) state "100 Hz / 5 = 20 Hz". The value and the comment
   diverge. Decide which is intended and align the number and the text.

3. **Name and unit of telemetry columns 14–15.** The firmware writes the name
   `cmd_r`/`cmd_p` in the header; `telemetry_logger.py` calls them
   `corr_r`/`corr_p`. It is the same column. The data is the attitude controller
   command as a velocity (deg/s), but the logger plot labels the axis "PID
   correction (deg)". The label unit is wrong (it is deg/s). The capture works
   because the logger uses the column order, not the names.

4. **`GAINS` command documented but absent.** The comment at the top of
   `supervisor.cpp` lists `GAINS ATT ...` and `GAINS BALL ...`, but
   `processCommand()` does not handle `GAINS`. Sending it returns
   `ERR: unknown command`.

5. **`IDLE` does not level the servos.** The comment in the `IDLE` case says
   "Servos stay at neutral", but nothing rewrites the servos when entering `IDLE`;
   they hold the last PWM. Leveling only happens at boot, via `writeNeutral()` in
   `init()`.

6. **Integral gains at zero.** `ATT_KI_ROLL/PITCH = 0` and `BALL_KI_* = 0` in the
   current build: both loops run as PD, despite being described as PID. Related
   items: `deadband_punch` zeroed in `attitude_ctrl.cpp` (dead code) and
   `ATT_DEADBAND = 0`.

7. **Old commented blocks in `config.h`.** `OLD` geometry and gains, plus
   end-of-line alternatives. They do not affect the build; removing them after
   consolidating reduces the chance of editing the wrong value.

---

## 9. Quick constant reference

Current values in `config.h`, for checking. Always treat `config.h` as the source;
this table is a summary.

| Constant | Value | Meaning |
|---|---|---|
| `LOOP_PERIOD_US` | 10000 | Loop target period (100 Hz) |
| `BALL_LOOP_DIV` | 3 | Outer-loop divisor (~33 Hz) |
| `SERIAL_PERIOD_MS` | 20 | Telemetry period (50 Hz) |
| `BNO_REPORT_US` | 10000 | BNO085 report period (100 Hz) |
| `L1`, `L2` | 24.95, 160.0 | Horn and rod (mm) |
| `Z_HOME` | 149.33 | Neutral height (mm) |
| `SERVO_MIN_DEG`/`MAX_DEG` | 15 / 165 | Servo envelope (degrees) |
| `POSE_TILT_MAX` | 18 | Integrated tilt limit (degrees) |
| `ATT_CMD_MAX` | 600 | Velocity command limit (deg/s) |
| `BALL_OUT_MAX` | 18 | Ball-loop output limit (degrees) |
| `BALL_TRANSITION_DIST` | 20 | Gain-scheduling threshold (mm) |
| `IMU_TIMEOUT_MS` | 50 | IMU freshness window |
| `ATT_KP_ROLL`/`PITCH` | 9.2 / 8.7 | Attitude proportional gain |
| `ATT_KD_ROLL`/`PITCH` | 0.48 / 0.38 | Attitude derivative gain |
| `BALL_KP_CLOSE`/`FAR` | 0.068 / 0.068 | Ball proportional gain |
| `BALL_KD_CLOSE`/`FAR` | 0.031 / 0.031 | Ball derivative gain |
| `BALL_DEADBAND` | 3 | Ball position deadband (mm) |
| `PLATE_X_MAX`/`Y_MAX` | 310 / 236 | Plate extent (mm) |
