# User guide — Stewart Ball-on-Plate

How to operate the lab firmware: compile, flash, send commands, read telemetry
and capture data for analysis. Aimed at people who will use the system, not
necessarily change it. For the internal description of the modules and how to
modify the code, see `TECHNICAL_MANUAL.md`.

Everything here describes the actual behavior of the code in the supplied files.
Where a code comment disagrees with what the code does, this guide follows the
code and points out the divergence.

A Portuguese version of this guide is available in `GUIA_DE_USO.md`.

---

## 1. What the system is

A 6-DOF Stewart platform driven by six rotary servos (horn plus fixed-length rod)
that balances a ball on a resistive plate. The microcontroller is a Raspberry Pi
Pico (RP2040). The architecture is cascaded: an outer loop (ball position)
produces a desired plate tilt, and an inner loop (attitude, with an IMU) applies
that tilt while rejecting disturbances. Inverse kinematics converts the pose into
six servo angles.

---

## 2. Requirements

### Hardware
- Raspberry Pi Pico (RP2040).
- PCA9685 servo driver (I2C), six MG90D servos.
- BNO085 IMU (SPI1).
- Resistive touchscreen via SN74HC4066N.
- 5 V supply able to feed all six servos.

The actual pins live in `config.h`, which is the single source of truth. The
header comment in `StewartBallOnPlate.ino` lists different pins and is outdated:
see `TECHNICAL_MANUAL.md`, section "Known inconsistencies".

### Firmware
- An RP2040 Arduino core. The code uses `MbedSPI` and the pin-argument `TwoWire`
  constructor, so the installed core must provide those classes. Use the same
  core already set up on the lab machine; confirm that it compiles `MbedSPI`.
- Libraries (install through the library manager):
  - Adafruit PWM Servo Driver Library
  - SparkFun BNO08x Arduino Library
- `SPI` and `Wire` ship with the core.

### Data analysis (optional, only for the capture tool)
- Python 3, `pyserial` (required to capture).
- `matplotlib`, `numpy`, `pandas` (only for plots and statistics).

```
pip install pyserial matplotlib numpy pandas
```

---

## 3. Compile and flash

1. Open `StewartBallOnPlate.ino` in the Arduino IDE. The `.h`/`.cpp` files must
   sit in the same folder as the `.ino` (the flat layout Arduino requires).
2. Select the Raspberry Pi Pico / RP2040 board in the installed core.
3. Compile and flash over USB.

The `tools/` folder (with `telemetry_logger.py` and the scripts) sits next to the
sketch and is not compiled: Arduino only compiles the sketch root and the `src/`
subfolder. Keeping `tools/` there is safe.

---

## 4. Startup

On power-up or reconnect, the firmware opens the serial port at 115200 baud and
prints the boot diagnostics, in order:

```
=== Stewart Ball-on-Plate ===
[OK] Kinematics
[OK] Servos (neutral)
[OK] IMU (BNO085)        (or [FAIL] IMU — check wiring)
[OK] Touchscreen
MODE → 0
time_ms,mode,bx,by,...   (telemetry header line)
Ready. Send MODE command to begin.
```

At boot the servos go to neutral (90 degrees plus each servo's trim). The system
enters `IDLE` and waits for a `MODE` command. If `[FAIL] IMU` appears, check the
SPI wiring before using any mode that depends on attitude (`ATT`, `CHICKEN`,
`BALL`).

---

## 5. Serial commands

A text line terminated by `\n` (the firmware ignores `\r`, so Windows line
endings work). All commands are uppercase. Angles in degrees, ball positions in
millimeters.

| Command | Effect | Where it applies |
|---|---|---|
| `MODE IDLE` | Stops control; sensors keep updating | always |
| `MODE OPEN` | Direct pose over serial, no feedback | open loop |
| `MODE ATT <r> <p>` | Hold attitude `<r> <p>` in the world frame | inner loop |
| `MODE CHICKEN` | Keep the plate level, rejecting base disturbances | inner loop |
| `MODE BALL` | Full ball balancing (cascade) | cascade |
| `MODE BALL_OPEN` | Ball → kinematics → servos, no IMU feedback | cascade without IMU |
| `MODE CAL_IMU` | Stream telemetry to measure the IMU offset | calibration |
| `MODE CAL_TOUCH` | Stream telemetry to calibrate the touchscreen | calibration |
| `POSE <r> <p>` | Set the desired tilt (roll, pitch) | `OPEN` and `ATT` |
| `TARGET <x> <y>` | Set the ball target (mm, plate frame) | `BALL` and `BALL_OPEN` |
| `TRIM <r> <p>` | Level bias applied to roll/pitch | all pose modes |

Usage notes:

- In `MODE ATT <r> <p>`, the two numbers are optional; without them, it keeps the
  last `POSE`/`ATT` received.
- `POSE` only changes the tilt in `OPEN` and `ATT`. In `CHICKEN` and `BALL` the
  inner-loop reference is set internally (level, or the ball loop output), so
  `POSE` is ignored.
- `TARGET` uses the plate frame: X from 0 to 310 mm, Y from 0 to 236 mm, center
  at (155, 118).
- `TRIM` adds a constant bias to roll and pitch. Use it to compensate a residual
  mechanical tilt without touching the code.
- The `GAINS` command appears in the comment at the top of `supervisor.cpp`, but
  it is **not implemented**. Sending it returns `ERR: unknown command`. To change
  gains, edit `config.h` and recompile.

Unknown commands return `ERR: unknown command` followed by the list of valid
commands.

---

## 6. Operating modes

The telemetry "mode" column carries the mode integer (set by the order of the
`enum OpMode` in `types.h`):

| no. | Mode | What it does | IMU | Touch |
|---|---|---|---|---|
| 0 | `IDLE` | No control. Servos hold the last PWM | reads | does not read |
| 1 | `OPEN_LOOP_POSE` | Pose = trim + `POSE` → kinematics → servos | reads | reads (telemetry only) |
| 2 | `ATTITUDE_HOLD` | Hold attitude = trim + `POSE` (inner loop) | requires fresh | reads (telemetry only) |
| 3 | `CHICKEN_HEAD` | Hold level = trim (reject base disturbance) | requires fresh | does not read |
| 4 | `BALL_BALANCE` | Full cascade, with IMU-loss handling | requires fresh | reads |
| 5 | `CALIBRATE_IMU` | Stream telemetry only, no control | reads | does not read |
| 6 | `CALIBRATE_TOUCH` | Update and stream the touch, no control | reads | reads |
| 7 | `BALL_OPEN` | Ball → kinematics → servos, no IMU feedback | ignores | reads |

Behavior on IMU loss:

- `ATT` and `CHICKEN`: if the IMU is not fresh (no new data within
  `IMU_TIMEOUT_MS`), the platform freezes at the last valid pose and the attitude
  integrators are reset (avoids windup over a stalled measurement). When data
  returns, control resumes.
- `BALL`: uses a three-phase machine. `CLOSED` is the normal cascade. If the IMU
  drops, it enters `OPEN` (open loop, holds the last ball tilt). When the IMU
  returns, it enters `RELEVELING`: the ball loop is suspended and the pose is
  ramped back to level at `RELEVEL_SLEW_DPS`, handing control back to the ball
  only after attitude settles at level. This avoids the abrupt sweep that could
  trip the sensor again.

Operational note: when leaving a control mode for `IDLE`, the servos do **not**
return to neutral automatically; they hold the last commanded PWM. Leveling only
happens at boot. To level manually, use `MODE OPEN` and `POSE 0 0` (with the
right `TRIM`).

---

## 7. Telemetry

The firmware streams CSV over serial, at most every `SERIAL_PERIOD_MS` (20 ms,
that is, up to 50 Hz). The header line is re-emitted on every mode change. The
capture tool treats header lines as information and uses the column order, so the
re-emission does not interfere with logging.

There are 23 columns, in this order:

| # | Column | Meaning | Unit |
|---|---|---|---|
| 1 | `time_ms` | Firmware time (`millis()`) | ms |
| 2 | `mode` | Integer of the current mode | — |
| 3 | `bx` | Ball X position | mm |
| 4 | `by` | Ball Y position | mm |
| 5 | `bvx` | Estimated X velocity | mm/s |
| 6 | `bvy` | Estimated Y velocity | mm/s |
| 7 | `bdet` | Ball detected (0/1) | — |
| 8 | `imu_r` | Measured roll | deg |
| 9 | `imu_p` | Measured pitch | deg |
| 10 | `gyro_r` | Roll rate (gyroscope) | deg/s |
| 11 | `gyro_p` | Pitch rate (gyroscope) | deg/s |
| 12 | `ref_r` | Inner-loop roll reference | deg |
| 13 | `ref_p` | Inner-loop pitch reference | deg |
| 14 | `cmd_r` | Attitude controller command (roll) | deg/s |
| 15 | `cmd_p` | Attitude controller command (pitch) | deg/s |
| 16 | `pose_r` | Roll pose sent to kinematics | deg |
| 17 | `pose_p` | Pitch pose sent to kinematics | deg |
| 18–23 | `s1`..`s6` | The six servo angles | deg |

Two important caveats about columns 14 and 15:

- The firmware writes the names `cmd_r`/`cmd_p` in the header; `telemetry_logger.py`
  labels the same column `corr_r`/`corr_p`. It is the same column, with the same
  data.
- The data in these columns is the attitude controller command as a **velocity**
  (deg/s, later integrated into pose). The plot produced by the logger labels the
  axis "PID correction (deg)", but the real unit is deg/s. Read the axis as deg/s.

The `ref_r`/`ref_p` reference is what the inner loop tracks: in `ATT` it is
trim + `POSE`; in `CHICKEN` it is just trim (level); in `BALL` it is trim plus
the ball loop output (time-varying).

---

## 8. Capture tool (`tools/telemetry_logger.py`)

Captures telemetry over USB, saves it to a timestamped CSV, optionally sends
commands and produces plots and statistics. The file lives in `tools/`; the
sequence scripts live in `tools/scripts/`. The examples below assume you run from
the project root.

### Three modes

1. Capture only, for a fixed time:

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name idle --duration 30
```

2. Interactive (type commands while logging):

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name tuning --interactive
```
Type commands (`MODE ATT`, `POSE 5 0`, `POSE 0 0`) and press Enter; Ctrl-C stops.
Sent commands are logged to a `.commands.txt` file for traceability.

3. Scripted (timed sequence, reproducible):

```
python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name step_roll5 \
    --script tools/scripts/step_roll5.txt
```

### Script format

Plain text, one event per line: `<seconds_from_start>  <command>`. Lines with `#`
are comments. `END` stops the capture.

```
# tools/scripts/step_roll5.txt
0.0  MODE ATT
1.0  POSE 0 0
3.0  POSE 5 0
8.0  POSE 0 0
12.0 END
```

### Replot an existing capture

```
python3 tools/telemetry_logger.py --plot logs/step_roll5_2026-04-20_14-33.csv
```

### Output

- `logs/<name>_<date-time>.csv`: the telemetry.
- `logs/<name>_<date-time>.commands.txt`: commands sent, with their time.
- `logs/<name>_<date-time>.png`: plot (roll, pitch, controller command, servo
  angles), saved after capture unless `--no-plot`.
- Quick statistics in the terminal, per axis: mean, standard deviation, peak to
  peak and RMS of the `ref - measured` error.

Logs go to `./logs` by default (relative to the working directory). Run from the
root, or pass `--log-dir <folder>`.

### Main options

| Option | Default | Function |
|---|---|---|
| `--port` | — | Serial port (e.g. `/dev/ttyACM0`, `COM4`). Required to capture |
| `--baud` | 115200 | Serial speed |
| `--name` | `run` | Name used in the output file |
| `--duration` | unlimited | Capture for N seconds, then stop |
| `--log-dir` | `logs` | CSV folder |
| `--interactive`, `-i` | — | Allow typing commands during capture |
| `--script` | — | Run a timed script |
| `--plot` | — | Only plot an existing CSV (no capture) |
| `--no-plot` | — | Do not plot after capture |

---

## 9. Reproducible procedures

The procedures below state every step so they can be repeated.

### 9.1 Attitude-loop step response (roll)

Goal: measure rise time, overshoot, settling time and steady-state error of the
inner loop, on a reference step.

1. Create `tools/scripts/step_roll5.txt` with the contents from the example in
   section 8.
2. Power the platform and wait for `Ready.` on the serial port.
3. Confirm the IMU came up (`[OK] IMU`). Without a fresh IMU, `ATT` does not
   control.
4. Run:
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name step_roll5 \
       --script tools/scripts/step_roll5.txt
   ```
5. The CSV will hold the `ref_r` step (from 0 to 5 degrees at t ≈ 3 s) and the
   response in `imu_r`. The metrics come from `imu_r` against `ref_r`.
6. For pitch, swap `POSE 5 0` for `POSE 0 5` and rename the output.

Note: the scripted capture is deterministic at the command instants, but the
telemetry sampling is not uniform (up to 50 Hz, subject to loop load). Use the
real `time_ms` timestamps when computing temporal metrics, not the sample index.

### 9.2 IMU offset calibration

Goal: measure the IMU mounting offset at the geometric zero, to fill
`IMU_OFFSET_ROLL` and `IMU_OFFSET_PITCH` in `config.h`.

1. Place the plate at the mechanical reference level (the real geometric zero).
2. Make sure `IMU_OFFSET_ROLL` and `IMU_OFFSET_PITCH` are 0 in the current build,
   so you measure the raw offset. (If they are not, add the current offset to the
   reading.)
3. `MODE CAL_IMU`. Capture for a fixed time:
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name cal_imu --duration 30
   ```
4. Compute the mean of `imu_r` and `imu_p` over the interval. The offset to apply
   is the negative of those means.
5. Write the values into `config.h`, recompile and repeat to confirm `imu_r` and
   `imu_p` stay close to zero at level.

### 9.3 Touchscreen calibration

Goal: obtain the affine ADC to mm transform (the `TS_CAL_*` coefficients in
`config.h`).

1. `MODE CAL_TOUCH`.
2. Capture, reading `bx`/`by` while you place the ball (or a stylus) at known
   plate positions, in measured millimeters.
   ```
   python3 tools/telemetry_logger.py --port /dev/ttyACM0 --name cal_touch --interactive
   ```
3. To get the raw ADC instead of the calibrated value, you need to stream the
   ADC. Today `touch_sensor.cpp` applies the calibration before streaming; the
   block that would stream the raw ADC is commented out. See
   `TECHNICAL_MANUAL.md`, touch module section, to enable the raw output during
   calibration.
4. Fit the six coefficients (`TS_CAL_AX`, `BX`, `DX`, `AY`, `BY`, `DY`) by least
   squares over the (ADC, measured mm) pairs, write them into `config.h` and
   recompile.

---

## 10. Operational safety

- The servos are limited to [`SERVO_MIN_DEG`, `SERVO_MAX_DEG`] (15 to 165
  degrees) and kinematics rejects poses outside the workspace. The integrated
  tilt is limited to `POSE_TILT_MAX` (18 degrees).
- The inner-loop velocity command is limited to `ATT_CMD_MAX` (600 deg/s). The
  ball loop output is limited to `BALL_OUT_MAX` (18 degrees).
- Under violent motion the BNO085 can reset. Recovery is non-blocking (a state
  machine in `imu_sensor.cpp`); open-loop modes keep running during recovery.
- Before enabling `BALL`, check the level with `TRIM` in `CHICKEN`, so the plate
  is actually level when the ball loop takes over.
