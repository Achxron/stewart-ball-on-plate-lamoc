/*============================================================
  supervisor.cpp — Central orchestrator.

  Implements the cascade architecture from the design document:

  ┌─────────────────────────────────────────────────────────┐
  │ BALL_BALANCE mode (full cascade):                       │
  │                                                         │
  │   ball_ctrl(ball) → tilt_ref                            │
  │   att_ctrl(tilt_ref + trim, imu) → correction           │
  │   pose = (0,0,z, trim+ball+corr, trim+ball+corr, 0)    │
  │   IK(pose) → servo_cmd                                  │
  │   servos.write(servo_cmd)                               │
  └─────────────────────────────────────────────────────────┘

  Other modes use subsets of this pipeline.

  Serial protocol (text commands):
    MODE IDLE           — stop all control
    MODE OPEN           — open-loop pose via serial
    MODE ATT <r> <p>    — hold world-frame attitude
    MODE CHICKEN        — level hold (chicken head)
    MODE BALL           — full ball balancing
    MODE CAL_IMU        — stream IMU data
    MODE CAL_TOUCH      — stream touchscreen data
    POSE <r> <p>        — set desired roll/pitch (open-loop or attitude)
    TARGET <x> <y>      — set ball target position
    TRIM <r> <p>        — set trim offsets
    GAINS ATT <kp> <ki> <kd>   — tune attitude loop (live)
    GAINS BALL <kp> <ki> <kd>  — tune ball loop (live)
============================================================*/

#include "supervisor.h"
#include "config.h"
#include "kinematics.h"
#include "imu_sensor.h"
#include "touch_sensor.h"
#include "attitude_ctrl.h"
#include "ball_ctrl.h"
#include "servo_driver.h"

namespace Supervisor {

// ─── State ─────────────────────────────────────────────────
static OpMode       current_mode = OpMode::IDLE;
static Attitude     imu_data;
static BallState    ball_data;
static TiltCmd      ball_tilt;      // output of ball controller
static TiltCmd      att_ref;        // reference for attitude loop
static TiltCmd      att_cmd;        // full attitude command (feedforward + feedback)
static Pose         pose_cmd;       // final pose sent to IK
static ServoCmd     servo_cmd;
static Telemetry    telem;

// Manual tilt target (for ATTITUDE_HOLD / OPEN_LOOP)
static float manual_roll  = 0.0f;
static float manual_pitch = 0.0f;

// Trim offsets (world-frame bias correction)
static float trim_roll  = 0.0f;
static float trim_pitch = 0.0f;

// Integrated pose command (velocity-form PID state).
// The attitude controller emits q̇_cmd (deg/s); this accumulator integrates
// it into an absolute pose command. See doc/velocity_form_pid.md.
static float pose_cmd_roll  = 0.0f;
static float pose_cmd_pitch = 0.0f;

// ── BALL_BALANCE recovery phase ────────────────────────────
// CLOSED:     full cascade (ball → attitude → IK).
// OPEN:       IMU down → open-loop, ball tilt held, no attitude feedback.
// RELEVELING: IMU just returned → ball loop suspended, pose ramped to level
//             at RELEVEL_SLEW_DPS; control handed back to the ball only once
//             attitude has settled at level. Prevents the violent sweep that
//             would otherwise re-trip the sensor.
enum class BalancePhase : uint8_t { CLOSED, OPEN, RELEVELING };
static BalancePhase bphase = BalancePhase::CLOSED;
static uint32_t relevel_ok_since_ms = 0;   // when attitude first met level criteria

// Timing
static uint32_t last_loop_us = 0;
static uint32_t last_telem_ms = 0;
static float    dt_s = 0.01f;

// Outer-loop (ball) rate division. The control tick runs at 100 Hz; the ball
// controller runs once every BALL_LOOP_DIV ticks (20 Hz). ball_dt_s accumulates
// the elapsed time between ball-loop executions so BallCtrl receives the correct
// step (~0.05 s), keeping its derivative and integral terms properly scaled.
// ball_tilt holds its last value between executions (zero-order hold), which the
// inner loop tracks at full 100 Hz.
static uint8_t ball_tick = 0;
static float   ball_dt_s = 0.0f;

// ─── Telemetry output ──────────────────────────────────────
static void sendTelemetry() {
  uint32_t now_ms = millis();
  if (now_ms - last_telem_ms < SERIAL_PERIOD_MS) return;
  last_telem_ms = now_ms;

  // CSV format: time, mode, bx, by, bvx, bvy, bdet,
  //             imu_r, imu_p, ref_r, ref_p, corr_r, corr_p,
  //             pose_r, pose_p, s1..s6
  Serial.print(now_ms);           Serial.print(',');
  Serial.print((int)current_mode); Serial.print(',');

  Serial.print(ball_data.x, 1);  Serial.print(',');
  Serial.print(ball_data.y, 1);  Serial.print(',');
  Serial.print(ball_data.vx, 1); Serial.print(',');
  Serial.print(ball_data.vy, 1); Serial.print(',');
  Serial.print(ball_data.detected); Serial.print(',');

  Serial.print(imu_data.roll, 3);  Serial.print(',');
  Serial.print(imu_data.pitch, 3); Serial.print(',');

  Serial.print(imu_data.roll_rate, 2);  Serial.print(',');
  Serial.print(imu_data.pitch_rate, 2); Serial.print(',');

  Serial.print(att_ref.roll, 3);  Serial.print(',');
  Serial.print(att_ref.pitch, 3); Serial.print(',');

  Serial.print(att_cmd.roll, 3);  Serial.print(',');
  Serial.print(att_cmd.pitch, 3); Serial.print(',');

  Serial.print(pose_cmd.roll, 3);  Serial.print(',');
  Serial.print(pose_cmd.pitch, 3); Serial.print(',');

  for (int i = 0; i < 6; i++) {
    Serial.print(servo_cmd.angle[i], 1);
    Serial.print(i < 5 ? ',' : '\n');
  }
}

static void sendTelemetryHeader() {
  Serial.println("time_ms,mode,bx,by,bvx,bvy,bdet,"
                 "imu_r,imu_p,gyro_r,gyro_p,ref_r,ref_p,cmd_r,cmd_p,"
                 "pose_r,pose_p,s1,s2,s3,s4,s5,s6");
}

// Integrate velocity command into absolute pose.
// - Freezes integration when IMU is stale (no windup on dead sensor).
// - Clamps pose to the physical workspace, NOT to the velocity limit.
static void integratePose(const TiltCmd& cmd, float dt_s) {
  if (!IMU::isFresh()) return;            // hold last good pose (fail-safe)
  pose_cmd_roll  = constrain(pose_cmd_roll  + cmd.roll  * dt_s,
                             -POSE_TILT_MAX, POSE_TILT_MAX);
  pose_cmd_pitch = constrain(pose_cmd_pitch + cmd.pitch * dt_s,
                             -POSE_TILT_MAX, POSE_TILT_MAX);
}

// Ramp one axis of the pose command toward zero (level) at a bounded rate.
// Returns the new value. Used during RELEVELING so the platform returns to
// level slowly enough not to re-disturb the sensor.
static float slewToZero(float value, float max_step) {
  if (value >  max_step) return value - max_step;
  if (value < -max_step) return value + max_step;
  return 0.0f;
}

// True when the measured attitude is at level and not moving — the condition
// for handing control back to the ball loop after a relevel.
static bool attitudeSettledLevel(const Attitude& a) {
  return fabsf(a.roll)       < RELEVEL_ANGLE_TOL &&
         fabsf(a.pitch)      < RELEVEL_ANGLE_TOL &&
         fabsf(a.roll_rate)  < RELEVEL_RATE_TOL  &&
         fabsf(a.pitch_rate) < RELEVEL_RATE_TOL;
}

// ─── Mode transitions ──────────────────────────────────────
void setMode(OpMode mode) {
  if (mode == current_mode) return;

  // Reset controllers on any mode change
  AttitudeCtrl::reset();
  BallCtrl::reset();
  ball_tilt = {0, 0};
  att_cmd = {0, 0};
  pose_cmd_roll  = 0.0f;
  pose_cmd_pitch = 0.0f;
  ball_tick = 0;
  ball_dt_s = 0.0f;
  bphase = BalancePhase::CLOSED;
  relevel_ok_since_ms = 0;

  current_mode = mode;
  Serial.print("MODE → ");
  Serial.println((int)mode);
  sendTelemetryHeader();
}

OpMode getMode() { return current_mode; }

// ─── Init ──────────────────────────────────────────────────
void init() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("=== Stewart Ball-on-Plate ===");

  Kinematics::init();
  Serial.println("[OK] Kinematics");

  Servos::init();
  Serial.println("[OK] Servos (neutral)");

  if (IMU::init()) {
    Serial.println("[OK] IMU (BNO085)");
  } else {
    Serial.println("[FAIL] IMU — check wiring");
  }

  Touch::init();
  Serial.println("[OK] Touchscreen");

  BallCtrl::setTarget(PLATE_CENTER_X, PLATE_CENTER_Y);

  last_loop_us = micros();
  last_telem_ms = millis();

  setMode(OpMode::IDLE);
  Serial.println("Ready. Send MODE command to begin.");
}

// ─── Main tick ─────────────────────────────────────────────
void tick() {
  // ── IMU poll (every loop) ────────────────────────────────
  // update() is non-blocking: it drains pending events (bounded) and
  // services the recovery state machine. It NEVER stalls the loop, so
  // open-loop modes keep running even while the IMU is down/recovering.
  // The IMU is no longer a global gate — each mode decides whether it
  // needs fresh attitude (see IMU::isFresh() checks below).
  IMU::update(imu_data);

  // ── Timing ───────────────────────────────────────────────
  uint32_t now_us = micros();
  uint32_t elapsed_us = now_us - last_loop_us;
  if (elapsed_us < LOOP_PERIOD_US) return;  // rate-limit control loop
  last_loop_us = now_us;
  dt_s = (float)elapsed_us * 1e-6f;

  // ── Outer-loop rate division ─────────────────────────────
  // Accumulate elapsed time every tick; fire the ball loop once every
  // BALL_LOOP_DIV ticks. ball_due is consumed inside the cascade modes.
  ball_dt_s += dt_s;
  bool ball_due = (++ball_tick >= BALL_LOOP_DIV);

  switch (current_mode) {

    // ────────────────────────────────────────────────────────
    case OpMode::IDLE:
      // Servos stay at neutral. Only sensors update.
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::OPEN_LOOP_POSE:
      Touch::update(ball_data);
      // Direct pose → IK → servos, no feedback
      pose_cmd = Kinematics::makeTiltPose(
        trim_roll + manual_roll,
        trim_pitch + manual_pitch
      );
      Kinematics::solve(pose_cmd, servo_cmd);
      Servos::write(servo_cmd);
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::ATTITUDE_HOLD:
      Touch::update(ball_data);
      // Velocity-form PID: AttitudeCtrl emits q̇_cmd (deg/s);
      // the supervisor integrates it into an absolute pose command.
      // This mode has no safe open-loop fallback (it IS an attitude loop),
      // so when the IMU is not fresh we hold the last valid pose and reset
      // the attitude integrators to prevent windup on frozen measurements.
      if (IMU::isFresh()) {
        att_ref.roll  = trim_roll  + manual_roll;
        att_ref.pitch = trim_pitch + manual_pitch;

        AttitudeCtrl::update(att_ref, imu_data, dt_s, att_cmd);
        integratePose(att_cmd, dt_s);   // self-freezes if IMU goes stale
      } else {
        AttitudeCtrl::reset();
        att_cmd = {0, 0};
        // pose_cmd_roll/pitch retain their last value → servos hold pose.
      }
      pose_cmd = Kinematics::makeTiltPose(pose_cmd_roll, pose_cmd_pitch);
      Kinematics::solve(pose_cmd, servo_cmd);
      Servos::write(servo_cmd);
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::CHICKEN_HEAD:
      // Level hold: reference = trim (world-level).
      // Same velocity-form integration as ATTITUDE_HOLD, and the same
      // hold-last-pose / reset-integrators behaviour when the IMU is down.
      if (IMU::isFresh()) {
        att_ref.roll  = trim_roll;
        att_ref.pitch = trim_pitch;

        AttitudeCtrl::update(att_ref, imu_data, dt_s, att_cmd);
        integratePose(att_cmd, dt_s);
      } else {
        AttitudeCtrl::reset();
        att_cmd = {0, 0};
      }
      pose_cmd = Kinematics::makeTiltPose(pose_cmd_roll, pose_cmd_pitch);
      Kinematics::solve(pose_cmd, servo_cmd);
      Servos::write(servo_cmd);
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::BALL_BALANCE:
      // Cascade with safe sensor-loss handling, via a 3-phase machine:
      //
      //   CLOSED     full cascade: ball (20 Hz) → attitude (100 Hz) → IK.
      //   OPEN       IMU lost → open-loop, ball tilt held, no feedback.
      //   RELEVELING IMU returned → ball loop SUSPENDED, pose ramped to level
      //              at RELEVEL_SLEW_DPS; control returns to the ball only
      //              after attitude settles at level. This is what prevents
      //              the violent sweep that used to re-trip the sensor.
      //
      // The ball loop (Touch + BallCtrl) only runs in CLOSED; ball_tilt is
      // held by zero-order hold elsewhere. Transitions are driven by
      // IMU::isFresh().

      // ── Phase transitions ────────────────────────────────
      if (!IMU::isFresh()) {
        // Sensor not trustworthy → open-loop hold. Suspend the ball loop and
        // clear its state so a stale error can't kick on return.
        if (bphase != BalancePhase::OPEN) {
          bphase = BalancePhase::OPEN;
          BallCtrl::reset();
          AttitudeCtrl::reset();
          relevel_ok_since_ms = 0;
        }
      } else if (bphase == BalancePhase::OPEN) {
        // Sensor just came back → relevel before touching the ball.
        bphase = BalancePhase::RELEVELING;
        BallCtrl::reset();
        AttitudeCtrl::reset();
        ball_tilt = {0, 0};
        relevel_ok_since_ms = 0;
      }

      // ── Phase behaviour ──────────────────────────────────
      if (bphase == BalancePhase::CLOSED) {
        // Run the ball loop at 20 Hz; inner loop tracks at 100 Hz.
        if (ball_due) {
          Touch::update(ball_data);
          BallCtrl::update(ball_data, ball_dt_s, ball_tilt);
        }
        att_ref.roll  = trim_roll  + ball_tilt.roll;
        att_ref.pitch = trim_pitch + ball_tilt.pitch;

        AttitudeCtrl::update(att_ref, imu_data, dt_s, att_cmd);
        integratePose(att_cmd, dt_s);
        pose_cmd = Kinematics::makeTiltPose(pose_cmd_roll, pose_cmd_pitch);
      }
      else if (bphase == BalancePhase::OPEN) {
        // Open-loop hold: keep the last ball tilt, sync the integrator so the
        // eventual relevel starts from the actually-applied pose (bumpless).
        att_cmd = {0, 0};
        pose_cmd_roll  = trim_roll  + ball_tilt.roll;
        pose_cmd_pitch = trim_pitch + ball_tilt.pitch;
        pose_cmd = Kinematics::makeTiltPose(pose_cmd_roll, pose_cmd_pitch);
      }
      else { // RELEVELING
        // Ball loop suspended. Ramp the pose toward level at a bounded rate.
        const float step = RELEVEL_SLEW_DPS * dt_s;   // deg this tick
        pose_cmd_roll  = slewToZero(pose_cmd_roll,  step);
        pose_cmd_pitch = slewToZero(pose_cmd_pitch, step);
        att_cmd = {0, 0};
        pose_cmd = Kinematics::makeTiltPose(pose_cmd_roll, pose_cmd_pitch);

        // Hand back to the ball only when attitude is settled at level for
        // RELEVEL_HOLD_MS continuously.
        uint32_t now = millis();
        if (attitudeSettledLevel(imu_data)) {
          if (relevel_ok_since_ms == 0) relevel_ok_since_ms = now;
          if (now - relevel_ok_since_ms >= RELEVEL_HOLD_MS) {
            bphase = BalancePhase::CLOSED;
            BallCtrl::reset();
            AttitudeCtrl::reset();
            ball_tilt = {0, 0};
            relevel_ok_since_ms = 0;
          }
        } else {
          relevel_ok_since_ms = 0;   // not settled → restart the hold timer
        }
      }

      Kinematics::solve(pose_cmd, servo_cmd);
      Servos::write(servo_cmd);
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::CALIBRATE_IMU:
      // Just stream IMU data — no control
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::CALIBRATE_TOUCH:
      Touch::update(ball_data);
      // Just stream touch data — no control
      break;

    // ────────────────────────────────────────────────────────
    case OpMode::BALL_OPEN:
      // Partial cascade: ball ctrl (20 Hz) → IK → servos (NO IMU feedback).
      // ball_tilt is held between ball-loop executions; IK/servo write stays
      // at 100 Hz on the held command.
      if (ball_due) {
        Touch::update(ball_data);
        BallCtrl::update(ball_data, ball_dt_s, ball_tilt);
      }

      // Feed the ball controller's desired tilt directly to the kinematics
      pose_cmd = Kinematics::makeTiltPose(
        trim_roll + ball_tilt.roll, 
        trim_pitch + ball_tilt.pitch
      );
      
      Kinematics::solve(pose_cmd, servo_cmd);
      Servos::write(servo_cmd);
      break;
  }

  // ── Consume the outer-loop trigger ───────────────────────
  // Reset the divider and dt accumulator once the ball loop has fired
  // (or would have fired). Done after the switch so every mode consumes
  // the trigger uniformly and ball_tick never overflows.
  if (ball_due) {
    ball_tick = 0;
    ball_dt_s = 0.0f;
  }

  // ── Telemetry ────────────────────────────────────────────
  sendTelemetry();
}

// ─── Serial command parser ─────────────────────────────────
// Non-blocking: accumulates chars in a buffer, only parses on '\n' or '\r'.
// This prevents the main loop from stalling (as Serial.readStringUntil does
// — it has a 1-second timeout that fires if only partial input arrives).
static char   cmd_buf[96];
static uint8_t cmd_len = 0;

static void processCommand(const char* raw);

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\r') continue;            // ignore CR from Windows line endings

    if (c == '\n') {
      cmd_buf[cmd_len] = 0;             // null-terminate
      if (cmd_len > 0) {
        processCommand(cmd_buf);
      }
      cmd_len = 0;
      continue;
    }

    if (cmd_len < sizeof(cmd_buf) - 1) {
      cmd_buf[cmd_len++] = c;
    } else {
      // Overflow: discard and reset
      cmd_len = 0;
    }
  }
}

static void processCommand(const char* raw) {
  String line(raw);
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("MODE")) {
    if      (line.indexOf("IDLE")      >= 0) setMode(OpMode::IDLE);
    else if (line.indexOf("BALL_OPEN") >= 0) setMode(OpMode::BALL_OPEN);
    else if (line.indexOf("OPEN")      >= 0) setMode(OpMode::OPEN_LOOP_POSE);
    else if (line.indexOf("ATT")       >= 0) {
      setMode(OpMode::ATTITUDE_HOLD);
      int idx = line.indexOf("ATT") + 3;
      String rest = line.substring(idx);
      rest.trim();
      int sp = rest.indexOf(' ');
      if (sp > 0) {
        manual_roll  = rest.substring(0, sp).toFloat();
        manual_pitch = rest.substring(sp + 1).toFloat();
      }
    }
    else if (line.indexOf("CHICKEN")   >= 0) setMode(OpMode::CHICKEN_HEAD);
    else if (line.indexOf("BALL")      >= 0) setMode(OpMode::BALL_BALANCE);
    else if (line.indexOf("CAL_IMU")   >= 0) setMode(OpMode::CALIBRATE_IMU);
    else if (line.indexOf("CAL_TOUCH") >= 0) setMode(OpMode::CALIBRATE_TOUCH);
    else Serial.println("ERR: unknown mode");
  }
  else if (line.startsWith("POSE")) {
    int sp1 = line.indexOf(' ', 5);
    if (sp1 > 0) {
      manual_roll  = line.substring(5, sp1).toFloat();
      manual_pitch = line.substring(sp1 + 1).toFloat();
      Serial.print("POSE → ");
      Serial.print(manual_roll); Serial.print(", ");
      Serial.println(manual_pitch);
    }
  }
  else if (line.startsWith("TARGET")) {
    int sp1 = line.indexOf(' ', 7);
    if (sp1 > 0) {
      float tx = line.substring(7, sp1).toFloat();
      float ty = line.substring(sp1 + 1).toFloat();
      BallCtrl::setTarget(tx, ty);
      Serial.print("TARGET → ");
      Serial.print(tx); Serial.print(", ");
      Serial.println(ty);
    }
  }
  else if (line.startsWith("TRIM")) {
    int sp1 = line.indexOf(' ', 5);
    if (sp1 > 0) {
      trim_roll  = line.substring(5, sp1).toFloat();
      trim_pitch = line.substring(sp1 + 1).toFloat();
      Serial.print("TRIM → ");
      Serial.print(trim_roll); Serial.print(", ");
      Serial.println(trim_pitch);
    }
  }
  else {
    Serial.println("ERR: unknown command");
    Serial.println("Commands: MODE IDLE|OPEN|ATT|CHICKEN|BALL|CAL_IMU|CAL_TOUCH");
    Serial.println("          POSE <r> <p> | TARGET <x> <y> | TRIM <r> <p>");
  }
}

} // namespace Supervisor