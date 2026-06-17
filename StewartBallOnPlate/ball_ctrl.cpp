/*============================================================
  ball_ctrl.cpp — Outer-loop ball PID.

  Computes world-frame tilt needed to drive ball to target.
  Uses gain scheduling (close vs far) from the original code.

  IMPORTANT: The output axes map plate-frame ball error to
  world-frame tilt. For a standard mounting:
    - Ball +X error → plate pitch θ  (tilt to move ball in X)
    - Ball +Y error → plate roll  φ  (tilt to move ball in Y)
  Adjust the sign/mapping if your axes differ.
============================================================*/

#include "ball_ctrl.h"
#include "config.h"
#include <math.h>

namespace BallCtrl {

static float target_x = PLATE_CENTER_X;
static float target_y = PLATE_CENTER_Y;

static float int_x = 0.0f, int_y = 0.0f;
static float prev_err_x = 0.0f, prev_err_y = 0.0f;

void reset() {
  int_x = 0.0f;
  int_y = 0.0f;
  prev_err_x = 0.0f;
  prev_err_y = 0.0f;
}

void setTarget(float x_ref, float y_ref) {
  target_x = x_ref;
  target_y = y_ref;
}

void update(const BallState& ball, float dt_s, TiltCmd& out) {
  if (!ball.detected || dt_s < 0.0001f) {
    // No ball or invalid dt — hold last command
    return;
  }

  float ex_raw = target_x - ball.x;
  float ey_raw = target_y - ball.y;

  // Deadband: suppress P and I for small errors.
  // prev_err is always updated with the raw error to avoid
  // a derivative spike when exiting the deadband.
  float ex = (fabsf(ex_raw) < BALL_DEADBAND) ? 0.0f : ex_raw;
  float ey = (fabsf(ey_raw) < BALL_DEADBAND) ? 0.0f : ey_raw;

  // ── Gain scheduling ──────────────────────────────────────
  float dist = fmaxf(fabsf(ex_raw), fabsf(ey_raw));  // use raw for scheduling
  float kp, ki, kd;
  if (dist > BALL_TRANSITION_DIST) {
    kp = BALL_KP_FAR;  ki = BALL_KI_FAR;  kd = BALL_KD_FAR;
  } else {
    kp = BALL_KP_CLOSE; ki = BALL_KI_CLOSE; kd = BALL_KD_CLOSE;
  }

  // ── PID for X axis ──────────────────────────────────────
  float px = kp * ex;
  int_x += ki * ex * dt_s;
  int_x = constrain(int_x, -BALL_I_MAX, BALL_I_MAX);
  float dx = kd * (ex_raw - prev_err_x) / dt_s;  // derivative on raw
  prev_err_x = ex_raw;
  float ux = constrain(px + int_x + dx, -BALL_OUT_MAX, BALL_OUT_MAX);

  // ── PID for Y axis ──────────────────────────────────────
  float py = kp * ey;
  int_y += ki * ey * dt_s;
  int_y = constrain(int_y, -BALL_I_MAX, BALL_I_MAX);
  float dy = kd * (ey_raw - prev_err_y) / dt_s;  // derivative on raw
  prev_err_y = ey_raw;
  float uy = constrain(py + int_y + dy, -BALL_OUT_MAX, BALL_OUT_MAX);

  // ── Map ball-frame error → world-frame tilt ──────────────
  // Convention: positive X error → positive pitch (θ) tilts plate
  //             positive Y error → positive roll  (φ) tilts plate
  // ADJUST SIGNS HERE if the platform axes are behaving differently.
  out.pitch = ux;
  out.roll  = -uy;
}

} // namespace BallCtrl
