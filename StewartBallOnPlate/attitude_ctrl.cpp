/*============================================================
  attitude_ctrl.cpp — Inner-loop attitude controller (2-DOF).

  Structure:

    cmd = ref + Kp·e + Ki·∫e·dt − Kd·ω_gyro (Feedfowart - not used right now)
    cmd = Kp·e + Ki·∫e·dt − Kd·ω_gyro (In use)

  where e = ref − meas and ω_gyro is the BNO085 calibrated
  gyroscope reading (deg/s), supplied through the Attitude
  struct's roll_rate / pitch_rate fields by imu_sensor.

  Using the gyro directly removes two failure modes of the
  previous numerical-derivative implementation:
    1. High-frequency amplification of attitude quantisation noise
    2. The (meas - prev_meas) / dt spike on any dt jitter

  The feedforward 'ref' is added explicitly so the integral
  only carries residual error and ATT_I_MAX can stay small.
  Output is clamped to ATT_CMD_MAX (workspace limit).
============================================================*/

#include "attitude_ctrl.h"
#include "config.h"

namespace AttitudeCtrl {

static float int_roll  = 0.0f;
static float int_pitch = 0.0f;

void reset() {
  int_roll  = 0.0f;
  int_pitch = 0.0f;
}

void update(const TiltCmd& ref, const Attitude& meas,
            float dt_s, TiltCmd& cmd) {

  // ── Roll ──────────────────────────────────────────────────
  float e_roll = ref.roll - meas.roll;
    if (fabsf(e_roll) < ATT_DEADBAND) e_roll = 0.0f;

  int_roll += ATT_KI_ROLL * e_roll * dt_s;
  int_roll  = constrain(int_roll, -ATT_I_MAX_ROLL, ATT_I_MAX_ROLL);

  // Derivative directly from gyroscope. Positive roll_rate means
  // the platform is rolling positive, so the D term must oppose it.
  float d_roll = -ATT_KD_ROLL * meas.roll_rate;

  float deadband_punch = 0.0f;

  // If we are close, but not perfectly on target
  if (abs(e_roll) > 0.05f && abs(e_roll) < 1.0f) { 
      // Force the command past the SERVO deadband. 
      // Tune this 0.5f up to 0.8f if the servo still ignores it.
      deadband_punch = (e_roll > 0) ? 0.0f : -0.0f; // set as 0, 0 to eliminate punch
  }

// Feedforward: tried to implement but it's not robust to base angle. Add IMU to base
//  cmd.roll = constrain(
//      ref.roll + ATT_KP_ROLL * e_roll + int_roll + d_roll + deadband_punch,
//      -ATT_CMD_MAX, ATT_CMD_MAX);

  cmd.roll = constrain(
      ATT_KP_ROLL * e_roll + int_roll + d_roll + deadband_punch,
      -ATT_CMD_MAX, ATT_CMD_MAX);

  // ── Pitch ─────────────────────────────────────────────────
  float e_pitch = ref.pitch - meas.pitch;
  if (fabsf(e_pitch) < ATT_DEADBAND) e_pitch = 0.0f;

  int_pitch += ATT_KI_PITCH * e_pitch * dt_s;
  int_pitch  = constrain(int_pitch, -ATT_I_MAX_PITCH, ATT_I_MAX_PITCH);

  float d_pitch = -ATT_KD_PITCH * meas.pitch_rate;

// Feedforward: tried to implement but it's not robust to base angle. Add IMU to base
//  cmd.pitch = constrain(
//      ref.pitch + ATT_KP_PITCH * e_pitch + int_pitch + d_pitch,
//      -ATT_CMD_MAX, ATT_CMD_MAX);

  cmd.pitch = constrain(
      ATT_KP_PITCH * e_pitch + int_pitch + d_pitch,
      -ATT_CMD_MAX, ATT_CMD_MAX);

}

} // namespace AttitudeCtrl