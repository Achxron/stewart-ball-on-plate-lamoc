/*============================================================
  kinematics.cpp — Stewart platform inverse kinematics.

  Matches the geometry from stewart_platform.py exactly.

  Each servo horn sweeps in the plane (dir[i], ẑ):

    horn_tip(θ) = base[i] + L1 · (dir[i]·cos θ  +  ẑ·sin θ)

  The rod of length L2 connects horn_tip to the platform joint.
  Given a platform pose, we solve for θ analytically.

  IK derivation
  ─────────────
  Let Q_i = R·plat[i] + T   (platform joint in world frame)
  Let D_i = Q_i − base[i]   (vector from base pivot to platform joint)

  Rod constraint:  |D_i − L1·(dir·cosθ + ẑ·sinθ)|² = L2²

  Expanding:
    |D|² − 2L1(D·dir)cosθ − 2L1·Dz·sinθ + L1² = L2²

  Rearranging:  A·cosθ + B·sinθ = C

    A = D_i · dir_i          (= Dx·DIR_X + Dy·DIR_Y)
    B = Dz                   (= D_i.z)
    C = (|D|² + L1² − L2²) / (2·L1)

  Solution:
    θ = asin(C / √(A²+B²)) − atan2(A, B)

  Servo conversion:
    θ = 0     → horn horizontal along dir[i] → servo = 90°
    θ > 0     → horn tilts up                → servo > 90°
    θ < 0     → horn tilts down              → servo < 90°
    servo_deg = 90 + θ·(180/π) + trim[i]
============================================================*/

#include "kinematics.h"
#include "config.h"
#include <math.h>

namespace Kinematics {

void init() {
  // Geometry is fully defined in config.h — nothing to precompute.
  // This function exists so the supervisor can call init() uniformly.
}

// ─── ZYX Euler rotation matrix ─────────────────────────────
static void eulerToR(float r_rad, float p_rad, float y_rad, float R[3][3]) {
  float cr = cosf(r_rad),  sr = sinf(r_rad);
  float cp = cosf(p_rad),  sp = sinf(p_rad);
  float cy = cosf(y_rad),  sy = sinf(y_rad);

  R[0][0] = cy * cp;
  R[0][1] = cy * sp * sr - sy * cr;
  R[0][2] = cy * sp * cr + sy * sr;
  R[1][0] = sy * cp;
  R[1][1] = sy * sp * sr + cy * cr;
  R[1][2] = sy * sp * cr - cy * sr;
  R[2][0] = -sp;
  R[2][1] = cp * sr;
  R[2][2] = cp * cr;
}

bool solve(const Pose& pose, ServoCmd& cmd) {
  float r_rad = pose.roll  * (M_PI / 180.0f);
  float p_rad = pose.pitch * (M_PI / 180.0f);
  float y_rad = pose.yaw   * (M_PI / 180.0f);

  float R[3][3];
  eulerToR(r_rad, p_rad, y_rad, R);

  bool all_valid = true;

  for (int i = 0; i < 6; i++) {
    // ── Platform joint in world frame: Q = R·plat + T ──────
    float qx = R[0][0] * PLAT_PX[i] + R[0][1] * PLAT_PY[i] + pose.x;
    float qy = R[1][0] * PLAT_PX[i] + R[1][1] * PLAT_PY[i] + pose.y;
    float qz = R[2][0] * PLAT_PX[i] + R[2][1] * PLAT_PY[i] + pose.z;
    // (plat_z = 0 for all joints, so the R[*][2] terms vanish)

    // ── Leg vector: D = Q − base ───────────────────────────
    float dx = qx - BASE_PX[i];
    float dy = qy - BASE_PY[i];
    float dz = qz;  // base_z = 0

    float D2 = dx * dx + dy * dy + dz * dz;

    // ── Solve A·cosθ + B·sinθ = C ──────────────────────────
    float A = dx * DIR_X[i] + dy * DIR_Y[i];   // D · dir
    float B = dz;                                // D · ẑ
    float C = (D2 + L1 * L1 - L2 * L2) / (2.0f * L1);

    float R2 = A * A + B * B;
    float disc = R2 - C * C;

    if (disc < 0.0f) {
      // Pose is outside the workspace for this leg
      all_valid = false;
      return false;  // keep previous angle
    }

    // θ = asin(C / √(A²+B²)) − atan2(A, B)
    float theta = asinf(C / sqrtf(R2)) - atan2f(A, B);

    // ── Convert to servo degrees ───────────────────────────
    // θ=0 → horn horizontal → 90° servo PWM
    float deg = 90.0f + SERVO_SIGN[i] * theta * (180.0f / M_PI) + SERVO_TRIM[i];

    cmd.angle[i] = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  }

  return all_valid;
}

Pose makeTiltPose(float roll_deg, float pitch_deg) {
  Pose p;
  p.x     = 0.0f;
  p.y     = 0.0f;
  p.z     = Z_HOME;
  p.roll  = roll_deg;
  p.pitch = pitch_deg;
  p.yaw   = 0.0f;
  return p;
}

void computeJoints(const ServoCmd& cmd, float joints_out[6][3]) {
  for (int i = 0; i < 6; i++) {
    // Convert servo degrees back to IK angle θ
    float theta = SERVO_SIGN[i] * (cmd.angle[i] - 90.0f - SERVO_TRIM[i]) * (M_PI / 180.0f);

    float ct = cosf(theta);
    float st = sinf(theta);

    // horn_tip = base + L1 · (dir·cosθ + ẑ·sinθ)
    joints_out[i][0] = BASE_PX[i] + L1 * DIR_X[i] * ct;
    joints_out[i][1] = BASE_PY[i] + L1 * DIR_Y[i] * ct;
    joints_out[i][2] = L1 * st;
  }
}

} // namespace Kinematics
