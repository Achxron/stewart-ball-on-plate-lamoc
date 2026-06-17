#ifndef TYPES_H
#define TYPES_H

/*============================================================
  types.h — Shared data structures used across all modules.

  These are plain-old-data structs passed by reference between
  modules. No module "owns" another; the supervisor owns all
  state and passes it explicitly.
============================================================*/

#include <Arduino.h>

// ─── Stewart platform pose (6-DOF) ────────────────────────
// Translational components in mm, rotational in degrees.
struct Pose {
  float x     = 0.0f;
  float y     = 0.0f;
  float z     = 0.0f;
  float roll  = 0.0f;   // φ  — rotation about X
  float pitch = 0.0f;   // θ  — rotation about Y
  float yaw   = 0.0f;   // ψ  — rotation about Z
};

// ─── Attitude (roll/pitch from IMU, world-referenced) ──────
struct Attitude {
  float roll  = 0.0f;   // degrees, gravity-referenced
  float pitch = 0.0f;   // degrees, gravity-referenced
  float yaw   = 0.0f;   // degrees (available but unused for control)
  float roll_rate  = 0.0f;   // deg/s, from BNO085 calibrated gyro
  float pitch_rate = 0.0f;   // deg/s, from BNO085 calibrated gyro
  float yaw_rate   = 0.0f;   // deg/s, from BNO085 calibrated gyro
  bool  valid = false;   // true if IMU is reporting
};

// ─── Ball state from touchscreen ───────────────────────────
struct BallState {
  float x  = 0.0f;      // mm, plate coordinates
  float y  = 0.0f;      // mm, plate coordinates
  float vx = 0.0f;      // mm/s, estimated
  float vy = 0.0f;      // mm/s, estimated
  bool  detected = false;
  uint32_t timestamp_ms = 0;
};

// ─── Servo command vector ──────────────────────────────────
struct ServoCmd {
  float angle[6] = { 90, 90, 90, 90, 90, 90 }; // degrees (0..180)
};

// ─── Tilt command (output of ball controller / input of attitude loop) ──
struct TiltCmd {
  float roll  = 0.0f;   // desired plate roll in world frame (degrees)
  float pitch = 0.0f;   // desired plate pitch in world frame (degrees)
};

// ─── Operating modes ───────────────────────────────────────
enum class OpMode : uint8_t {
  IDLE,              // servos at neutral, no control
  OPEN_LOOP_POSE,    // direct pose command via serial, no feedback
  ATTITUDE_HOLD,     // inner loop only — hold a fixed world-frame tilt
  CHICKEN_HEAD,      // inner loop rejecting base disturbances (level hold)
  BALL_BALANCE,      // full cascade — outer + inner loop
  CALIBRATE_IMU,     // stream IMU data for offset measurement
  CALIBRATE_TOUCH,   // stream touchscreen data for calibration
  BALL_OPEN,         // complete cascade but open loop on the platform
};

// ─── Telemetry packet (for serial logging) ─────────────────
struct Telemetry {
  uint32_t   timestamp_ms;
  OpMode     mode;
  BallState  ball;
  Attitude   imu;
  TiltCmd    tilt_cmd;
  Pose       pose_cmd;
  ServoCmd   servos;
  uint8_t    flags;   // see supervisor for flag definitions
};

#endif // TYPES_H
