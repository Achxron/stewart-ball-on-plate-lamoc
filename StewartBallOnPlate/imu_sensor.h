#ifndef IMU_SENSOR_H
#define IMU_SENSOR_H

/*============================================================
  imu_sensor.h — BNO085 IMU reader and filter.

  Reads rotation vector from BNO085 via SPI1, converts
  quaternion → Euler, applies mounting offsets, and provides
  the world-referenced attitude (φ_m, θ_m).

  Also gives the Calibrated gyro  → angular rates (φ̇, θ̇) in deg/s
============================================================*/

#include "types.h"

namespace IMU {

  // Call once in setup(). Returns true if BNO085 is online.
  bool init();

  // Call every loop iteration. Returns true if new data was read.
  // Updates 'att' in-place with world-referenced roll/pitch.
  // Non-blocking: drains pending events up to an internal cap and
  // services the recovery state machine; never stalls the caller.
  bool update(Attitude& att);

  // Re-enable reports after a sensor reset (software-level).
  // Returns true only if BOTH reports were re-enabled successfully.
  bool reEnableReports();

  // true if a valid rotation vector arrived within IMU_TIMEOUT_MS.
  bool isFresh();

  // true while the recovery state machine is actively trying to bring
  // the sensor back (software re-enable or hardware reset in progress).
  bool isRecovering();
}

#endif // IMU_SENSOR_H