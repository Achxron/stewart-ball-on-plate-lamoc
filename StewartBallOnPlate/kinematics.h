#ifndef KINEMATICS_H
#define KINEMATICS_H

/*============================================================
  kinematics.h — Stewart platform inverse kinematics.

  Geometry from stewart_platform.py:
    - 6 servo pivots at arbitrary base positions
    - 6 platform joints at arbitrary body-frame positions
    - Each servo horn sweeps in the plane (dir_i, ẑ)
    - Horn length L1, rod length L2

  Takes a commanded Pose and returns 6 servo angles.
  This is the ONLY module that knows about platform geometry.
============================================================*/

#include "types.h"

namespace Kinematics {

  // Call once in setup() — nothing to precompute with explicit geometry
  void init();

  // Inverse kinematics: Pose → ServoCmd
  // Returns true if all 6 legs have a valid solution.
  // On failure, the failed servo(s) keep their previous value.
  bool solve(const Pose& pose, ServoCmd& cmd);

  // Build a neutral-height, tilt-only pose:
  //   q = (0, 0, Z_HOME, roll, pitch, 0)
  Pose makeTiltPose(float roll_deg, float pitch_deg);

  // Forward check: given a ServoCmd, compute the horn-tip positions
  // (useful for debugging / visualization via serial)
  void computeJoints(const ServoCmd& cmd, float joints_out[6][3]);
}

#endif // KINEMATICS_H
