#ifndef BALL_CTRL_H
#define BALL_CTRL_H

/*============================================================
  ball_ctrl.h — Outer-loop ball position controller.

  Takes ball position (x_b, y_b) and target (x_ref, y_ref),
  computes desired plate tilt in world frame (φ_ball, θ_ball).

  The output is a WORLD-FRAME tilt command, fed as the reference
  to the inner attitude loop.
============================================================*/

#include "types.h"

namespace BallCtrl {

  // Reset integrators (call on mode change)
  void reset();

  // Set the target ball position (mm, plate frame)
  void setTarget(float x_ref, float y_ref);

  // Compute desired world-frame tilt from ball error.
  // ball: current ball state (position + velocity)
  // dt_s: time step in seconds
  // out: desired plate tilt (degrees, world frame)
  void update(const BallState& ball, float dt_s, TiltCmd& out);
}

#endif // BALL_CTRL_H
