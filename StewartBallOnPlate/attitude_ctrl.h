#ifndef ATTITUDE_CTRL_H
#define ATTITUDE_CTRL_H

/*============================================================
  attitude_ctrl.h — Inner-loop attitude controller (2-DOF).
 
  Implements a feedforward + feedback structure:
    - Feedforward : the reference attitude (ref) passed directly
                    through to the IK via the command output.
    - Feedback    : PID on attitude error, with the derivative
                    computed on the MEASUREMENT only (setpoint
                    weight c = 0, Åström & Hägglund notation),
                    eliminating derivative kick on reference steps.
 
  Output 'cmd' is the full attitude COMMAND (feedforward +
  feedback correction), clamped to the mechanical workspace.
  The supervisor feeds cmd directly to makeTiltPose(); it does
  NOT add cmd to att_ref.
============================================================*/

#include "types.h"

namespace AttitudeCtrl {

  // Reset integrators (call on mode change)
  void reset();

  // Compute corrective tilt from attitude error.
  // ref: desired world-frame tilt (from ball controller or manual)
  // meas: IMU-measured world-frame attitude
  // dt_s: time step in seconds
  // cmd   : full attitude command to pass to IK (not a correction)
  void update(const TiltCmd& ref, const Attitude& meas,
              float dt_s, TiltCmd& cmd);
}

#endif // ATTITUDE_CTRL_H
