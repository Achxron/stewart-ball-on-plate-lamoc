#ifndef SUPERVISOR_H
#define SUPERVISOR_H

/*============================================================
  supervisor.h — Mode management, timing, and orchestration.

  The supervisor owns all state, calls the right modules per
  operating mode, and enforces the cascade architecture:

    Ball ctrl → Attitude ctrl → IK → Servos

  It also handles:
    - Mode transitions (via serial commands)
    - Telemetry output
    - Timing regulation
============================================================*/

#include "types.h"

namespace Supervisor {

  // Call once in setup(). Initializes all subsystems.
  void init();

  // Call every loop iteration. Runs the full control pipeline
  // for the current operating mode.
  void tick();

  // Change operating mode. Resets integrators as needed.
  void setMode(OpMode mode);

  // Get current mode
  OpMode getMode();

  // Process a serial command (called from main loop when data available)
  void handleSerial();
}

#endif // SUPERVISOR_H
