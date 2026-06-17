#ifndef SERVO_DRIVER_H
#define SERVO_DRIVER_H

/*============================================================
  servo_driver.h — PCA9685 servo output driver.

  Accepts a ServoCmd (6 angles in degrees) and writes PWM.
  This is the ONLY module that talks to the PCA9685.
============================================================*/

#include "types.h"

namespace Servos {

  // Call once in setup(). Initializes I2C and PCA9685.
  void init();

  // Write a servo command to hardware.
  void write(const ServoCmd& cmd);

  // Write all servos to neutral (90° + trim). Safe startup pose.
  void writeNeutral();
}

#endif // SERVO_DRIVER_H
