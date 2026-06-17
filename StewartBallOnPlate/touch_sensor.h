#ifndef TOUCH_SENSOR_H
#define TOUCH_SENSOR_H

/*============================================================
  touch_sensor.h — Resistive touchscreen reader.

  Reads ball position via the SN74HC4066N-switched resistive
  touchscreen. Applies trimmed-mean ADC filtering, affine
  calibration, and threshold-based outlier rejection.

  Output: BallState with (x, y) in plate-frame mm and
  estimated velocity (vx, vy).
============================================================*/

#include "types.h"

namespace Touch {

  // Call once in setup()
  void init();

  // Call every loop iteration.
  // Reads touchscreen, filters, calibrates, estimates velocity.
  // Updates 'ball' in-place.
  void update(BallState& ball);
}

#endif // TOUCH_SENSOR_H
