/*============================================================
  servo_driver.cpp — PCA9685 servo output.

  Channels 1..6 on the PCA9685 (matching your wiring).
  Angles clamped to [SERVO_MIN_DEG, SERVO_MAX_DEG].
============================================================*/

#include "servo_driver.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace Servos {

static TwoWire i2c(PCA_SDA_PIN, PCA_SCL_PIN);
static Adafruit_PWMServoDriver pwm(PCA_ADDR, i2c);

void init() {
  i2c.begin();
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  writeNeutral();
}

void write(const ServoCmd& cmd) {
  for (int i = 0; i < 6; i++) {
    float angle = constrain(cmd.angle[i], SERVO_MIN_DEG, SERVO_MAX_DEG);
    uint16_t pulse = map((long)(angle), 0, 180, SERVO_PWM_MIN, SERVO_PWM_MAX);
    pwm.setPWM(i + 1, 0, pulse);  // channels 1-6
  }
}

void writeNeutral() {
  ServoCmd neutral;
  for (int i = 0; i < 6; i++) {
    neutral.angle[i] = 90.0f + SERVO_TRIM[i];
  }
  write(neutral);
}

} // namespace Servos
