/*============================================================
  StewartBallOnPlate.ino
  ──────────────────────────────────────────────────────────

  Stewart Platform + Ball-on-Plate cascade control system.

  Hardware:
    MCU:   Raspberry Pi Pico (RP2040)
    Servos: 6× MG90D via PCA9685 (I2C0, SDA=GP8, SCL=GP9)
    IMU:    BNO085 via SPI1 (SCK=GP10, MOSI=GP11, MISO=GP12,
            CS=GP13, INT=GP14, RST=GP15)
    Touch:  Resistive screen via SN74HC4066N
            (D2-D7 digital, GP26/GP27 analog)

  Architecture:
    Outer loop:  ball position → desired plate tilt
    Inner loop:  desired tilt + IMU → corrected pose
    IK:          corrected pose → 6 servo angles
    Driver:      servo angles → PWM

  Serial commands (115200 baud):
    MODE IDLE / OPEN / ATT / CHICKEN / BALL / CAL_IMU / CAL_TOUCH
    POSE <roll> <pitch>
    TARGET <x> <y>
    TRIM <roll> <pitch>

  Libraries:
    - Adafruit PWM Servo Driver Library
    - SparkFun BNO08x Arduino Library

  Project structure:
    config.h          — all pins, geometry, tuning constants
    types.h           — shared data structures
    kinematics.h/cpp  — inverse kinematics (pose → servos)
    imu_sensor.h/cpp  — BNO085 reader
    touch_sensor.h/cpp — touchscreen reader
    attitude_ctrl.h/cpp — inner attitude PID
    ball_ctrl.h/cpp   — outer ball PID
    servo_driver.h/cpp — PCA9685 output
    supervisor.h/cpp  — mode management & orchestration
============================================================*/

#include "supervisor.h"

void setup() {
  Supervisor::init();
}

void loop() {
  Supervisor::handleSerial();
  Supervisor::tick();
}
