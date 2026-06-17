Read this in: English | [Português](README.pt-BR.md)

# Stewart Ball-on-Plate

Cascade control of a 6-DOF Stewart platform driven by six rotary servos, balancing
a ball on a resistive plate. Microcontroller: Raspberry Pi Pico (RP2040). The
outer loop (ball position) produces the desired tilt; the inner loop (attitude,
with a BNO085 IMU) applies that tilt while rejecting disturbances; inverse
kinematics converts the pose into six servo angles.

Code supporting the undergraduate thesis of Arthur Rolinski, UFPR (Universidade Federal do Paraná), advised by PhD. João Victor de Carvalho Fontes. 
Title: 'Atualização Estrutural e Aplicação de Controle PID em Cascata para Plataforma de Stewart-Gough com 6 Graus de Liberdade e Sistema Bola e Placa'.

## Hardware

- Raspberry Pi Pico (RP2040)
- PCA9685 servo driver (I2C), six MG90D servos
- BNO085 IMU (SPI1)
- Resistive touchscreen via SN74HC4066N
- 5 V supply for the servos

The effective pins live in `config.h`, which is the single source of truth.

## Structure

```
StewartBallOnPlate/        (root = Arduino sketch folder)
├── StewartBallOnPlate.ino
├── config.h, types.h
├── kinematics.*, imu_sensor.*, touch_sensor.*, servo_driver.*
├── attitude_ctrl.*, ball_ctrl.*, supervisor.*
├── USER_GUIDE.md         how to operate (commands, telemetry, capture)
├── TECHNICAL_MANUAL.md   modules, maintenance, extension, inconsistencies
└── tools/
    ├── telemetry_logger.py   capture/command/plot (runs on the PC)
    ├── make_qr.py            generates the repository-link QR code
    └── scripts/              timed sequences (.txt)
```

The repository root is the sketch folder itself, so the folder name must match the
`.ino` name (`StewartBallOnPlate`) for the Arduino IDE to open it.

## Build and flash

1. Open `StewartBallOnPlate.ino` in the Arduino IDE with the RP2040 core
   installed.
2. Install the libraries: Adafruit PWM Servo Driver Library and SparkFun BNO08x
   Arduino Library.
3. Select the Raspberry Pi Pico board and flash over USB.

Full step-by-step, commands and telemetry: see `USER_GUIDE.md`.

## Documentation

Operation (serial commands, modes, telemetry format, capture tool, reproducible
procedures):
- English: `USER_GUIDE.md`
- Português: `GUIA_DE_USO.md`

Internal reference (per-module description, maintenance, how to extend, known
divergences between comments and code):
- English: `TECHNICAL_MANUAL.md`
- Português: `MANUAL_TECNICO.md`

## License

See `LICENSE`. Confirm your institution's intellectual property policy before
publishing.
