# Stewart Ball-on-Plate

Controle em cascata de uma plataforma de Stewart de 6 graus de liberdade, atuada
por seis servos rotativos, equilibrando uma bola sobre uma placa resistiva.
Microcontrolador Raspberry Pi Pico (RP2040). Malha externa (posicao da bola) gera
a inclinacao desejada; malha interna (atitude, com IMU BNO085) aplica essa
inclinacao corrigindo disturbios; a cinematica inversa converte a pose em seis
angulos de servo.

Codigo de apoio ao TCC de `<seu nome>`, `<instituicao / laboratorio>`,
orientacao de `<orientador(a)>`. Titulo: `<titulo do TCC>`.

## Hardware

- Raspberry Pi Pico (RP2040)
- Driver de servo PCA9685 (I2C), seis servos MG90D
- IMU BNO085 (SPI1)
- Touchscreen resistivo via SN74HC4066N
- Fonte 5 V para os servos

Os pinos efetivos estao em `config.h`, que e a fonte unica de verdade.

## Estrutura

```
StewartBallOnPlate/        (raiz = pasta do sketch Arduino)
├── StewartBallOnPlate.ino
├── config.h, types.h
├── kinematics.*, imu_sensor.*, touch_sensor.*, servo_driver.*
├── attitude_ctrl.*, ball_ctrl.*, supervisor.*
├── GUIA_DE_USO.md         como operar (comandos, telemetria, captura)
├── MANUAL_TECNICO.md      modulos, manutencao, extensao, inconsistencias
└── tools/
    ├── telemetry_logger.py   captura/comando/plot (roda no PC)
    ├── make_qr.py            gera QR do link do repositorio
    └── scripts/              sequencias temporizadas (.txt)
```

A raiz do repositorio e a propria pasta do sketch, entao o nome da pasta precisa
casar com o nome do `.ino` (`StewartBallOnPlate`) para o Arduino IDE abri-la.

## Compilar e gravar

1. Abra `StewartBallOnPlate.ino` no Arduino IDE com o nucleo RP2040 instalado.
2. Instale as bibliotecas: Adafruit PWM Servo Driver Library e SparkFun BNO08x
   Arduino Library.
3. Selecione a placa Raspberry Pi Pico e grave pela USB.

Passo a passo completo, comandos e telemetria: ver `GUIA_DE_USO.md`.

## Documentacao

Operacao (comandos seriais, modos, formato da telemetria, ferramenta de captura,
procedimentos reproduziveis):
- Portugues: `GUIA_DE_USO.md`
- English: `USER_GUIDE.md`

Referencia interna (descricao de cada modulo, manutencao, como estender,
divergencias conhecidas entre comentarios e codigo):
- Portugues: `MANUAL_TECNICO.md`
- English: `TECHNICAL_MANUAL.md`

## Licenca

Ver `LICENSE`. Confirme a politica de propriedade intelectual da instituicao
antes de publicar.
