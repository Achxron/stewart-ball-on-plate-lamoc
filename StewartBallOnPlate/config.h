#ifndef CONFIG_H
#define CONFIG_H

/*============================================================
  config.h — Single source of truth for all hardware pins,
  geometry constants, and tuning parameters.

  RULE: No magic numbers anywhere else in the project.
        If you tune something, tune it here.
============================================================*/

#include <Arduino.h>

// ─── PCA9685 servo driver (I2C1) ───────────────────────────
constexpr uint8_t PCA_SDA_PIN   = 18;
constexpr uint8_t PCA_SCL_PIN   = 19;
constexpr uint8_t PCA_ADDR      = 0x40;
constexpr uint16_t SERVO_PWM_MIN = 100;   // ~1 ms pulse
constexpr uint16_t SERVO_PWM_MAX = 520;   // ~2 ms pulse
constexpr float SERVO_FREQ       = 50.0f; // Hz

// Per-servo mechanical trim (degrees) — zero these with horns level
constexpr float SERVO_TRIM[6] = { -10.0f, -8.0f, -12.0f, -3.0f, 10.5f, 8.0f };
constexpr float SERVO_SIGN[6] = { +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f };

// Safe angular envelope for servos (degrees)
constexpr float SERVO_MIN_DEG = 15.0f;
constexpr float SERVO_MAX_DEG = 165.0f;

// ─── BNO085 IMU (SPI1) ────────────────────────────────────
constexpr uint8_t BNO_SCK_PIN  = 14;
constexpr uint8_t BNO_MOSI_PIN = 15;
constexpr uint8_t BNO_MISO_PIN = 12;
constexpr uint8_t BNO_CS_PIN   = 13;
constexpr uint8_t BNO_INT_PIN  = 11;
constexpr uint8_t BNO_RST_PIN  = 10;
constexpr uint32_t BNO_SPI_SPEED   = 3000000;   // 3 MHz — SPI
constexpr uint32_t BNO_REPORT_US   = 10000;     // 100 Hz rotation vector

// IMU mounting bias (degrees) — measured at geometric zero
constexpr float IMU_OFFSET_ROLL  = -0.4468f;// -0.2968f;
constexpr float IMU_OFFSET_PITCH = 0.6079f;// 0.4079f;

// IMU Z-axis misalignment correction (radians).
// Currently NOT applied — below statistical significance threshold.
constexpr float IMU_COUPLING_ANGLE = 0.0f; // -0.03002f; // radians (= -1.72°) // 0.02374f; // radians (= 1.36°)

// IMU gyro low pass filter strength
constexpr float GYRO_ALPHA = 0.0f;

// ─── Touchscreen (resistive, via SN74HC4066N) ──────────────
constexpr uint8_t TS_D2 = 2;
constexpr uint8_t TS_D3 = 3;
constexpr uint8_t TS_D4 = 4;
constexpr uint8_t TS_D5 = 5;
constexpr uint8_t TS_D6 = 6;
constexpr uint8_t TS_D7 = 7;
constexpr uint8_t TS_A0 = 26;   // GP26 / ADC0
constexpr uint8_t TS_A1 = 27;   // GP27 / ADC1

// Touchscreen acquisition
constexpr int    TS_ACQ_DELAY_US   = 300;
constexpr int    TS_SAMPLES        = 8;      // must be >= 6
constexpr int    TS_MAX_ITERATIONS = 5;
constexpr float  TS_THRESHOLD_MM   = 50.0f;
constexpr int    TS_THRESHOLD_RESET = 5;
constexpr int    TS_MAX_RANGE_ADC  = 50;
constexpr int    TS_X_DEADZONE     = 15;
constexpr int    TS_Y_DEADZONE     = 16;

// Touchscreen ADC-to-mm calibration (affine transform)
constexpr float TS_CAL_AX = 0.335181395f; // 0.411206064f;
constexpr float TS_CAL_BX = -0.000348814f;// -0.000519147f;
constexpr float TS_CAL_DX = -20.13955602f; // -88.07095145f;
constexpr float TS_CAL_AY = 0.00350547f; // 0.002731717f;
constexpr float TS_CAL_BY = 0.297268325f;// 0.28390922f;
constexpr float TS_CAL_DY = -37.58133657f; //-28.9708877133884f;

// Plate physical extents (mm)
constexpr float PLATE_X_MAX = 310.0f;
constexpr float PLATE_Y_MAX = 236.0f;
constexpr float PLATE_CENTER_X = 155.0f;
constexpr float PLATE_CENTER_Y = 118.0f;


// ═══════════════════════════════════════════════════════════
//  Stewart platform geometry
//  Derived from technical drawings (PLAT.STEW.003-01, 004-01, 005-01)
//  and physical measurements (2026-04-15)
// ═══════════════════════════════════════════════════════════

// Arm and rod lengths (mm)
constexpr float L1 = 24.95f;   // servo horn length (drawing: shaft center to rod hole)
constexpr float L2 = 160.0f;   // connecting rod length

// Nominal platform height (mm) — computed from corrected geometry.
// With symmetric base+platform, all 6 legs give the same neutral height.
// At Z=158, all servo angles = 90.5° (RMS = 0.47°).
constexpr float Z_HOME = 149.33f;

// Base servo pivot positions (mm, base frame).
// Hexagonal base plate: 3 servo edges at 69.28mm from center (40mm each),
// 3 non-servo edges at 51.96mm from center.
// Servo shafts sit at the hexagon vertices where servo and non-servo edges meet.
// Ordering: top-right, right-upper, right-lower, left-lower, left-upper, top-left.
//OLD constexpr float BASE_PX[6] = {  -20.0f,  20.0f,   70.0f,   50.0f,  -50.0f,  -70.0f };
//OLD constexpr float BASE_PY[6] = {   69.28f, 69.28f, -17.32f, -51.96f, -51.96f, -17.32f };
constexpr float BASE_PX[6] = {  -40.000f, 40.000f, 70.040f,  30.040f, -30.040f, -70.040f };
constexpr float BASE_PY[6] = {   57.780f, 57.780f,  5.750f, -63.530f, -63.530f,   5.750f };


// Platform rod-joint positions (mm, platform body frame).
// Equilateral triangle, side=90mm, pointing UP (Star of David with base).
// Joints are 25mm from each vertex along the adjacent edges.
// Vertex assignment: (J0,J5)→V_top, (J1,J2)→V_bottom_right, (J3,J4)→V_bottom_left.
//OLD constexpr float PLAT_PX[6] = {  -12.5f,   12.5f,   32.5f,   20.0f,  -20.0f,  -32.5f  };
//OLD constexpr float PLAT_PY[6] = {   30.31f,  30.31f,  -4.33f, -25.98f, -25.98f,  -4.33f };
constexpr float PLAT_PX[6] = {  -7.500f,  7.500f,  53.775f,  46.275f, -46.275f, -53.775f  };
constexpr float PLAT_PY[6] = {  57.764f, 57.764f, -22.387f, -35.377f, -35.377f, -22.387f  };


// Servo horn direction vectors (unit, XY-plane).
// At θ=0 (servo at 90°), the horn points ALONG the adjacent non-servo
// hex edge, toward the nearest servo edge.
//
// Geometry:
//   horn_tip(θ) = base[i] + L1 · (dir[i]·cosθ + ẑ·sinθ)
//
// The direction vectors are exactly the unit vectors along the hex
// non-servo edges, from each servo's vertex toward the adjacent vertex.
//constexpr float DIR_X[6] = {     -0.5f,  0.5f,    -0.5f,    -1.0f,   1.0f,     0.5f    };
//constexpr float DIR_Y[6] = { -0.86603f, -0.86603f, 0.86603f,  0.0f,   0.0f,    0.86603f };
constexpr float DIR_X[6] = { -1.0f, 1.0f,   0.5f,   -0.5f,    0.5f,  -0.5f };
constexpr float DIR_Y[6] = {  0.0f, 0.0f, 0.866f, -0.866f, -0.866f, 0.866f };
// (0.86603 ≈ √3/2)


// ─── Inner attitude controller (PI/PID) ────────────────────
// Per-axis gains: the rectangular acrylic plate has different lengths
// along each axis, so pitch and roll have different inertias. Step-response
// analysis (2026-04-20) showed pitch overshoots 33% and settles 4x slower
// than roll at identical gains, and rings at 3.5Hz vs 1.5Hz.
//
// Starting point: match what was working for roll, then detune pitch.
// Roll uses the gains that were driving the original symmetric loop.

// Roll gains
constexpr float ATT_KP_ROLL    = 9.2f; // 9.20f; // 11.0f; // 11.0f; // 11.2f; // 9.2f; // 11.7f;// 11.30f; // 14.2f; // 14.5f; // 1.75f;
constexpr float ATT_KI_ROLL    = 0.00f; // 0.00f; // 0.0f; // 0.8f;
constexpr float ATT_KD_ROLL    = 0.48f; // 0.31f; // 0.42f; // 0.62f; // 0.44f; // 0.31f; // 0.5865; // 0.48f; // 0.95f; // 0.9f; // 0.055f;
constexpr float ATT_I_MAX_ROLL = 3.00f; // 3.0f;

// Pitch gains
constexpr float ATT_KP_PITCH    = 8.7f; // 8.70f; // 10.0f; // 10.2f; // 8.5f; // 10.5f; // 11.2f; // 11.8f; // 0.85f;
constexpr float ATT_KI_PITCH    = 0.00f; // 0.00f; // 0.0f; // 0.8f;
constexpr float ATT_KD_PITCH    = 0.38f; // 0.24f; // 0.57f; // 0.36f; // 0.19f; // 0.735f; // 0.73f; // 0.055f;
constexpr float ATT_I_MAX_PITCH = 3.0f; // 3.0f;

// Output clamp shared by both axes (Max Velocity in °/s)
constexpr float ATT_CMD_MAX = 600.0f;

// ─── IMU safety limits ─────────────────────────────────────
constexpr uint32_t IMU_TIMEOUT_MS = 50;     // RV considerado stall acima disso
constexpr float    GYRO_MAX_VALID = 800.0f; // deg/s
constexpr float    ATT_JUMP_MAX   = 30.0f;  // deg

// ─── IMU recovery state machine (non-blocking) ─────────────
// Drain cap: max events processed per update() call, bounds tick time
// even when the FIFO is backed up after a reset.
constexpr uint8_t  IMU_DRAIN_MAX        = 8;
// Frescor (IMU_TIMEOUT_MS) governs whether the CONTROL loop trusts the
// attitude. Recovery uses a SEPARATE, longer threshold so a couple of
// missed frames don't trigger re-enable churn: recovery only starts once
// data has been absent this long.
constexpr uint32_t IMU_RECOVERY_TRIGGER_MS = 150;
// Software re-enable: how long to wait for fresh data after a software
// re-enable before escalating to the next software attempt.
constexpr uint32_t IMU_REENABLE_WAIT_MS = 120;
// Number of software re-enable attempts before escalating to HW reset.
constexpr uint8_t  IMU_SW_ATTEMPTS_MAX  = 3;
// Hardware reset: BNO085 boot time before reports can be re-requested.
// Non-blocking — measured with millis(), not delay().
constexpr uint32_t IMU_BOOT_MS          = 550;
// Reset pulse width.
constexpr uint32_t IMU_RST_PULSE_MS     = 10;

// ─── Limite físico da pose integrada (workspace) ───────────
// Independente de ATT_CMD_MAX (limite de velocidade).
constexpr float POSE_TILT_MAX = 18.0f;      // deg — conservador

// ─── Religação segura após perda de IMU (relevel) ──────────
// Quando o IMU volta, a plataforma pode ter derivado e a bola rolado.
// Em vez de devolver o controle à bola de imediato (que satura o comando
// e gera uma varredura violenta capaz de re-derrubar o sensor), passamos
// por uma fase RELEVELING: a malha da bola fica suspensa e a pose é
// rampada suavemente até o nível, a uma taxa MUITO abaixo de ATT_CMD_MAX.
// Só após a atitude convergir ao nível (ângulo e taxa baixos, sustentados)
// o controle da bola é religado.
constexpr float RELEVEL_SLEW_DPS   = 25.0f;  // deg/s — taxa máx. da rampa de pose
constexpr float RELEVEL_ANGLE_TOL  = 1.5f;   // deg — |atitude| considerada nivelada
constexpr float RELEVEL_RATE_TOL   = 20.0f;  // deg/s — |taxa| considerada parada
constexpr uint32_t RELEVEL_HOLD_MS = 200;    // ms nivelado e parado antes de religar


// ─── Outer ball controller (PID) ───────────────────────────
//Closed Inner Loop gains
 constexpr float BALL_KP_CLOSE = 0.068f; // 0.04f; // 0.05f; // 0.075f;
 constexpr float BALL_KI_CLOSE = 0.00f; // 0.01f; // 0.000f;
 constexpr float BALL_KD_CLOSE = 0.031f; // 0.022f; // 0.02f;

 constexpr float BALL_KP_FAR   = 0.068f; // 0.063f; // 0.04f; // 0.05f; // 0.040f;
 constexpr float BALL_KI_FAR   = 0.00f; // 0.00f; // 0.01f; // 0.000f;
 constexpr float BALL_KD_FAR   = 0.031f; // 0.031f; // 0.022f; // 0.01f;

//Open Inner Loop gains
// constexpr float BALL_KP_CLOSE = 0.06f; // 0.04f; // 0.05f; // 0.075f;
// constexpr float BALL_KI_CLOSE = 0.00f; // 0.01f; // 0.000f;
// constexpr float BALL_KD_CLOSE = 0.015125f; // 0.022f; // 0.02f;

// constexpr float BALL_KP_FAR   = 0.06f; // 0.04f; // 0.05f; // 0.040f;
// constexpr float BALL_KI_FAR   = 0.00f; // 0.01f; // 0.000f;
// constexpr float BALL_KD_FAR   = 0.015125f; // 0.022f; // 0.01f;

constexpr float BALL_TRANSITION_DIST = 20.0f;  // mm — gain scheduling threshold
constexpr float BALL_I_MAX    = 10.0f;  // degrees, anti-windup
constexpr float BALL_OUT_MAX  = 18.0f;  // degrees, max tilt command

// ─── Controller deadbands ──────────────────────────────────
// Errors below these thresholds are treated as zero.
// ATT: degrees. BALL: mm.
constexpr float ATT_DEADBAND  = 0.0f;   // deg
constexpr float BALL_DEADBAND = 3.0f;   // mm

// ─── Timing ────────────────────────────────────────────────
constexpr uint32_t LOOP_PERIOD_US  = 10000;  // 100 Hz main loop target
constexpr uint32_t SERIAL_PERIOD_MS = 20;     // 50 Hz telemetry output

// Cascade rate separation: inner attitude loop runs every control tick
// (100 Hz); outer ball loop runs once every BALL_LOOP_DIV ticks.
// 100 Hz / 5 = 20 Hz. Maintains a 5x separation,
// so the inner loop appears as a static gain to the outer loop.
constexpr uint8_t BALL_LOOP_DIV = 3;          // 100 Hz / 5 = 20 Hz outer loop

#endif // CONFIG_H