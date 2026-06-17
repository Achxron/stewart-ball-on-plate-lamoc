/*============================================================
  imu_sensor.cpp — BNO085 via SPI1 on RP2040.

  Reads two reports at BNO_REPORT_US period:
    SENSOR_REPORTID_GAME_ROTATION_VECTOR → att.roll, pitch, yaw
    SENSOR_REPORTID_GYROSCOPE_CALIBRATED → att.roll_rate, pitch_rate, yaw_rate

  Angular rates come directly from the BNO085 calibrated
  gyroscope (bias-compensated on-chip), avoiding numerical
  differentiation of the attitude estimate.

  CALL PATTERN
  ────────────
  getSensorEvent() runs sh2_service(), whose SPI HAL waits for the INT
  line before transferring. When the FIFO is empty INT is not asserted
  and that wait spins (delay(1) internally) until timeout — tens of ms —
  which stalls the main loop if called speculatively. We therefore poll
  the INT pin (active-low, hardwired to BNO_INT_PIN) and only call
  getSensorEvent() when a report is actually pending. This keeps the hot
  path free of the wait-for-INT cost. IMU_DRAIN_MAX caps the per-call
  burst when several reports are queued.

  RECOVERY (non-blocking state machine)
  ─────────────────────────────────────
  A violent disturbance can trip the BNO085's internal watchdog;
  it resets and re-advertises every ~500 ms, during which no
  valid reports arrive. The old code called reEnableReports()
  once with no retry and no return check, so reports could stay
  dead forever. Recovery is now staged:

    HEALTHY → (reset/stale) → SW_REENABLE → (retry up to
    IMU_SW_ATTEMPTS_MAX) → HW_RESET → (boot wait) → SW_REENABLE …

  No stage blocks: every transition is timed with millis() and
  returns control to the supervisor each tick, so open-loop
  modes keep running while the IMU recovers.
============================================================*/

#include "imu_sensor.h"
#include "config.h"
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>

namespace IMU {

static BNO08x bno;
static MbedSPI mySPI1(BNO_MISO_PIN, BNO_MOSI_PIN, BNO_SCK_PIN);
static float roll_rate_filt  = 0.0f;
static float pitch_rate_filt = 0.0f;
static uint32_t last_rv_ms = 0;

// ─── Recovery state machine ────────────────────────────────
enum class RecoveryState : uint8_t {
  HEALTHY,        // reports flowing
  SW_REENABLE,    // software re-enable issued, waiting for fresh data
  HW_RESET_PULSE, // hardware reset pulse asserted
  HW_RESET_BOOT,  // waiting for BNO085 boot after hardware reset
};

static RecoveryState rstate = RecoveryState::HEALTHY;
static uint32_t rstate_ms   = 0;   // timestamp of last state transition
static uint8_t  sw_attempts = 0;   // software re-enable attempts since last HW reset

// ─── Low-level helpers ─────────────────────────────────────

// Issue both report-enable requests. Returns true only if BOTH succeed.
bool reEnableReports() {
  bool rv = bno.enableGameRotationVector(BNO_REPORT_US);
  bool gy = bno.enableGyro(BNO_REPORT_US);
  return rv && gy;
}

// Assert the hardware reset line (non-blocking: just drives the pin LOW).
static void startHardwareReset() {
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, LOW);
  rstate   = RecoveryState::HW_RESET_PULSE;
  rstate_ms = millis();
}

bool init() {
  mySPI1.begin();

  // Init order matters: beginSPI sets up library internal state to match
  // the sensor; doing the hardware reset before beginSPI desynchronises
  // them and reports never arrive.
  Serial.println("IMU: beginSPI...");
  if (!bno.beginSPI(BNO_CS_PIN, BNO_INT_PIN, BNO_RST_PIN,
                     BNO_SPI_SPEED, mySPI1)) {
    return false;
  }
  Serial.println("IMU: beginSPI done");

  // Hardware reset after beginSPI. delay() is acceptable HERE (setup only).
  pinMode(BNO_RST_PIN, OUTPUT);
  digitalWrite(BNO_RST_PIN, LOW);
  delay(IMU_RST_PULSE_MS);
  digitalWrite(BNO_RST_PIN, HIGH);
  delay(IMU_BOOT_MS);   // BNO085 boot time

  // Enable rotation vector (with retries).
  bool rv_ok = false;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (bno.enableGameRotationVector(BNO_REPORT_US)) { rv_ok = true; break; }
    delay(50);
  }
  if (!rv_ok) return false;

  // Enable calibrated gyroscope (with retries).
  for (int attempt = 0; attempt < 5; attempt++) {
    if (bno.enableGyro(BNO_REPORT_US)) {
      rstate = RecoveryState::HEALTHY;
      rstate_ms = millis();
      sw_attempts = 0;
      return true;
    }
    delay(50);
  }

  return false;
}

bool isFresh() {
  return (last_rv_ms != 0) && (millis() - last_rv_ms < IMU_TIMEOUT_MS);
}

bool isRecovering() {
  return rstate != RecoveryState::HEALTHY;
}

// ─── Process exactly one already-fetched event into 'att' ──
// Returns true if the event updated attitude or rates.
static bool processEvent(Attitude& att) {
  const uint8_t id = bno.getSensorEventID();

  if (id == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    float new_roll  = bno.getRoll()  * RAD_TO_DEG + IMU_OFFSET_ROLL;
    float new_pitch = bno.getPitch() * RAD_TO_DEG + IMU_OFFSET_PITCH;

    // Z-axis misalignment correction (2D rotation). Currently below the
    // significance threshold; IMU_COUPLING_ANGLE governs whether it acts.
    const float ca = cosf(IMU_COUPLING_ANGLE);
    const float sa = sinf(IMU_COUPLING_ANGLE);
    const float r  = new_roll;
    const float p  = new_pitch;
    new_roll  = r * ca - p * sa;
    new_pitch = r * sa + p * ca;

    // Glitch rejection: drop single-frame jumps beyond any physical motion.
    if (last_rv_ms != 0 &&
        (fabsf(new_roll  - att.roll)  > ATT_JUMP_MAX ||
         fabsf(new_pitch - att.pitch) > ATT_JUMP_MAX)) {
      return false;   // drop frame; timestamp untouched → goes stale
    }

    att.roll  = new_roll;
    att.pitch = new_pitch;
    att.yaw   = bno.getYaw() * RAD_TO_DEG;
    att.valid = true;
    last_rv_ms = millis();
    return true;
  }
  else if (id == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
    const float gx = bno.getGyroX() * RAD_TO_DEG;
    const float gy = bno.getGyroY() * RAD_TO_DEG;

    if (fabsf(gx) > GYRO_MAX_VALID || fabsf(gy) > GYRO_MAX_VALID) {
      return false;   // corrupt gyro frame
    }

    // Same 2D coupling rotation as attitude, so the D term lives in the
    // same frame as the P/I terms.
    const float ca = cosf(IMU_COUPLING_ANGLE);
    const float sa = sinf(IMU_COUPLING_ANGLE);
    const float raw_roll_rate  = gx * ca - gy * sa;
    const float raw_pitch_rate = gx * sa + gy * ca;

    roll_rate_filt =
        GYRO_ALPHA * roll_rate_filt + (1.0f - GYRO_ALPHA) * raw_roll_rate;
    pitch_rate_filt =
        GYRO_ALPHA * pitch_rate_filt + (1.0f - GYRO_ALPHA) * raw_pitch_rate;

    att.roll_rate  = roll_rate_filt;
    att.pitch_rate = pitch_rate_filt;
    att.yaw_rate   = bno.getGyroZ() * RAD_TO_DEG;
    return true;
  }

  return false;
}

// ─── Recovery state machine step (non-blocking) ────────────
// Called once per update() when the sensor is not delivering fresh data.
static void serviceRecovery() {
  const uint32_t now = millis();

  switch (rstate) {

    case RecoveryState::HEALTHY:
      // Entered here because data went stale. Begin with a software
      // re-enable — cheapest recovery, handles the common case.
      sw_attempts = 0;
      if (reEnableReports()) {
        // Request accepted; wait for fresh data to confirm.
        rstate = RecoveryState::SW_REENABLE;
      } else {
        rstate = RecoveryState::SW_REENABLE;  // wait/retry regardless
      }
      rstate_ms = now;
      sw_attempts = 1;
      break;

    case RecoveryState::SW_REENABLE:
      // If fresh data has arrived, processEvent() already set rstate via
      // the caller. Here we only handle the no-data-yet timeout.
      if (now - rstate_ms >= IMU_REENABLE_WAIT_MS) {
        if (sw_attempts < IMU_SW_ATTEMPTS_MAX) {
          reEnableReports();
          sw_attempts++;
          rstate_ms = now;
        } else {
          // Software path exhausted → hardware reset (last resort).
          startHardwareReset();
        }
      }
      break;

    case RecoveryState::HW_RESET_PULSE:
      // Hold reset low for the pulse width, then release and wait for boot.
      if (now - rstate_ms >= IMU_RST_PULSE_MS) {
        digitalWrite(BNO_RST_PIN, HIGH);
        rstate = RecoveryState::HW_RESET_BOOT;
        rstate_ms = now;
      }
      break;

    case RecoveryState::HW_RESET_BOOT:
      // After boot time, re-request reports and return to the software
      // wait state. If this fails too, the cycle repeats from SW_REENABLE.
      if (now - rstate_ms >= IMU_BOOT_MS) {
        reEnableReports();
        sw_attempts = 0;
        rstate = RecoveryState::SW_REENABLE;
        rstate_ms = now;
      }
      break;
  }
}

bool update(Attitude& att) {
  bool got_data = false;

  // ── Drain pending events, gated by the INT pin ───────────
  // CRITICAL: getSensorEvent() runs sh2_service(), which calls the SPI HAL's
  // wait-for-INT routine. When the FIFO is EMPTY, INT is not asserted and that
  // wait spins (with delay(1) internally) until it times out — tens of ms.
  // Calling getSensorEvent() speculatively therefore stalls the main loop.
  //
  // The BNO085 pulls INT LOW (active-low) only when a report is ready. We poll
  // the pin directly and call getSensorEvent() ONLY when data is actually
  // pending, so we never pay the wait-for-INT cost. IMU_DRAIN_MAX caps the
  // burst in case INT stays asserted (e.g. multiple queued reports).
  for (uint8_t n = 0; n < IMU_DRAIN_MAX; n++) {
    if (digitalRead(BNO_INT_PIN) != LOW) break;   // no report pending → done
    if (!bno.getSensorEvent()) break;             // safety: nothing fetched

    if (bno.wasReset()) {
      // Sensor reset detected. Re-enable and enter recovery; do not trust
      // the current attitude until fresh RV confirms.
      att.valid = false;
      reEnableReports();
      rstate = RecoveryState::SW_REENABLE;
      rstate_ms = millis();
      sw_attempts = 1;
      // keep draining — a valid event may already be queued behind the reset
      continue;
    }

    if (processEvent(att)) got_data = true;
  }

  // ── Health / recovery ────────────────────────────────────
  if (isFresh()) {
    // Fresh RV within timeout → healthy. Clear any recovery in progress.
    if (rstate != RecoveryState::HEALTHY) {
      rstate = RecoveryState::HEALTHY;
      sw_attempts = 0;
    }
  } else {
    // No fresh RV. Advance the recovery state machine (non-blocking).
    if (rstate == RecoveryState::HEALTHY) {
      // Only escalate to recovery once data has been absent long enough
      // (IMU_RECOVERY_TRIGGER_MS ≫ IMU_TIMEOUT_MS), so a couple of missed
      // frames don't cause re-enable churn.
      if (last_rv_ms != 0 &&
          (millis() - last_rv_ms) >= IMU_RECOVERY_TRIGGER_MS) {
        serviceRecovery();   // transitions HEALTHY → SW_REENABLE
      }
    } else {
      serviceRecovery();
    }
  }

  return got_data;
}

} // namespace IMU