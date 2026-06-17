/*============================================================
  touch_sensor.cpp — Resistive touchscreen reader.

  Ported from SBP_PSG_PID_control_V2_0.ino, restructured
  as a clean module. Key improvements:
    - Dedicated readAxis() helper (DRY)
    - Velocity estimation via finite difference
    - Configurable from config.h
============================================================*/

#include "touch_sensor.h"
#include "config.h"

namespace Touch {

// ─── Internal state ────────────────────────────────────────
static float last_x = 0.0f, last_y = 0.0f;
static uint32_t last_time_ms = 0;
static int reset_count = 0;

// ─── Trimmed-mean ADC reader for one axis ──────────────────
// Takes TS_SAMPLES readings, discards 2 smallest + 2 largest,
// checks remaining range against TS_MAX_RANGE_ADC.
// Returns the trimmed mean, or -1 on persistent noise.
static float readAxis(uint8_t analog_pin) {
  const int S = TS_SAMPLES;
  const int useful = S - 4;
  int buf[S]; // use stack, not heap

  for (int attempt = 0; attempt < TS_MAX_ITERATIONS; attempt++) {
    delayMicroseconds(TS_ACQ_DELAY_US);

    for (int i = 0; i < S; i++) {
      buf[i] = analogRead(analog_pin);
    }

    // Find 2 smallest indices
    int min1_idx = -1, min2_idx = -1;
    int min1_val = 1024, min2_val = 1024;
    for (int i = 0; i < S; i++) {
      if (buf[i] < min1_val) {
        min2_val = min1_val; min2_idx = min1_idx;
        min1_val = buf[i];   min1_idx = i;
      } else if (buf[i] < min2_val) {
        min2_val = buf[i]; min2_idx = i;
      }
    }

    // Find 2 largest indices (skipping min indices)
    int max1_idx = -1, max2_idx = -1;
    int max1_val = -1,  max2_val = -1;
    for (int i = 0; i < S; i++) {
      if (i == min1_idx || i == min2_idx) continue;
      if (buf[i] > max1_val) {
        max2_val = max1_val; max2_idx = max1_idx;
        max1_val = buf[i];   max1_idx = i;
      } else if (buf[i] > max2_val) {
        max2_val = buf[i]; max2_idx = i;
      }
    }

    // Compute trimmed statistics
    long sum = 0;
    int lo = 1024, hi = -1, count = 0;
    for (int i = 0; i < S; i++) {
      if (i == min1_idx || i == min2_idx || i == max1_idx || i == max2_idx) continue;
      sum += buf[i];
      if (buf[i] < lo) lo = buf[i];
      if (buf[i] > hi) hi = buf[i];
      count++;
    }

    if (count == useful && (hi - lo) <= TS_MAX_RANGE_ADC) {
      return (float)sum / (float)useful;
    }
    // else retry
  }

  // Fallback: return simple average after max retries
  long sum = 0;
  for (int i = 0; i < S; i++) sum += buf[i];
  return (float)sum / (float)S;
}

// ─── Public API ────────────────────────────────────────────

void init() {
  analogReadResolution(10);
  last_x = 0.0f;
  last_y = 0.0f;
  last_time_ms = millis();
  reset_count = 0;
}

void update(BallState& ball) {
  // ── Configure and read X axis ────────────────────────────
  pinMode(TS_A1, INPUT);
  pinMode(TS_D6, OUTPUT);
  pinMode(TS_D7, OUTPUT);
  digitalWrite(TS_D7, LOW);
  pinMode(TS_D2, OUTPUT);
  digitalWrite(TS_D2, HIGH);
  pinMode(TS_D3, INPUT);
  digitalWrite(TS_D3, LOW);
  pinMode(TS_D4, OUTPUT);
  digitalWrite(TS_D4, LOW);
  pinMode(TS_D5, INPUT);
  digitalWrite(TS_D5, LOW);
  digitalWrite(TS_D6, HIGH);

  float raw_xp = readAxis(TS_A1);

  // ── Configure and read Y axis ────────────────────────────
  pinMode(TS_A0, INPUT);
  digitalWrite(TS_D6, LOW);
  pinMode(TS_D2, INPUT);
  digitalWrite(TS_D2, LOW);
  pinMode(TS_D3, OUTPUT);
  digitalWrite(TS_D3, HIGH);
  pinMode(TS_D4, INPUT);
  digitalWrite(TS_D4, LOW);
  pinMode(TS_D5, OUTPUT);
  digitalWrite(TS_D5, LOW);
  digitalWrite(TS_D7, HIGH);

  float raw_yp = readAxis(TS_A0);
  
  //DEBUGGIN - REMOVE THE TWO LINES BELOW LATER
  //Serial.print("RAW "); Serial.print(raw_xp, 0);
  //Serial.print(" "); Serial.println(raw_yp, 0);


  // ── Deadzone check (no-touch detection) ──────────────────
  if (raw_xp < TS_X_DEADZONE || raw_yp < TS_Y_DEADZONE) {
    // Likely no ball on plate
    if (reset_count < TS_THRESHOLD_RESET) {
      reset_count++;
      // Keep previous valid position, mark as not detected
      ball.detected = false;
      return;
    } else {
      // Too many consecutive no-touch: zero out
      ball.x = 0;
      ball.y = 0;
      ball.vx = 0;
      ball.vy = 0;
      ball.detected = false;
      ball.timestamp_ms = millis();
      last_x = 0;
      last_y = 0;
      reset_count = 0;
      return;
    }
  }

  // ── Affine calibration: ADC → mm ─────────────────────────
  float mx = raw_xp * TS_CAL_AX + raw_yp * TS_CAL_BX + TS_CAL_DX;
  float my = raw_xp * TS_CAL_AY + raw_yp * TS_CAL_BY + TS_CAL_DY;

  // ── Range check ──────────────────────────────────────────
  if (mx <= 0 || my <= 0 || mx > PLATE_X_MAX || my > PLATE_Y_MAX) {
    ball.detected = false;
    return;
  }

  // ── Threshold check (outlier rejection) ──────────────────
  bool first_reading = (last_x == 0 && last_y == 0);
  bool within_threshold = (fabsf(mx - last_x) <= TS_THRESHOLD_MM) &&
                          (fabsf(my - last_y) <= TS_THRESHOLD_MM);

  if (!first_reading && !within_threshold) {
    if (reset_count < TS_THRESHOLD_RESET) {
      reset_count++;
      ball.detected = false;
      return;
    }
    // If we've been rejecting too long, accept the new position
    reset_count = 0;
  } else {
    reset_count = 0;
  }

  // ── Velocity estimation (finite difference) ──────────────
  uint32_t now_ms = millis();
  float dt_s = (float)(now_ms - last_time_ms) * 0.001f;

  if (dt_s > 0.001f && last_time_ms > 0 && !first_reading) {
    ball.vx = (mx - last_x) / dt_s;
    ball.vy = (my - last_y) / dt_s;
  } else {
    ball.vx = 0;
    ball.vy = 0;
  }

  // ── Store result ─────────────────────────────────────────
  ball.x = mx;
  //ball.x = raw_xp;
  ball.y = my;
  //ball.y = raw_yp;
  ball.detected = true;
  ball.timestamp_ms = now_ms;

  last_x = mx;
  //last_x = raw_xp;
  last_y = my;
  //last_y = raw_yp;
  last_time_ms = now_ms;
}

} // namespace Touch
