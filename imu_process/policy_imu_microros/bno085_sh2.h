#pragma once

#include <Arduino.h>
#include <Wire.h>
// The SparkFun BNO08x library supplies CEVA's SH-2 C driver. We use its public
// callback API directly, rather than the wrapper which exposes only the last
// event in a packet. No library debug prints may share the micro-ROS serial port.
#include <sh2.h>
#include <sh2_SensorValue.h>
#include <sh2_err.h>
#include "body_imu_math.h"

namespace bno085 {
constexpr int SDA_PIN = 5, SCL_PIN = 4, INT_PIN = 6, RESET_PIN = 7;
constexpr uint32_t REPORT_INTERVAL_US = 10000;  // All reports request 100 Hz.
constexpr uint32_t MAX_AGE_US = 30000;
constexpr uint32_t MAX_SKEW_US = 20000;

// q_BS rotates sensor-frame vectors into the body-aligned published frame.
// Identity is a bench assumption, NOT a measured mounting calibration. Adjust
// this one quaternion after verifying the sensor's physical axis markings.
constexpr body_imu::Quaternion BODY_FROM_SENSOR = {1, 0, 0, 0};

struct Sample {
  body_imu::Quaternion orientation{};  // q_WS: sensor -> Z-up world.
  body_imu::Vector acceleration{}, gyro{};  // m/s^2 (with gravity), rad/s.
  uint32_t stamp[3]{};  // Host micros reconstructed by SH-2 for q, accel, gyro.
  uint32_t generation[3]{};
  bool valid[3]{};
};
static Sample sample;
static uint8_t address = 0;
static bool opened = false, reset_pending = false, reports_ready = false;
static uint32_t last_attempt_ms = 0, last_event_ms = 0;
static const char *stage = "waiting";
static int last_error = 0;
static uint32_t packets = 0, read_errors = 0, header_errors = 0;
static uint32_t peek_header = 0, data_header = 0;
static volatile uint32_t interrupt_us = 0;
static portMUX_TYPE interrupt_lock = portMUX_INITIALIZER_UNLOCKED;

static void ARDUINO_ISR_ATTR on_interrupt() {
  portENTER_CRITICAL_ISR(&interrupt_lock);
  interrupt_us = micros();
  portEXIT_CRITICAL_ISR(&interrupt_lock);
}
static uint32_t interrupt_stamp() {
  portENTER_CRITICAL(&interrupt_lock);
  const uint32_t stamp = interrupt_us;
  portEXIT_CRITICAL(&interrupt_lock);
  return stamp;
}

static int hal_open(sh2_Hal_t *) {
  digitalWrite(RESET_PIN, LOW);
  delay(10);
  // Keep the host clock running across sensor resets: SH-2 timestamps are in
  // host micros, not the sensor's boot clock. Capture the next INT falling edge.
  digitalWrite(RESET_PIN, HIGH);
  delay(300);
  return SH2_OK;
}
static void hal_close(sh2_Hal_t *) { digitalWrite(RESET_PIN, LOW); }
static uint32_t hal_time(sh2_Hal_t *) { return micros(); }

/** Read a complete SHTP transfer. BNO08x repeats its four-byte header when the
 * host re-reads the packet after inspecting the header. A 512-byte Wire buffer
 * allows one complete SH-2 transfer (maximum 384 bytes) without chunk assembly.
 * Never wait for INT inside a control callback; no data returns immediately.
 */
static int hal_read(sh2_Hal_t *, uint8_t *buffer, unsigned capacity, uint32_t *t_us) {
  if (digitalRead(INT_PIN) != LOW) return 0;
  const uint32_t edge = interrupt_stamp();
  uint8_t header[4];
  // Explicit stop argument selects core 2.x's size_t overload; its two-argument
  // uint8_t overload truncates boot advertisements larger than 255 bytes.
  if (Wire.requestFrom(address, size_t(4), true) != 4) { ++read_errors; return 0; }
  for (unsigned i = 0; i < 4; ++i) header[i] = Wire.read();
  peek_header = uint32_t(header[0]) | (uint32_t(header[1]) << 8) |
      (uint32_t(header[2]) << 16) | (uint32_t(header[3]) << 24);
  const unsigned size = (header[0] | (unsigned(header[1]) << 8)) & 0x7fff;
  if (size < 4 || size > capacity || size > SH2_HAL_MAX_TRANSFER_IN) { ++header_errors; return 0; }
  if (Wire.requestFrom(address, size_t(size), true) != size) { ++read_errors; return 0; }
  for (unsigned i = 0; i < size; ++i) buffer[i] = Wire.read();
  data_header = uint32_t(buffer[0]) | (uint32_t(buffer[1]) << 8) |
      (uint32_t(buffer[2]) << 16) | (uint32_t(buffer[3]) << 24);
  // The header-only I2C read itself advances SHTP transfer sequence/continuation
  // state. Live BNO085 returned 00020014 then 01028014: same 20-byte report,
  // new sequence and continuation flag. Do not require identical headers.
  // Pass the actual returned header to SHTP, which owns channel/sequence checks
  // and continuation assembly. If the actual transfer differs in size, return
  // only its bytes (or the received prefix) so SHTP can finish a continuation.
  const unsigned actual_size = (buffer[0] | (unsigned(buffer[1]) << 8)) & 0x7fff;
  if (actual_size < 4 || actual_size > capacity) { ++header_errors; return 0; }
  *t_us = edge;
  ++packets;
  return int(min(size, actual_size));
}
static int hal_write(sh2_Hal_t *, uint8_t *buffer, unsigned length) {
  if (length > SH2_HAL_MAX_TRANSFER_OUT) return 0;
  Wire.beginTransmission(address);
  const size_t written = Wire.write(buffer, length);
  const uint8_t error = Wire.endTransmission();
  return error == 0 && written == length ? int(length) : 0;
}
static sh2_Hal_t hal = {hal_open, hal_close, hal_read, hal_write, hal_time};

static void on_reset(void *, sh2_AsyncEvent_t *event) {
  if (event->eventId == SH2_RESET) {
    reset_pending = true;
    reports_ready = false;
    sample = Sample{};  // Never combine readings from opposite sides of a reset.
  }
}
static void on_sensor(void *, sh2_SensorEvent_t *event) {
  sh2_SensorValue_t value{};
  if (sh2_decodeSensorEvent(&value, event) != SH2_OK) return;
  unsigned index;
  switch (value.sensorId) {
    case SH2_GAME_ROTATION_VECTOR: {
      index = 0;
      const auto &q = value.un.gameRotationVector;
      sample.orientation = {q.real, q.i, q.j, q.k};
      sample.valid[index] = body_imu::normalize(sample.orientation);
      break;
    }
    case SH2_ACCELEROMETER:
      index = 1;
      sample.acceleration = {value.un.accelerometer.x, value.un.accelerometer.y,
                             value.un.accelerometer.z};
      sample.valid[index] = body_imu::finite(sample.acceleration);
      break;
    case SH2_GYROSCOPE_CALIBRATED:
      index = 2;
      sample.gyro = {value.un.gyroscope.x, value.un.gyroscope.y, value.un.gyroscope.z};
      sample.valid[index] = body_imu::finite(sample.gyro);
      break;
    default: return;
  }
  // SH-2 subtracts report delay from the host INT time. Keeping its low 32 bits
  // makes wrap-safe freshness checks independent of SH-2's rollover extension.
  sample.stamp[index] = uint32_t(value.timestamp);
  sample.generation[index]++;
  last_event_ms = millis();
}

static bool enable_reports() {
  sh2_SensorConfig_t config{};
  config.reportInterval_us = REPORT_INTERVAL_US;
  // Game RV supplies fused roll/pitch and relative yaw, without magnetic heading
  // corrections. Consumers here need tilt/gravity, not absolute compass yaw.
  for (sh2_SensorId_t id : {SH2_GAME_ROTATION_VECTOR, SH2_ACCELEROMETER,
                           SH2_GYROSCOPE_CALIBRATED}) {
    last_error = sh2_setSensorConfig(id, &config);
    if (last_error != SH2_OK) return false;
  }
  return true;
}
static void begin() {
  pinMode(INT_PIN, INPUT_PULLUP);
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  Wire.setBufferSize(512);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(10);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), on_interrupt, FALLING);
}
/** Service/recover the IMU independently of Agent discovery. Reinitialization
 * is rate limited; blocking SH-2 configuration may pause publication, but never
 * fabricates fresh samples. The host watchdog must see such pauses as outages.
 */
static void service() {
  if (!opened) {
    if (uint32_t(millis() - last_attempt_ms) < 2000) return;
    last_attempt_ms = millis();
    stage = "address_scan";
    for (uint8_t candidate : {uint8_t(0x4a), uint8_t(0x4b)}) {
      Wire.beginTransmission(candidate);
      if (Wire.endTransmission() != 0) continue;
      address = candidate;
      stage = "sh2_open";
      last_error = sh2_open(&hal, on_reset, nullptr);
      if (last_error != SH2_OK) {
        sh2_close(); digitalWrite(RESET_PIN, HIGH); delay(300); continue;
      }
      sh2_ProductIds_t ids{};
      stage = "product_id";
      last_error = sh2_getProdIds(&ids);
      if (last_error != SH2_OK || ids.numEntries == 0) {
        sh2_close(); digitalWrite(RESET_PIN, HIGH); delay(300); continue;
      }
      sh2_setSensorCallback(on_sensor, nullptr);
      opened = true;
      reset_pending = true;
      last_event_ms = millis();
      break;
    }
    return;
  }
  // Drain several packets, with a time budget, so normal mixed-report packets
  // are decoded without discarding all but the final sensor event.
  const uint32_t start = micros();
  for (unsigned i = 0; i < 8 && digitalRead(INT_PIN) == LOW; ++i) {
    sh2_service();
    if (uint32_t(micros() - start) > 3000) break;
  }
  if (reset_pending) {
    reset_pending = false;
    sample = Sample{};
    stage = "enable_reports";
    reports_ready = enable_reports();
    if (reports_ready) stage = "streaming";
    last_event_ms = millis();
  }
  if (!reports_ready || uint32_t(millis() - last_event_ms) > 1000) {
    sh2_close();
    opened = reports_ready = false;
    sample = Sample{};
    last_attempt_ms = millis();
    digitalWrite(RESET_PIN, HIGH);  // Allow address detection on the next attempt.
    delay(10);
  }
}
}  // namespace bno085
