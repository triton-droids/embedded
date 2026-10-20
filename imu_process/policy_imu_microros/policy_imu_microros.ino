/**
 * Body-mounted MPU-family raw IMU -> micro-ROS, ESP32-S3, GPIO4/5 I2C.
 * No motor/servo outputs. Binary UART 460800; never print ASCII on Serial.
 * Sensor axes are unchanged. Configure the host's imu_to_body_rotation after
 * measuring the mount. No orientation fusion or physical sensor-edge timestamp.
 */
#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>

#ifndef IMU_SDA
#define IMU_SDA 4
#endif
#ifndef IMU_SCL
#define IMU_SCL 5
#endif
constexpr uint32_t TRANSPORT_BAUD = 460800;
constexpr uint32_t SYNC_PERIOD_MS = 5000;
constexpr uint32_t SYNC_TTL_MS = 15000;
uint8_t imu_address = 0;
bool imu_ready = false;
extern "C" bool arduino_transport_open(struct uxrCustomTransport *transport) {
  (void)transport;
  Serial.begin(TRANSPORT_BAUD);
  return true;
}

/** @brief Close UART when the transport lifecycle requests it. */
extern "C" bool arduino_transport_close(struct uxrCustomTransport *transport) {
  (void)transport;
  Serial.end();
  return true;
}

/** @brief Send binary protocol bytes and report the number accepted by Serial.
 * The library owns framing; application-level ASCII diagnostics would corrupt it.
 */
extern "C" size_t arduino_transport_write(
    struct uxrCustomTransport *transport, const uint8_t *buffer, size_t length,
    uint8_t *error) {
  (void)transport;
  (void)error;
  return Serial.write(buffer, length);
}

/** @brief Read up to length protocol bytes, using the requested millisecond timeout.
 * @return Actual bytes received; a short read is handled by the transport layer.
 */
extern "C" size_t arduino_transport_read(
    struct uxrCustomTransport *transport, uint8_t *buffer, size_t length,
    int timeout_ms, uint8_t *error) {
  (void)transport;
  (void)error;
  Serial.setTimeout(timeout_ms);
  return Serial.readBytes(reinterpret_cast<char *>(buffer), length);
}

bool imu_read_registers(uint8_t reg, uint8_t *buffer, size_t count) {
  Wire.beginTransmission(imu_address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(imu_address, uint8_t(count)) != count) return false;
  for (size_t i = 0; i < count; ++i) buffer[i] = Wire.read();
  return true;
}

/** @brief Write a register at the detected IMU address. */
bool imu_write_register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(imu_address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

/** @brief Decode one big-endian two's-complement sensor sample. */
int16_t signed16(const uint8_t *bytes) {
  return int16_t((uint16_t(bytes[0]) << 8U) | bytes[1]);
}

bool imu_begin() {
  for (uint8_t address : {uint8_t(0x68), uint8_t(0x69)}) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      imu_address = address;
      break;
    }
  }
  if (imu_address == 0) return false;

  uint8_t who = 0;
  if (!imu_read_registers(0x75, &who, 1)) return false;
  if (who != 0x68 && who != 0x70 && who != 0x71 && who != 0x73) return false;

  // PWR_MGMT_1: reset first; later select the gyro PLL clock and wake the sensor.
  if (!imu_write_register(0x6B, 0x80)) return false;
  delay(100);
  // PWR_MGMT_2 enables axes; GYRO_CONFIG and ACCEL_CONFIG select full scales;
  // CONFIG selects the digital low-pass filter; SMPLRT_DIV divides sample rate.
  bool ok = imu_write_register(0x6B, 0x01) &&
            imu_write_register(0x6C, 0x00) &&
            imu_write_register(0x1B, 0x00) &&
            imu_write_register(0x1C, 0x00) &&
            imu_write_register(0x1A, 0x03) &&
            imu_write_register(0x19, 0x04);
  // Newer supported variants also expose ACCEL_CONFIG2 for accelerometer filtering.
  if (who != 0x68) ok = ok && imu_write_register(0x1D, 0x03);
  delay(100);
  return ok;
}


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
rcl_timer_t timer;
rclc_executor_t executor;
sensor_msgs__msg__Imu message;
bool support_ok = false, node_ok = false, publisher_ok = false;
bool timer_ok = false, executor_ok = false, clock_ok = false;
bool connected = false;
uint32_t last_sync_attempt = 0, last_sync_success = 0, last_ping = 0;

/** Sync outside the sensor callback, with a bounded wait; failed sync revokes
 * publication immediately. Reconnection always establishes a new time offset.
 */
void synchronize_clock(int timeout_ms) {
  last_sync_attempt = millis();
  clock_ok = rmw_uros_sync_session(timeout_ms) == RMW_RET_OK &&
             rmw_uros_epoch_synchronized();
  if (clock_ok) last_sync_success = millis();
}

/** Timestamp at the START of the register read, never after serial transport.
 * This bounds host transport age but does not measure the IMU's internal filter
 * delay or align physical CAN/IMU sample edges. Failed/expired sync skips samples.
 */
void sample_callback(rcl_timer_t *source, int64_t last_call) {
  (void)last_call;
  if (!source || !imu_ready || !clock_ok ||
      uint32_t(millis() - last_sync_success) > SYNC_TTL_MS) return;
  const int64_t capture_ns = rmw_uros_epoch_nanos();
  if (capture_ns <= 0) return;
  uint8_t data[14];
  if (!imu_read_registers(0x3B, data, sizeof(data))) return;
  message.header.stamp.sec = int32_t(capture_ns / 1000000000LL);
  message.header.stamp.nanosec = uint32_t(capture_ns % 1000000000LL);
  constexpr double accel_scale = 9.80665 / 16384.0;
  constexpr double gyro_scale = (PI / 180.0) / 131.0;
  message.linear_acceleration.x = signed16(data) * accel_scale;
  message.linear_acceleration.y = signed16(data + 2) * accel_scale;
  message.linear_acceleration.z = signed16(data + 4) * accel_scale;
  message.angular_velocity.x = signed16(data + 8) * gyro_scale;
  message.angular_velocity.y = signed16(data + 10) * gyro_scale;
  message.angular_velocity.z = signed16(data + 12) * gyro_scale;
  (void)rcl_publish(&publisher, &message, nullptr);
}

/** Clean partially constructed entities on every failed connection attempt. */
void disconnect_agent() {
  clock_ok = connected = false;
  if (support_ok) rmw_uros_set_context_entity_destroy_session_timeout(
      &support.context, 0);
  if (executor_ok) (void)rclc_executor_fini(&executor);
  if (timer_ok) (void)rcl_timer_fini(&timer);
  if (publisher_ok) (void)rcl_publisher_fini(&publisher, &node);
  if (node_ok) (void)rcl_node_fini(&node);
  if (support_ok) (void)rclc_support_fini(&support);
  executor_ok = timer_ok = publisher_ok = node_ok = support_ok = false;
}

bool connect_agent() {
  allocator = rcl_get_default_allocator();
  support = {}; node = rcl_get_zero_initialized_node();
  publisher = rcl_get_zero_initialized_publisher();
  timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) return false;
  support_ok = true;
  if (rclc_node_init_default(&node, "body_imu_mcu", "", &support) != RCL_RET_OK) return false;
  node_ok = true;
  if (rclc_publisher_init_best_effort(&publisher, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "/imu/data_raw") != RCL_RET_OK) return false;
  publisher_ok = true;
  if (rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(5), sample_callback) != RCL_RET_OK) return false;
  timer_ok = true;
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;
  executor_ok = true;
  if (rclc_executor_add_timer(&executor, &timer) != RCL_RET_OK) return false;
  synchronize_clock(1000);  // Startup only; no control or servo loop runs here.
  last_ping = millis();
  return true;
}

void setup() {
  set_microros_transports();
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(10);
  imu_ready = imu_begin();
  sensor_msgs__msg__Imu__init(&message);
  rosidl_runtime_c__String__assign(&message.header.frame_id, "imu_link");
  message.orientation_covariance[0] = -1.0;  // Raw six-axis data only.
}

void loop() {
  const uint32_t now = millis();
  if (!connected) {
    if (uint32_t(now - last_ping) >= 500) {
      last_ping = now;
      if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
        connected = connect_agent();
        if (!connected) disconnect_agent();
      }
    }
  } else {
    (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
    if (uint32_t(now - last_ping) >= 500) {
      last_ping = now;
      if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) disconnect_agent();
    }
    if (connected && uint32_t(now - last_sync_attempt) >= SYNC_PERIOD_MS)
      synchronize_clock(10);  // May skip IMU periods; host never re-stamps samples.
  }
  delay(1);
}
