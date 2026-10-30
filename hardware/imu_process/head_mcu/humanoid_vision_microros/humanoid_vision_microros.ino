/**
 * @file humanoid_vision_microros.ino
 * @brief ESP32-S3 head interface: raw MPU-family IMU and one PCA9685/DS3225 servo.
 *
 * Shared I2C: SDA GPIO4, SCL GPIO5, 100 kHz; PCA9685 address 0x40, servo channel
 * 8 (the ninth connector). Supply servo V+ separately and share signal ground.
 * USB serial carries binary micro-XRCE-DDS at 460800 baud: never print debug
 * text on Serial while the Agent owns it.
 *
 * /head/servo_target_rad accepts absolute joint-command radians; /head/imu emits
 * acceleration (m/s^2) and angular velocity (rad/s), with no orientation estimate.
 * The host computes visual control. This sketch maps angle to pulse and checks
 * command age; it has no external joint encoder or software position controller.
 */
#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string_functions.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>

#ifndef IMU_SDA
#define IMU_SDA 4
#endif
#ifndef IMU_SCL
#define IMU_SCL 5
#endif

// Publication rate, host command refresh, and PWM carrier are independent rates.
constexpr uint32_t TRANSPORT_BAUD = 460800;
constexpr uint32_t IMU_PERIOD_MS = 10;       // Publish at 100 Hz.
constexpr uint32_t SERVO_TIMEOUT_MS = 250;   // Fail-safe command watchdog.
constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr uint8_t SERVO_CHANNEL = 8;         // Ninth physical connector.
constexpr float SERVO_FREQUENCY_HZ = 50.0f;
constexpr uint16_t SERVO_MIN_US = 1000;
constexpr uint16_t SERVO_CENTER_US = 1500;
constexpr uint16_t SERVO_MAX_US = 2000;
// Initial joint calibration: angle_rad -> pulse_us. Measure the real linkage
// and then adjust angle limits, scale, center, and direction for the real joint.
constexpr float SERVO_MIN_RAD = -1.0f;
constexpr float SERVO_MAX_RAD = 1.0f;
constexpr float SERVO_US_PER_RAD = 500.0f;
constexpr float SERVO_DIRECTION = 1.0f;  // Change to -1.0 if motion is reversed.

// PCA9685 register map and bit masks. A channel has ON_L/H and OFF_L/H bytes.
constexpr uint8_t PCA_MODE1 = 0x00;
constexpr uint8_t PCA_MODE2 = 0x01;
constexpr uint8_t PCA_LED0_ON_L = 0x06;
constexpr uint8_t PCA_PRESCALE = 0xFE;
constexpr uint8_t PCA_MODE1_RESTART = 0x80;
constexpr uint8_t PCA_MODE1_SLEEP = 0x10;
constexpr uint8_t PCA_MODE1_AUTO_INCREMENT = 0x20;
constexpr uint8_t PCA_FULL_OFF = 0x10;

// Override the library's weak 115200-baud serial transport. The Agent must use
// the same 460800 baud rate.
/** @brief Open the UART transport at the baud rate configured on the host Agent. */
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

// Hardware state is initialized once; Agent reconnection rebuilds ROS entities.
uint8_t imu_address = 0;
bool imu_ready = false;
bool pca_ready = false;
bool servo_enabled = false;
uint32_t last_servo_command_ms = 0;

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t imu_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t servo_subscription;
rcl_timer_t imu_timer;
rclc_executor_t executor;

sensor_msgs__msg__Imu imu_message;
std_msgs__msg__Int32 status_message;
std_msgs__msg__Float32 servo_command;

// Track partial initialization so a failed connection can be cleaned up safely.
bool support_initialized = false;
bool node_initialized = false;
bool imu_publisher_initialized = false;
bool status_publisher_initialized = false;
bool servo_subscription_initialized = false;
bool timer_initialized = false;
bool executor_initialized = false;

/** @brief Connection lifecycle: probe, build entities, run, clean up, then retry. */
enum AgentState {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED,
};

AgentState agent_state = WAITING_AGENT;
uint32_t last_agent_ping_ms = 0;
uint16_t status_divider = 0;

/** @brief Write one register using a 7-bit Wire address; true means bus ACK only. */
bool i2c_write_register(uint8_t address, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

/** @brief Read one register with a repeated START; leave output unused on failure. */
bool i2c_read_register(uint8_t address, uint8_t reg, uint8_t *value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, uint8_t(1)) != 1) return false;
  *value = Wire.read();
  return true;
}

/** @brief Burst-read count consecutive IMU registers into caller-owned storage.
 * Returns false on failed register selection or a short Wire response.
 */
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
  return i2c_write_register(imu_address, reg, value);
}

/** @brief Decode one big-endian two's-complement sensor sample. */
int16_t signed16(const uint8_t *bytes) {
  return int16_t((uint16_t(bytes[0]) << 8U) | bytes[1]);
}

/** @brief Request channel-8 full-off and clear the software enabled flag.
 * This releases PWM; it is not a center command or an active brake. A false
 * return means the hardware write was not confirmed, so release is not guaranteed.
 * Other PCA channels are not modified by this firmware's shutdown operation.
 */
bool servo_off() {
  servo_enabled = false;
  if (!pca_ready) return false;

  const uint8_t base = PCA_LED0_ON_L + 4U * SERVO_CHANNEL;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(base);
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(PCA_FULL_OFF);
  return Wire.endTransmission() == 0;
}

/** @brief Convert a validated pulse width in microseconds into 12-bit PWM ticks.
 * Range checking belongs to servo_callback. Successful I2C transfer confirms
 * register acceptance, not physical joint movement or actual position.
 */
bool servo_set_pulse_us(uint16_t pulse_us) {
  if (!pca_ready) return false;

  const uint16_t ticks = uint16_t(lroundf(
      float(pulse_us) * SERVO_FREQUENCY_HZ * 4096.0f / 1000000.0f));
  const uint8_t base = PCA_LED0_ON_L + 4U * SERVO_CHANNEL;
  Wire.beginTransmission(PCA9685_ADDRESS);
  Wire.write(base);
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(0));
  Wire.write(uint8_t(ticks & 0xFFU));
  Wire.write(uint8_t((ticks >> 8U) & 0x0FU));
  return Wire.endTransmission() == 0;
}

/** @brief Configure the nominal 25 MHz PCA oscillator for a 50 Hz PWM carrier.
 * Prescale may only change in sleep. Clear SLEEP explicitly when waking: keeping
 * an old SLEEP bit produces I2C ACKs with no PWM output. Register writes are
 * checked for ACK; this routine does not read back the final configuration.
 */
bool pca_begin() {
  Wire.beginTransmission(PCA9685_ADDRESS);
  if (Wire.endTransmission() != 0) return false;

  uint8_t old_mode = 0;
  if (!i2c_read_register(PCA9685_ADDRESS, PCA_MODE1, &old_mode)) return false;

  const uint8_t sleep_mode =
      (old_mode & ~PCA_MODE1_RESTART) | PCA_MODE1_SLEEP;
  // Auto-increment is required for four-byte channel writes below.
  const uint8_t wake_mode =
      (old_mode & ~PCA_MODE1_SLEEP) | PCA_MODE1_AUTO_INCREMENT;
  const uint8_t prescale = uint8_t(
      lroundf(25000000.0f / (4096.0f * SERVO_FREQUENCY_HZ)) - 1.0f);

  if (!i2c_write_register(PCA9685_ADDRESS, PCA_MODE1, sleep_mode) ||
      !i2c_write_register(PCA9685_ADDRESS, PCA_PRESCALE, prescale) ||
      !i2c_write_register(PCA9685_ADDRESS, PCA_MODE2, 0x04) ||
      !i2c_write_register(PCA9685_ADDRESS, PCA_MODE1, wake_mode)) {
    return false;
  }
  // Allow oscillator startup before issuing RESTART.
  delay(1);
  if (!i2c_write_register(PCA9685_ADDRESS, PCA_MODE1,
                          wake_mode | PCA_MODE1_RESTART)) {
    return false;
  }
  return true;
}

/** @brief Detect a supported WHO_AM_I and set raw accel/gyro measurement ranges.
 * Registers configure +/-2 g, +/-250 deg/s, a low-pass filter, and a sample
 * divider. This is raw six-axis telemetry; no magnetometer/fusion is enabled.
 */
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

/** @brief Convert the Agent-synchronized epoch into ROS seconds/nanoseconds.
 * Check the status sync bit before treating this timestamp as trusted wall time.
 */
void fill_timestamp(builtin_interfaces__msg__Time *stamp) {
  const int64_t epoch_ns = rmw_uros_epoch_nanos();
  stamp->sec = int32_t(epoch_ns / 1000000000LL);
  stamp->nanosec = uint32_t(epoch_ns % 1000000000LL);
}

/** @brief Attempt best-effort publication of diagnostic state bits.
 * bit0: IMU initialized; bit1: PCA initialized; bit2: servo software-enabled;
 * bit3: epoch synchronized. Initialization bits do not prove ongoing bus health.
 * The publish result is currently ignored, so this is not a delivery guarantee.
 */
void publish_status() {
  status_message.data = (imu_ready ? 1 : 0) |
                        (pca_ready ? 2 : 0) |
                        (servo_enabled ? 4 : 0) |
                        (rmw_uros_epoch_synchronized() ? 8 : 0);
  (void)rcl_publish(&status_publisher, &status_message, nullptr);
}

/** @brief Read one sensor sample and attempt 100 Hz SI telemetry publication.
 * Bytes 0..5 are accel, 6..7 temperature (unused), and 8..13 gyro. Axes stay in
 * the sensor's native mounting frame: no body-frame transform or bias correction
 * is performed. Failed reads skip a sample; status is attempted once per second.
 */
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == nullptr) return;

  if (imu_ready) {
    uint8_t data[14];
    if (imu_read_registers(0x3B, data, sizeof(data))) {
      constexpr double gravity = 9.80665;
      constexpr double accel_scale = gravity / 16384.0;
      constexpr double gyro_scale = (PI / 180.0) / 131.0;

      fill_timestamp(&imu_message.header.stamp);
      imu_message.linear_acceleration.x = signed16(&data[0]) * accel_scale;
      imu_message.linear_acceleration.y = signed16(&data[2]) * accel_scale;
      imu_message.linear_acceleration.z = signed16(&data[4]) * accel_scale;
      imu_message.angular_velocity.x = signed16(&data[8]) * gyro_scale;
      imu_message.angular_velocity.y = signed16(&data[10]) * gyro_scale;
      imu_message.angular_velocity.z = signed16(&data[12]) * gyro_scale;
      (void)rcl_publish(&imu_publisher, &imu_message, nullptr);
    }
  }

  if (++status_divider >= (1000U / IMU_PERIOD_MS)) {
    status_divider = 0;
    publish_status();
  }
}

/** @brief Validate absolute target radians, map to PWM, and refresh the watchdog.
 * Invalid/out-of-range commands release the output instead of clamping silently.
 * Only a successful pulse write refreshes last_servo_command_ms. There is no
 * local trajectory generation; the host tracker limits command speed/acceleration.
 */
void servo_callback(const void *message) {
  const auto *command = static_cast<const std_msgs__msg__Float32 *>(message);
  const float target_rad = command->data;

  if (!isfinite(target_rad) || target_rad < SERVO_MIN_RAD ||
      target_rad > SERVO_MAX_RAD || !pca_ready) {
    servo_off();
    return;
  }

  const float pulse = float(SERVO_CENTER_US) +
                      SERVO_DIRECTION * SERVO_US_PER_RAD * target_rad;
  const uint16_t pulse_us = uint16_t(lroundf(pulse));
  if (pulse_us < SERVO_MIN_US || pulse_us > SERVO_MAX_US) {
    servo_off();
    return;
  }

  if (servo_set_pulse_us(pulse_us)) {
    servo_enabled = true;
    last_servo_command_ms = millis();
  } else {
    servo_off();
  }
}

/** @brief Restore zero-initialized handles before creating a new Agent session. */
void reset_ros_handles() {
  node = rcl_get_zero_initialized_node();
  imu_publisher = rcl_get_zero_initialized_publisher();
  status_publisher = rcl_get_zero_initialized_publisher();
  servo_subscription = rcl_get_zero_initialized_subscription();
  imu_timer = rcl_get_zero_initialized_timer();
  executor = rclc_executor_get_zero_initialized_executor();
}

/** @brief Build the node, two publishers, command subscription, timer, and executor.
 * @return True after setup; on false the caller must invoke destroy_entities()
 * to clean up the subset identified by initialization flags. Time-sync failure
 * does not reject the session; consumers can inspect the diagnostic sync bit.
 */
bool create_entities() {
  allocator = rcl_get_default_allocator();
  reset_ros_handles();

  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK)
    return false;
  support_initialized = true;

  if (rclc_node_init_default(&node, "head_mcu", "", &support) != RCL_RET_OK)
    return false;
  node_initialized = true;

  // Sensor/diagnostic telemetry uses best effort to avoid waiting on old samples.
  if (rclc_publisher_init_best_effort(
          &imu_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          "/head/imu") != RCL_RET_OK)
    return false;
  imu_publisher_initialized = true;

  if (rclc_publisher_init_best_effort(
          &status_publisher, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
          "/head/mcu_status") != RCL_RET_OK)
    return false;
  status_publisher_initialized = true;

  // Default subscription QoS is reliable; the host command publisher matches it.
  if (rclc_subscription_init_default(
          &servo_subscription, &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
          "/head/servo_target_rad") != RCL_RET_OK)
    return false;
  servo_subscription_initialized = true;

  if (rclc_timer_init_default(&imu_timer, &support,
                              RCL_MS_TO_NS(IMU_PERIOD_MS),
                              imu_timer_callback) != RCL_RET_OK)
    return false;
  timer_initialized = true;

  // Exactly two executor handles are needed: one timer and one subscription.
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) !=
      RCL_RET_OK)
    return false;
  executor_initialized = true;

  if (rclc_executor_add_timer(&executor, &imu_timer) != RCL_RET_OK)
    return false;
  if (rclc_executor_add_subscription(
          &executor, &servo_subscription, &servo_command, servo_callback,
          ON_NEW_DATA) != RCL_RET_OK)
    return false;

  (void)rmw_uros_sync_session(1000);
  publish_status();
  return true;
}

/** @brief Request servo release and tear down initialized ROS resources in reverse.
 * Session destruction is nonblocking when the Agent is gone. Resetting flags
 * permits the next state-machine cycle to establish a fresh connection.
 */
void destroy_entities() {
  servo_off();

  if (support_initialized) {
    rmw_context_t *context = rcl_context_get_rmw_context(&support.context);
    (void)rmw_uros_set_context_entity_destroy_session_timeout(context, 0);
  }
  if (executor_initialized) rclc_executor_fini(&executor);
  if (timer_initialized) rcl_timer_fini(&imu_timer);
  if (servo_subscription_initialized)
    rcl_subscription_fini(&servo_subscription, &node);
  if (status_publisher_initialized)
    rcl_publisher_fini(&status_publisher, &node);
  if (imu_publisher_initialized) rcl_publisher_fini(&imu_publisher, &node);
  if (node_initialized) rcl_node_fini(&node);
  if (support_initialized) rclc_support_fini(&support);

  support_initialized = false;
  node_initialized = false;
  imu_publisher_initialized = false;
  status_publisher_initialized = false;
  servo_subscription_initialized = false;
  timer_initialized = false;
  executor_initialized = false;
}

/** @brief Initialize I2C/devices/messages, release channel 8, and wait for the Agent.
 * The IMU quaternion is a placeholder identity; covariance[0]=-1 declares that
 * orientation is unavailable. No automatic boot sweep or center move is issued.
 */
void setup() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(20);

  pca_ready = pca_begin();
  servo_off();
  imu_ready = imu_begin();

  sensor_msgs__msg__Imu__init(&imu_message);
  std_msgs__msg__Int32__init(&status_message);
  std_msgs__msg__Float32__init(&servo_command);
  rosidl_runtime_c__String__assign(&imu_message.header.frame_id,
                                   "head_imu_link");
  imu_message.orientation.w = 1.0;
  imu_message.orientation_covariance[0] = -1.0;  // No orientation estimate.

  set_microros_transports();
  reset_ros_handles();
  agent_state = WAITING_AGENT;
}

/** @brief Service command-age checks and the reconnecting Agent state machine.
 * Unsigned subtraction tolerates millis() wraparound. The watchdog is checked
 * once per loop, so blocking I2C/transport/ping work can delay its response; it
 * is not a separate hard real-time safety task.
 */
void loop() {
  if (servo_enabled &&
      uint32_t(millis() - last_servo_command_ms) > SERVO_TIMEOUT_MS) {
    servo_off();
  }

  const uint32_t now = millis();
  switch (agent_state) {
    case WAITING_AGENT:
      // Retry discovery at 500 ms intervals without moving the joint.
      if (uint32_t(now - last_agent_ping_ms) >= 500U) {
        last_agent_ping_ms = now;
        agent_state =
            (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) ? AGENT_AVAILABLE
                                                        : WAITING_AGENT;
      }
      break;

    case AGENT_AVAILABLE:
      // All ROS entities belong to this Agent session and must be rebuilt on loss.
      if (create_entities()) {
        agent_state = AGENT_CONNECTED;
      } else {
        destroy_entities();
        agent_state = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      // Bounded executor spinning services both IMU and incoming angle commands.
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      if (uint32_t(now - last_agent_ping_ms) >= 500U) {
        last_agent_ping_ms = now;
        if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK)
          agent_state = AGENT_DISCONNECTED;
      }
      break;

    case AGENT_DISCONNECTED:
      destroy_entities();
      agent_state = WAITING_AGENT;
      break;
  }

  delay(1);
}
