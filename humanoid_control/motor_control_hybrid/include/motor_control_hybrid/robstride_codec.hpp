// RobStride private CAN protocol codec and joint<->motor conversion, without ROS.
//
// C++ port of the audited Python path used on the robot:
//   motor_control_hybrid/can_transport.py   PROTOCOL_PROFILES, MIT encoding,
//                                           enable/disable, feedback decoding
//   motor_control_hybrid/python_can_node.py run-mode / parameter write frames
//   motor_control_hybrid/motor_configuration.py  MotorSettings, registry rules
//   motor_control_hybrid/ankle_mapping.py   four-bar ankle (Newton branch; the
//                                           deployed rosenv has no SciPy)
// Arithmetic follows the Python expressions term by term so encoded frames are
// bit-identical; test/test_robstride_codec.cpp checks this against vectors
// generated from the Python modules. Protocol audit: docs/can_protocol_audit.md.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace robstride {

// Communication types (bits 28..24 of the 29-bit identifier).
constexpr uint8_t kOperationControl = 1;
constexpr uint8_t kOperationStatus = 2;
constexpr uint8_t kEnable = 3;
constexpr uint8_t kDisable = 4;
constexpr uint8_t kReadParameter = 17;
constexpr uint8_t kWriteParameter = 18;
constexpr uint8_t kFaultReport = 21;

// Parameter indices used by the node.
constexpr uint16_t kParamMode = 0x7005;              // int8 run_mode
constexpr uint16_t kParamVelocityTarget = 0x700A;    // float spd_ref
constexpr uint16_t kParamCurrentLimit = 0x7018;      // float limit_cur
constexpr uint16_t kParamVelAccelerationTarget = 0x7022;  // float acc_rad

constexpr double kPi = 3.141592653589793;  // Python math.pi

// (position magnitude, velocity magnitude, torque magnitude, kp max, kd max)
struct Profile {
  double pmax;
  double vmax;
  double tmax;
  double kpmax;
  double kdmax;
};

// Only audited profiles; any other model is refused (can_transport.py).
inline std::optional<Profile> profile_for(const std::string& model) {
  static const std::map<std::string, Profile> profiles = {
      {"rs-02", {4 * kPi, 44.0, 17.0, 500.0, 5.0}},     // 2026-07-13
      {"rs-03", {4 * kPi, 20.0, 60.0, 5000.0, 100.0}},  // 2026-10-13
      {"rs-04", {4 * kPi, 15.0, 120.0, 5000.0, 100.0}},  // 2026-07-13
  };
  const auto it = profiles.find(model);
  if (it == profiles.end()) {
    return std::nullopt;
  }
  return it->second;
}

struct Frame {
  uint32_t id{0};  // 29-bit extended identifier, without CAN_EFF_FLAG
  std::array<uint8_t, 8> data{};
};

inline uint32_t arbitration_id(int type, int extra, int device) {
  if (type < 0 || type > 31 || extra < 0 || extra > 65535 || device < 1 || device > 254) {
    throw std::invalid_argument("Invalid RobStride frame");
  }
  return (static_cast<uint32_t>(type) << 24) | (static_cast<uint32_t>(extra) << 8) |
         static_cast<uint32_t>(device);
}

// int((clamp(value) - low) * 65535.0 / (high - low)), evaluated left to right.
inline uint16_t encode_uint16(double value, double low, double high) {
  value = std::max(low, std::min(high, value));
  return static_cast<uint16_t>(static_cast<int64_t>((value - low) * 65535.0 / (high - low)));
}

inline void put_be16(std::array<uint8_t, 8>& data, size_t offset, uint16_t v) {
  data[offset] = static_cast<uint8_t>(v >> 8);
  data[offset + 1] = static_cast<uint8_t>(v & 0xFF);
}

// MIT operation frame: big-endian position, velocity, kp, kd; torque in the ID.
inline Frame operation_frame(const Profile& p, int device, double position, double kp, double kd,
                             double velocity, double torque) {
  for (const double v : {position, velocity, torque, kp, kd}) {
    if (!std::isfinite(v)) {
      throw std::invalid_argument("MIT values must be finite");
    }
  }
  Frame f;
  put_be16(f.data, 0, encode_uint16(position, -p.pmax, p.pmax));
  put_be16(f.data, 2, encode_uint16(velocity, -p.vmax, p.vmax));
  put_be16(f.data, 4, encode_uint16(kp, 0, p.kpmax));
  put_be16(f.data, 6, encode_uint16(kd, 0, p.kdmax));
  f.id = arbitration_id(kOperationControl, encode_uint16(torque, -p.tmax, p.tmax), device);
  return f;
}

inline Frame enable_frame(int host, int device) {
  Frame f;
  f.id = arbitration_id(kEnable, host, device);
  return f;
}

inline Frame disable_frame(int host, int device) {
  Frame f;
  f.id = arbitration_id(kDisable, host, device);
  return f;
}

// struct.pack('<HH', param_id, 0) + value bytes (little endian).
inline Frame parameter_frame(int host, int device, uint16_t param, const std::array<uint8_t, 4>& v) {
  Frame f;
  f.id = arbitration_id(kWriteParameter, host, device);
  f.data[0] = static_cast<uint8_t>(param & 0xFF);
  f.data[1] = static_cast<uint8_t>(param >> 8);
  for (size_t i = 0; i < 4; ++i) {
    f.data[4 + i] = v[i];
  }
  return f;
}

inline Frame write_int8(int host, int device, uint16_t param, int value) {
  return parameter_frame(host, device, param,
                         {static_cast<uint8_t>(static_cast<int8_t>(value)), 0, 0, 0});
}

// Type 17 read request: index in bytes 0-1, the rest zero.
inline Frame read_parameter(int host, int device, uint16_t param) {
  Frame f;
  f.id = arbitration_id(kReadParameter, host, device);
  f.data[0] = static_cast<uint8_t>(param & 0xFF);
  f.data[1] = static_cast<uint8_t>(param >> 8);
  return f;
}

inline Frame write_uint32(int host, int device, uint16_t param, uint32_t value) {
  return parameter_frame(host, device, param,
                         {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8),
                          static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 24)});
}

inline Frame write_float(int host, int device, uint16_t param, double value) {
  const float f = static_cast<float>(value);  // struct.pack('<f', ...)
  uint32_t bits;
  static_assert(sizeof(bits) == sizeof(f));
  std::memcpy(&bits, &f, sizeof(bits));
  return parameter_frame(host, device, param,
                         {static_cast<uint8_t>(bits), static_cast<uint8_t>(bits >> 8),
                          static_cast<uint8_t>(bits >> 16), static_cast<uint8_t>(bits >> 24)});
}

struct Feedback {
  int motor_id{0};
  double position{0.0};  // raw motor rad
  double velocity{0.0};  // rad/s
  double torque{0.0};    // Nm
  double temperature{0.0};
  int mode{0};
  int fault_bits{0};
  // Decoded::kParameter only: type-17 reply index and data bytes 4..7 (little endian).
  uint16_t param_index{0};
  uint32_t param_value{0};
};

enum class Decoded { kIgnored, kStatus, kFault, kErrorFrame, kParameter };

// Mirrors NonblockingRobstrideBus.decode_feedback. The caller supplies the
// profile of the reporting motor (or nullopt when the device is unknown).
template <typename ProfileLookup>
Decoded decode(uint32_t id, bool extended, bool remote, bool error, uint8_t dlc,
               const uint8_t* data, int host_id, ProfileLookup&& lookup, Feedback& out) {
  if (error) {
    return Decoded::kErrorFrame;
  }
  if (!extended || remote || id > 0x1FFFFFFF || dlc != 8) {
    return Decoded::kIgnored;
  }
  const int kind = static_cast<int>((id >> 24) & 31);
  const int extra = static_cast<int>((id >> 8) & 65535);
  if (static_cast<int>(id & 255) != host_id) {
    return Decoded::kIgnored;
  }
  const int device = extra & 255;
  const std::optional<Profile> p = lookup(device);
  if (!p) {
    return Decoded::kIgnored;  // unknown device (or a model without a profile)
  }
  out.motor_id = device;
  if (kind == kFaultReport) {
    return Decoded::kFault;
  }
  if (kind == kReadParameter) {  // NonblockingRobstrideBus.decode_parameter_reply
    out.param_index = static_cast<uint16_t>(data[0] | (data[1] << 8));
    out.param_value = 0;
    for (int i = 3; i >= 0; --i) {
      out.param_value = (out.param_value << 8) | data[4 + i];
    }
    return Decoded::kParameter;
  }
  if (kind != kOperationStatus) {
    return Decoded::kIgnored;
  }
  // A firmware-query reply also uses type 2; never scale it into joint angles.
  if (data[0] == 0x00 && data[1] == 0xC4 && data[2] == 0x56) {
    return Decoded::kIgnored;
  }
  const auto be16 = [&](size_t i) {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[i]) << 8) | data[i + 1]);
  };
  out.position = (be16(0) / 65535.0 * 2 - 1) * p->pmax;
  out.velocity = (be16(2) / 65535.0 * 2 - 1) * p->vmax;
  out.torque = (be16(4) / 65535.0 * 2 - 1) * p->tmax;
  out.temperature = be16(6) * 0.1;
  out.mode = (extra >> 14) & 3;
  out.fault_bits = (extra >> 8) & 63;
  return Decoded::kStatus;
}

// ------------------------------------------------------------ ankle linkage
// Python float % (result takes the divisor's sign).
inline double py_mod(double a, double b) {
  double m = std::fmod(a, b);
  if (m != 0.0) {
    if ((b < 0) != (m < 0)) {
      m += b;
    }
  } else {
    m = std::copysign(0.0, b);
  }
  return m;
}

inline double wrap_to_pi(double x) { return py_mod(x + kPi, 2.0 * kPi) - kPi; }
inline double unwrap_to_near(double x, double ref) { return ref + wrap_to_pi(x - ref); }
inline double py_radians(double deg) { return deg * (kPi / 180.0); }
inline double py_degrees(double rad) { return rad * (180.0 / kPi); }

struct AnkleMapper {
  double k1{0}, k2{0}, k3{0};
  double theta2_offset_rad{0}, t4_to_motor_offset_rad{0}, ankle_to_theta2_sign{1};

  AnkleMapper() = default;
  AnkleMapper(const std::array<double, 4>& links, double theta2_offset_deg,
              double t4_to_motor_offset_deg, double sign) {
    const double l1 = links[0], l2 = links[1], l3 = links[2], l4 = links[3];
    k1 = l1 / l4;
    k2 = l1 / l2;
    k3 = (std::pow(l2, 2.0) - std::pow(l3, 2.0) + std::pow(l4, 2.0) + std::pow(l1, 2.0)) /
         (2.0 * l2 * l4);
    theta2_offset_rad = py_radians(theta2_offset_deg);
    t4_to_motor_offset_rad = py_radians(t4_to_motor_offset_deg);
    ankle_to_theta2_sign = sign;
  }

  double solve_foot_to_motor_theta2(double target_foot_deg, double t2) const {
    const double theta4 = py_radians(target_foot_deg);
    for (int i = 0; i < 50; ++i) {
      const double ft = k1 * std::cos(theta4) - k2 * std::cos(t2) - std::cos(t2 - theta4) + k3;
      const double dft = k2 * std::sin(t2) + std::sin(t2 - theta4);
      if (std::abs(dft) < 1e-12) break;
      const double step = ft / dft;
      t2 -= step;
      if (std::abs(step) < 1e-12) break;
    }
    return t2;
  }

  double solve_motor_to_foot_theta4(double t2, double t4) const {
    for (int i = 0; i < 50; ++i) {
      const double ft = k1 * std::cos(t4) - k2 * std::cos(t2) - std::cos(t2 - t4) + k3;
      const double dft = -k1 * std::sin(t4) - std::sin(t2 - t4);
      if (std::abs(dft) < 1e-12) break;
      const double step = ft / dft;
      t4 -= step;
      if (std::abs(step) < 1e-12) break;
    }
    return t4;
  }

  double ankle_to_motor(double ankle_rad, double motor_guess) const {
    const double theta2_model = theta2_offset_rad + ankle_to_theta2_sign * ankle_rad;
    const double t4_guess = motor_guess - t4_to_motor_offset_rad;
    const double t4 = unwrap_to_near(solve_motor_to_foot_theta4(theta2_model, t4_guess), t4_guess);
    return unwrap_to_near(t4 + t4_to_motor_offset_rad, motor_guess);
  }

  double motor_to_ankle(double motor_rad, double ankle_guess) const {
    const double theta4_model = motor_rad - t4_to_motor_offset_rad;
    const double theta2_guess = theta2_offset_rad + ankle_to_theta2_sign * ankle_guess;
    const double raw = solve_foot_to_motor_theta2(py_degrees(theta4_model), theta2_guess);
    const double theta2 = unwrap_to_near(raw, theta2_guess);
    return (theta2 - theta2_offset_rad) / ankle_to_theta2_sign;
  }
};

// Stateful guesses, as AnkleCalibration: continuity across calls.
struct AnkleState {
  double motor{0.0};
  double joint{0.0};
  std::optional<double> last_time;
};

// ------------------------------------------------------------ motor settings
struct MotorCommandValues {
  double position{0}, velocity{0}, acceleration{0}, torque{0}, kp{0}, kd{0};
};

// Peak torque (N.m) and output no-load speed (rad/s) at 48 V, RobStride product
// pages; same table as motor_configuration.MODEL_LIMITS.
inline std::optional<std::pair<double, double>> model_limits(const std::string& model) {
  constexpr double kRpm = 2.0 * 3.14159265358979323846 / 60.0;
  if (model == "rs-02") return std::make_pair(17.0, 410 * kRpm);
  if (model == "rs-03") return std::make_pair(60.0, 195 * kRpm);
  if (model == "rs-04") return std::make_pair(120.0, 200 * kRpm);
  return std::nullopt;
}
constexpr double kFeedbackJumpMarginRad = 0.05;
constexpr double kFeedbackJumpWindowS = 0.1;

struct MotorSettings {
  int direction{1};
  double offset{0.0};
  double lower{-std::numeric_limits<double>::infinity()};
  double upper{std::numeric_limits<double>::infinity()};
  double kp{10.0};
  double kd{0.2};
  double max_velocity{4.5};
  std::optional<double> max_torque;
  std::optional<AnkleMapper> ankle;
  // Gain ceilings (requests above are reduced), the largest |target - measured|
  // a position frame may ask for (motor rad), and the fastest physical motor
  // speed for feedback-jump detection. See motor_configuration.MotorSettings.
  double max_kp{std::numeric_limits<double>::infinity()};
  double max_kd{std::numeric_limits<double>::infinity()};
  double max_position_error{std::numeric_limits<double>::infinity()};
  double max_motor_speed{std::numeric_limits<double>::infinity()};

  // Throws when raw motor position moved faster than the motor can:
  // a coordinate change (external zero write) or an encoder fault.
  void check_feedback_step(double p0, double t0, double p1, double t1) const {
    const double dt = t1 - t0;
    if (dt < 0.0 || dt > kFeedbackJumpWindowS || !std::isfinite(max_motor_speed)) {
      return;
    }
    const double limit = max_motor_speed * 1.5 * dt + kFeedbackJumpMarginRad;
    if (std::abs(p1 - p0) > limit) {
      throw std::runtime_error("feedback position jumped " + std::to_string(p1 - p0) +
                               " rad in " + std::to_string(dt * 1000.0) +
                               " ms; coordinate change or encoder fault");
    }
  }

  void validate() const {
    const bool ok = (direction == 1 || direction == -1) && std::isfinite(offset) &&
                    !std::isnan(lower) && !std::isnan(upper) && lower < upper &&
                    std::isfinite(kp) && kp >= 0 && std::isfinite(kd) && kd >= 0 &&
                    std::isfinite(max_velocity) && max_velocity >= 0 &&
                    (!max_torque || (std::isfinite(*max_torque) && *max_torque > 0)) &&
                    max_kp > 0 && max_kd > 0 && max_position_error > 0 && max_motor_speed > 0;
    if (!ok) {
      throw std::invalid_argument("Invalid motor calibration, limits or gains");
    }
  }

  // Motor -> joint coordinates. `now` is host monotonic seconds (ankle velocity).
  void feedback(double& position, double& velocity, double& torque, AnkleState& state,
                double now) const {
    if (!std::isfinite(position) || !std::isfinite(velocity) || !std::isfinite(torque)) {
      throw std::runtime_error("Non-finite motor feedback");
    }
    position = direction * (position - offset);
    velocity = direction * velocity;
    torque = direction * torque;
    if (!ankle) {
      return;
    }
    const double joint = ankle->motor_to_ankle(position, state.joint);
    const double dt = state.last_time ? now - *state.last_time : 0.0;
    const double joint_velocity = dt > 1e-4 ? (joint - state.joint) / dt : 0.0;
    if (!std::isfinite(joint)) {
      throw std::runtime_error("Invalid ankle conversion");
    }
    state.motor = position;
    state.joint = joint;
    state.last_time = now;
    position = joint;
    velocity = joint_velocity;
  }

  // Joint target -> motor target (clamped, calibrated). For position modes
  // (`position_mode`), the target stays within max_position_error of
  // `measured_motor` (fresh raw motor rad); without it the frame is a probe.
  MotorCommandValues command(MotorCommandValues c, const AnkleState& state,
                             bool position_mode = false,
                             std::optional<double> measured_motor = std::nullopt) const {
    for (const double v : {c.position, c.velocity, c.acceleration, c.torque, c.kp, c.kd}) {
      if (!std::isfinite(v)) {
        throw std::invalid_argument("Non-finite command or negative gains");
      }
    }
    if (c.kp < 0 || c.kd < 0) {
      throw std::invalid_argument("Non-finite command or negative gains");
    }
    double position = std::min(std::max(c.position, lower), upper);
    const double velocity = std::min(std::max(c.velocity, -max_velocity), max_velocity);
    double torque = c.torque;
    if (max_torque) {
      torque = std::min(std::max(torque, -*max_torque), *max_torque);
    }
    if (ankle) {
      position = ankle->ankle_to_motor(position, state.motor);
      if (!std::isfinite(position)) {
        throw std::invalid_argument("Invalid ankle conversion");
      }
    }
    c.kp = std::min(c.kp, max_kp);
    c.kd = std::min(c.kd, max_kd);
    double motor_position = offset + direction * position;
    if (position_mode) {
      if (!measured_motor || !std::isfinite(*measured_motor)) {
        // Zero-gain, zero-torque probe: RobStride reports status only in reply to
        // a command, so refusing to send would never produce the measurement.
        c.position = offset + direction * position;
        c.velocity = c.torque = c.kp = c.kd = 0.0;
        return c;
      }
      // A coarse guard only; no torque budget (it made the simulated robot fall).
      const double error = max_position_error;
      motor_position = std::min(std::max(motor_position, *measured_motor - error),
                                *measured_motor + error);
    }
    c.position = motor_position;
    c.velocity = direction * velocity;
    c.torque = direction * torque;
    return c;
  }
};

}  // namespace robstride
