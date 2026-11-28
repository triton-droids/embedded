// Motor registry parsing and validation (port of motor_configuration.py).
#pragma once

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "motor_control_hybrid/robstride_codec.hpp"

namespace robstride {

// One firmware_* registry parameter written at ENABLE (motor_configuration.FIRMWARE_PARAMETERS).
struct FirmwareWrite {
  uint16_t index{0};
  bool is_float{true};
  double value{0.0};
};

struct JointConfig {
  std::string name;
  std::string can_interface;
  int motor_id{0};
  std::string model;
  int master_id{255};
  MotorSettings settings;
  std::vector<FirmwareWrite> firmware_writes;
};



struct Registry {
  std::string default_interface{"can1"};
  int default_master{255};
  double max_vel_rad_s{4.5};
  std::vector<JointConfig> joints;  // registry (YAML) order
};

namespace detail {
// cfg.get(key, default) followed by float(): a present null is an error.
inline double get_double(const YAML::Node& node, const std::string& key, double fallback) {
  const YAML::Node v = node[key];
  if (!v) {
    return fallback;
  }
  if (v.IsNull() || !v.IsScalar()) {
    throw std::invalid_argument(key + " must be a number");
  }
  return v.as<double>();
}

inline int64_t get_strict_int64(const YAML::Node& v, const std::string& what) {
  if (!v || !v.IsScalar() || v.Tag() == "tag:yaml.org,2002:bool") {
    throw std::invalid_argument(what);
  }
  const std::string s = v.Scalar();
  try {
    size_t used = 0;
    const long long value = std::stoll(s, &used, 0);
    if (used != s.size()) throw std::invalid_argument(what);
    return value;
  } catch (const std::logic_error&) {
    throw std::invalid_argument(what);
  }
}

inline int get_strict_int(const YAML::Node& v, const std::string& what) {
  // Python rejects bool (isinstance(x, bool)) and non-int values.
  if (!v || !v.IsScalar() || v.Tag() == "tag:yaml.org,2002:bool") {
    throw std::invalid_argument(what);
  }
  const std::string s = v.Scalar();
  if (s == "true" || s == "false" || s == "True" || s == "False") {
    throw std::invalid_argument(what);
  }
  try {
    size_t used = 0;
    const long value = std::stol(s, &used, 0);
    if (used != s.size()) {
      throw std::invalid_argument(what);
    }
    return static_cast<int>(value);
  } catch (const std::logic_error&) {
    throw std::invalid_argument(what);
  }
}

inline AnkleMapper parse_ankle(const YAML::Node& cfg) {
  const YAML::Node links = cfg["link_lengths"];
  if (!links || !links.IsSequence() || links.size() != 4) {
    throw std::invalid_argument("ankle_mapping.link_lengths must be [L1, L2, L3, L4]");
  }
  std::array<double, 4> l{};
  for (size_t i = 0; i < 4; ++i) {
    l[i] = links[i].as<double>();
  }
  for (const char* key : {"theta2_offset_deg", "t4_to_motor_offset_deg", "ankle_to_theta2_sign"}) {
    if (!cfg[key]) {
      throw std::invalid_argument(std::string("ankle_mapping.") + key + " is required");
    }
  }
  return AnkleMapper(l, cfg["theta2_offset_deg"].as<double>(),
                     cfg["t4_to_motor_offset_deg"].as<double>(),
                     cfg["ankle_to_theta2_sign"].as<double>());
}
}  // namespace detail

// motor_configuration.firmware_writes: the motor-side CAN watchdog, canTimeout
// (0x7028, 20000 = 1 s per the RobStride manual), from firmware_can_timeout_s or
// the registry's FIRMWARE_CAN_TIMEOUT_S; null leaves the motor's value.
constexpr uint16_t kCanTimeoutIndex = 0x7028;
constexpr double kCanTimeoutTicksPerS = 20000.0;
constexpr double kCanTimeoutMaxS = 5.0;

inline std::vector<FirmwareWrite> parse_firmware_writes(const YAML::Node& cfg,
                                                        const YAML::Node& defaults) {
  const YAML::Node own = cfg["firmware_can_timeout_s"];
  const YAML::Node v = own ? own : defaults["FIRMWARE_CAN_TIMEOUT_S"];  // never assign a Node
  if (!v || v.IsNull()) return {};
  if (!v.IsScalar() || v.Tag() == "tag:yaml.org,2002:bool" || v.Scalar() == "true" ||
      v.Scalar() == "false" || v.Tag() == "!") {
    throw std::invalid_argument("firmware_can_timeout_s must be a number of seconds");
  }
  double seconds = 0.0;
  try {
    size_t used = 0;
    seconds = std::stod(v.Scalar(), &used);
    if (used != v.Scalar().size()) throw std::invalid_argument("");
  } catch (const std::logic_error&) {
    throw std::invalid_argument("firmware_can_timeout_s must be a number of seconds");
  }
  if (!(std::isfinite(seconds) && seconds > 0 && seconds <= kCanTimeoutMaxS)) {
    throw std::invalid_argument("firmware_can_timeout_s must be in (0, 5]");
  }
  return {FirmwareWrite{kCanTimeoutIndex, false, std::nearbyint(seconds * kCanTimeoutTicksPerS)}};
}

// MotorSettings.from_config; `defaults` is the registry's top-level parameters.
inline MotorSettings settings_from_config(const YAML::Node& cfg, const YAML::Node& defaults) {
  MotorSettings s;
  const YAML::Node direction = cfg["direction"];
  s.direction = direction ? detail::get_strict_int(direction, "direction must be +1 or -1") : 1;
  if (s.direction != 1 && s.direction != -1) {
    throw std::invalid_argument("direction must be +1 or -1");
  }
  const double inf = std::numeric_limits<double>::infinity();
  s.offset = detail::get_double(cfg, "encoder_offset_rad", 0.0);
  s.lower = detail::get_double(cfg, "min_position", -inf);
  s.upper = detail::get_double(cfg, "max_position", inf);
  s.kp = detail::get_double(cfg, "kp", detail::get_double(defaults, "KP", 10.0));
  s.kd = detail::get_double(cfg, "kd", detail::get_double(defaults, "KD", 0.2));
  s.max_velocity =
      detail::get_double(cfg, "max_vel_rad_s", detail::get_double(defaults, "MAX_VEL_RAD_S", 4.5));
  if (cfg["max_torque_nm"] && !cfg["max_torque_nm"].IsNull()) {
    s.max_torque = cfg["max_torque_nm"].as<double>();
  }
  const std::string model = cfg["model"] ? cfg["model"].as<std::string>() : "";
  const auto limits = model_limits(model);
  s.max_kp = detail::get_double(cfg, "max_kp", detail::get_double(defaults, "MAX_KP", s.kp));
  s.max_kd = detail::get_double(cfg, "max_kd", detail::get_double(defaults, "MAX_KD", s.kd));
  s.max_position_error = detail::get_double(
      cfg, "max_position_error_rad", detail::get_double(defaults, "MAX_POSITION_ERROR_RAD", inf));
  s.max_motor_speed =
      detail::get_double(cfg, "max_motor_speed_rad_s", limits ? limits->second : inf);
  for (const char* key : {"hard_min_position", "hard_max_position"}) {
    const YAML::Node v = cfg[key];
    if (!v || v.IsNull()) continue;
    const double hard = v.as<double>();
    const bool lower = std::string(key) == "hard_min_position";
    if (lower ? s.lower < hard : s.upper > hard) {
      throw std::invalid_argument("min/max_position leave the mechanical range (" +
                                  std::string(key) + ")");
    }
  }
  if (limits && s.max_torque && *s.max_torque > limits->first) {
    throw std::invalid_argument("max_torque_nm exceeds the " + model + " peak torque");
  }
  const YAML::Node ankle = cfg["ankle_mapping"];
  if (ankle && !ankle.IsNull() && !(ankle.IsMap() && ankle.size() == 0)) {
    s.ankle = detail::parse_ankle(ankle);
  }
  s.validate();
  return s;
}

// validate_registry + the node's per-joint identity, before any socket opens.
inline Registry parse_registry(const YAML::Node& root) {
  const YAML::Node params = root["motor_control_node"]
                                ? root["motor_control_node"]["ros__parameters"]
                                : root;
  // Same rule as python_can_node: only an explicit false is refused.
  if (params["hardware_verified"] && !params["hardware_verified"].IsNull() &&
      !params["hardware_verified"].as<bool>()) {
    throw std::invalid_argument(
        "Motor registry is not hardware verified: verify CAN IDs, models, directions and "
        "offsets before enabling hardware_verified");
  }
  Registry reg;
  if (params["default_can_interface"]) {
    reg.default_interface = params["default_can_interface"].as<std::string>();
  }
  if (params["default_master_id"]) {
    reg.default_master = detail::get_strict_int(params["default_master_id"], "default_master_id");
  }
  reg.max_vel_rad_s = detail::get_double(params, "MAX_VEL_RAD_S", 4.5);

  const YAML::Node motors = params["motors"];
  if (!motors || !motors.IsMap() || motors.size() == 0) {
    throw std::invalid_argument("Motor registry is empty");
  }
  std::vector<std::string> order;
  for (const auto& entry : motors) {
    order.push_back(entry.first.as<std::string>());
  }
  const YAML::Node contract = params["model_contract"];
  if (contract && contract["joint_order"]) {
    if (contract["joint_order"].as<std::vector<std::string>>() != order) {
      throw std::invalid_argument("Motor registry order does not match the tracking contract");
    }
  }
  std::set<std::pair<std::string, int>> seen;
  std::map<std::string, int> masters;
  for (const auto& entry : motors) {
    const std::string name = entry.first.as<std::string>();
    const YAML::Node cfg = entry.second;
    JointConfig j;
    j.name = name;
    const std::string bad = name + ": valid motor_id, can_interface and model are required";
    j.motor_id = detail::get_strict_int(cfg["motor_id"], bad);
    j.can_interface =
        cfg["can_interface"] ? cfg["can_interface"].as<std::string>() : reg.default_interface;
    j.model = cfg["model"] && !cfg["model"].IsNull() ? cfg["model"].as<std::string>() : "";
    if (j.motor_id < 1 || j.motor_id >= 255 || j.can_interface.empty() || j.model.empty()) {
      throw std::invalid_argument(bad);
    }
    if (!seen.insert({j.can_interface, j.motor_id}).second) {
      throw std::invalid_argument("Duplicate CAN motor mapping: (" + j.can_interface + ", " +
                                  std::to_string(j.motor_id) + ")");
    }
    j.master_id = cfg["master_id"]
                      ? detail::get_strict_int(cfg["master_id"], name + ": invalid master_id")
                      : reg.default_master;
    if (j.master_id < 1 || j.master_id > 255 || j.master_id == j.motor_id) {
      throw std::invalid_argument(name + ": invalid master_id");
    }
    const auto m = masters.find(j.can_interface);
    if (m != masters.end() && m->second != j.master_id) {
      throw std::invalid_argument(j.can_interface + ": all motors must share a master_id");
    }
    masters[j.can_interface] = j.master_id;
    // The C++ node refuses a model without an audited profile at load time
    // (python_can_node fails later, when the bus is created).
    if (!profile_for(j.model)) {
      throw std::invalid_argument(name + ": CAN model lacks an audited protocol profile");
    }
    try {
      j.settings = settings_from_config(cfg, params);
      j.firmware_writes = parse_firmware_writes(cfg, params);
    } catch (const std::exception& exc) {
      throw std::invalid_argument(name + ": invalid motor configuration: " + exc.what());
    }
    reg.joints.push_back(std::move(j));
  }
  return reg;
}

}  // namespace robstride
