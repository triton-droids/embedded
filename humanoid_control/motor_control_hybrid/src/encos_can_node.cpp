// ROS 2 ENCOS adapter: same topic contract as python_can_node, built on
// encos_driver.
//
// SKELETON — build-checked against encos_driver headers only. Not run against
// real motors. See the "UNVERIFIED" notes before connecting hardware.
//
// Contract kept from python_can_node:
//   sub  motor_commands  (MotorCommand)  per-joint mode/pos/vel/torque/kp/kd
//   sub  /safety/estop   (Bool, reliable + transient_local) latched stop
//   pub  joint_states    (JointState)    registry order, fresh samples only
//   pub  motor_status    (String JSON)   {"<index>": {temperature, torque}}
//   pub  motor_telemetry (String JSON)   {schema_version: 1, motors: [...]}
//   pub  can_timing      (String JSON)   1 Hz TX/RX counters
//   Watchdogs: source-age + valid_until deadline per joint, explicit ENABLE to
//   recover after a timeout, latched transport fault, estop rejects commands.
//
// Where ENCOS differs from RobStride (and so this node differs):
//   - There is no enable/disable command. ENABLE is a software arm: the joint
//     accepts motion once a fresh error-free status arrives. DISABLE and every
//     unarmed/idle joint get Stop() = firmware damping brake, NOT torque-off.
//   - Each control command carries its own mode (PVT / speed), so there are no
//     run-mode parameter writes before switching modes.
//   - Feedback only arrives in reply to a command, so idle joints are polled by
//     re-sending Stop() each TX period (feedback_poll_when_idle).
//   - Feedback reports current (A); effort = current * kt from the model table.
//   - MODE_VELOCITY ignores acceleration (SpdControl has no ramp parameter).
//   - Linkage (ankle_mapping) joints are rejected until the four-bar solver is
//     ported from ankle_mapping.py; driving them without it would be wrong.

#include <encos/encos_driver.h>
#include <pthread.h>
#include <sched.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <map>
#include <motor_control_interfaces/msg/motor_command.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {
using MotorCommand = motor_control_interfaces::msg::MotorCommand;
using Clock = std::chrono::steady_clock;
using TimePoint = Clock::time_point;

constexpr double kInf = std::numeric_limits<double>::infinity();

double seconds_between(TimePoint a, TimePoint b) {
  return std::chrono::duration<double>(b - a).count();
}

TimePoint after(TimePoint t, double seconds) {
  return t + std::chrono::duration_cast<Clock::duration>(
                 std::chrono::duration<double>(seconds));
}

// Exact encos::MotorModel names from encos_driver motor_models.csv. Keep this
// list to models we own so a typo cannot select another motor's PVT ranges.
std::optional<encos::MotorModel> parse_model(const std::string& name) {
  static const std::map<std::string, encos::MotorModel> models = {
      {"EC_A2806_P2", encos::MotorModel::EC_A2806_P2},
      {"EC_A4310_P2", encos::MotorModel::EC_A4310_P2},
      {"EC_A6408_P2", encos::MotorModel::EC_A6408_P2},
      {"EC_A6416_P2", encos::MotorModel::EC_A6416_P2},
  };
  const auto it = models.find(name);
  if (it == models.end()) {
    return std::nullopt;
  }
  return it->second;
}

// JSON has no NaN/Inf (python_can_node uses allow_nan=False); emit null.
std::string json_number(double v) {
  if (!std::isfinite(v)) {
    return "null";
  }
  std::ostringstream os;
  os << v;
  return os.str();
}

std::string json_escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    if (static_cast<unsigned char>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

// Port of motor_configuration.MotorSettings: calibration, limits and gains.
// Joint coordinates are what ROS sees; motor coordinates go on the wire.
struct MotorSettings {
  int direction{1};
  double offset{0.0};
  double lower{-kInf};
  double upper{kInf};
  double kp{10.0};
  double kd{0.2};
  double max_velocity{4.5};
  std::optional<double> max_torque;

  // Motor (rad, rad/s, Nm) -> joint coordinates.
  void feedback(double& position, double& velocity, double& torque) const {
    if (!std::isfinite(position) || !std::isfinite(velocity) ||
        !std::isfinite(torque)) {
      throw std::runtime_error("Non-finite motor feedback");
    }
    position = direction * (position - offset);
    velocity = direction * velocity;
    torque = direction * torque;
  }
};

struct Command {
  std::string joint;
  uint8_t mode{MotorCommand::MODE_VELOCITY};
  TimePoint received_at{};
  TimePoint deadline{};
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double torque{0.0};
  double kp{0.0};
  double kd{0.0};
};

// Port of can_transport.LatestCommandMailbox: at most one lifecycle request
// (ENABLE/DISABLE) and one latest motion target per joint. DISABLE discards
// pending motion; motion after a pending DISABLE is ignored until ENABLE.
class LatestCommandMailbox {
 public:
  void put(const std::vector<Command>& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& command : commands) {
      auto& slot = find_or_add(command.joint);
      if (command.mode == MotorCommand::MODE_ENABLE ||
          command.mode == MotorCommand::MODE_DISABLE) {
        slot.lifecycle = command;
        slot.motion.reset();
      } else if (!slot.lifecycle ||
                 slot.lifecycle->mode != MotorCommand::MODE_DISABLE) {
        slot.motion = command;
      }
    }
  }

  std::optional<Command> get() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slots_.empty()) {
      return std::nullopt;
    }
    auto& slot = slots_.front().second;
    std::optional<Command> out;
    if (slot.lifecycle) {
      out = std::move(slot.lifecycle);
      slot.lifecycle.reset();
    } else {
      out = std::move(slot.motion);
      slot.motion.reset();
    }
    if (!slot.lifecycle && !slot.motion) {
      slots_.erase(slots_.begin());
    }
    return out;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_.clear();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto& entry : slots_) {
      n += entry.second.lifecycle.has_value() + entry.second.motion.has_value();
    }
    return n;
  }

 private:
  struct Slot {
    std::optional<Command> lifecycle;
    std::optional<Command> motion;
  };

  Slot& find_or_add(const std::string& joint) {
    for (auto& entry : slots_) {
      if (entry.first == joint) {
        return entry.second;
      }
    }
    slots_.emplace_back(joint, Slot{});
    return slots_.back().second;
  }

  std::mutex mutex_;
  std::vector<std::pair<std::string, Slot>>
      slots_;  // insertion order, <= 2 per joint
};

struct Sample {
  double position{0.0};
  double velocity{0.0};
  double torque{0.0};
  double temperature{0.0};
  TimePoint received_at{};
  double source_stamp{0.0};
};

struct Joint {
  std::string name;
  size_t index{0};
  std::string can_interface;
  int motor_id{0};
  std::string model;
  MotorSettings settings;
  encos::Motor* motor{nullptr};  // non-owning; owned by encos_driver manager
  encos::Bus* bus{nullptr};      // non-owning
  float kt{0.0F};
};

}  // namespace

class EncosCanNode : public rclcpp::Node {
 public:
  EncosCanNode() : Node("encos_can_node") {
    declare_parameter<std::string>("motor_config_file", "");
    declare_parameter<std::string>("adapter_type", "Can");
    declare_parameter<bool>("can_fd", false);
    declare_parameter<bool>("can_rt_enabled", false);
    // encos_driver's own comm thread runs SCHED_FIFO 50; stay below it.
    declare_parameter<int>("can_tx_rt_priority", 45);
    declare_parameter<double>("publish_rate_hz", 50.0);
    declare_parameter<bool>("feedback_poll_when_idle", true);
    declare_parameter<double>("feedback_timeout_s", 0.1);
    declare_parameter<double>("command_timeout_s", 0.5);
    // UNVERIFIED: braking current cap passed to Stop(); bench-check behaviour.
    declare_parameter<double>("stop_current_a", 5.0);
    // UNVERIFIED: current cap for MODE_VELOCITY (RobStride path used 5 A).
    declare_parameter<double>("velocity_current_limit_a", 5.0);
    // Motor-side CAN watchdog in ms; 0 leaves the firmware setting untouched.
    // UNVERIFIED: what the motor does on expiry must be measured before relying
    // on it.
    declare_parameter<int>("motor_can_timeout_ms", 0);

    can_rt_enabled_ = get_parameter("can_rt_enabled").as_bool();
    can_tx_rt_priority_ =
        static_cast<int>(get_parameter("can_tx_rt_priority").as_int());
    if (can_tx_rt_priority_ < 1 || can_tx_rt_priority_ > 99) {
      throw std::invalid_argument(
          "can_tx_rt_priority must be between 1 and 99");
    }
    command_timeout_s_ = get_parameter("command_timeout_s").as_double();
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0.0) {
      throw std::invalid_argument(
          "command_timeout_s must be finite and positive");
    }
    feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
    if (!(feedback_timeout_s_ > 0.0 && feedback_timeout_s_ <= 1.0)) {
      throw std::invalid_argument("feedback_timeout_s must be in (0, 1]");
    }
    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    if (!std::isfinite(publish_rate) || publish_rate <= 0.0) {
      throw std::invalid_argument(
          "publish_rate_hz must be finite and positive");
    }
    tx_period_s_ = 1.0 / publish_rate;
    poll_when_idle_ = get_parameter("feedback_poll_when_idle").as_bool();
    stop_current_a_ =
        static_cast<float>(get_parameter("stop_current_a").as_double());
    velocity_current_a_ = static_cast<float>(
        get_parameter("velocity_current_limit_a").as_double());
    if (!(stop_current_a_ > 0.0F) || !(velocity_current_a_ > 0.0F)) {
      throw std::invalid_argument(
          "stop_current_a and velocity_current_limit_a must be positive");
    }

    load_registry(get_parameter("motor_config_file").as_string());
    connect_motors();

    for (const auto& joint : joints_) {
      inhibited_since_[joint.name] = TimePoint::min();  // unarmed until ENABLE
    }

    joint_state_pub_ =
        create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    motor_status_pub_ =
        create_publisher<std_msgs::msg::String>("motor_status", 10);
    telemetry_pub_ =
        create_publisher<std_msgs::msg::String>("motor_telemetry", 10);
    timing_pub_ = create_publisher<std_msgs::msg::String>("can_timing", 1);
    cmd_sub_ = create_subscription<MotorCommand>(
        "motor_commands", 10,
        std::bind(&EncosCanNode::cmd_callback, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&EncosCanNode::estop_callback, this, std::placeholders::_1));

    publish_timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::duration<double>(tx_period_s_)),
                          std::bind(&EncosCanNode::publish_states, this));
    timing_timer_ =
        create_wall_timer(std::chrono::seconds(1),
                          std::bind(&EncosCanNode::publish_timing, this));

    running_ = true;
    tx_thread_ = std::thread(&EncosCanNode::tx_loop, this);

    RCLCPP_INFO(get_logger(),
                "ENCOS CAN node started with %zu motors on %zu interface(s)",
                joints_.size(), adapters_.size());
  }

  ~EncosCanNode() override {
    RCLCPP_INFO(get_logger(), "Shutting down ENCOS CAN node...");
    running_ = false;
    wake_tx();
    if (tx_thread_.joinable()) {
      tx_thread_.join();
    }
    // TX thread is gone: this thread now owns the buses for the final stop.
    for (auto& joint : joints_) {
      try {
        send_stop(joint);
      } catch (const std::exception&) {
      }
    }
    commit_all();
    for (auto& entry : adapters_) {
      encos::DeleteAdapter(
          entry.second);  // cascades to Bus/Motor; pointers now invalid
    }
  }

 private:
  // ---------------------------------------------------------------- config
  void load_registry(const std::string& path) {
    if (path.empty()) {
      throw std::invalid_argument("motor_config_file is required");
    }
    RCLCPP_INFO(get_logger(), "Loading motor config from: %s", path.c_str());
    YAML::Node root = YAML::LoadFile(path);
    YAML::Node params = root["motor_control_node"]
                            ? root["motor_control_node"]["ros__parameters"]
                            : root;

    // Stricter than python_can_node (which only rejects an explicit false):
    // new actuators must be confirmed, not merely "not marked unverified".
    if (!params["hardware_verified"] ||
        !params["hardware_verified"].as<bool>()) {
      throw std::invalid_argument(
          "Motor registry is not hardware verified: verify CAN IDs, models, "
          "directions and "
          "offsets, then set hardware_verified: true");
    }
    const auto default_iface =
        params["default_can_interface"].as<std::string>("can0");
    const double default_kp = params["KP"].as<double>(10.0);
    const double default_kd = params["KD"].as<double>(0.2);
    max_vel_rad_s_ = params["MAX_VEL_RAD_S"].as<double>(4.5);

    const YAML::Node motors = params["motors"];
    if (!motors || !motors.IsMap() || motors.size() == 0) {
      throw std::invalid_argument("Motor registry is empty");
    }
    std::set<std::pair<std::string, int>> seen;
    for (const auto& entry : motors) {
      const auto name = entry.first.as<std::string>();
      const YAML::Node cfg = entry.second;
      Joint joint;
      joint.name = name;
      joint.index = joints_.size();
      joint.can_interface = cfg["can_interface"].as<std::string>(default_iface);
      joint.motor_id = cfg["motor_id"].as<int>(0);
      joint.model = cfg["model"].as<std::string>("");
      if (joint.motor_id < 1 || joint.motor_id > 254 ||
          joint.can_interface.empty()) {
        throw std::invalid_argument(
            name + ": valid motor_id and can_interface are required");
      }
      if (!parse_model(joint.model)) {
        throw std::invalid_argument(name + ": unknown ENCOS model '" +
                                    joint.model + "'");
      }
      if (!seen.insert({joint.can_interface, joint.motor_id}).second) {
        throw std::invalid_argument(name + ": duplicate CAN motor mapping");
      }
      if (cfg["ankle_mapping"] && !cfg["ankle_mapping"].IsNull()) {
        throw std::invalid_argument(
            name +
            ": ankle_mapping is not implemented in encos_can_node yet "
            "(port ankle_mapping.py before driving linkage joints)");
      }

      auto& s = joint.settings;
      s.direction = cfg["direction"].as<int>(1);
      s.offset = cfg["encoder_offset_rad"].as<double>(0.0);
      s.lower = cfg["min_position"].as<double>(-kInf);
      s.upper = cfg["max_position"].as<double>(kInf);
      s.kp = cfg["kp"].as<double>(default_kp);
      s.kd = cfg["kd"].as<double>(default_kd);
      s.max_velocity = cfg["max_vel_rad_s"].as<double>(max_vel_rad_s_);
      if (cfg["max_torque_nm"] && !cfg["max_torque_nm"].IsNull()) {
        s.max_torque = cfg["max_torque_nm"].as<double>();
      }
      if ((s.direction != 1 && s.direction != -1) || !std::isfinite(s.offset) ||
          std::isnan(s.lower) || std::isnan(s.upper) || s.lower >= s.upper ||
          !(std::isfinite(s.kp) && s.kp >= 0) ||
          !(std::isfinite(s.kd) && s.kd >= 0) ||
          !(std::isfinite(s.max_velocity) && s.max_velocity >= 0) ||
          (s.max_torque &&
           !(std::isfinite(*s.max_torque) && *s.max_torque > 0))) {
        throw std::invalid_argument(
            name + ": invalid motor calibration, limits or gains");
      }
      joint_index_[name] = joint.index;
      joints_.push_back(joint);
    }

    const YAML::Node order = params["model_contract"]["joint_order"];
    if (order) {
      std::vector<std::string> expected = order.as<std::vector<std::string>>();
      std::vector<std::string> actual;
      for (const auto& joint : joints_) {
        actual.push_back(joint.name);
      }
      if (expected != actual) {
        throw std::invalid_argument(
            "Motor registry order does not match the tracking contract");
      }
    }
  }

  // All scanning/creation/parameter I/O happens here, BEFORE sync mode is
  // enabled; encos_driver requests issued after SetSyncMode wait for Commit().
  void connect_motors() {
    const auto adapter_type = get_parameter("adapter_type").as_string();
    const bool can_fd = get_parameter("can_fd").as_bool();
    const auto can_timeout_ms = get_parameter("motor_can_timeout_ms").as_int();
    if (can_timeout_ms < 0 || can_timeout_ms > 65535) {
      throw std::invalid_argument("motor_can_timeout_ms must be in [0, 65535]");
    }

    for (auto& joint : joints_) {
      auto& adapter = adapters_[joint.can_interface];
      if (adapter == nullptr) {
        adapter = encos::MakeAdapter(adapter_type,
                                     joint.can_interface);  // throws on failure
        if (adapter == nullptr || adapter->GetBus(0) == nullptr) {
          throw std::runtime_error("No ENCOS bus on " + joint.can_interface);
        }
        tx_submitted_[joint.can_interface] =
            0;  // keys fixed before threads start
      }
      joint.bus = adapter->GetBus(0);
      joint.motor = joint.bus->GetMotor(joint.motor_id,
                                        *parse_model(joint.model), can_fd);
      if (joint.motor == nullptr) {
        throw std::runtime_error(joint.name + ": GetMotor failed");
      }
      // Cached statuses expire after two reads so a silent motor cannot look
      // alive.
      joint.motor->SetMaxStatusLifeCycle(2);
      if (can_timeout_ms > 0 &&
          !joint.motor->SetCanTimeout(static_cast<uint16_t>(can_timeout_ms))) {
        throw std::runtime_error(joint.name +
                                 ": SetCanTimeout not acknowledged");
      }
      const auto props = joint.motor->GetProperties();
      joint.kt = props.kt;
      RCLCPP_INFO(get_logger(),
                  "Configured motor[%zu]: %s on %s, motor_id=%d, model=%s, "
                  "kt=%.3f, fw=0x%08x%s",
                  joint.index, joint.name.c_str(), joint.can_interface.c_str(),
                  joint.motor_id, joint.model.c_str(), props.kt,
                  props.fw_version,
                  props.fw_version == 0 ? " (unknown/old firmware)" : "");

      const size_t index = joint.index;
      // Runs on the encos_driver RX thread: copy and return, no I/O.
      joint.motor->SetOnStatus([this, index](const encos::MotorStatus& status) {
        on_status(index, status);
      });
    }
    for (auto& entry : adapters_) {
      entry.second->GetBus(0)->SetSyncMode(true);
    }
  }

  // ---------------------------------------------------------------- RX
  void on_status(size_t index, const encos::MotorStatus& status) {
    const auto& joint = joints_[index];
    rx_frames_++;
    const auto received_at = Clock::now();
    if (status.error == encos::MotorError::OverTemperatureWarning ||
        status.error == encos::MotorError::TemperatureWarning) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "%s: temperature warning (code %d)",
                           joint.name.c_str(), static_cast<int>(status.error));
    } else if (status.error != encos::MotorError::NoError) {
      latch_transport_fault(joint.name + ": motor error code " +
                            std::to_string(static_cast<int>(status.error)));
      return;
    }

    // UNVERIFIED: kt from the model table is assumed to be output-side Nm/A.
    double position = status.position;
    double velocity = status.speed;
    double torque = static_cast<double>(status.current) * joint.kt;
    try {
      joint.settings.feedback(position, velocity, torque);
    } catch (const std::exception& e) {
      latch_transport_fault(joint.name + ": " + e.what());
      return;
    }
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      auto& sample = state_[joint.name];
      sample = Sample{position,    velocity,
                      torque,      status.motor_temperature,
                      received_at, get_clock()->now().seconds()};
      if (status.error == encos::MotorError::NoError) {
        last_healthy_status_[joint.name] = received_at;
      }
    }
    if (awaiting_enable_ack_) {
      wake_tx();
    }
  }

  // A local transport/motor fault cannot be cleared by a healthy supervisor
  // publishing false; restart the node after diagnosis.
  void latch_transport_fault(const std::string& reason) {
    if (!transport_fault_.exchange(true)) {
      estop_disable_pending_ = true;
      RCLCPP_ERROR(get_logger(), "ENCOS transport fault (latched): %s",
                   reason.c_str());
    }
    wake_tx();
  }

  // ---------------------------------------------------------------- TX
  void configure_tx_scheduling() {
    if (!can_rt_enabled_) {
      RCLCPP_INFO(get_logger(), "CAN TX: existing scheduling");
      return;
    }
    sched_param param{};
    param.sched_priority = can_tx_rt_priority_;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (rc != 0) {
      RCLCPP_WARN(get_logger(),
                  "CAN TX: cannot set SCHED_FIFO priority=%d: %s; continuing",
                  can_tx_rt_priority_, std::strerror(rc));
    } else {
      RCLCPP_INFO(get_logger(), "CAN TX: SCHED_FIFO priority=%d",
                  can_tx_rt_priority_);
    }
  }

  void wake_tx() {
    {
      std::lock_guard<std::mutex> lock(wake_mutex_);
      wake_requested_ = true;
    }
    wake_cv_.notify_one();
  }

  void tx_loop() {
    configure_tx_scheduling();
    while (running_ && rclcpp::ok()) {
      const auto begin = Clock::now();
      {
        std::lock_guard<std::mutex> lock(wake_mutex_);
        wake_requested_ = false;
      }
      expire_commands();
      drain_command_queue();
      process_enable_reports();
      if (estop_active_ || transport_fault_) {
        enforce_estop();
      }
      for (auto& joint : joints_) {
        send_cycle(joint);
      }
      commit_all();

      const auto end = Clock::now();
      tx_cycles_++;
      const double work_ms = seconds_between(begin, end) * 1000.0;
      tx_max_work_ms_ = std::max(tx_max_work_ms_.load(), work_ms);

      // Sleep the remainder of the period, waking early for a new command,
      // estop, enable ack, or an earlier per-joint deadline.
      auto wake_at = after(begin, tx_period_s_);
      for (const auto& entry : command_deadlines_) {
        wake_at = std::min(wake_at, entry.second);
      }
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_cv_.wait_until(lock, wake_at,
                          [this]() { return wake_requested_ || !running_; });
    }
  }

  void commit_all() {
    for (auto& entry : adapters_) {
      try {
        entry.second->GetBus(0)->Commit();
        tx_submitted_.at(entry.first)++;
      } catch (const std::exception& e) {
        tx_errors_++;
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "Commit failed on %s: %s", entry.first.c_str(),
                              e.what());
      }
    }
  }

  void send_stop(Joint& joint) {
    // Non-blocking; feedback=1 requests a status reply for the RX callback.
    joint.motor->Stop<0>(encos::MotorStopMode::DynamicBrake, stop_current_a_,
                         1);
  }

  // One wire command per joint per pass: the active target when armed,
  // otherwise a damping Stop that doubles as the idle feedback poll.
  void send_cycle(Joint& joint) {
    expire_commands();  // scheduling may exhaust a deadline mid-pass
    const auto active = active_commands_.find(joint.name);
    const bool armed = !estop_active_ && !transport_fault_ &&
                       inhibited_since_.count(joint.name) == 0 &&
                       active != active_commands_.end();
    try {
      if (armed) {
        send_active_command(joint, active->second);
      } else if (pending_disable_.count(joint.name) != 0 ||
                 enable_pending_.count(joint.name) != 0 || poll_when_idle_) {
        // An enabling joint is always polled: its ack is the status reply.
        send_stop(joint);
        pending_disable_.erase(joint.name);
      }
    } catch (const std::exception& e) {
      tx_errors_++;
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Failed to send to %s: %s", joint.name.c_str(),
                            e.what());
    }
  }

  void send_active_command(Joint& joint, const Command& cmd) {
    const auto& s = joint.settings;
    for (const double v : {cmd.position, cmd.velocity, cmd.acceleration,
                           cmd.torque, cmd.kp, cmd.kd}) {
      if (!std::isfinite(v)) {
        throw std::invalid_argument("Non-finite command");
      }
    }
    if (cmd.kp < 0 || cmd.kd < 0) {
      throw std::invalid_argument("Negative gains");
    }
    // Joint-space limits, then joint -> motor coordinates.
    const double position = std::clamp(cmd.position, s.lower, s.upper);
    const double vel_limit = std::min(s.max_velocity, max_vel_rad_s_);
    const double velocity = std::clamp(cmd.velocity, -vel_limit, vel_limit);
    double torque = cmd.torque;
    if (s.max_torque) {
      torque = std::clamp(torque, -*s.max_torque, *s.max_torque);
    }
    const auto motor_pos =
        static_cast<float>(s.offset + s.direction * position);
    const auto motor_vel = static_cast<float>(s.direction * velocity);
    const auto motor_tor = static_cast<float>(s.direction * torque);
    const auto kp = static_cast<float>(cmd.kp);
    const auto kd = static_cast<float>(cmd.kd);

    // UNVERIFIED: PVT kp/kd are assumed to be Nm/rad and Nm*s/rad like the
    // RobStride MIT frame the policy was trained against. Step-test on a bench.
    switch (cmd.mode) {
      case MotorCommand::MODE_MOTION:
        joint.motor->PVTControl<0>(kp, kd, motor_pos, motor_vel, motor_tor);
        break;
      case MotorCommand::MODE_POSITION:
        // python_can_node passes |velocity| as the MIT velocity target here,
        // which biases the D term in one direction. Use a zero velocity target.
        joint.motor->PVTControl<0>(kp, kd, motor_pos, 0.0F, 0.0F);
        break;
      case MotorCommand::MODE_VELOCITY:
        joint.motor->SpdControl<0>(motor_vel, velocity_current_a_, 1);
        break;
      default:
        break;
    }
  }

  void process_enable_reports() {
    std::map<std::string, TimePoint> healthy;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      healthy = last_healthy_status_;
    }
    const auto now = Clock::now();
    for (auto it = enable_pending_.begin(); it != enable_pending_.end();) {
      const auto report = healthy.find(it->first);
      const auto deadline = command_deadlines_.find(it->first);
      if (report != healthy.end() && report->second >= it->second &&
          deadline != command_deadlines_.end() && now < deadline->second) {
        inhibited_since_.erase(it->first);
        RCLCPP_INFO(get_logger(), "Enable confirmed for %s", it->first.c_str());
        it = enable_pending_.erase(it);
      } else {
        ++it;
      }
    }
    awaiting_enable_ack_ = !enable_pending_.empty();
  }

  void enforce_estop() {
    if (!estop_disable_pending_) {
      return;
    }
    command_deadlines_.clear();
    enable_pending_.clear();
    awaiting_enable_ack_ = false;
    active_commands_.clear();
    const auto now = Clock::now();
    for (const auto& joint : joints_) {
      inhibited_since_[joint.name] = now;
      pending_disable_.insert(joint.name);
    }
    estop_disable_pending_ = false;
    RCLCPP_FATAL(
        get_logger(),
        "ESTOP: damping stop queued for all joints; verify via feedback");
  }

  // Stop expired joints. All deadlines are steady-clock, including queue time.
  // A fresh explicit ENABLE is required to recover after a timeout.
  void expire_commands() {
    const auto now = Clock::now();
    for (auto it = command_deadlines_.begin();
         it != command_deadlines_.end();) {
      if (now >= it->second) {
        const auto joint = it->first;
        it = command_deadlines_.erase(it);
        enable_pending_.erase(joint);
        inhibited_since_[joint] = now;
        active_commands_.erase(joint);
        pending_disable_.insert(joint);
        RCLCPP_ERROR(get_logger(),
                     "Command timeout for %s; explicit enable required",
                     joint.c_str());
      } else {
        ++it;
      }
    }
    if (enable_pending_.empty()) {
      awaiting_enable_ack_ = false;
    }
  }

  void drain_command_queue() {
    // Bound work even when producers continuously update targets.
    for (size_t i = 0; i < 2 * joints_.size(); ++i) {
      expire_commands();
      auto next = mailbox_.get();
      if (!next) {
        return;
      }
      Command cmd = std::move(*next);
      const auto now = Clock::now();

      // Queue residence consumes the original lifetime. A delayed ENABLE
      // must not restart a joint after the watchdog has already stopped it.
      if (cmd.mode != MotorCommand::MODE_DISABLE) {
        if (estop_active_ || transport_fault_ || now >= cmd.deadline) {
          continue;
        }
        const auto inhibited = inhibited_since_.find(cmd.joint);
        if (cmd.mode == MotorCommand::MODE_ENABLE) {
          if (pending_disable_.count(cmd.joint) != 0) {
            continue;
          }
          if (inhibited != inhibited_since_.end() &&
              cmd.received_at <= inhibited->second) {
            continue;
          }
        } else if (inhibited != inhibited_since_.end() &&
                   enable_pending_.count(cmd.joint) == 0) {
          continue;
        }
      }

      if (cmd.mode == MotorCommand::MODE_ENABLE) {
        // Software arm only: ENCOS has no enable frame. Motion is held until a
        // fresh error-free status (elicited by the idle Stop poll) arrives.
        active_commands_.erase(cmd.joint);
        command_deadlines_[cmd.joint] = cmd.deadline;
        enable_pending_[cmd.joint] = now;
        inhibited_since_.emplace(cmd.joint, cmd.received_at);
        awaiting_enable_ack_ = true;
        RCLCPP_INFO(get_logger(), "Enable requested for %s", cmd.joint.c_str());
        continue;
      }
      if (cmd.mode == MotorCommand::MODE_DISABLE) {
        active_commands_.erase(cmd.joint);
        command_deadlines_.erase(cmd.joint);
        enable_pending_.erase(cmd.joint);
        inhibited_since_[cmd.joint] = now;
        pending_disable_.insert(cmd.joint);
        RCLCPP_INFO(get_logger(), "Disable (damping stop) queued for %s",
                    cmd.joint.c_str());
        continue;
      }
      if (cmd.mode == MotorCommand::MODE_VELOCITY && cmd.acceleration != 0.0) {
        RCLCPP_WARN_ONCE(
            get_logger(),
            "MODE_VELOCITY acceleration is ignored on ENCOS motors");
      }
      if (enable_pending_.count(cmd.joint) == 0) {
        command_deadlines_[cmd.joint] = cmd.deadline;
      }
      active_commands_[cmd.joint] = std::move(cmd);
    }
  }

  // ---------------------------------------------------------------- ROS
  // callbacks
  void estop_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (msg->data) {
      const bool newly_asserted = !estop_active_.exchange(true);
      estop_disable_pending_ = true;
      mailbox_.clear();
      wake_tx();
      if (newly_asserted) {
        RCLCPP_FATAL(
            get_logger(),
            "Safety estop asserted; rejecting commands and stopping motors");
      }
    } else if (estop_active_.exchange(false)) {
      RCLCPP_WARN(
          get_logger(),
          "Safety estop reset; motors remain stopped until explicitly enabled");
    }
  }

  void cmd_callback(const MotorCommand::SharedPtr msg) {
    if (estop_active_ || transport_fault_ || msg->joint_name.empty()) {
      return;
    }
    const auto received_at = Clock::now();
    const double now_ros = get_clock()->now().seconds();
    const auto& hs = msg->header.stamp;
    const double stamp = hs.sec + hs.nanosec * 1e-9;
    const double age = now_ros - stamp;
    bool source_valid = hs.sec >= 0 && hs.nanosec < 1000000000u && stamp > 0 &&
                        age >= -0.01 && age < command_timeout_s_;
    // The source age and local queue time both count against the lifetime.
    auto deadline = after(received_at, command_timeout_s_ - std::max(0.0, age));
    const auto& vu = msg->valid_until;
    const double expiry = vu.sec + vu.nanosec * 1e-9;
    if (expiry != 0) {
      source_valid = source_valid && vu.sec >= 0 && vu.nanosec < 1000000000u &&
                     expiry > now_ros && expiry > stamp;
      deadline = std::min(deadline, after(received_at, expiry - now_ros));
    }

    std::vector<Command> commands;
    for (size_t i = 0; i < msg->joint_name.size(); ++i) {
      const auto& name = msg->joint_name[i];
      const auto it = joint_index_.find(name);
      if (it == joint_index_.end()) {
        continue;
      }
      uint8_t mode = MotorCommand::MODE_VELOCITY;
      if (msg->mode.size() == 1) {
        mode = msg->mode[0];
      } else if (i < msg->mode.size()) {
        mode = msg->mode[i];
      }
      if (mode != MotorCommand::MODE_DISABLE && !source_valid) {
        RCLCPP_WARN(get_logger(),
                    "Rejecting stale or invalid command timestamp for %s",
                    name.c_str());
        continue;
      }
      const auto& s = joints_[it->second].settings;
      auto at = [i](const std::vector<double>& v, double fallback) {
        return i < v.size() ? v[i] : fallback;
      };
      Command cmd;
      cmd.joint = name;
      cmd.mode = mode;
      cmd.received_at = received_at;
      cmd.deadline = deadline;
      cmd.position = at(msg->position, 0.0);
      cmd.velocity = at(msg->velocity, 0.0);
      cmd.acceleration = at(msg->acceleration, 0.0);
      cmd.torque = at(msg->torque, 0.0);
      cmd.kp = at(msg->kp, s.kp);
      cmd.kd = at(msg->kd, s.kd);
      commands.push_back(std::move(cmd));
    }
    if (!commands.empty()) {
      mailbox_.put(commands);  // one ROS message is installed atomically
      wake_tx();
    }
  }

  void publish_states() {
    sensor_msgs::msg::JointState msg;
    const auto now_ros = get_clock()->now();
    double oldest_age = 0.0;
    std::ostringstream status;
    std::ostringstream telemetry;
    status << "{";
    telemetry << "{\"schema_version\": 1, \"motors\": [";
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const auto now = Clock::now();
      bool first = true;
      for (const auto& joint : joints_) {
        const auto it = state_.find(joint.name);
        if (it == state_.end()) {
          continue;
        }
        const auto& sample = it->second;
        const double age = seconds_between(sample.received_at, now);
        if (age > feedback_timeout_s_) {
          continue;
        }
        oldest_age = std::max(oldest_age, age);
        msg.name.push_back(joint.name);
        msg.position.push_back(sample.position);
        msg.velocity.push_back(sample.velocity);
        msg.effort.push_back(sample.torque);

        status << (first ? "" : ", ") << "\"" << joint.index
               << "\": {\"temperature\": " << json_number(sample.temperature)
               << ", \"torque\": " << json_number(sample.torque) << "}";
        telemetry << (first ? "" : ", ") << "{\"joint_name\": \""
                  << json_escape(joint.name) << "\", \"can_interface\": \""
                  << json_escape(joint.can_interface)
                  << "\", \"motor_id\": " << joint.motor_id << ", \"model\": \""
                  << json_escape(joint.model) << "\", \"source_stamp\": "
                  << json_number(sample.source_stamp)
                  << ", \"temperature_c\": " << json_number(sample.temperature)
                  << "}";
        first = false;
      }
    }
    status << "}";
    telemetry << "]}";

    if (!msg.name.empty()) {
      // JointState has one stamp: use the oldest included receipt, rather
      // than making cached data look new.
      msg.header.stamp = now_ros - rclcpp::Duration::from_seconds(oldest_age);
      joint_state_pub_->publish(msg);
    }
    std_msgs::msg::String out;
    out.data = status.str();
    motor_status_pub_->publish(out);
    out.data = telemetry.str();
    telemetry_pub_->publish(out);
  }

  void publish_timing() {
    std::ostringstream os;
    os << "{\"tx_cycles\": " << tx_cycles_
       << ", \"tx_max_work_ms\": " << tx_max_work_ms_
       << ", \"tx_errors\": " << tx_errors_ << ", \"rx_frames\": " << rx_frames_
       << ", \"rx_ignored\": 0, \"pending_commands\": " << mailbox_.size()
       << ", \"transport_fault\": " << (transport_fault_ ? "true" : "false")
       << ", \"repeat_hz\": " << 1.0 / tx_period_s_ << ", \"interfaces\": {";
    bool first = true;
    for (const auto& entry : adapters_) {
      // tx_submitted counts bus commits (one batch per pass), not frames.
      os << (first ? "" : ", ") << "\"" << json_escape(entry.first)
         << "\": {\"tx_submitted\": " << tx_submitted_.at(entry.first).load()
         << "}";
      first = false;
    }
    os << "}}";
    std_msgs::msg::String out;
    out.data = os.str();
    timing_pub_->publish(out);
  }

  // ---------------------------------------------------------------- state
  // Parameters
  bool can_rt_enabled_{false};
  int can_tx_rt_priority_{45};
  double command_timeout_s_{0.5};
  double feedback_timeout_s_{0.1};
  double tx_period_s_{0.02};
  double max_vel_rad_s_{4.5};
  bool poll_when_idle_{true};
  float stop_current_a_{5.0F};
  float velocity_current_a_{5.0F};

  // Registry and driver objects (fixed after construction)
  std::vector<Joint> joints_;
  std::unordered_map<std::string, size_t> joint_index_;
  std::map<std::string, encos::BaseAdapter*>
      adapters_;  // non-owning; see DeleteAdapter

  // Shared between threads
  LatestCommandMailbox mailbox_;
  std::atomic<bool> running_{false};
  std::atomic<bool> estop_active_{false};
  std::atomic<bool> estop_disable_pending_{false};
  std::atomic<bool> transport_fault_{false};
  std::atomic<bool> awaiting_enable_ack_{false};
  std::mutex
      state_mutex_;  // guards state_ and last_healthy_status_ (RX vs ROS/TX)
  std::map<std::string, Sample> state_;
  std::map<std::string, TimePoint> last_healthy_status_;
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  bool wake_requested_{false};

  // Owned exclusively by the TX thread
  std::map<std::string, Command> active_commands_;
  std::map<std::string, TimePoint> command_deadlines_;
  std::map<std::string, TimePoint> inhibited_since_;
  std::map<std::string, TimePoint> enable_pending_;
  std::set<std::string> pending_disable_;

  // Counters
  std::atomic<uint64_t> tx_cycles_{0};
  std::atomic<double> tx_max_work_ms_{0.0};
  std::atomic<uint64_t> tx_errors_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::map<std::string, std::atomic<uint64_t>>
      tx_submitted_;  // keys set in connect_motors()

  std::thread tx_thread_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::Subscription<MotorCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr timing_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<EncosCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(rclcpp::get_logger("encos_can_node"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
