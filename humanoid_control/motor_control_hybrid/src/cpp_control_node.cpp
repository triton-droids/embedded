#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <motor_control_interfaces/msg/motor_command.hpp>

#include <atomic>
#include <chrono>
#include <sched.h>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

// New non-ROS control core API (per-joint independent mode)
#include "control_core/api.hpp"


class CppControlNode : public rclcpp::Node
{
public:
  CppControlNode()
  : Node("cpp_control_node")
  {
    this->declare_parameter<bool>("control_rt_enabled", false);
    this->declare_parameter<int>("control_rt_priority", 10);
    const auto priority = this->get_parameter("control_rt_priority").as_int();
    if (priority < 1 || priority > 99) {
      throw std::invalid_argument("control_rt_priority must be between 1 and 99");
    }
    // Parameters
    this->declare_parameter<double>("control_rate_hz", 50.0);
    this->declare_parameter<double>("state_timeout_s", 0.2);
    this->declare_parameter<bool>("auto_append_new_joints", false);
    this->declare_parameter<double>("cmd_timeout_s", 0.5);
    this->declare_parameter<double>("supervisor_warning_timeout_s", 0.5);
    this->declare_parameter<double>("supervisor_warning_startup_grace_s", 3.0);
    supervisor_timeout_s_ = this->get_parameter("supervisor_warning_timeout_s").as_double();
    supervisor_grace_s_ = this->get_parameter("supervisor_warning_startup_grace_s").as_double();
    if (!std::isfinite(supervisor_timeout_s_) || supervisor_timeout_s_ <= 0 ||
        !std::isfinite(supervisor_grace_s_) || supervisor_grace_s_ < 0) {
      throw std::invalid_argument("Supervisor warning timeout must be positive; grace nonnegative");
    }
    // Deployments pass the full registry order so a partial first feedback
    // message cannot silently shrink the controlled robot.
    this->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>{});

    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    auto_append_new_joints_ = this->get_parameter("auto_append_new_joints").as_bool();
    cmd_timeout_s_   = this->get_parameter("cmd_timeout_s").as_double();
    if (!std::isfinite(cmd_timeout_s_) || cmd_timeout_s_ <= 0.0) {
      throw std::invalid_argument("cmd_timeout_s must be finite and positive");
    }
    if (!std::isfinite(state_timeout_s_) || state_timeout_s_ <= 0.0) {
      throw std::invalid_argument("state_timeout_s must be finite and positive");
    }
    joint_order_ = this->get_parameter("joint_names").as_string_array();
    if (!joint_order_.empty()) {
      std::unordered_set<std::string> names(joint_order_.begin(), joint_order_.end());
      if (names.size() != joint_order_.size() || names.count("") != 0) {
        throw std::invalid_argument("joint_names must be unique and nonempty");
      }
      core_.configure(joint_order_);
      core_configured_ = true;
    } else {
      RCLCPP_WARN(get_logger(), "joint_names unset: inferring order from first valid feedback; "
        "set joint_names to the complete hardware registry for deployment");
    }

    // Prefer absolute topic names to avoid namespace surprises
    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CppControlNode::state_callback, this, std::placeholders::_1)
    );

    // Recommended external command interface: MotorCommand subset
    desired_motor_sub_ = this->create_subscription<motor_control_interfaces::msg::MotorCommand>(
      "/desired_motor_subset", rclcpp::QoS(1).best_effort(),
      std::bind(&CppControlNode::desired_motor_callback, this, std::placeholders::_1)
    );

    // Backward-compatible: velocity subset via JointState
    desired_vel_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/desired_velocity_subset", 1,
      std::bind(&CppControlNode::desired_velocity_callback, this, std::placeholders::_1)
    );

    // Latched safety supervisor input. Transient local receives the latest
    // estop state even when this control node starts after the safety node.
    safety_estop_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&CppControlNode::safety_estop_callback, this, std::placeholders::_1)
    );

    // The independent supervisor is advisory when absent/unhealthy. This does
    // not alter the explicit /safety/estop or command/state watchdog contracts.
    supervisor_sub_ = this->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/safety/status", rclcpp::QoS(1).best_effort(),
      std::bind(&CppControlNode::supervisor_callback, this, std::placeholders::_1));
    supervisor_warning_timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      const auto reason = supervisor_warning_reason();
      if (!reason.empty()) {
        RCLCPP_WARN_THROTTLE(get_logger(), warning_clock_, 5000,
          "Safety supervisor warning (advisory only): %s", reason.c_str());
      }
    });

    // Output to Python CAN node
    cmd_pub_ = this->create_publisher<motor_control_interfaces::msg::MotorCommand>("/motor_commands", 10);

    // Control timer
    const double rate = std::max(1.0, control_rate_hz_);
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate)
    );
    control_timer_ = this->create_wall_timer(period, std::bind(&CppControlNode::control_loop, this));

    RCLCPP_INFO(
      this->get_logger(),
      "CppControlNode started (rate=%.1f Hz, state_timeout=%.3fs, cmd_timeout=%.3fs)",
      control_rate_hz_, state_timeout_s_, cmd_timeout_s_
    );
  }

private:
  friend struct CppControlNodeTestPeer;
  struct JointDataRT {
    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
    std::chrono::steady_clock::time_point received_at{};
    double source_stamp{0.0};
  };

  // -------- mode mapping helpers --------
  static inline control_core::Mode from_ros_mode(uint8_t m) {
    using MC = motor_control_interfaces::msg::MotorCommand;
    switch (m) {
      case MC::MODE_VELOCITY: return control_core::Mode::Velocity;
      case MC::MODE_POSITION: return control_core::Mode::Position;
      case MC::MODE_MOTION:   return control_core::Mode::Motion;
      case MC::MODE_ENABLE:   return control_core::Mode::Enable;
      case MC::MODE_DISABLE:  return control_core::Mode::Disable;
      default:                return control_core::Mode::Velocity;
    }
  }

  static inline uint8_t to_ros_mode(control_core::Mode m) {
    using MC = motor_control_interfaces::msg::MotorCommand;
    switch (m) {
      case control_core::Mode::Velocity: return MC::MODE_VELOCITY;
      case control_core::Mode::Position: return MC::MODE_POSITION;
      case control_core::Mode::Motion:   return MC::MODE_MOTION;
      case control_core::Mode::Enable:   return MC::MODE_ENABLE;
      case control_core::Mode::Disable:  return MC::MODE_DISABLE;
      default:                           return MC::MODE_VELOCITY;
    }
  }

  // -------- callbacks --------
  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty()) return;

    std::lock_guard<std::mutex> lock(state_mutex_);
    const std::unordered_set<std::string> names(msg->name.begin(), msg->name.end());
    if (names.size() != msg->name.size() || names.count("") != 0 ||
        msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u ||
        msg->position.size() != msg->name.size() ||
        msg->velocity.size() != msg->name.size() ||
        (!msg->effort.empty() && msg->effort.size() != msg->name.size()) ||
        !std::all_of(msg->position.begin(), msg->position.end(), [](double v) {return std::isfinite(v);}) ||
        !std::all_of(msg->velocity.begin(), msg->velocity.end(), [](double v) {return std::isfinite(v);}) ||
        !std::all_of(msg->effort.begin(), msg->effort.end(), [](double v) {return std::isfinite(v);})) {
      // Do not let a malformed update keep an earlier valid frame alive.
      latest_state_map_.clear();
      return;
    }
    const auto received_at = std::chrono::steady_clock::now();
    const double source_stamp = rclcpp::Time(msg->header.stamp).seconds();

    // Lock joint order once
    if (joint_order_.empty()) {
      joint_order_ = msg->name;
      RCLCPP_INFO(this->get_logger(),
                  "Joint order locked from first /joint_states (%zu joints).",
                  joint_order_.size());
      try {
        core_.configure(joint_order_);
        core_configured_ = true;
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "control_core::Controller configure failed: %s", e.what());
        core_configured_ = false;
      }
    }

    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& jn = msg->name[i];
      JointDataRT& jd = latest_state_map_[jn];

      if (i < msg->position.size()) jd.pos = msg->position[i];
      if (i < msg->velocity.size()) jd.vel = msg->velocity[i];
      if (i < msg->effort.size())   jd.eff = msg->effort[i];
      jd.received_at = received_at;
      jd.source_stamp = source_stamp;

      if (auto_append_new_joints_) {
        if (std::find(joint_order_.begin(), joint_order_.end(), jn) == joint_order_.end()) {
          joint_order_.push_back(jn);
          RCLCPP_WARN(this->get_logger(), "New joint appended: %s", jn.c_str());
          try {
            core_.configure(joint_order_);
            core_configured_ = true;
          } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "re-configure failed: %s", e.what());
            core_configured_ = false;
          }
        }
      }
    }
  }

  struct CommandTime {
    double source;
    double expiry;
    std::chrono::steady_clock::time_point deadline;
  };

  bool command_time(const builtin_interfaces::msg::Time& stamp,
    const builtin_interfaces::msg::Time& expiry, CommandTime& result) const
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1000000000u ||
        expiry.sec < 0 || expiry.nanosec >= 1000000000u) return false;
    const double source = rclcpp::Time(stamp).seconds();
    const double now = this->now().seconds();
    const double explicit_expiry = rclcpp::Time(expiry).seconds();
    const double end = explicit_expiry > 0 ?
      std::min(source + cmd_timeout_s_, explicit_expiry) : source + cmd_timeout_s_;
    if (source <= 0 || source > now + 0.01 || end <= now) return false;
    result = {source, end, std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(end - now))};
    return true;
  }

  // Preferred: full per-joint MotorCommand subset
  void desired_motor_callback(const motor_control_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (!msg || msg->joint_name.empty()) return;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    CommandTime timing;
    const bool disable_only = !msg->mode.empty() && std::all_of(
      msg->mode.begin(), msg->mode.end(), [](uint8_t mode) {
        return mode == motor_control_interfaces::msg::MotorCommand::MODE_DISABLE;
      });
    if (!command_time(msg->header.stamp, msg->valid_until, timing)) {
      if (!disable_only) return;
      const double now = this->now().seconds();
      timing = {now, now + cmd_timeout_s_, std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(cmd_timeout_s_))};
    }
    last_cmd_time_ = this->now();
    has_cmd_ = true;
    desired_subset_.clear();

    const size_t n = msg->joint_name.size();

    auto mode_at = [&](size_t i) -> uint8_t {
      if (msg->mode.empty()) return motor_control_interfaces::msg::MotorCommand::MODE_VELOCITY;
      if (msg->mode.size() == 1) return msg->mode[0];
      return (i < msg->mode.size()) ? msg->mode[i] : msg->mode.back();
    };

    auto get = [&](const std::vector<double>& arr, size_t i, double defv) -> double {
      return (i < arr.size()) ? arr[i] : defv;
    };

    for (size_t i = 0; i < n; ++i) {
      const std::string& jn = msg->joint_name[i];

      control_core::Command c;
      c.mode = from_ros_mode(mode_at(i));
      c.position      = get(msg->position, i, 0.0);
      c.velocity      = get(msg->velocity, i, 0.0);
      c.acceleration  = get(msg->acceleration, i, 0.0);
      c.torque        = get(msg->torque, i, 0.0);
      c.kp            = get(msg->kp, i, 40.0);
      c.kd            = get(msg->kd, i, 1.5);

      desired_subset_[jn] = c;
      command_times_[jn] = timing;
    }
  }

  void supervisor_callback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
  {
    if (!msg || msg->header.stamp.sec < 0 || msg->header.stamp.nanosec >= 1000000000u) return;
    const double source = rclcpp::Time(msg->header.stamp).seconds();
    const double age = this->now().seconds() - source;
    if (source <= 0 || age < -0.01 || age > supervisor_timeout_s_ ||
        source <= supervisor_source_stamp_) return;
    for (const auto& status : msg->status) {
      if (status.name != "humanoid_safety/supervisor") continue;
      supervisor_seen_ = true;
      supervisor_source_stamp_ = source;
      supervisor_received_ = std::chrono::steady_clock::now();
      supervisor_level_ = status.level;
      supervisor_message_ = status.message;
      return;
    }
  }

  std::string supervisor_warning_reason() const
  {
    const auto now = std::chrono::steady_clock::now();
    if (!supervisor_seen_) {
      return std::chrono::duration<double>(now - supervisor_started_).count() > supervisor_grace_s_ ?
        "no supervisor heartbeat received" : "";
    }
    const double source_age = this->now().seconds() - supervisor_source_stamp_;
    if (std::chrono::duration<double>(now - supervisor_received_).count() > supervisor_timeout_s_ ||
        source_age < -0.01 || source_age > supervisor_timeout_s_) {
      return "supervisor heartbeat expired";
    }
    if (supervisor_level_ != diagnostic_msgs::msg::DiagnosticStatus::OK) {
      return "supervisor reports unhealthy: " + supervisor_message_;
    }
    return "";
  }

  void safety_estop_callback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (!msg) return;
    const bool previous = safety_estop_active_.exchange(msg->data);
    if (msg->data && !previous) {
      RCLCPP_FATAL(
        this->get_logger(),
        "Safety estop asserted; all motor outputs will be disabled."
      );
    } else if (!msg->data && previous) {
      RCLCPP_WARN(
        this->get_logger(),
        "Safety estop reset; motors remain disabled until explicitly enabled."
      );
    }
  }

  // Backward-compatible: velocity subset via JointState
  void desired_velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty()) return;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    CommandTime timing;
    if (!command_time(msg->header.stamp, builtin_interfaces::msg::Time{}, timing)) return;
    last_cmd_time_ = this->now();
    has_cmd_ = true;

    // Only overwrite joints provided; others remain whatever was already in desired_subset_
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& jn = msg->name[i];
      if (i < msg->velocity.size()) {
        control_core::Command c;
        c.mode = control_core::Mode::Velocity;
        c.velocity = msg->velocity[i];
        desired_subset_[jn] = c;
        command_times_[jn] = timing;
      }
    }
  }

  // -------- control loop --------
  bool state_is_fresh(const std::vector<std::string>& order,
    const std::unordered_map<std::string, JointDataRT>& state) const
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double ros_now = this->now().seconds();
    for (const auto& name : order) {
      const auto it = state.find(name);
      if (it == state.end()) return false;
      const auto& sample = it->second;
      const double receipt_age = std::chrono::duration<double>(steady_now - sample.received_at).count();
      const double source_age = ros_now - sample.source_stamp;
      if (sample.source_stamp <= 0.0 || receipt_age < 0.0 || receipt_age > state_timeout_s_ ||
          source_age < -0.01 || source_age > state_timeout_s_) return false;
    }
    return true;
  }

  void control_loop()
  {
    // Snapshot state
    std::vector<std::string> joint_order;
    std::unordered_map<std::string, JointDataRT> state_map;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joint_order = joint_order_;
      state_map = latest_state_map_;
    }

    if (joint_order.empty()) return;

    // This gate precedes all command handling and continuously publishes
    // DISABLE while the supervisor latch is active.
    if (safety_estop_active_.load()) {
      publish_disable_all(joint_order);
      return;
    }

    if (!core_configured_) {
      // Fail-safe: publish DISABLE for all joints (or zero velocity if you prefer)
      publish_disable_all(joint_order);
      return;
    }

    // Every configured joint must have its own recent, valid q/dq sample.
    // Other joints' traffic cannot renew a missing joint's lifetime.
    if (!state_is_fresh(joint_order, state_map)) {
      publish_disable_all(joint_order);
      return;
    }

    // Snapshot desired subset
    control_core::CommandSubset desired_subset;
    rclcpp::Time last_cmd_time;
    bool has_cmd;

    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      desired_subset = desired_subset_;
      last_cmd_time = last_cmd_time_;
      has_cmd = has_cmd_;
    }

    // No upstream command has been received yet. Do not synthesize a default
    // velocity command, because motor backends start disabled and would reject it
    // every control tick.
    if (!has_cmd) {
      return;
    }

    // Command watchdog: if the upstream command source stops updating,
    // stop forwarding hold-last motion into the CAN node.
    if (cmd_timeout_s_ > 0.0) {
      const double dt_cmd = (this->now() - last_cmd_time).seconds();
      if (dt_cmd > cmd_timeout_s_) {
        publish_disable_all(joint_order);
        return;
      }
    }

    // Held joints retain their own source time and monotonic deadline. Traffic
    // for another joint, or a ROS clock pause, cannot renew their lifetime.
    const auto steady_now = std::chrono::steady_clock::now();
    for (const auto& item : command_times_) {
      if (steady_now >= item.second.deadline || this->now().seconds() >= item.second.expiry) {
        publish_disable_all(joint_order);
        return;
      }
    }

    // Build control_core::State from ROS snapshot
    control_core::State s;
    s.joints.reserve(state_map.size());
    for (const auto& kv : state_map) {
      control_core::JointData jd;
      jd.pos = kv.second.pos;
      jd.vel = kv.second.vel;
      jd.eff = kv.second.eff;
      s.joints[kv.first] = jd;
    }

    const double dt = 1.0 / std::max(1.0, control_rate_hz_);

    // Run core (per-joint independent mode + hold-last)
    auto out = core_.step(s, desired_subset, dt);
    if (!out.warnings.empty()) {
      std::string reasons;
      for (const auto& warning : out.warnings) {
        if (!reasons.empty()) reasons += "; ";
        reasons += warning;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), warning_clock_, 5000,
        "SafetyFeature warning (advisory only): %s", reasons.c_str());
    }

    // Publish MotorCommand per joint (no broadcast)
    publish_output(out);
  }

  // -------- publishing helpers --------
  void publish_output(const control_core::Output& out)
  {
    using MC = motor_control_interfaces::msg::MotorCommand;

    MC msg;
    // A single envelope conservatively covers all held per-joint commands.
    // Preserve the oldest source and earliest deadline instead of re-stamping.
    if (command_times_.empty()) return;
    double source = this->now().seconds(), expiry = source + cmd_timeout_s_;
    for (const auto& item : command_times_) {
      source = std::min(source, item.second.source);
      expiry = std::min(expiry, item.second.expiry);
    }
    msg.header.stamp = rclcpp::Time(static_cast<int64_t>(source * 1e9));
    msg.valid_until = rclcpp::Time(static_cast<int64_t>(expiry * 1e9));
    msg.joint_name = out.joint_name;

    const size_t n = out.joint_name.size();
    msg.mode.resize(n, MC::MODE_VELOCITY);

    // Always size arrays to n (receiver reads per index safely)
    msg.position.resize(n, 0.0);
    msg.velocity.resize(n, 0.0);
    msg.acceleration.resize(n, 0.0);
    msg.torque.resize(n, 0.0);
    msg.kp.resize(n, 40.0);
    msg.kd.resize(n, 1.5);

    for (size_t i = 0; i < n && i < out.commands.size(); ++i) {
      const auto& c = out.commands[i];
      msg.mode[i] = to_ros_mode(c.mode);
      msg.position[i] = c.position;
      msg.velocity[i] = c.velocity;
      msg.acceleration[i] = c.acceleration;
      msg.torque[i] = c.torque;
      msg.kp[i] = c.kp;
      msg.kd[i] = c.kd;
    }

    cmd_pub_->publish(msg);
  }

  void publish_disable_all(const std::vector<std::string>& joint_order)
  {
    using MC = motor_control_interfaces::msg::MotorCommand;

    // Recovery must not replay a cached ENABLE or a pre-fault hold-last target.
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      desired_subset_.clear();
      command_times_.clear();
      has_cmd_ = false;
    }
    if (core_configured_) core_.configure(joint_order);

    MC msg;
    msg.header.stamp = this->now();
    msg.joint_name = joint_order;
    msg.mode = {MC::MODE_DISABLE};  // broadcast disable
    cmd_pub_->publish(msg);
  }

private:
  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Subscription<motor_control_interfaces::msg::MotorCommand>::SharedPtr desired_motor_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr desired_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr safety_estop_sub_;

  rclcpp::Publisher<motor_control_interfaces::msg::MotorCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr supervisor_sub_;
  rclcpp::TimerBase::SharedPtr supervisor_warning_timer_;
  rclcpp::Clock warning_clock_{RCL_STEADY_TIME};
  double supervisor_timeout_s_{0.5}, supervisor_grace_s_{3.0};
  bool supervisor_seen_{false};
  double supervisor_source_stamp_{0};
  uint8_t supervisor_level_{0};
  std::string supervisor_message_;
  std::chrono::steady_clock::time_point supervisor_started_{std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point supervisor_received_{};

  // Parameters
  double control_rate_hz_{50.0};
  double state_timeout_s_{0.2};
  bool auto_append_new_joints_{false};
  double cmd_timeout_s_{0.5};

  // Thread-safe state storage
  std::mutex state_mutex_;
  std::unordered_map<std::string, JointDataRT> latest_state_map_;
  std::vector<std::string> joint_order_;

  // Desired subset commands (external input): per-joint Command (mode + fields)
  std::mutex cmd_mutex_;
  control_core::CommandSubset desired_subset_;
  std::unordered_map<std::string, CommandTime> command_times_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};

  // Control core
  control_core::Controller core_;
  bool core_configured_{false};
  std::atomic_bool safety_estop_active_{false};
};

#ifndef MOTOR_CONTROL_HYBRID_NO_MAIN
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppControlNode>();
  // Configure after ROS/DDS initialization so their existing worker threads
  // keep their policies. This thread executes both timer and subscriptions.
  if (node->get_parameter("control_rt_enabled").as_bool()) {
    sched_param param{};
    param.sched_priority = static_cast<int>(node->get_parameter("control_rt_priority").as_int());
    if (sched_setscheduler(0, SCHED_RR, &param) == 0) {
      RCLCPP_INFO(node->get_logger(), "Control executor: SCHED_RR priority=%d", param.sched_priority);
    } else {
      RCLCPP_WARN(node->get_logger(),
        "Cannot set control executor SCHED_RR: %s; continuing with existing scheduling",
        std::strerror(errno));
    }
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
