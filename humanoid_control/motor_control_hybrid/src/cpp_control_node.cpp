#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <motor_control_interfaces/msg/motor_command.hpp>
#include <motor_control_interfaces/msg/abstract_command.hpp>
#include <motor_control_interfaces/msg/abstract_command_status.hpp>
#include <motor_control_interfaces/srv/submit_abstract_command.hpp>
#include <motor_control_interfaces/srv/execute_motor_operation.hpp>
#include <motor_control_interfaces/srv/get_motor_feedback.hpp>

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
#include "control_core/operation.hpp"


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

    // Explicit commissioning switch; ordinary launch files leave this false.
    home_lower_ = declare_parameter<std::vector<double>>("home_joint_lower", std::vector<double>{});
    home_upper_ = declare_parameter<std::vector<double>>("home_joint_upper", std::vector<double>{});
    simulation_zero_ = declare_parameter<bool>("simulation_zero_enabled", false);
    actions_enabled_ = declare_parameter<bool>("abstract_actions_enabled", false) || simulation_zero_;
    if (simulation_zero_) RCLCPP_WARN(get_logger(), "SIMULATION zero enabled: using only /simulation/motor_backend services");
    operation_limits_.acceleration = declare_parameter<double>("zero_stop_acceleration", 0.2);
    operation_limits_.velocity = declare_parameter<double>("zero_admission_velocity", 0.2);
    operation_limits_.max_effort = declare_parameter<double>("zero_measured_effort_trip", 1.0);
    operation_limits_.kp = declare_parameter<double>("zero_stop_kp", 2.0);
    operation_limits_.kd = declare_parameter<double>("zero_stop_kd", 0.2);
    operation_limits_.settle_s = declare_parameter<double>("zero_settle_s", 0.3);
    operation_limits_.timeout_s = declare_parameter<double>("zero_timeout_s", 10.0);
    backend_client_ = create_client<BackendOperation>(simulation_zero_ ? "/simulation/motor_backend/execute_operation" : "/motor_backend/execute_operation");
    feedback_client_ = create_client<FeedbackService>(simulation_zero_ ? "/simulation/motor_backend/get_feedback" : "/motor_backend/get_feedback");
    operation_status_pub_ = create_publisher<ActionStatus>(
      "/control_core/abstract_command_status", rclcpp::QoS(1).reliable().transient_local());
    action_service_ = create_service<ActionService>("/control_core/submit_abstract_command",
      [this](const ActionService::Request::SharedPtr req, ActionService::Response::SharedPtr res) {
        res->accepted = submit_action(req->command, res->message);
        res->status = res->accepted ? action_status() : rejected_status(req->command, res->message);
      });
    action_sub_ = create_subscription<Action>("/control_core/abstract_command", 10,
      [this](const Action::SharedPtr msg) {
        std::string reason;
        if (!submit_action(*msg, reason)) {
          RCLCPP_WARN(get_logger(), "Action rejected: %s", reason.c_str());
          operation_status_pub_->publish(rejected_status(*msg, reason));
        }
      });
    feedback_timer_ = create_wall_timer(std::chrono::milliseconds(50),
      std::bind(&CppControlNode::poll_management_feedback, this));
    publish_action_status();

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
  using Action = motor_control_interfaces::msg::AbstractCommand;
  using ActionStatus = motor_control_interfaces::msg::AbstractCommandStatus;
  using ActionService = motor_control_interfaces::srv::SubmitAbstractCommand;
  using BackendOperation = motor_control_interfaces::srv::ExecuteMotorOperation;
  using FeedbackService = motor_control_interfaces::srv::GetMotorFeedback;
  using Steady = std::chrono::steady_clock;

  struct JointDataRT {
    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
    bool effort_known{false};
    std::chrono::steady_clock::time_point received_at{};
    double source_stamp{0.0};
  };

  struct CommandTime {
    double source;
    double expiry;
    std::chrono::steady_clock::time_point deadline;
  };

  ActionStatus action_status() const {
    if (home_status_valid_) {auto msg = home_status_; msg.header.stamp = now(); return msg;}
    const auto& status = zero_.status();
    ActionStatus msg; msg.header.stamp = now(); msg.request_id = status.request_id;
    msg.joint_name = status.joint; msg.stage = static_cast<uint8_t>(status.stage);
    msg.message = status.message; msg.zero_may_have_changed = status.zero_may_have_changed;
    msg.calibration_epoch = status.calibration_epoch;
    if (simulation_zero_) msg.message = "SIMULATION: " + msg.message;
    return msg;
  }
  void publish_action_status() {operation_status_pub_->publish(action_status());}
  ActionStatus rejected_status(const Action& action, const std::string& reason) const {
    ActionStatus msg; msg.header.stamp = now(); msg.request_id = action.request_id;
    msg.joint_name = action.joint_name; msg.stage = ActionStatus::REJECTED;
    msg.message = reason; msg.calibration_epoch = zero_.status().calibration_epoch; return msg;
  }
  void poll_management_feedback() {
    if (!actions_enabled_ && !lifecycle_active_ && idle_enabled_.empty()) return;
    const auto current = Steady::now();
    for (auto it = feedback_pending_.begin(); it != feedback_pending_.end();) {
      if (current >= it->second.second) {
        feedback_client_->remove_pending_request(it->second.first);
        it = feedback_pending_.erase(it);
      } else ++it;
    }
    if (!feedback_client_->service_is_ready()) return;
    for (const auto& name : joint_order_) {
      if (feedback_pending_.count(name)) continue;
      auto req = std::make_shared<FeedbackService::Request>(); req->joint_name = name;
      auto future = feedback_client_->async_send_request(req, [this, name](rclcpp::Client<FeedbackService>::SharedFuture result) {
        feedback_pending_.erase(name);
        try {
          const auto response = result.get(); const auto& f = response->feedback;
          const double age = (now() - rclcpp::Time(f.stamp)).seconds();
          const double freshness = simulation_zero_ ? 0.2 : 0.1;
          if (!response->success || !f.available || !f.fresh || f.joint_name != name ||
              f.fault_bits || !std::isfinite(f.age_s) || f.age_s < 0 || f.age_s > freshness ||
              age < -0.01 || age > freshness || (f.status_mode != 0 && f.status_mode != 2)) {
            enabled_evidence_.erase(name); return;
          }
          enabled_evidence_[name] = {f.status_mode == 2, Steady::now() +
            std::chrono::duration_cast<Steady::duration>(std::chrono::duration<double>(freshness - std::max(age, f.age_s)))};
        } catch (const std::exception&) {enabled_evidence_.erase(name);}
      });
      feedback_pending_[name] = {future.request_id, current + std::chrono::milliseconds(200)};
    }
  }
  bool submit_action(const Action& action, std::string& reason) {
    CommandTime timing;
    if (!command_time(action.header.stamp, action.valid_until, timing)) {
      reason = "Expired, future or unstamped abstract command"; return false;
    }
    if (action.operation == Action::ENABLE || action.operation == Action::DISABLE)
      return begin_lifecycle(action, reason);
    if (lifecycle_active_) {reason = "Enable/disable verification pending"; return false;}
    if (action.operation == Action::HOME) return begin_home(action, reason);
    if (home_active_) {reason = "Home owns control; wait for completion or assert estop"; return false;}
    if (action.operation == Action::CANCEL) {
      if (action.request_id != zero_.status().request_id || !zero_.blocks_normal()) {
        reason = "No matching operation to cancel"; return false;
      }
      zero_.abort("Cancelled; remain disabled", true); reason = "Cancelled";
      publish_action_status(); return true;
    }
    if (action.operation == Action::RESET) {
      if (action.request_id != zero_.status().request_id || safety_estop_active_.load()) {
        reason = "Reset requires matching request ID and released estop"; return false;
      }
      const bool accepted = zero_.reset(reason);
      if (accepted) {minimum_command_source_ = now().seconds(); publish_disable_all(joint_order_);}
      publish_action_status(); return accepted;
    }
    if (!actions_enabled_) {reason = "Hardware Set Zero unavailable: commissioning and firmware/readback verification are not implemented. Use fake mode for simulated zero"; return false;}
    if (action.operation != Action::SET_MECHANICAL_ZERO) {reason = "Unsupported abstract operation"; return false;}
    if (!core_configured_ || safety_estop_active_.load() || !state_is_fresh(joint_order_, latest_state_map_)) {
      reason = "Estop, unconfigured core or stale joint feedback"; return false;
    }
    if (!backend_client_->service_is_ready()) {reason = "Motor backend service unavailable"; return false;}
    control_core::State state;
    std::unordered_map<std::string, bool> enabled;
    for (const auto& name : joint_order_) {
      const auto evidence = enabled_evidence_.find(name);
      if (evidence == enabled_evidence_.end() || Steady::now() >= evidence->second.second) {
        reason = "Fresh raw enabled-state evidence required for every joint"; return false;
      }
      enabled[name] = evidence->second.first;
      const auto& jd = latest_state_map_.at(name);
      if (!jd.effort_known) {reason = "Measured effort required for action admission"; return false;}
      state.joints[name] = {jd.pos, jd.vel, jd.eff};
    }
    home_status_valid_ = false;
    if (!zero_.begin(action.request_id, action.joint_name, state, joint_order_, enabled, operation_limits_, reason)) return false;
    idle_enabled_.clear();
    // No pre-zero policy/manual target survives admission or operation reset.
    desired_subset_.clear(); command_times_.clear(); has_cmd_ = false;
    minimum_command_source_ = now().seconds(); action_tick_ = Steady::now();
    publish_action_status(); return true;
  }
  void publish_lifecycle(const std::vector<std::string>& names, uint8_t mode) {
    if (names.empty()) return;
    motor_control_interfaces::msg::MotorCommand msg;
    msg.header.stamp = now(); msg.valid_until = now() + rclcpp::Duration::from_seconds(0.3);
    msg.joint_name = names; msg.mode = {mode};
    msg.position.assign(names.size(), 0); msg.velocity.assign(names.size(), 0);
    msg.torque.assign(names.size(), 0); msg.kp.assign(names.size(), 0); msg.kd.assign(names.size(), 0);
    // Idle enable sends zero-gain/zero-feedforward MIT frames, never an old pose target.
    cmd_pub_->publish(msg);
  }
  bool begin_lifecycle(const Action& action, std::string& reason) {
    const bool enable = action.operation == Action::ENABLE;
    const auto selected = action.joint_name == "*" ? joint_order_ : std::vector<std::string>{action.joint_name};
    if (action.request_id.empty() || action.request_id.size() > 128 || selected.empty()) {
      reason = "Invalid lifecycle request"; return false;
    }
    for (const auto& name : selected) if (std::find(joint_order_.begin(), joint_order_.end(), name) == joint_order_.end()) {
      reason = "Unknown lifecycle joint"; return false;
    }
    if (enable && (lifecycle_active_ || home_active_ || zero_.blocks_normal() || !core_configured_ ||
        safety_estop_active_.load() || !supervisor_seen_ || !supervisor_warning_reason().empty() ||
        !state_is_fresh(joint_order_, latest_state_map_))) {
      reason = "Enable requires fresh complete feedback, healthy released safety and no active operation"; return false;
    }
    if (!enable && (home_active_ || zero_.blocks_normal() || lifecycle_active_)) publish_disable_all(joint_order_);
    desired_subset_.clear(); command_times_.clear(); has_cmd_ = false;
    minimum_command_source_ = now().seconds();
    if (core_configured_) core_.configure(joint_order_);
    lifecycle_joints_ = selected; lifecycle_enable_ = enable; lifecycle_active_ = true;
    lifecycle_deadline_ = Steady::now() + std::chrono::seconds(2);
    home_status_ = ActionStatus{}; home_status_valid_ = true;
    home_status_.request_id = action.request_id; home_status_.joint_name = action.joint_name;
    home_status_.stage = ActionStatus::WAITING_BACKEND;
    home_status_.zero_may_have_changed = zero_.status().zero_may_have_changed;
    home_status_.message = enable ? "Enable submitted; waiting for fresh enabled-state feedback" : "Disable submitted; waiting for fresh disabled-state feedback";
    for (const auto& name : selected) {
      if (enable) idle_enabled_.insert(name); else idle_enabled_.erase(name);
      enabled_evidence_.erase(name); // A pre-command sample cannot verify this request.
    }
    publish_lifecycle(selected, enable ? motor_control_interfaces::msg::MotorCommand::MODE_ENABLE : motor_control_interfaces::msg::MotorCommand::MODE_DISABLE);
    reason = home_status_.message; publish_action_status(); return true;
  }
  void lifecycle_step() {
    if (!lifecycle_active_) return;
    bool verified = true;
    for (const auto& name : lifecycle_joints_) {
      const auto it = enabled_evidence_.find(name);
      verified = verified && it != enabled_evidence_.end() && Steady::now() < it->second.second && it->second.first == lifecycle_enable_;
    }
    if (verified) {
      lifecycle_active_ = false; home_status_.stage = ActionStatus::COMPLETED;
      home_status_.message = lifecycle_enable_ ? "Fresh feedback verifies enabled; idle commands have zero gains and feedforward" : "Fresh feedback verifies disabled";
      publish_action_status();
    } else if (Steady::now() >= lifecycle_deadline_) {
      lifecycle_active_ = false;
      for (const auto& name : lifecycle_joints_) idle_enabled_.erase(name);
      publish_lifecycle(lifecycle_joints_, motor_control_interfaces::msg::MotorCommand::MODE_DISABLE);
      home_status_.stage = ActionStatus::FAILED;
      home_status_.message = "Enable/disable feedback timed out; disable submitted, physical state unverified";
      publish_action_status();
    }
  }
  void drive_idle_enabled() {
    if (idle_enabled_.empty()) return;
    if (!supervisor_seen_ || !supervisor_warning_reason().empty()) {publish_disable_all(joint_order_); return;}
    publish_lifecycle(std::vector<std::string>(idle_enabled_.begin(), idle_enabled_.end()), motor_control_interfaces::msg::MotorCommand::MODE_MOTION);
  }
  bool begin_home(const Action& action, std::string& reason) {
    if (home_status_valid_ && action.request_id == home_status_.request_id) {reason = "Duplicate Home request ID"; return false;}
    if (home_active_ || zero_.blocks_normal()) {reason = "Another operation owns control"; return false;}
    if (action.request_id.empty() || action.request_id.size() > 128 ||
        !core_configured_ || safety_estop_active_.load() || !supervisor_seen_ ||
        !supervisor_warning_reason().empty() ||
        !state_is_fresh(joint_order_, latest_state_map_)) {
      reason = "Invalid request, estop or stale/incomplete joint feedback"; return false;
    }
    const std::vector<std::string> selected = action.joint_name == "*" ? joint_order_ : std::vector<std::string>{action.joint_name};
    if (home_lower_.size() != joint_order_.size() || home_upper_.size() != joint_order_.size()) {
      reason = "Missing Home limits"; return false;
    }
    for (const auto& name : selected) {
      const auto it = std::find(joint_order_.begin(), joint_order_.end(), name);
      if (it == joint_order_.end()) {reason = "Unknown Home joint"; return false;}
      const size_t i = std::distance(joint_order_.begin(), it);
      const auto& state = latest_state_map_.at(name);
      if (!std::isfinite(home_lower_[i]) || !std::isfinite(home_upper_[i]) ||
          home_lower_[i] > 0 || home_upper_[i] < 0 || home_lower_[i] >= home_upper_[i] ||
          state.pos < home_lower_[i] || state.pos > home_upper_[i] || std::abs(state.vel) > 0.05) {
        reason = "Home needs settled joints and finite limits containing current positions and zero"; return false;
      }
    }
    idle_enabled_.clear();
    home_targets_.clear();
    for (const auto& name : selected) home_targets_[name] = latest_state_map_.at(name).pos;
    home_status_ = ActionStatus{};
    home_status_.request_id = action.request_id; home_status_.joint_name = action.joint_name;
    home_status_.stage = ActionStatus::RESUMING;
    home_status_.message = "Moving to current coordinate zero; no encoder write or automatic enable";
    home_status_valid_ = home_active_ = true;
    home_start_ = home_tick_ = Steady::now(); home_settle_ = home_start_;
    desired_subset_.clear(); command_times_.clear(); has_cmd_ = false;
    minimum_command_source_ = now().seconds();
    reason = "Home admitted; completion requires fresh zero-position feedback";
    publish_action_status(); return true;
  }
  void step_home(const std::unordered_map<std::string, JointDataRT>& state) {
    const auto current = Steady::now();
    if (!supervisor_warning_reason().empty()) {publish_disable_all(joint_order_); return;}
    if (std::chrono::duration<double>(current - home_start_).count() > 30.0) {
      publish_disable_all(joint_order_); return;
    }
    const double dt = std::min(0.05, std::chrono::duration<double>(current - home_tick_).count());
    home_tick_ = current;
    bool settled = true;
    for (auto& item : home_targets_) {
      const auto& measured = state.at(item.first);
      if (std::abs(item.second - measured.pos) > 0.2) {publish_disable_all(joint_order_); return;}
      item.second += std::clamp(-item.second, -0.1 * dt, 0.1 * dt);
      settled = settled && std::abs(measured.pos) <= 0.01 && std::abs(measured.vel) <= 0.02 && std::abs(item.second) <= 1e-6;
    }
    if (!settled) home_settle_ = current;
    if (std::chrono::duration<double>(current - home_settle_).count() >= 0.3) {
      home_active_ = false;
      minimum_command_source_ = now().seconds();
      home_status_.stage = ActionStatus::COMPLETED;
      home_status_.message = "Verified current coordinate zero; outputs disabled";
      publish_disable_all(joint_order_); publish_action_status(); return;
    }
    control_core::Output output; output.joint_name = joint_order_;
    for (const auto& name : joint_order_) {
      control_core::Command cmd; cmd.mode = control_core::Mode::Disable;
      if (home_targets_.count(name)) {
        cmd.mode = control_core::Mode::Position; cmd.position = home_targets_.at(name);
        cmd.velocity = 0.1; cmd.kp = 2.0; cmd.kd = 0.2;
      }
      output.commands.push_back(cmd);
    }
    publish_output(output, true); publish_action_status();
  }
  void dispatch_zero() {
    if (!zero_.take_backend_request()) return;
    const auto id = zero_.status().request_id;
    if (!backend_client_->service_is_ready()) {
      zero_.backend_reply(id, false, "Backend unavailable before submission"); return;
    }
    auto req = std::make_shared<BackendOperation::Request>();
    req->request_id = id; req->joint_name = zero_.status().joint;
    req->operation = BackendOperation::Request::SET_MECHANICAL_ZERO;
    try {
      auto future = backend_client_->async_send_request(req, [this, id](rclcpp::Client<BackendOperation>::SharedFuture result) {
        backend_pending_id_ = 0;
        try {
          const auto reply = result.get();
          if (reply->request_id != id ||
              (reply->accepted && reply->state != BackendOperation::Response::ACCEPTED) ||
              (!reply->accepted && reply->state != BackendOperation::Response::REJECTED)) {
            zero_.abort("Invalid backend acknowledgement; outcome unknown");
          } else {
            zero_.backend_reply(id, reply->accepted, reply->message);
            if (simulation_zero_ && reply->accepted) simulated_zero_reply_stamp_ = now().seconds();
          }
        } catch (const std::exception& error) {zero_.abort(std::string("Unknown backend outcome: ") + error.what());}
        publish_action_status();
      });
      backend_pending_id_ = future.request_id;
    } catch (const std::exception& error) {zero_.abort(std::string("Backend submission outcome unknown: ") + error.what());}
  }



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
      jd.effort_known = i < msg->effort.size();
      if (jd.effort_known) jd.eff = msg->effort[i];
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
    if (home_active_ || lifecycle_active_) {
      if (std::find(msg->mode.begin(), msg->mode.end(), motor_control_interfaces::msg::MotorCommand::MODE_DISABLE) != msg->mode.end())
        publish_disable_all(joint_order_);
      return;
    }
    if (zero_.blocks_normal()) {
      if (std::find(msg->mode.begin(), msg->mode.end(), motor_control_interfaces::msg::MotorCommand::MODE_DISABLE) != msg->mode.end())
        zero_.abort("External disable during operation");
      return;
    }

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
    if (!disable_only && timing.source <= minimum_command_source_) return;
    for (const auto& name : msg->joint_name) idle_enabled_.erase(name);
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
    if (!msg || msg->name.empty() || zero_.blocks_normal() || home_active_ || lifecycle_active_) return;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
    CommandTime timing;
    if (!command_time(msg->header.stamp, builtin_interfaces::msg::Time{}, timing)) return;
    if (timing.source <= minimum_command_source_) return;
    for (const auto& name : msg->name) idle_enabled_.erase(name);
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

    lifecycle_step();
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

    if (home_active_) {step_home(state_map); return;}

    if (zero_.blocks_normal()) {
      control_core::State state;
      if (joint_order != zero_.joint_order()) {
        publish_disable_all(joint_order); return;
      }
      for (const auto& item : state_map) {
        if (!item.second.effort_known) zero_.abort("Effort feedback unavailable during operation");
        state.joints[item.first] = {item.second.pos, item.second.vel, item.second.eff};
      }
      const auto current = Steady::now();
      const double dt = std::chrono::duration<double>(current - action_tick_).count();
      action_tick_ = current;
      if (simulation_zero_ && zero_.status().stage == control_core::OperationStage::WaitingRebind &&
          simulated_zero_reply_stamp_ > 0) {
        const auto& selected = state_map.at(zero_.status().joint);
        bool new_samples = true;
        for (const auto& item : state_map) new_samples = new_samples && item.second.source_stamp > simulated_zero_reply_stamp_;
        if (new_samples && std::abs(selected.pos) <= 0.001 && std::abs(selected.vel) <= 0.01) {
          zero_.confirm_rebind(zero_.status().request_id, zero_.status().calibration_epoch + 1, state);
          simulated_zero_reply_stamp_ = 0;
          minimum_command_source_ = now().seconds();
          desired_subset_.clear(); command_times_.clear(); has_cmd_ = false;
        }
      }
      const auto output = zero_.step(state, dt);
      publish_output(output, true);
      dispatch_zero();
      const auto stage = zero_.status().stage;
      if ((stage == control_core::OperationStage::Failed || stage == control_core::OperationStage::Cancelled) && backend_pending_id_) {
        backend_client_->remove_pending_request(backend_pending_id_); backend_pending_id_ = 0;
      }
      publish_action_status(); return;
    }

    drive_idle_enabled();

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
    for (size_t i = 0; i < out.joint_name.size(); ++i) if (idle_enabled_.count(out.joint_name[i])) {
      auto& cmd = out.commands[i]; cmd = control_core::Command{};
      cmd.mode = control_core::Mode::Motion; cmd.kp = cmd.kd = 0;
    }
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
  void publish_output(const control_core::Output& out, bool operation_owned = false)
  {
    using MC = motor_control_interfaces::msg::MotorCommand;

    MC msg;
    // A single envelope conservatively covers all held per-joint commands.
    // Preserve the oldest source and earliest deadline instead of re-stamping.
    if (!operation_owned && command_times_.empty()) return;
    double source = this->now().seconds(), expiry = source + (operation_owned ? std::min(0.1, cmd_timeout_s_) : cmd_timeout_s_);
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

    idle_enabled_.clear();
    if (lifecycle_active_) {
      lifecycle_active_ = false; home_status_.stage = ActionStatus::FAILED;
      home_status_.message = "Lifecycle operation interrupted: outputs disabled; inspect safety/feedback";
    }
    if (home_active_) {
      home_active_ = false; home_status_.stage = ActionStatus::FAILED;
      minimum_command_source_ = now().seconds();
      home_status_.message = "Home interrupted by watchdog, disable, tracking error or timeout";
    }
    zero_.abort("Estop or control watchdog: outputs disabled");
    if (backend_pending_id_) {backend_client_->remove_pending_request(backend_pending_id_); backend_pending_id_ = 0;}
    if (operation_status_pub_) publish_action_status();
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
  bool lifecycle_active_{false}, lifecycle_enable_{false};
  std::vector<std::string> lifecycle_joints_;
  std::unordered_set<std::string> idle_enabled_;
  Steady::time_point lifecycle_deadline_;
  bool simulation_zero_{false};
  double simulated_zero_reply_stamp_{0};
  bool home_active_{false}, home_status_valid_{false};
  ActionStatus home_status_;
  std::unordered_map<std::string, double> home_targets_;
  Steady::time_point home_start_, home_tick_, home_settle_;
  std::vector<double> home_lower_, home_upper_;
  control_core::ZeroOperation zero_;
  control_core::OperationLimits operation_limits_;
  bool actions_enabled_{false};
  double minimum_command_source_{0};
  Steady::time_point action_tick_{};
  int64_t backend_pending_id_{0};
  std::unordered_map<std::string, std::pair<bool, Steady::time_point>> enabled_evidence_;
  std::unordered_map<std::string, std::pair<int64_t, Steady::time_point>> feedback_pending_;
  rclcpp::Client<BackendOperation>::SharedPtr backend_client_;
  rclcpp::Client<FeedbackService>::SharedPtr feedback_client_;
  rclcpp::Service<ActionService>::SharedPtr action_service_;
  rclcpp::Subscription<Action>::SharedPtr action_sub_;
  rclcpp::Publisher<ActionStatus>::SharedPtr operation_status_pub_;
  rclcpp::TimerBase::SharedPtr feedback_timer_;

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
