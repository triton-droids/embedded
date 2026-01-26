#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <motor_control_interfaces/msg/motor_command.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <algorithm>

// New non-ROS control core API (per-joint independent mode)
#include "control_core/api.hpp"


class CppControlNode : public rclcpp::Node
{
public:
  CppControlNode()
  : Node("cpp_control_node")
  {
    // Parameters
    this->declare_parameter<double>("control_rate_hz", 50.0);
    this->declare_parameter<double>("state_timeout_s", 0.2);
    this->declare_parameter<bool>("auto_append_new_joints", false);
    this->declare_parameter<double>("cmd_timeout_s", 0.5);

    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    auto_append_new_joints_ = this->get_parameter("auto_append_new_joints").as_bool();
    cmd_timeout_s_   = this->get_parameter("cmd_timeout_s").as_double();

    // Prefer absolute topic names to avoid namespace surprises
    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CppControlNode::state_callback, this, std::placeholders::_1)
    );

    // Recommended external command interface: MotorCommand subset
    desired_motor_sub_ = this->create_subscription<motor_control_interfaces::msg::MotorCommand>(
      "/desired_motor_subset", 10,
      std::bind(&CppControlNode::desired_motor_callback, this, std::placeholders::_1)
    );

    // Backward-compatible: velocity subset via JointState
    desired_vel_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/desired_velocity_subset", 10,
      std::bind(&CppControlNode::desired_velocity_callback, this, std::placeholders::_1)
    );

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
  struct JointDataRT {
    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
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
    last_state_time_ = this->now();

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

  // Preferred: full per-joint MotorCommand subset
  void desired_motor_callback(const motor_control_interfaces::msg::MotorCommand::SharedPtr msg)
  {
    if (!msg || msg->joint_name.empty()) return;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
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
    }
  }

  // Backward-compatible: velocity subset via JointState
  void desired_velocity_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty()) return;

    std::lock_guard<std::mutex> lock(cmd_mutex_);
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
      }
    }
  }

  // -------- control loop --------
  void control_loop()
  {
    // Snapshot state
    std::vector<std::string> joint_order;
    std::unordered_map<std::string, JointDataRT> state_map;
    rclcpp::Time last_state_time;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joint_order = joint_order_;
      state_map = latest_state_map_;
      last_state_time = last_state_time_;
    }

    if (joint_order.empty()) return;

    if (!core_configured_) {
      // Fail-safe: publish DISABLE for all joints (or zero velocity if you prefer)
      publish_disable_all(joint_order);
      return;
    }

    // State watchdog
    if (state_timeout_s_ > 0.0) {
      const double dt_state = (this->now() - last_state_time).seconds();
      if (dt_state > state_timeout_s_) {
        publish_disable_all(joint_order);
        return;
      }
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

    // Command watchdog: if stale, clear subset => control_core will hold-last
    if (cmd_timeout_s_ > 0.0 && has_cmd) {
      const double dt_cmd = (this->now() - last_cmd_time).seconds();
      if (dt_cmd > cmd_timeout_s_) {
        desired_subset.clear();
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

    // Publish MotorCommand per joint (no broadcast)
    publish_output(out);
  }

  // -------- publishing helpers --------
  void publish_output(const control_core::Output& out)
  {
    using MC = motor_control_interfaces::msg::MotorCommand;

    MC msg;
    msg.header.stamp = this->now();
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

  rclcpp::Publisher<motor_control_interfaces::msg::MotorCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  double control_rate_hz_{50.0};
  double state_timeout_s_{0.2};
  bool auto_append_new_joints_{false};
  double cmd_timeout_s_{0.5};

  // Thread-safe state storage
  std::mutex state_mutex_;
  std::unordered_map<std::string, JointDataRT> latest_state_map_;
  std::vector<std::string> joint_order_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  // Desired subset commands (external input): per-joint Command (mode + fields)
  std::mutex cmd_mutex_;
  control_core::CommandSubset desired_subset_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  bool has_cmd_{false};

  // Control core
  control_core::Controller core_;
  bool core_configured_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CppControlNode>());
  rclcpp::shutdown();
  return 0;
}
