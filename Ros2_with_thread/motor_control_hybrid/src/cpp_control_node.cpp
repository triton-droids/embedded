#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <memory>
#include <algorithm>

/**
 * C++ Control Node (Order-Safe)
 *
 * Purpose:
 * - Subscribe to /joint_states (from Python CAN node)
 * - Run a fixed-rate control loop (default 50 Hz)
 * - Publish /joint_commands (to Python CAN node)
 *
 * Key fix vs. the original:
 * - Do NOT rely on JointState array order being stable.
 * - Build a name->state map, and use a locked joint order for consistent action mapping.
 *
 * Notes about Python CAN node compatibility:
 * - Your Python _cmd_callback uses a heuristic:
 *     if position[i] != 0.0 -> position control; else velocity control
 * - To force velocity control robustly, this node publishes ONLY velocity[] and leaves position[] empty.
 */
class CppControlNode : public rclcpp::Node
{
public:
  CppControlNode()
  : Node("cpp_control_node")
  {
    // Parameters
    this->declare_parameter<double>("control_rate_hz", 50.0);
    this->declare_parameter<bool>("enable_rl", false);
    this->declare_parameter<std::string>("rl_model_path", "");
    this->declare_parameter<double>("state_timeout_s", 0.2);   // Safety: if no state for this long, command zeros
    this->declare_parameter<bool>("auto_append_new_joints", false); // Option: if new joints appear later, append to order

    control_rate_hz_ = this->get_parameter("control_rate_hz").as_double();
    enable_rl_       = this->get_parameter("enable_rl").as_bool();
    rl_model_path_   = this->get_parameter("rl_model_path").as_string();
    state_timeout_s_ = this->get_parameter("state_timeout_s").as_double();
    auto_append_new_joints_ = this->get_parameter("auto_append_new_joints").as_bool();

    // Subscriber: joint states from Python CAN node
    state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&CppControlNode::state_callback, this, std::placeholders::_1)
    );

    // Publisher: joint commands to Python CAN node
    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_commands", 10);

    // Initialize RL policy if enabled (placeholder)
    if (enable_rl_ && !rl_model_path_.empty()) {
      RCLCPP_INFO(this->get_logger(), "RL policy enabled (placeholder). Path: %s", rl_model_path_.c_str());
      // TODO: Load model (e.g., ONNX Runtime) and set rl_policy_
    }

    // Control timer at control_rate_hz
    const double rate = std::max(1.0, control_rate_hz_);
    auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(1.0 / rate)
    );

    control_timer_ = this->create_wall_timer(
      period,
      std::bind(&CppControlNode::control_loop, this)
    );

    RCLCPP_INFO(
      this->get_logger(),
      "CppControlNode started (rate=%.1f Hz, RL=%s, timeout=%.3fs)",
      control_rate_hz_, enable_rl_ ? "enabled" : "disabled", state_timeout_s_
    );
  }

private:
  struct JointData {
    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
  };

  void state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!msg || msg->name.empty()) return;

    std::lock_guard<std::mutex> lock(state_mutex_);

    last_state_time_ = this->now();

    // Lock in a stable joint order on first message to ensure action alignment.
    if (joint_order_.empty()) {
      joint_order_ = msg->name;
      RCLCPP_INFO(this->get_logger(), "Joint order locked from first /joint_states (%zu joints).",
                  joint_order_.size());
    }

    // Update map by name (order-independent).
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string &jn = msg->name[i];
      JointData &jd = latest_state_map_[jn];

      if (i < msg->position.size()) jd.pos = msg->position[i];
      if (i < msg->velocity.size()) jd.vel = msg->velocity[i];
      if (i < msg->effort.size())   jd.eff = msg->effort[i];

      // Optionally append any new joints that were not in the initial locked order.
      if (auto_append_new_joints_) {
        if (std::find(joint_order_.begin(), joint_order_.end(), jn) == joint_order_.end()) {
          joint_order_.push_back(jn);
          RCLCPP_WARN(this->get_logger(), "New joint discovered and appended to order: %s", jn.c_str());
        }
      }
    }
  }

  void control_loop()
  {
    // Copy state atomically to avoid holding the mutex during computation.
    std::vector<std::string> joint_order;
    std::unordered_map<std::string, JointData> state_map;
    rclcpp::Time last_time;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      joint_order = joint_order_;
      state_map = latest_state_map_;
      last_time = last_state_time_;
    }

    if (joint_order.empty()) {
      // No state received yet.
      return;
    }

    // Safety: if state is stale, publish zero-velocity command.
    if (state_timeout_s_ > 0.0) {
      const double dt = (this->now() - last_time).seconds();
      if (dt > state_timeout_s_) {
        publish_velocity_command(joint_order, std::vector<float>(joint_order.size(), 0.0f));
        return;
      }
    }

    // Build observation vector in stable joint order: [pos..., vel..., eff...] per joint (pos, vel, eff interleaved).
    std::vector<float> observation;
    observation.reserve(joint_order.size() * 3);

    for (const auto &jn : joint_order) {
      JointData jd;
      auto it = state_map.find(jn);
      if (it != state_map.end()) jd = it->second;
      // If missing, use zeros (or you can choose to hold last value / use NaN).
      observation.push_back(static_cast<float>(jd.pos));
      observation.push_back(static_cast<float>(jd.vel));
      observation.push_back(static_cast<float>(jd.eff));
    }

    // Compute actions aligned with joint_order.
    std::vector<float> actions;
    if (rl_policy_ != nullptr) {
      // TODO: actions = rl_policy_->infer(observation);
      actions = compute_simple_control(observation, joint_order.size()); // Fallback
    } else {
      actions = compute_simple_control(observation, joint_order.size());
    }

    // Publish velocity-only commands (position[] left empty to force Python side to interpret as velocity control).
    publish_velocity_command(joint_order, actions);
  }

  std::vector<float> compute_simple_control(const std::vector<float>& /*observation*/, size_t num_joints)
  {
    // Placeholder controller: command zero velocity on all joints.
    // Replace with your real control logic (PID, impedance, MPC, RL inference, etc.).
    return std::vector<float>(num_joints, 0.0f);
  }

  void publish_velocity_command(const std::vector<std::string>& joint_order,
                                const std::vector<float>& actions)
  {
    sensor_msgs::msg::JointState cmd;
    cmd.header.stamp = this->now();

    cmd.name = joint_order;

    // Velocity array aligned with cmd.name
    cmd.velocity.assign(joint_order.size(), 0.0);
    for (size_t i = 0; i < joint_order.size() && i < actions.size(); ++i) {
      cmd.velocity[i] = static_cast<double>(actions[i]);
    }

    // IMPORTANT:
    // Leave cmd.position empty to avoid Python heuristic switching to position mode.
    // cmd.effort can be filled if you want to pass torque, but your Python currently treats msg.effort as torque.
    // cmd.effort.assign(...);

    cmd_pub_->publish(cmd);
  }

private:
  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters
  double control_rate_hz_{50.0};
  bool enable_rl_{false};
  std::string rl_model_path_;
  double state_timeout_s_{0.2};
  bool auto_append_new_joints_{false};

  // Thread-safe state storage
  std::mutex state_mutex_;
  std::unordered_map<std::string, JointData> latest_state_map_;
  std::vector<std::string> joint_order_;
  rclcpp::Time last_state_time_{0, 0, RCL_ROS_TIME};

  // RL policy placeholder
  std::shared_ptr<void> rl_policy_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CppControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
