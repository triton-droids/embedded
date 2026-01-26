#include "control_core/api.hpp"
#include <stdexcept>

namespace control_core {

void Controller::configure(const std::vector<std::string>& joint_order) {
  if (joint_order.empty()) {
    throw std::runtime_error("Controller::configure: joint_order is empty");
  }

  joint_order_ = joint_order;
  name_to_idx_.clear();
  name_to_idx_.reserve(joint_order_.size());

  for (size_t i = 0; i < joint_order_.size(); ++i) {
    const auto& jn = joint_order_[i];
    if (jn.empty()) {
      throw std::runtime_error("Controller::configure: empty joint name");
    }
    name_to_idx_[jn] = i;
  }

  // Initialize hold-last to zero velocity for all joints
  last_cmd_.assign(joint_order_.size(), Command{});
  configured_ = true;
}

void Controller::add_safety(std::shared_ptr<SafetyFeature> f) {
  if (f) safety_.push_back(std::move(f));
}

void Controller::reset_hold(double value) {
  if (!configured_) return;

  for (auto& c : last_cmd_) {
    c.mode = Mode::Velocity;
    c.velocity = value;

    // Reset the rest to defaults
    c.position = 0.0;
    c.acceleration = 0.0;
    c.torque = 0.0;
    c.kp = 40.0;
    c.kd = 1.5;
  }
}

Output Controller::step(const State& state, const CommandSubset& desired_subset, double /*dt*/) {
  if (!configured_) {
    throw std::runtime_error("Controller::step: controller not configured");
  }

  // Safety placeholder: if any safety fails, keep hold-last unchanged.
  for (auto& s : safety_) {
    std::string reason;
    if (!s->check(state, &reason)) {
      Output out;
      out.joint_name = joint_order_;
      out.commands = last_cmd_;
      return out;
    }
  }

  // Start from hold-last commands
  Output out;
  out.joint_name = joint_order_;
  out.commands = last_cmd_;

  // Override only specified joints (independent per joint mode)
  for (const auto& kv : desired_subset) {
    const std::string& jn = kv.first;
    const Command& cmd = kv.second;

    auto it = name_to_idx_.find(jn);
    if (it == name_to_idx_.end()) {
      // Unknown joint => ignore
      continue;
    }
    out.commands[it->second] = cmd;
  }

  // Update hold-last memory
  last_cmd_ = out.commands;
  return out;
}

} // namespace control_core
