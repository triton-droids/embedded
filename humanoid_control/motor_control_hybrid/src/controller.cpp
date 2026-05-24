#include "control_core/controller.hpp"

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
    const auto& name = joint_order_[i];
    if (name.empty()) {
      throw std::runtime_error("Controller::configure: joint name is empty");
    }
    // If duplicates exist, later one overwrites earlier mapping; usually you want unique names.
    name_to_idx_[name] = i;
  }

  last_vel_cmd_.assign(joint_order_.size(), 0.0);
  configured_ = true;
}

void Controller::reset_hold(double value) {
  if (!configured_) return;
  last_vel_cmd_.assign(joint_order_.size(), value);
}

CommandOut Controller::step_velocity_subset(
    const JointStateIn& /*state*/,
    const std::unordered_map<std::string, double>& desired_vel_by_name,
    double /*dt*/) {

  if (!configured_) {
    throw std::runtime_error("Controller::step_velocity_subset: controller not configured");
  }

  CommandOut out;
  out.name = joint_order_;

  // Default: hold last velocity for all joints
  out.velocity = last_vel_cmd_;

  // Override only joints specified by the caller
  for (const auto& kv : desired_vel_by_name) {
    const std::string& joint = kv.first;
    const double v = kv.second;

    auto it = name_to_idx_.find(joint);
    if (it == name_to_idx_.end()) {
      // Unknown joint name: ignore silently (keeps hold-last)
      continue;
    }
    out.velocity[it->second] = v;
  }

  // Safety layer placeholder (requested "leave blank")
  out.ok = true;
  out.reason.clear();

  // Update hold-last memory
  last_vel_cmd_ = out.velocity;

  return out;
}

CommandOut Controller::step_velocity_subset(
    const JointStateIn& state,
    const std::vector<std::string>& subset_names,
    const std::vector<double>& subset_vel,
    double dt) {

  if (subset_names.size() != subset_vel.size()) {
    throw std::runtime_error("Controller::step_velocity_subset: subset_names and subset_vel size mismatch");
  }

  std::unordered_map<std::string, double> desired;
  desired.reserve(subset_names.size());
  for (size_t i = 0; i < subset_names.size(); ++i) {
    desired[subset_names[i]] = subset_vel[i];
  }

  return step_velocity_subset(state, desired, dt);
}

} // namespace control_core
