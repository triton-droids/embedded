#pragma once
#include "control_core/types.hpp"

#include <string>
#include <unordered_map>
#include <vector>

namespace control_core {

/**
 * Controller (Non-ROS)
 *
 * Key behavior:
 * - configure(joint_order) once to define canonical output order.
 * - step_velocity_subset():
 *   - takes a subset of desired velocities (by joint name)
 *   - joints NOT specified keep last commanded velocity (hold-last)
 *   - returns full velocity vector aligned to joint_order
 *
 * Safety layer: placeholder (ok/reason reserved), currently no filtering.
 */
class Controller {
public:
  Controller() = default;

  // Configure canonical joint order for output. Must be called before step().
  void configure(const std::vector<std::string>& joint_order);

  // Whether configure() has been called successfully.
  bool is_configured() const noexcept { return configured_; }

  // Return canonical joint order.
  const std::vector<std::string>& joint_order() const { return joint_order_; }

  // Reset hold-last velocities for all joints.
  void reset_hold(double value = 0.0);

  /**
   * Step with velocity subset control.
   *
   * @param state  Current state snapshot (optional; currently unused but kept for future safety)
   * @param desired_vel_by_name  Subset: joint_name -> target velocity (rad/s)
   * @param dt  Timestep seconds (currently unused; kept for future safety)
   * @return CommandOut with full velocity aligned to joint_order
   */
  CommandOut step_velocity_subset(
      const JointStateIn& state,
      const std::unordered_map<std::string, double>& desired_vel_by_name,
      double dt);

  /**
   * Convenience overload: desired subset as parallel arrays.
   * Useful when integrating with non-ROS systems that store arrays.
   */
  CommandOut step_velocity_subset(
      const JointStateIn& state,
      const std::vector<std::string>& subset_names,
      const std::vector<double>& subset_vel,
      double dt);

private:
  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, size_t> name_to_idx_;

  // Hold-last command memory (size == joint_order_.size())
  std::vector<double> last_vel_cmd_;

  bool configured_{false};
};

} // namespace control_core
