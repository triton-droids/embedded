#pragma once
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace control_core {

// Per-joint command mode (independent per joint)
enum class Mode : uint8_t {
  Velocity = 0,
  Position = 1,
  Motion   = 2,
  Enable   = 3,
  Disable  = 4
};

// Basic per-joint state (optional; can be partial/missing)
struct JointData {
  double pos{0.0};
  double vel{0.0};
  double eff{0.0};
};

// Input state snapshot
struct State {
  std::unordered_map<std::string, JointData> joints;
};

// One per-joint command (independent mode)
struct Command {
  Mode mode{Mode::Velocity};

  // Common fields
  double position{0.0};      // rad
  double velocity{0.0};      // rad/s
  double acceleration{0.0};  // rad/s^2

  // Motion mode fields
  double torque{0.0};        // Nm (or driver units)
  double kp{40.0};
  double kd{1.5};
};

// Subset command: only joints present here are overwritten this step.
// Missing joints will keep last command (hold-last).
using CommandSubset = std::unordered_map<std::string, Command>;

// Full output aligned to canonical joint order
struct Output {
  std::vector<std::string> joint_name;
  std::vector<Command> commands;  // same length as joint_name
};

// Safety plugin interface (optional, you said "leave blank" for now)
class SafetyFeature {
public:
  virtual ~SafetyFeature() = default;
  virtual bool check(const State& state, std::string* reason) = 0;
};

class Controller {
public:
  Controller() = default;

  // Must be called once before step()
  void configure(const std::vector<std::string>& joint_order);

  bool is_configured() const noexcept { return configured_; }
  const std::vector<std::string>& joint_order() const { return joint_order_; }

  // Optional safety modules (can ignore for now)
  void add_safety(std::shared_ptr<SafetyFeature> f);

  // Reset hold-last memory:
  // - sets all joints to Mode::Velocity with velocity=value
  void reset_hold(double value = 0.0);

  // Main step:
  // - Start from hold-last commands
  // - Override only joints in desired_subset
  // - Safety failure => return hold-last unchanged (as placeholder)
  Output step(const State& state, const CommandSubset& desired_subset, double dt);

private:
  std::vector<std::string> joint_order_;
  std::unordered_map<std::string, size_t> name_to_idx_;

  std::vector<Command> last_cmd_;  // hold-last commands aligned to joint_order_
  std::vector<std::shared_ptr<SafetyFeature>> safety_;

  bool configured_{false};
};

} // namespace control_core
