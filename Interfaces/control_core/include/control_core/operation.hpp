#pragma once
#include "control_core/api.hpp"
#include <unordered_set>

namespace control_core {
// Pure state machine: no ROS, CAN, sleeps or automatic motor enable.
enum class OperationStage : uint8_t {
  Idle, Decelerating, Settling, WaitingBackend, WaitingRebind, Resuming,
  Completed, Failed, Cancelled
};
struct OperationLimits {
  double acceleration{0.2};  // commanded rad/s^2; not an actuator torque guarantee
  double velocity{0.2};      // admission bound on measured rad/s
  double settled_velocity{0.01};
  double settle_s{0.3};
  double timeout_s{10.0};
  double max_step_s{0.1};
  double kp{2.0}, kd{0.2};   // must be commissioned for the supported mechanism
  double max_effort{1.0};    // measured Nm trip; cannot prevent sub-cycle damage
};
struct OperationStatus {
  std::string request_id, joint, message;
  OperationStage stage{OperationStage::Idle};
  bool backend_request{false};  // consumed once by take_backend_request()
  bool zero_may_have_changed{false};
  uint64_t calibration_epoch{0};
};
class ZeroOperation {
public:
  // All joints require an explicitly observed enable state. Disabled joints
  // stay disabled. Position, velocity and effort must be fresh at the adapter.
  bool begin(const std::string& id, const std::string& joint, const State& state,
             const std::vector<std::string>& order,
             const std::unordered_map<std::string, bool>& enabled,
             const OperationLimits& limits, std::string& reason);
  Output step(const State& state, double dt);
  bool take_backend_request();
  void backend_reply(const std::string& id, bool accepted, const std::string& reason);
  // This is separate from backend acceptance: requires verified zero AND a
  // completed coordinate/producer-history rebind. The current adapter cannot
  // manufacture this evidence and therefore never calls it for a real zero.
  bool confirm_rebind(const std::string& id, uint64_t epoch, const State& state);
  void abort(const std::string& reason, bool cancelled = false);
  // Explicit recovery only before a potentially submitted zero write.
  bool reset(std::string& reason);
  bool blocks_normal() const;
  const OperationStatus& status() const { return status_; }
  const std::vector<std::string>& joint_order() const { return order_; }
private:
  OperationStatus status_;
  OperationLimits limits_;
  std::vector<std::string> order_;
  std::unordered_map<std::string, bool> enabled_;
  std::unordered_map<std::string, double> position_, velocity_;
  std::unordered_set<std::string> seen_ids_;
  double elapsed_{0}, settled_{0};
  bool valid_state(const State& state) const;
  Output output(bool disable = false) const;
};
} // namespace control_core
