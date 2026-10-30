#include "control_core/operation.hpp"
#include <algorithm>
#include <cmath>

namespace control_core {
bool ZeroOperation::blocks_normal() const {
  return status_.stage != OperationStage::Idle && status_.stage != OperationStage::Completed;
}
bool ZeroOperation::valid_state(const State& state) const {
  for (const auto& name : order_) {
    const auto it = state.joints.find(name);
    if (it == state.joints.end() || !std::isfinite(it->second.pos) ||
        !std::isfinite(it->second.vel) || !std::isfinite(it->second.eff)) return false;
  }
  return true;
}
bool ZeroOperation::begin(const std::string& id, const std::string& joint,
    const State& state, const std::vector<std::string>& order,
    const std::unordered_map<std::string, bool>& enabled,
    const OperationLimits& limits, std::string& reason) {
  if (blocks_normal()) {reason = "Operation owns control; recover explicitly first"; return false;}
  if (id.empty() || id.size() > 128 || seen_ids_.count(id) || seen_ids_.size() >= 256) {
    reason = "Invalid, duplicate or exhausted request ID"; return false;
  }
  std::unordered_set<std::string> names(order.begin(), order.end());
  if (order.empty() || names.size() != order.size() || names.count("") || !names.count(joint)) {
    reason = "Unknown joint or invalid joint order"; return false;
  }
  for (double value : {limits.acceleration, limits.velocity, limits.settled_velocity,
      limits.settle_s, limits.timeout_s, limits.max_step_s, limits.max_effort}) {
    if (!std::isfinite(value) || value <= 0) {reason = "Limits must be finite and positive"; return false;}
  }
  if (!std::isfinite(limits.kp) || !std::isfinite(limits.kd) || limits.kp < 0 || limits.kd < 0) {
    reason = "Invalid stop gains"; return false;
  }
  for (const auto& name : order) {
    const auto it = state.joints.find(name);
    if (!enabled.count(name) || it == state.joints.end() || !std::isfinite(it->second.pos) ||
        !std::isfinite(it->second.vel) || !std::isfinite(it->second.eff) ||
        std::abs(it->second.vel) > limits.velocity || std::abs(it->second.eff) > limits.max_effort) {
      reason = "Missing enable evidence or feedback outside admission limits"; return false;
    }
  }
  order_ = order; enabled_ = enabled; limits_ = limits;
  position_.clear(); velocity_.clear();
  for (const auto& name : order) {
    position_[name] = state.joints.at(name).pos;
    velocity_[name] = state.joints.at(name).vel;
  }
  const auto epoch = status_.calibration_epoch;
  status_ = {id, joint, "Decelerating all core-owned joints", OperationStage::Decelerating, false, false, epoch};
  seen_ids_.insert(id); elapsed_ = settled_ = 0; reason = "Accepted";
  return true;
}
Output ZeroOperation::output(bool disable) const {
  Output out; out.joint_name = order_;
  for (const auto& name : order_) {
    Command cmd;
    cmd.mode = disable || !enabled_.at(name) ? Mode::Disable : Mode::Motion;
    cmd.position = position_.at(name); cmd.velocity = velocity_.at(name);
    cmd.acceleration = limits_.acceleration;
    cmd.kp = limits_.kp; cmd.kd = limits_.kd; cmd.torque = 0;
    out.commands.push_back(cmd);
  }
  return out;
}
Output ZeroOperation::step(const State& state, double dt) {
  if (status_.stage == OperationStage::Idle) return {};
  if (status_.stage == OperationStage::Failed || status_.stage == OperationStage::Cancelled) return output(true);
  if (!std::isfinite(dt) || dt <= 0 || dt > limits_.max_step_s || !valid_state(state)) {
    abort("Invalid timing or feedback"); return output(true);
  }
  for (const auto& name : order_) {
    const auto& sample = state.joints.at(name);
    if (std::abs(sample.eff) > limits_.max_effort || std::abs(sample.vel) > limits_.velocity) {
      abort("Measured velocity/effort limit exceeded"); return output(true);
    }
  }
  if (status_.stage == OperationStage::Completed) return output();
  elapsed_ += dt;
  if (elapsed_ > limits_.timeout_s) {abort("Operation timeout; remain disabled"); return output(true);}
  if (status_.stage == OperationStage::Decelerating) {
    bool commanded_stop = true;
    for (const auto& name : order_) {
      const double previous = velocity_.at(name);
      const double next = std::copysign(std::max(0.0, std::abs(previous) - limits_.acceleration * dt), previous);
      position_[name] += (previous + next) * 0.5 * dt;
      velocity_[name] = next;
      commanded_stop = commanded_stop && next == 0;
    }
    if (commanded_stop) {status_.stage = OperationStage::Settling; status_.message = "Waiting for measured stop";}
  } else if (status_.stage == OperationStage::Settling) {
    bool stopped = true;
    for (const auto& name : order_) stopped = stopped && std::abs(state.joints.at(name).vel) <= limits_.settled_velocity;
    settled_ = stopped ? settled_ + dt : 0;
    if (settled_ >= limits_.settle_s) {
      status_.stage = OperationStage::WaitingBackend;
      status_.backend_request = true;
      status_.message = "Stopped; awaiting backend zero admission";
    }
  } else if (status_.stage == OperationStage::WaitingBackend || status_.stage == OperationStage::WaitingRebind) {
    for (const auto& name : order_) {
      if (std::abs(state.joints.at(name).vel) > limits_.settled_velocity) {
        abort("Motion detected while waiting for zero/rebind"); return output(true);
      }
    }
  } else if (status_.stage == OperationStage::Resuming) {
    status_.stage = OperationStage::Completed;
    status_.message = "Verified coordinate rebind complete; awaiting fresh producer commands";
  }
  // After a zero could have been sent, old-coordinate hold targets must never
  // reach CAN. Disable while waiting; a future backend must own exclusion and
  // lifecycle recovery. The existing backend rejects the request entirely.
  return output(status_.zero_may_have_changed || status_.stage == OperationStage::WaitingBackend ||
                status_.stage == OperationStage::WaitingRebind);
}
bool ZeroOperation::take_backend_request() {
  if (!status_.backend_request) return false;
  status_.backend_request = false;
  status_.zero_may_have_changed = true; // Conservative even if RPC outcome is lost.
  return true;
}
void ZeroOperation::backend_reply(const std::string& id, bool accepted, const std::string& reason) {
  if (id != status_.request_id || status_.stage != OperationStage::WaitingBackend ||
      !status_.zero_may_have_changed) return;
  if (!accepted) {
    // A definitive rejection promises no write. Timeout/transport failure must
    // instead call abort() and retain the unknown-outcome latch.
    status_.zero_may_have_changed = false; abort("Backend rejected: " + reason); return;
  }
  status_.stage = OperationStage::WaitingRebind;
  status_.message = "Accepted only; verified zero and coordinate rebind still required";
}
bool ZeroOperation::confirm_rebind(const std::string& id, uint64_t epoch, const State& state) {
  if (id != status_.request_id || status_.stage != OperationStage::WaitingRebind ||
      epoch <= status_.calibration_epoch || !valid_state(state)) return false;
  for (const auto& name : order_) {
    const auto& sample = state.joints.at(name);
    if (std::abs(sample.vel) > limits_.settled_velocity || std::abs(sample.eff) > limits_.max_effort) return false;
  }
  for (const auto& name : order_) {position_[name] = state.joints.at(name).pos; velocity_[name] = 0;}
  status_.calibration_epoch = epoch; status_.zero_may_have_changed = false;
  status_.stage = OperationStage::Resuming;
  return true;
}
void ZeroOperation::abort(const std::string& reason, bool cancelled) {
  if (!blocks_normal() || status_.stage == OperationStage::Failed ||
      status_.stage == OperationStage::Cancelled) return;
  status_.backend_request = false;
  status_.stage = cancelled ? OperationStage::Cancelled : OperationStage::Failed;
  status_.message = reason;
}
bool ZeroOperation::reset(std::string& reason) {
  if (status_.stage != OperationStage::Failed && status_.stage != OperationStage::Cancelled) {
    reason = "Only failed/cancelled operations can be reset"; return false;
  }
  if (status_.zero_may_have_changed) {reason = "Zero outcome unknown; coordinate reconciliation required"; return false;}
  const auto epoch = status_.calibration_epoch;
  status_ = {}; status_.calibration_epoch = epoch;
  reason = "Reset; explicit enable and fresh commands required"; return true;
}
} // namespace control_core
