#pragma once
#include <string>
#include <unordered_map>
#include <vector>

namespace control_core {

// Per-joint numeric state (optional; can be missing for some joints)
struct JointData {
  double pos{0.0};
  double vel{0.0};
  double eff{0.0};
};

// Input state snapshot (by name). You can provide partial data.
struct JointStateIn {
  std::unordered_map<std::string, JointData> joints;
};

// Output command (always full-sized, aligned to joint_order)
struct CommandOut {
  std::vector<std::string> name;     // canonical joint order
  std::vector<double> velocity;      // rad/s, same length as name
  bool ok{true};                     // reserved for safety layer
  std::string reason;                // reserved for safety layer
};

} // namespace control_core
