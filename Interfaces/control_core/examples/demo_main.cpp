#include "control_core/api.hpp"
#include <iostream>

int main() {
  using namespace control_core;

  Controller ctrl;
  ctrl.configure({"shoulder_pitch", "elbow_pitch", "wrist_roll"});

  State s; // state can be empty for now

  CommandSubset desired;

  Command c0;
  c0.mode = Mode::Position;
  c0.position = 0.0;
  c0.velocity = 0.5;
  c0.acceleration = 1.0;
  desired["shoulder_pitch"] = c0;

  Command c1;
  c1.mode = Mode::Velocity;
  c1.velocity = -0.2;
  desired["elbow_pitch"] = c1;

  Command c2;
  c2.mode = Mode::Motion;
  c2.position = 0.3;
  c2.velocity = 0.0;
  c2.torque = 0.0;
  c2.kp = 40.0;
  c2.kd = 1.5;
  desired["wrist_roll"] = c2;

  auto out = ctrl.step(s, desired, 0.02);

  for (size_t i = 0; i < out.joint_name.size(); ++i) {
    const auto& jn = out.joint_name[i];
    const auto& cmd = out.commands[i];
    std::cout << jn << " mode=" << (int)cmd.mode << " vel=" << cmd.velocity
              << " pos=" << cmd.position << "\n";
  }

  // Step again with empty desired => hold-last (keeps previous per-joint modes)
  CommandSubset empty;
  auto out2 = ctrl.step(s, empty, 0.02);
  std::cout << "\nHold-last step:\n";
  for (size_t i = 0; i < out2.joint_name.size(); ++i) {
    std::cout << out2.joint_name[i] << " mode=" << (int)out2.commands[i].mode
              << " vel=" << out2.commands[i].velocity << "\n";
  }

  return 0;
}
