// Host-only physical contract checks: g++ -std=c++17 this_file.cpp -o /tmp/imu_math
#include "../body_imu_math.h"
#include <cassert>

int main() {
  using namespace body_imu;
  constexpr double h = 0.7071067811865475244;
  // Sensor mounted 90 degrees around body X: sensor +Y is body +Z.
  const Quaternion mount{h, h, 0, 0};
  const Vector force = rotate(mount, {0, 9.80665, 0});
  assert(std::abs(force.x) < 1e-9 && std::abs(force.y) < 1e-9);
  assert(std::abs(force.z - 9.80665) < 1e-9);
  // A level body with that mount has q_WS=q_BS. Compensating it must yield
  // upright body orientation, independently of the sensor's tilted orientation.
  const Quaternion q_WB = multiply(mount, conjugate(mount));
  assert(std::abs(q_WB.w - 1) < 1e-9 && std::abs(q_WB.x) < 1e-9);
  // A genuine 90-degree body roll must survive mount compensation.
  const Quaternion q_WS = multiply(mount, mount);
  const Quaternion rolled = multiply(q_WS, conjugate(mount));
  const Vector down = rotate(conjugate(rolled), {0, 0, -1});
  assert(std::abs(down.y + 1) < 1e-9 && std::abs(down.z) < 1e-9);
  // Wrap must preserve a fresh 20-us-old sample; a future sample is invalid.
  assert(fresh(10, UINT32_MAX - 9, 30));
  assert(!fresh(10, 11, 30000));
  assert(!fresh(30001, 0, 30000));
  Quaternion invalid{0, 0, 0, 0};
  assert(!normalize(invalid));
}
