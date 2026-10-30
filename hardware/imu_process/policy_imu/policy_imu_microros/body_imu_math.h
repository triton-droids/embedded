#pragma once

#include <cmath>
#include <cstdint>

namespace body_imu {
// Quaternion order throughout this file is w,x,y,z; ROS messages use named fields.
struct Quaternion { double w, x, y, z; };
struct Vector { double x, y, z; };

inline bool normalize(Quaternion &q) {
  const double n = std::sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
  if (!std::isfinite(n) || n < 0.9 || n > 1.1) return false;
  q = {q.w/n, q.x/n, q.y/n, q.z/n};
  return true;
}
inline Quaternion conjugate(Quaternion q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quaternion multiply(Quaternion a, Quaternion b) {
  return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
          a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
          a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
          a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
inline Vector rotate(Quaternion q, Vector v) {
  const Quaternion r = multiply(multiply(q, {0, v.x, v.y, v.z}), conjugate(q));
  return {r.x, r.y, r.z};
}
inline bool finite(Vector v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
// Unsigned subtraction keeps sample age correct across the 32-bit micros wrap.
inline bool fresh(uint32_t now, uint32_t sample, uint32_t limit_us) {
  return uint32_t(now - sample) <= limit_us;
}
}  // namespace body_imu
