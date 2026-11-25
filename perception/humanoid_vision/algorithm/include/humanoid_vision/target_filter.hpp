/** @file target_filter.hpp
 * @brief Constant-velocity alpha-beta filter for one target's image-plane box.
 *
 * Units are pixels and pixels per second in the color image. The camera sits on
 * the moving head, so the estimated velocity is image-plane motion: it mixes the
 * target's own motion with head rotation and is not a world-frame velocity.
 */
#pragma once

#include <opencv2/core/types.hpp>

namespace humanoid_vision {

/** @brief Filtered box center, center velocity, and box size. */
struct TargetState {
  cv::Point2f center;    ///< Box center, pixels.
  cv::Point2f velocity;  ///< Center velocity, pixels/second (image plane).
  cv::Size2f size;       ///< Box width/height, pixels.
};

/** @brief Alpha-beta (g-h) filter: a fixed-gain steady-state Kalman filter.
 * alpha in (0,1] weights the measured position against the prediction; beta in
 * [0,alpha] converts the residual into a velocity correction. Larger values react
 * faster but pass more box jitter. A constant-velocity target is followed with
 * no steady-state lag. White position noise keeps a variance fraction of
 * (2a^2 + b(2 - 3a)) / (a(4 - 2a - b)): 0.32 (std 0.57x) for a=0.5, b=0.1.
 * Gains are per update, so the effective bandwidth scales with the frame rate.
 */
class AlphaBetaFilter {
public:
  /** @throws std::invalid_argument When the gains are outside the stable range. */
  AlphaBetaFilter(float alpha, float beta, float size_alpha);

  /** @brief Start from one measurement with zero velocity. */
  void reset(const cv::Rect2f &box);
  /** @brief State extrapolated dt seconds past the last update (size held). */
  TargetState predict(double dt) const;
  /** @brief Fuse a measurement taken dt > 0 seconds after the last update. */
  void update(const cv::Rect2f &box, double dt);
  /** @brief State at the last update. */
  const TargetState &state() const { return state_; }

private:
  float alpha_;
  float beta_;
  float size_alpha_;
  TargetState state_{};
};

}  // namespace humanoid_vision
