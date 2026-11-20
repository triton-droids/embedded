/** @file target_filter.cpp
 * @brief Alpha-beta filtering of one image-plane box (see target_filter.hpp).
 */
#include "humanoid_vision/target_filter.hpp"

#include <cmath>
#include <stdexcept>

namespace humanoid_vision {

namespace {
cv::Point2f box_center(const cv::Rect2f &box) {
  return {box.x + box.width * 0.5F, box.y + box.height * 0.5F};
}
}  // namespace

AlphaBetaFilter::AlphaBetaFilter(float alpha, float beta, float size_alpha)
    : alpha_(alpha), beta_(beta), size_alpha_(size_alpha) {
  // beta <= alpha (with alpha <= 1) keeps the g-h filter stable and non-ringing.
  if (!(alpha > 0.0F && alpha <= 1.0F) || !(beta >= 0.0F && beta <= alpha) ||
      !(size_alpha > 0.0F && size_alpha <= 1.0F)) {
    throw std::invalid_argument(
        "AlphaBetaFilter requires 0 < alpha <= 1, 0 <= beta <= alpha, 0 < size_alpha <= 1");
  }
}

void AlphaBetaFilter::reset(const cv::Rect2f &box) {
  state_ = TargetState{box_center(box), cv::Point2f(0.0F, 0.0F), box.size()};
}

TargetState AlphaBetaFilter::predict(double dt) const {
  TargetState predicted = state_;
  predicted.center += state_.velocity * static_cast<float>(dt);
  return predicted;
}

void AlphaBetaFilter::update(const cv::Rect2f &box, double dt) {
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("AlphaBetaFilter::update requires dt > 0");
  }
  const TargetState predicted = predict(dt);
  const cv::Point2f residual = box_center(box) - predicted.center;
  state_.center = predicted.center + alpha_ * residual;
  state_.velocity += (beta_ / static_cast<float>(dt)) * residual;
  // Box size has no motion model; exponential smoothing only removes jitter.
  state_.size.width += size_alpha_ * (box.width - state_.size.width);
  state_.size.height += size_alpha_ * (box.height - state_.size.height);
}

}  // namespace humanoid_vision
