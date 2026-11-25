/** @file target_tracker.cpp
 * @brief Gated nearest-neighbour association for one target (see header).
 */
#include "humanoid_vision/target_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace humanoid_vision {

TargetTracker::TargetTracker(const TrackerConfig &config)
    : config_(config),
      filter_(config.alpha, config.beta, config.size_alpha) {
  if (!(config.gate_min_px > 0.0F) || !(config.gate_size_factor >= 0.0F) ||
      config.confirm_frames < 1 || !(config.max_coast_s >= 0.0) ||
      !std::isfinite(config.max_coast_s)) {
    throw std::invalid_argument(
        "TargetTracker requires gate_min_px > 0, gate_size_factor >= 0, "
        "confirm_frames >= 1, and finite max_coast_s >= 0");
  }
}

TrackOutput TargetTracker::output(bool measured, const TargetState &state) const {
  return TrackOutput{track_id_, class_id_, confidence_, measured, state};
}

std::optional<TrackOutput> TargetTracker::start(
    const std::vector<Detection2D> &detections, double time_s) {
  active_ = false;
  if (detections.empty()) return std::nullopt;
  const auto best = std::max_element(
      detections.begin(), detections.end(),
      [](const Detection2D &a, const Detection2D &b) { return a.confidence < b.confidence; });
  filter_.reset(best->box);
  active_ = true;
  ++track_id_;
  hits_ = 1;
  class_id_ = best->class_id;
  confidence_ = best->confidence;
  last_match_s_ = time_s;
  if (hits_ < config_.confirm_frames) return std::nullopt;
  return output(true, filter_.state());
}

std::optional<TrackOutput> TargetTracker::update(
    const std::vector<Detection2D> &detections, double time_s) {
  if (!std::isfinite(time_s)) {
    throw std::invalid_argument("TargetTracker::update requires a finite time");
  }
  if (!active_ || time_s <= last_match_s_) {
    return start(detections, time_s);
  }

  const double dt = time_s - last_match_s_;
  const TargetState predicted = filter_.predict(dt);
  const float gate = std::max(
      config_.gate_min_px,
      config_.gate_size_factor * std::max(predicted.size.width, predicted.size.height));

  // Nearest detection to the prediction inside the gate.
  const Detection2D *match = nullptr;
  float match_distance = gate;
  for (const Detection2D &detection : detections) {
    const cv::Point2f center(detection.box.x + detection.box.width * 0.5F,
                             detection.box.y + detection.box.height * 0.5F);
    const cv::Point2f offset = center - predicted.center;
    const float distance = std::hypot(offset.x, offset.y);
    if (distance <= match_distance) {
      match = &detection;
      match_distance = distance;
    }
  }

  if (match) {
    filter_.update(match->box, dt);
    last_match_s_ = time_s;
    hits_ = std::min(hits_ + 1, config_.confirm_frames);
    class_id_ = match->class_id;
    confidence_ = match->confidence;
    if (hits_ < config_.confirm_frames) return std::nullopt;
    return output(true, filter_.state());
  }

  // A tentative track that misses was probably a false positive; a confirmed one
  // may coast briefly. Either way, an expired track lets a new target take over.
  if (hits_ >= config_.confirm_frames && dt <= config_.max_coast_s) {
    return output(false, predicted);
  }
  return start(detections, time_s);
}

}  // namespace humanoid_vision
