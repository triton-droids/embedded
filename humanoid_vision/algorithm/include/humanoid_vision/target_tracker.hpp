/** @file target_tracker.hpp
 * @brief Single-target lock-on: frame-to-frame association, smoothing, coasting.
 *
 * The tracker keeps one target. While it has a track, only detections inside a
 * gate around the predicted center can update it, so a second ball or a one-frame
 * false positive elsewhere cannot steal the target. Short detection gaps are
 * bridged by prediction; after max_coast_s without a match the track is dropped
 * and the highest-confidence detection starts a new one. This is not a
 * multi-object tracker: other detections are ignored, not tracked.
 */
#pragma once

#include <optional>
#include <vector>

#include "humanoid_vision/target_filter.hpp"
#include "humanoid_vision/types.hpp"

namespace humanoid_vision {

/** @brief Tracker tuning; pixel values refer to the original color image. */
struct TrackerConfig {
  float alpha = 0.5F;       ///< Position gain, see AlphaBetaFilter.
  float beta = 0.1F;        ///< Velocity gain, see AlphaBetaFilter.
  float size_alpha = 0.3F;  ///< Box-size smoothing gain.
  /// Association radius around the predicted center is
  /// max(gate_min_px, gate_size_factor * max(predicted width, height)).
  float gate_min_px = 40.0F;
  float gate_size_factor = 1.5F;
  /// Consecutive matched frames before a new track is reported; 1 reports at once.
  int confirm_frames = 2;
  /// Longest gap (seconds since the last match) bridged by prediction.
  double max_coast_s = 0.1;
};

/** @brief One reported target state. */
struct TrackOutput {
  int track_id;       ///< Increments whenever a new track starts.
  int class_id;       ///< Class of the most recent matched detection.
  float confidence;   ///< Confidence of the most recent matched detection.
  bool measured;      ///< False while coasting: state is a prediction only.
  TargetState state;  ///< At the time passed to update().
};

class TargetTracker {
public:
  /** @throws std::invalid_argument For invalid gains, gates, or durations. */
  explicit TargetTracker(const TrackerConfig &config = TrackerConfig{});

  /** @brief Advance to time_s (seconds, monotonic per stream) with this frame's
   * detections, already restricted to the wanted class.
   * @return The target state, or nothing when no confirmed track exists.
   * A time that does not increase (for example a looping bag) resets the track.
   */
  std::optional<TrackOutput> update(const std::vector<Detection2D> &detections,
                                    double time_s);

  /** @brief Forget the current track. */
  void reset() { active_ = false; }

private:
  /** @brief Start a tentative track from the highest-confidence detection. */
  std::optional<TrackOutput> start(const std::vector<Detection2D> &detections,
                                   double time_s);
  TrackOutput output(bool measured, const TargetState &state) const;

  TrackerConfig config_;
  AlphaBetaFilter filter_;
  bool active_ = false;
  int track_id_ = 0;
  int hits_ = 0;
  int class_id_ = -1;
  float confidence_ = 0.0F;
  double last_match_s_ = 0.0;
};

}  // namespace humanoid_vision
