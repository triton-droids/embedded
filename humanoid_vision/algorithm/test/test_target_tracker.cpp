/** @file test_target_tracker.cpp
 * @brief Deterministic checks for AlphaBetaFilter and TargetTracker; no ROS/camera.
 * Exits nonzero on the first failed expectation.
 */
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <stdexcept>
#include <vector>

#include "humanoid_vision/target_filter.hpp"
#include "humanoid_vision/target_tracker.hpp"

using humanoid_vision::AlphaBetaFilter;
using humanoid_vision::TargetTracker;
using humanoid_vision::TrackerConfig;

namespace {
int failures = 0;

void expect(bool condition, const char *what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++failures;
  }
}

/** @brief A 30 px ball detection centered at (x, y). */
Detection2D ball(float x, float y, float confidence = 0.8F, float size = 30.0F) {
  return Detection2D{0, confidence, cv::Rect2f(x - size / 2, y - size / 2, size, size)};
}

constexpr double kDt = 1.0 / 60.0;

void filter_follows_constant_velocity() {
  AlphaBetaFilter filter(0.5F, 0.1F, 0.3F);
  filter.reset(cv::Rect2f(85, 85, 30, 30));  // center (100, 100)
  for (int i = 1; i <= 120; ++i) {
    const float x = 100.0F + 120.0F * static_cast<float>(i * kDt);  // 120 px/s
    filter.update(cv::Rect2f(x - 15, 85, 30, 30), kDt);
  }
  const auto &state = filter.state();
  expect(std::abs(state.velocity.x - 120.0F) < 1.0F, "filter velocity converges to 120 px/s");
  expect(std::abs(state.velocity.y) < 1e-3F, "filter keeps zero vertical velocity");
  expect(std::abs(state.center.x - 340.0F) < 0.5F, "filter has no steady-state lag");
}

void filter_reduces_jitter() {
  std::mt19937 rng(7);
  std::normal_distribution<float> noise(0.0F, 3.0F);
  AlphaBetaFilter filter(0.5F, 0.1F, 0.3F);
  filter.reset(cv::Rect2f(185, 185, 30, 30));
  double raw = 0.0, filtered = 0.0;
  const int n = 2000;
  for (int i = 0; i < n; ++i) {
    const float e = noise(rng);
    filter.update(cv::Rect2f(185 + e, 185, 30, 30), kDt);
    raw += e * e;
    const double f = filter.state().center.x - 200.0;
    filtered += f * f;
  }
  const double ratio = std::sqrt(filtered / raw);
  // Theory for a=0.5, b=0.1 is 0.57; allow sampling error.
  expect(ratio > 0.45 && ratio < 0.70, "filtered jitter is about 0.57x of raw");
}

void confirmation_delays_new_track() {
  TargetTracker tracker;  // confirm_frames = 2
  expect(!tracker.update({ball(100, 100)}, 1.0), "first sighting is tentative");
  const auto out = tracker.update({ball(102, 100)}, 1.0 + kDt);
  expect(out && out->measured && out->track_id == 1, "second sighting confirms track 1");
}

void single_false_positive_is_not_reported() {
  TargetTracker tracker;
  expect(!tracker.update({ball(300, 200)}, 1.0), "false positive frame is tentative");
  expect(!tracker.update({}, 1.0 + kDt), "tentative miss reports nothing");
  expect(!tracker.update({}, 1.0 + 2 * kDt), "still nothing afterwards");
}

void distractor_cannot_steal_track() {
  TargetTracker tracker;
  double t = 1.0;
  tracker.update({ball(100, 100)}, t);
  for (int i = 0; i < 10; ++i) {
    t += kDt;
    // A second, more confident ball far away appears in every frame.
    const auto out = tracker.update({ball(100, 100, 0.5F), ball(500, 300, 0.95F)}, t);
    expect(out && out->track_id == 1 && std::abs(out->state.center.x - 100.0F) < 1.0F,
           "track stays on the original ball");
  }
}

void short_gap_coasts_then_hands_over() {
  TargetTracker tracker;
  double t = 1.0;
  for (int i = 0; i < 30; ++i, t += kDt) {
    tracker.update({ball(100.0F + 60.0F * static_cast<float>(i * kDt), 100)}, t);
  }
  const double last_match = t - kDt;
  // Missed frames within max_coast_s (0.1 s): prediction keeps moving right.
  const auto coast = tracker.update({}, last_match + 0.05);
  expect(coast && !coast->measured && coast->track_id == 1, "short gap coasts on track 1");
  expect(coast && coast->state.center.x > 100.0F + 60.0F * 29 * kDt,
         "coasting prediction extrapolates motion");
  // After the coast limit the other ball becomes a new tentative, then confirmed, track.
  expect(!tracker.update({ball(500, 300)}, last_match + 0.15), "expired track restarts tentatively");
  const auto next = tracker.update({ball(500, 300)}, last_match + 0.15 + kDt);
  expect(next && next->track_id == 2, "new target becomes track 2");
}

void reacquire_inside_gate_keeps_id() {
  TargetTracker tracker;
  tracker.update({ball(100, 100)}, 1.0);
  tracker.update({ball(100, 100)}, 1.0 + kDt);
  tracker.update({}, 1.0 + 2 * kDt);  // one missed frame
  const auto out = tracker.update({ball(104, 100)}, 1.0 + 3 * kDt);
  expect(out && out->measured && out->track_id == 1, "brief miss keeps the same track id");
}

void time_reset_restarts_track() {
  TargetTracker tracker;
  tracker.update({ball(100, 100)}, 5.0);
  tracker.update({ball(100, 100)}, 5.0 + kDt);
  expect(!tracker.update({ball(100, 100)}, 1.0), "time going backwards starts a new tentative track");
}

void invalid_configuration_throws() {
  bool threw = false;
  try {
    TrackerConfig config;
    config.beta = 0.9F;  // > alpha
    TargetTracker tracker(config);
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  expect(threw, "beta > alpha is rejected");
}
}  // namespace

int main() {
  filter_follows_constant_velocity();
  filter_reduces_jitter();
  confirmation_delays_new_track();
  single_false_positive_is_not_reported();
  distractor_cannot_steal_track();
  short_gap_coasts_then_hands_over();
  reacquire_inside_gate_keeps_id();
  time_reset_restarts_track();
  invalid_configuration_throws();
  if (failures == 0) std::puts("target tracker: all checks passed");
  return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
