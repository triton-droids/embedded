#include "control_core/operation.hpp"
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
using namespace control_core;
namespace {
State state(double velocity = 0) {State s; s.joints["a"] = {.4, velocity, .1}; s.joints["b"] = {-.2, 0, 0}; return s;}
struct Fixture : ::testing::Test {
  ZeroOperation op; OperationLimits limits; std::string reason;
  void begin(double velocity = 0) {
    ASSERT_TRUE(op.begin("zero-1", "a", state(velocity), {"a", "b"}, {{"a", true}, {"b", false}}, limits, reason)) << reason;
  }
  void stop() {
    for (int i = 0; i < 100 && op.status().stage != OperationStage::WaitingBackend; ++i) op.step(state(), .02);
    ASSERT_EQ(op.status().stage, OperationStage::WaitingBackend);
  }
};
TEST_F(Fixture, DecelerationStartsFromMeasuredPositionAndBoundsVelocityChange) {
  begin(.1); double previous = .1, position = .4;
  for (int i = 0; i < 30; ++i) {
    auto out = op.step(state(.1), .02);
    EXPECT_EQ(out.commands[0].mode, Mode::Motion);
    EXPECT_EQ(out.commands[1].mode, Mode::Disable);
    const auto velocity = out.commands[0].velocity;
    EXPECT_LE(std::abs(velocity - previous), limits.acceleration * .02 + 1e-12);
    EXPECT_NEAR(out.commands[0].position - position, (previous + velocity) * .01, 1e-12);
    previous = velocity; position = out.commands[0].position;
  }
  EXPECT_EQ(op.status().stage, OperationStage::Settling);
  EXPECT_FALSE(op.take_backend_request()); // Commanded zero is not measured stopping.
}
TEST_F(Fixture, MeasuredSettlingMustBeContinuousBeforeExactlyOneRequest) {
  begin(); op.step(state(), .02);
  for (int i = 0; i < 10; ++i) op.step(state(), .02);
  op.step(state(.02), .02); // Reset dwell timer.
  for (int i = 0; i < 10; ++i) op.step(state(), .02);
  EXPECT_FALSE(op.take_backend_request()); stop();
  EXPECT_TRUE(op.take_backend_request()); EXPECT_FALSE(op.take_backend_request());
  auto waiting = op.step(state(), .02);
  EXPECT_EQ(waiting.commands[0].mode, Mode::Disable); // No old-coordinate hold.
}
TEST_F(Fixture, RejectionStaysDisabledAndRequiresExplicitReset) {
  begin(); stop(); ASSERT_TRUE(op.take_backend_request());
  op.backend_reply("zero-1", false, "unsupported firmware");
  EXPECT_EQ(op.status().stage, OperationStage::Failed); EXPECT_TRUE(op.blocks_normal());
  EXPECT_EQ(op.step(state(), .02).commands[0].mode, Mode::Disable);
  EXPECT_FALSE(op.status().zero_may_have_changed); EXPECT_TRUE(op.reset(reason));
  EXPECT_FALSE(op.begin("zero-1", "a", state(), {"a"}, {{"a", true}}, limits, reason));
}
TEST_F(Fixture, AcceptanceIsNotCompletionAndUnknownOutcomeCannotBeReset) {
  begin(); stop(); op.take_backend_request(); op.backend_reply("zero-1", true, "queued");
  EXPECT_EQ(op.status().stage, OperationStage::WaitingRebind); EXPECT_TRUE(op.blocks_normal());
  EXPECT_EQ(op.step(state(), .02).commands[0].mode, Mode::Disable);
  op.abort("RPC timeout"); EXPECT_FALSE(op.reset(reason));
  op.backend_reply("zero-1", false, "late rejection"); EXPECT_TRUE(op.status().zero_may_have_changed);
}
TEST_F(Fixture, VerifiedRebindUsesNewMeasuredTargetsNeverPreZeroPositions) {
  begin(); stop(); op.take_backend_request(); op.backend_reply("zero-1", true, "queued");
  State rebound = state(); rebound.joints["a"].pos = .03;
  EXPECT_FALSE(op.confirm_rebind("other", 1, rebound));
  EXPECT_FALSE(op.confirm_rebind("zero-1", 0, rebound));
  ASSERT_TRUE(op.confirm_rebind("zero-1", 1, rebound));
  auto out = op.step(rebound, .02);
  EXPECT_EQ(op.status().stage, OperationStage::Completed); EXPECT_FALSE(op.blocks_normal());
  EXPECT_DOUBLE_EQ(out.commands[0].position, .03);
  EXPECT_EQ(out.commands[1].mode, Mode::Disable); // Never enable a previously disabled motor.
  EXPECT_EQ(op.status().calibration_epoch, 1u);
}
TEST_F(Fixture, InvalidFeedbackTimingAndEffortTripDisableImmediately) {
  for (int scenario = 0; scenario < 4; ++scenario) {
    ZeroOperation x; ASSERT_TRUE(x.begin("x", "a", state(), {"a"}, {{"a", true}}, limits, reason));
    auto s = state(); double dt = .02;
    if (scenario == 0) s.joints.clear();
    if (scenario == 1) s.joints["a"].pos = std::numeric_limits<double>::quiet_NaN();
    if (scenario == 2) s.joints["a"].eff = 2;
    if (scenario == 3) dt = .2;
    EXPECT_EQ(x.step(s, dt).commands[0].mode, Mode::Disable);
    EXPECT_EQ(x.status().stage, OperationStage::Failed);
  }
}
TEST_F(Fixture, TimeoutCannotCauseAutomaticRetryOrResume) {
  limits.timeout_s = .5; begin(); stop(); op.take_backend_request();
  for (int i = 0; i < 30; ++i) op.step(state(), .02);
  EXPECT_EQ(op.status().stage, OperationStage::Failed); EXPECT_FALSE(op.take_backend_request());
  EXPECT_TRUE(op.status().zero_may_have_changed); EXPECT_FALSE(op.reset(reason));
}
TEST_F(Fixture, CancellationAndUnexpectedMotionCannotCompleteZero) {
  begin(); op.abort("cancel", true); EXPECT_EQ(op.status().stage, OperationStage::Cancelled);
  EXPECT_TRUE(op.reset(reason));
  ASSERT_TRUE(op.begin("zero-2", "a", state(), {"a"}, {{"a", true}}, limits, reason));
  stop(); op.take_backend_request();
  EXPECT_EQ(op.step(state(.05), .02).commands[0].mode, Mode::Disable);
  EXPECT_EQ(op.status().stage, OperationStage::Failed);
  EXPECT_FALSE(op.confirm_rebind("zero-2", 1, state()));
}
TEST_F(Fixture, RejectsMissingEnableEvidenceAndInvalidLimitsBeforeMotion) {
  EXPECT_FALSE(op.begin("a", "a", state(), {"a"}, {}, limits, reason));
  EXPECT_FALSE(op.begin("a", "unknown", state(), {"a"}, {{"a", true}}, limits, reason));
  limits.acceleration = -1;
  EXPECT_FALSE(op.begin("a", "a", state(), {"a"}, {{"a", true}}, limits, reason));
  EXPECT_EQ(op.status().stage, OperationStage::Idle);
}
} // namespace
