// Exercise real control-node callbacks without any CAN or motor backend.
#define MOTOR_CONTROL_HYBRID_NO_MAIN
#include "../src/cpp_control_node.cpp"
#include <gtest/gtest.h>
#include <limits>

// A narrow test peer keeps production callbacks private while testing their
// state transition and watchdog behavior directly, without timing sleeps.
struct CppControlNodeTestPeer {
  static void observe(CppControlNode& node, const std::vector<std::string>& names) {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->header.stamp = node.now();
    msg->name = names;
    msg->position.assign(names.size(), 0.1);
    msg->velocity.assign(names.size(), 0.0);
    node.state_callback(msg);
  }
  static bool fresh(const CppControlNode& node) {
    return node.state_is_fresh(node.joint_order_, node.latest_state_map_);
  }
  static void expire(CppControlNode& node, const std::string& name) {
    node.latest_state_map_.at(name).received_at -= std::chrono::seconds(1);
  }
  static void shift_stamp(CppControlNode& node, double delta) {
    for (auto& entry : node.latest_state_map_) entry.second.source_stamp += delta;
  }
  static void zero_stamps(CppControlNode& node) {
    for (auto& entry : node.latest_state_map_) entry.second.source_stamp = 0;
  }
  static void invalidate(CppControlNode& node, int kind) {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->header.stamp = node.now();
    msg->name = {"a", "b"};
    msg->position = {0.0, 0.0};
    msg->velocity = {0.0, 0.0};
    if (kind == 0) msg->velocity.clear();
    if (kind == 1) msg->position[0] = std::numeric_limits<double>::quiet_NaN();
    if (kind == 2) msg->name = {"a", "a"};
    node.state_callback(msg);
  }
  static void check_fault_clears_enable(CppControlNode& node) {
    auto cmd = std::make_shared<motor_control_interfaces::msg::MotorCommand>();
    cmd->header.stamp = node.now();
    cmd->joint_name = {"a", "b"};
    cmd->mode = {motor_control_interfaces::msg::MotorCommand::MODE_ENABLE};
    node.desired_motor_callback(cmd);
    ASSERT_TRUE(node.has_cmd_);
    node.control_loop();  // Expired b must disable and discard this cached enable.
    EXPECT_FALSE(node.has_cmd_);
    EXPECT_TRUE(node.desired_subset_.empty());
  }
  static void command(CppControlNode& node, const std::string& name, double age,
    double remaining = 0.05) {
    auto msg = std::make_shared<motor_control_interfaces::msg::MotorCommand>();
    msg->header.stamp = node.now() - rclcpp::Duration::from_seconds(age);
    msg->valid_until = node.now() + rclcpp::Duration::from_seconds(remaining);
    msg->joint_name = {name};
    msg->mode = {motor_control_interfaces::msg::MotorCommand::MODE_MOTION};
    node.desired_motor_callback(msg);
  }
  static void check_envelope(CppControlNode& node) {
    command(node, "a", 0.01);
    ASSERT_TRUE(node.has_cmd_);
    const auto original = node.command_times_.at("a");
    EXPECT_NEAR(original.source, node.now().seconds() - 0.01, 0.01);
    EXPECT_LT(original.expiry - original.source, 0.07);
    command(node, "b", 0.0);
    EXPECT_EQ(node.command_times_.at("a").deadline, original.deadline);
    // Even with fresh receipt/source traffic on b, expired held a stops output.
    node.command_times_.at("a").deadline -= std::chrono::seconds(1);
    node.control_loop();
    EXPECT_FALSE(node.has_cmd_);
    EXPECT_TRUE(node.command_times_.empty());
  }
  static void check_rejected(CppControlNode& node) {
    command(node, "a", 1.0);
    EXPECT_FALSE(node.has_cmd_);
    command(node, "a", 0.01, -0.001);
    EXPECT_FALSE(node.has_cmd_);
    command(node, "a", -1.0);
    EXPECT_FALSE(node.has_cmd_);
  }

  static void check_supervisor_advisory(CppControlNode& node) {
    observe(node, {"a", "b"});
    command(node, "a", 0.0);
    command(node, "b", 0.0);
    EXPECT_TRUE(node.supervisor_warning_reason().empty()); // Startup grace.
    node.supervisor_started_ -= std::chrono::seconds(4);
    EXPECT_NE(node.supervisor_warning_reason().find("no supervisor"), std::string::npos);
    node.control_loop();
    EXPECT_TRUE(node.has_cmd_);  // Missing supervisor must not disable/freeze commands.

    auto heartbeat = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    heartbeat->header.stamp = node.now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "humanoid_safety/supervisor";
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    heartbeat->status.push_back(status);
    node.supervisor_callback(heartbeat);
    EXPECT_TRUE(node.supervisor_warning_reason().empty());
    node.supervisor_received_ -= std::chrono::seconds(1);
    EXPECT_NE(node.supervisor_warning_reason().find("expired"), std::string::npos);
    const auto original_receipt = node.supervisor_received_;
    node.supervisor_callback(heartbeat); // Replaying a stamp cannot renew liveness.
    EXPECT_EQ(node.supervisor_received_, original_receipt);
    node.control_loop();
    EXPECT_TRUE(node.has_cmd_);

    heartbeat->header.stamp = node.now();
    heartbeat->status[0].level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    heartbeat->status[0].message = "test unhealthy";
    node.supervisor_callback(heartbeat);
    EXPECT_NE(node.supervisor_warning_reason().find("test unhealthy"), std::string::npos);
    node.control_loop();
    EXPECT_TRUE(node.has_cmd_);
    // Explicit estop remains independent and authoritative.
    auto estop = std::make_shared<std_msgs::msg::Bool>();
    estop->data = true;
    node.safety_estop_callback(estop);
    node.control_loop();
    EXPECT_FALSE(node.has_cmd_);
  }

};

class CppStateWatchdogTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    const char* argv[] = {"watchdog_test", "--ros-args", "-p", "joint_names:=[a,b]"};
    rclcpp::init(4, argv);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(CppStateWatchdogTest, PartialFirstFrameDoesNotShrinkExpectedRobot) {
  CppControlNode node;
  CppControlNodeTestPeer::observe(node, {"a"});
  EXPECT_FALSE(CppControlNodeTestPeer::fresh(node));
  CppControlNodeTestPeer::observe(node, {"b"});
  EXPECT_TRUE(CppControlNodeTestPeer::fresh(node));
}

TEST_F(CppStateWatchdogTest, OtherJointTrafficDoesNotRenewExpiredJoint) {
  CppControlNode node;
  CppControlNodeTestPeer::observe(node, {"a", "b"});
  CppControlNodeTestPeer::expire(node, "b");
  CppControlNodeTestPeer::observe(node, {"a"});
  EXPECT_FALSE(CppControlNodeTestPeer::fresh(node));
  CppControlNodeTestPeer::check_fault_clears_enable(node);
  CppControlNodeTestPeer::observe(node, {"b"});
  EXPECT_TRUE(CppControlNodeTestPeer::fresh(node));
}

TEST_F(CppStateWatchdogTest, RejectsStaleFutureAndZeroSourceStamps) {
  CppControlNode node;
  for (double delta : {-1.0, 1.0}) {
    CppControlNodeTestPeer::observe(node, {"a", "b"});
    CppControlNodeTestPeer::shift_stamp(node, delta);
    EXPECT_FALSE(CppControlNodeTestPeer::fresh(node));
  }
  CppControlNodeTestPeer::zero_stamps(node);
  EXPECT_FALSE(CppControlNodeTestPeer::fresh(node));
}

TEST_F(CppStateWatchdogTest, MalformedFramesInvalidateCachedState) {
  CppControlNode node;
  for (int kind = 0; kind < 3; ++kind) {
    CppControlNodeTestPeer::observe(node, {"a", "b"});
    ASSERT_TRUE(CppControlNodeTestPeer::fresh(node));
    CppControlNodeTestPeer::invalidate(node, kind);
    EXPECT_FALSE(CppControlNodeTestPeer::fresh(node));
  }
}

TEST_F(CppStateWatchdogTest, HeldCommandsRetainTheirOwnDeadline) {
  CppControlNode node;
  CppControlNodeTestPeer::observe(node, {"a", "b"});
  CppControlNodeTestPeer::check_envelope(node);
}

TEST_F(CppStateWatchdogTest, RejectsExpiredSourceAndExplicitExpiry) {
  CppControlNode node;
  CppControlNodeTestPeer::check_rejected(node);
}

TEST_F(CppStateWatchdogTest, SupervisorFailureIsWarningOnlyAndEstopStillStops) {
  CppControlNode node;
  CppControlNodeTestPeer::check_supervisor_advisory(node);
}

// These are test doubles only; production still registers zero SafetyFeatures.
class FailingAdvisoryFeature : public control_core::SafetyFeature {
public:
  explicit FailingAdvisoryFeature(bool throws = false) : throws_(throws) {}
  bool check(const control_core::State&, std::string* reason) override {
    if (throws_) throw std::runtime_error("test exception");
    *reason = "test check failure";
    return false;
  }
private:
  bool throws_;
};

TEST(ControlCoreAdvisoryTest, FailedFeaturesDoNotFreezeOrReplaceDesiredAction) {
  control_core::Controller core;
  core.configure({"a"});
  control_core::Command command;
  command.mode = control_core::Mode::Motion;
  command.position = 0.1;
  auto first = core.step({}, {{"a", command}}, .02);
  ASSERT_TRUE(first.warnings.empty()); // No features registered by default.
  core.add_safety(std::make_shared<FailingAdvisoryFeature>());
  command.position = 0.2;
  auto failed = core.step({}, {{"a", command}}, .02);
  ASSERT_EQ(failed.warnings.size(), 1u);
  EXPECT_EQ(failed.warnings[0], "test check failure");
  EXPECT_DOUBLE_EQ(failed.commands[0].position, 0.2);
  core.add_safety(std::make_shared<FailingAdvisoryFeature>(true));
  command.position = 0.3;
  auto throwing = core.step({}, {{"a", command}}, .02);
  ASSERT_EQ(throwing.warnings.size(), 2u);
  EXPECT_NE(throwing.warnings[1].find("test exception"), std::string::npos);
  EXPECT_DOUBLE_EQ(throwing.commands[0].position, 0.3);
}
