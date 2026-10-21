// Exercise real control-node callbacks without any CAN or motor backend.
#define MOTOR_CONTROL_HYBRID_NO_MAIN
#include "../src/cpp_control_node.cpp"
#include <gtest/gtest.h>
#include <limits>
#include <thread>

// A narrow test peer keeps production callbacks private while testing their
// state transition and watchdog behavior directly, without timing sleeps.
struct CppControlNodeTestPeer {
  static void observe(CppControlNode& node, const std::vector<std::string>& names) {
    auto msg = std::make_shared<sensor_msgs::msg::JointState>();
    msg->header.stamp = node.now();
    msg->name = names;
    msg->position.assign(names.size(), 0.1);
    msg->velocity.assign(names.size(), 0.0);
    msg->effort.assign(names.size(), 0.0);
    node.state_callback(msg);
  }
  static void home_checks(CppControlNode& node) {
    observe(node, {"a", "b"});
    node.home_lower_ = {-1, -1}; node.home_upper_ = {1, 1};
    node.safety_estop_active_.store(false);
    auto heartbeat = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    heartbeat->header.stamp = node.now();
    diagnostic_msgs::msg::DiagnosticStatus summary;
    summary.name = "humanoid_safety/supervisor"; summary.hardware_id = "humanoid";
    summary.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    heartbeat->status.push_back(summary); node.supervisor_callback(heartbeat);
    CppControlNode::Action action;
    action.request_id = "home-test"; action.joint_name = "a"; action.operation = CppControlNode::Action::HOME;
    action.header.stamp = node.now();
    action.valid_until = node.now() + rclcpp::Duration::from_seconds(0.1);
    std::string reason;
    node.home_lower_[0] = 0.01;
    EXPECT_FALSE(node.submit_action(action, reason));
    node.home_lower_[0] = -1;
    node.safety_estop_active_.store(true);
    EXPECT_FALSE(node.submit_action(action, reason));
    node.safety_estop_active_.store(false);
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    EXPECT_TRUE(node.home_active_);
    EXPECT_FALSE(node.action_status().zero_may_have_changed);
    EXPECT_FALSE(node.submit_action(action, reason));
    command(node, "b", 0.0);
    EXPECT_FALSE(node.has_cmd_);  // Competing targets cannot steal Home ownership.
    node.home_tick_ -= std::chrono::milliseconds(20);
    node.step_home(node.latest_state_map_);
    EXPECT_LT(node.home_targets_.at("a"), 0.1);
    EXPECT_GT(node.home_targets_.at("a"), 0.09);  // Bounded target step, no jump to zero.
    node.home_targets_.at("a") = 0;
    node.latest_state_map_.at("a").pos = 0;
    node.home_settle_ -= std::chrono::seconds(1);
    node.step_home(node.latest_state_map_);
    EXPECT_FALSE(node.home_active_);
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::COMPLETED);
    EXPECT_FALSE(node.has_cmd_);
    action.request_id = "home-fault"; action.header.stamp = node.now();
    action.valid_until = node.now() + rclcpp::Duration::from_seconds(0.1);
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    expire(node, "b"); node.control_loop();
    EXPECT_FALSE(node.home_active_);
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::FAILED);
  }
  static void lifecycle_checks(CppControlNode& node) {
    observe(node, {"a", "b"}); node.safety_estop_active_.store(false);
    auto heartbeat = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>(); heartbeat->header.stamp = node.now();
    diagnostic_msgs::msg::DiagnosticStatus summary; summary.name = "humanoid_safety/supervisor";
    summary.hardware_id = "humanoid"; summary.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    heartbeat->status.push_back(summary); node.supervisor_callback(heartbeat);
    CppControlNode::Action action; action.header.stamp = node.now();
    action.valid_until = node.now() + rclcpp::Duration::from_seconds(0.2);
    action.request_id = "enable-all"; action.joint_name = "*"; action.operation = CppControlNode::Action::ENABLE;
    std::string reason;
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    EXPECT_EQ(node.idle_enabled_.size(), 2u); EXPECT_TRUE(node.lifecycle_active_);
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::WAITING_BACKEND);
    auto expiry = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    node.enabled_evidence_["a"] = {true, expiry}; node.enabled_evidence_["b"] = {true, expiry};
    node.lifecycle_step(); EXPECT_FALSE(node.lifecycle_active_);
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::COMPLETED);
    action.header.stamp = node.now(); action.request_id = "disable-a"; action.joint_name = "a";
    action.operation = CppControlNode::Action::DISABLE;
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    EXPECT_EQ(node.idle_enabled_.count("a"), 0u); EXPECT_EQ(node.idle_enabled_.count("b"), 1u);
    node.enabled_evidence_["a"] = {false, expiry}; node.lifecycle_step();
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::COMPLETED);
    node.latest_state_map_.clear(); node.safety_estop_active_.store(true);
    action.header.stamp = node.now(); action.request_id = "disable-offline"; action.joint_name = "*";
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    EXPECT_TRUE(node.idle_enabled_.empty());
    action.operation = CppControlNode::Action::ENABLE; action.request_id = "enable-unsafe";
    EXPECT_FALSE(node.submit_action(action, reason));
    node.lifecycle_deadline_ -= std::chrono::seconds(3); node.lifecycle_step();
    EXPECT_EQ(node.action_status().stage, CppControlNode::ActionStatus::FAILED);
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

  static void enable_test_actions(CppControlNode& node) {
    node.actions_enabled_ = true; node.operation_limits_.settle_s = .04;
  }
  static void start_zero(CppControlNode& node) {
    observe(node, {"a", "b"});
    CppControlNode::Action action; action.header.stamp = node.now();
    action.request_id = "mock-zero"; action.joint_name = "a";
    action.operation = CppControlNode::Action::SET_MECHANICAL_ZERO;
    std::string reason;
    ASSERT_TRUE(node.submit_action(action, reason)) << reason;
    command(node, "a", 0); EXPECT_FALSE(node.has_cmd_); // Paused targets are discarded.
  }
  static bool backend_ready(CppControlNode& node) {
    return node.backend_client_->service_is_ready() && node.enabled_evidence_.size() == 2;
  }
  static bool zero_failed(CppControlNode& node) {return node.zero_.status().stage == control_core::OperationStage::Failed;}
  static std::string zero_reason(CppControlNode& node) {return node.zero_.status().message;}
  static void check_default_action_gate(CppControlNode& node) {
    CppControlNode::Action action; action.header.stamp = node.now();
    action.request_id = "default-off"; action.joint_name = "a";
    action.operation = CppControlNode::Action::SET_MECHANICAL_ZERO;
    std::string reason; EXPECT_FALSE(node.submit_action(action, reason));
    EXPECT_NE(reason.find("Hardware Set Zero unavailable"), std::string::npos);
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

TEST_F(CppStateWatchdogTest, AbstractActionsDefaultOff) {
  CppControlNode node; CppControlNodeTestPeer::check_default_action_gate(node);
}

TEST_F(CppStateWatchdogTest, AsyncMockBackendRejectionKeepsCorePaused) {
  auto node = std::make_shared<CppControlNode>();
  auto backend = std::make_shared<rclcpp::Node>("mock_zero_backend");
  using Service = motor_control_interfaces::srv::ExecuteMotorOperation;
  size_t calls = 0;
  auto service = backend->create_service<Service>("/motor_backend/execute_operation",
    [&calls](const Service::Request::SharedPtr req, Service::Response::SharedPtr res) {
      ++calls; res->request_id = req->request_id; res->accepted = false;
      res->state = Service::Response::REJECTED; res->message = "mock unsupported profile";
    });
  using Feedback = motor_control_interfaces::srv::GetMotorFeedback;
  auto feedback_service = backend->create_service<Feedback>("/motor_backend/get_feedback",
    [clock = backend->get_clock()](const Feedback::Request::SharedPtr req, Feedback::Response::SharedPtr res) {
      res->success = true; res->feedback.joint_name = req->joint_name;
      res->feedback.available = res->feedback.fresh = true;
      res->feedback.stamp = clock->now(); res->feedback.age_s = 0;
      res->feedback.status_mode = req->joint_name == "a" ? 2 : 0;
    });
  CppControlNodeTestPeer::enable_test_actions(*node);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node); executor.add_node(backend);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!CppControlNodeTestPeer::backend_ready(*node) && std::chrono::steady_clock::now() < deadline) {
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_TRUE(CppControlNodeTestPeer::backend_ready(*node));
  CppControlNodeTestPeer::start_zero(*node);
  while (!CppControlNodeTestPeer::zero_failed(*node) && std::chrono::steady_clock::now() < deadline) {
    CppControlNodeTestPeer::observe(*node, {"a", "b"});
    executor.spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_TRUE(CppControlNodeTestPeer::zero_failed(*node)); EXPECT_EQ(calls, 1u);
  EXPECT_NE(CppControlNodeTestPeer::zero_reason(*node).find("mock unsupported profile"), std::string::npos);
  executor.remove_node(node); executor.remove_node(backend);
}

TEST_F(CppStateWatchdogTest, HomeUsesCurrentZeroAndHonorsOwnershipLimitsAndWatchdog) {
  auto node = std::make_shared<CppControlNode>();
  CppControlNodeTestPeer::home_checks(*node);
}

TEST_F(CppStateWatchdogTest, LifecycleVerifiesFeedbackAndDisableWorksOffline) {
  auto node = std::make_shared<CppControlNode>();
  CppControlNodeTestPeer::lifecycle_checks(*node);
}
