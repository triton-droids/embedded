// C++ RobStride codec vs vectors generated from the Python reference modules
// (test/gen_robstride_vectors.py). Frames must match bit for bit; conversions
// to 1e-12. No ROS, CAN socket or motor is used.
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <string>

#include "motor_control_hybrid/robstride_codec.hpp"
#include "motor_control_hybrid/robstride_registry.hpp"

namespace {

const YAML::Node& vectors() {
  static const YAML::Node v = YAML::LoadFile(std::string(TEST_DATA_DIR) + "/robstride_vectors.json");
  return v;
}

std::string hex(const std::array<uint8_t, 8>& data) {
  std::string out;
  char buf[3];
  for (const uint8_t b : data) {
    std::snprintf(buf, sizeof(buf), "%02x", b);
    out += buf;
  }
  return out;
}

std::array<uint8_t, 8> unhex(const std::string& s) {
  std::array<uint8_t, 8> out{};
  for (size_t i = 0; i < 8; ++i) {
    out[i] = static_cast<uint8_t>(std::stoul(s.substr(2 * i, 2), nullptr, 16));
  }
  return out;
}

TEST(RobstrideCodec, OperationFramesMatchPythonBitForBit) {
  const auto& rows = vectors()["operation"];
  ASSERT_GE(rows.size(), 100u);
  for (const auto& r : rows) {
    const auto p = robstride::profile_for(r["model"].as<std::string>());
    ASSERT_TRUE(p);
    const auto f = robstride::operation_frame(*p, r["device"].as<int>(), r["position"].as<double>(),
                                              r["kp"].as<double>(), r["kd"].as<double>(),
                                              r["velocity"].as<double>(), r["torque"].as<double>());
    EXPECT_EQ(f.id, r["id"].as<uint32_t>());
    EXPECT_EQ(hex(f.data), r["data"].as<std::string>());
  }
}

TEST(RobstrideCodec, LifecycleAndParameterFramesMatchPython) {
  const int host = vectors()["host_id"].as<int>();
  for (const auto& r : vectors()["lifecycle"]) {
    const int device = r["device"].as<int>();
    const auto f = r["kind"].as<std::string>() == "enable" ? robstride::enable_frame(host, device)
                                                           : robstride::disable_frame(host, device);
    EXPECT_EQ(f.id, r["id"].as<uint32_t>());
    EXPECT_EQ(hex(f.data), r["data"].as<std::string>());
  }
  for (const auto& r : vectors()["parameters"]) {
    const int device = r["device"].as<int>();
    const auto param = static_cast<uint16_t>(r["param"].as<int>());
    const auto f = r["kind"].as<std::string>() == "int8"
                       ? robstride::write_int8(host, device, param, r["value"].as<int>())
                       : robstride::write_float(host, device, param, r["value"].as<double>());
    EXPECT_EQ(f.id, r["id"].as<uint32_t>());
    EXPECT_EQ(hex(f.data), r["data"].as<std::string>()) << "param " << param;
  }
}

TEST(RobstrideCodec, FeedbackDecodingMatchesPython) {
  const int host = vectors()["host_id"].as<int>();
  const std::map<int, std::string> models = {{1, "rs-02"}, {2, "rs-03"}, {3, "rs-04"}};
  const auto lookup = [&](int device) -> std::optional<robstride::Profile> {
    const auto it = models.find(device);
    return it == models.end() ? std::nullopt : robstride::profile_for(it->second);
  };
  int status = 0, faults = 0, parameters = 0;
  for (const auto& r : vectors()["decode"]) {
    const auto data = unhex(r["data"].as<std::string>());
    robstride::Feedback fb;
    const auto res = robstride::decode(r["id"].as<uint32_t>(), true, false, false, 8, data.data(),
                                       host, lookup, fb);
    const std::string want = r["result"].as<std::string>();
    if (want == "status") {
      ASSERT_EQ(res, robstride::Decoded::kStatus) << r["id"].as<uint32_t>();
      ++status;
      EXPECT_EQ(fb.motor_id, r["motor_id"].as<int>());
      EXPECT_DOUBLE_EQ(fb.position, r["position"].as<double>());
      EXPECT_DOUBLE_EQ(fb.velocity, r["velocity"].as<double>());
      EXPECT_DOUBLE_EQ(fb.torque, r["torque"].as<double>());
      EXPECT_DOUBLE_EQ(fb.temperature, r["temperature"].as<double>());
      EXPECT_EQ(fb.mode, r["mode"].as<int>());
      EXPECT_EQ(fb.fault_bits, r["fault_bits"].as<int>());
    } else if (want == "fault") {
      EXPECT_EQ(res, robstride::Decoded::kFault);
      ++faults;
    } else if (want == "parameter") {
      ASSERT_EQ(res, robstride::Decoded::kParameter) << r["id"].as<uint32_t>();
      ++parameters;
      EXPECT_EQ(fb.motor_id, r["motor_id"].as<int>());
      EXPECT_EQ(fb.param_index, r["index"].as<int>());
      EXPECT_EQ(fb.param_value, r["value"].as<uint32_t>());
    } else {
      EXPECT_EQ(res, robstride::Decoded::kIgnored) << r["id"].as<uint32_t>();
    }
  }
  EXPECT_GT(status, 50);
  EXPECT_GT(faults, 3);
  EXPECT_GT(parameters, 3);
}

TEST(RobstrideCodec, MalformedFramesAreIgnoredAndErrorFramesReported) {
  const auto lookup = [](int) { return robstride::profile_for("rs-03"); };
  const uint8_t data[8] = {};
  robstride::Feedback fb;
  const uint32_t id = (2u << 24) | (5u << 8) | 255u;
  EXPECT_EQ(robstride::decode(id, false, false, false, 8, data, 255, lookup, fb),
            robstride::Decoded::kIgnored);
  EXPECT_EQ(robstride::decode(id, true, true, false, 8, data, 255, lookup, fb),
            robstride::Decoded::kIgnored);
  EXPECT_EQ(robstride::decode(id, true, false, false, 6, data, 255, lookup, fb),
            robstride::Decoded::kIgnored);
  EXPECT_EQ(robstride::decode(id, true, false, true, 8, data, 255, lookup, fb),
            robstride::Decoded::kErrorFrame);
  EXPECT_THROW(robstride::arbitration_id(1, 0, 255), std::invalid_argument);
  EXPECT_THROW(robstride::operation_frame(*robstride::profile_for("rs-02"), 1, NAN, 1, 1, 0, 0),
               std::invalid_argument);
  EXPECT_FALSE(robstride::profile_for("rs-01"));
}

TEST(RobstrideCodec, JointConversionsMatchPythonIncludingAnkleLinkage) {
  int ankles = 0;
  for (const auto& c : vectors()["settings"]) {
    const auto settings = robstride::settings_from_config(c["config"], c["defaults"]);
    ankles += settings.ankle.has_value();
    robstride::AnkleState state;
    for (const auto& s : c["steps"]) {
      double p = s["fb_in"][0].as<double>(), v = s["fb_in"][1].as<double>(),
             t = s["fb_in"][2].as<double>();
      settings.feedback(p, v, t, state, s["now"].as<double>());
      EXPECT_NEAR(p, s["fb_out"][0].as<double>(), 1e-12) << c["joint"].as<std::string>();
      EXPECT_NEAR(v, s["fb_out"][1].as<double>(), 1e-9) << c["joint"].as<std::string>();
      EXPECT_NEAR(t, s["fb_out"][2].as<double>(), 1e-12);
      robstride::MotorCommandValues in;
      const auto& ci = s["cmd_in"];
      in.position = ci["position"].as<double>();
      in.velocity = ci["velocity"].as<double>();
      in.acceleration = ci["acceleration"].as<double>();
      in.torque = ci["torque"].as<double>();
      in.kp = ci["kp"].as<double>();
      in.kd = ci["kd"].as<double>();
      const auto out = settings.command(in, state);
      const auto& co = s["cmd_out"];
      EXPECT_NEAR(out.position, co["position"].as<double>(), 1e-12) << c["joint"].as<std::string>();
      EXPECT_NEAR(out.velocity, co["velocity"].as<double>(), 1e-12);
      EXPECT_NEAR(out.torque, co["torque"].as<double>(), 1e-12);
      EXPECT_DOUBLE_EQ(out.kp, co["kp"].as<double>());
      EXPECT_DOUBLE_EQ(out.kd, co["kd"].as<double>());
    }
  }
  EXPECT_EQ(ankles, 2);
}

TEST(RobstrideRegistry, DeployedRegistryParsesAndRulesMatchPython) {
  const YAML::Node root = YAML::LoadFile(std::string(CONFIG_DIR) + "/motors.yaml");
  // The deployed registry is (correctly) marked unverified.
  EXPECT_THROW(robstride::parse_registry(root), std::invalid_argument);
  YAML::Node copy = YAML::Clone(root);
  YAML::Node params = copy["motor_control_node"] ? copy["motor_control_node"]["ros__parameters"]
                                                 : copy;
  params["hardware_verified"] = true;
  const auto reg = robstride::parse_registry(copy);
  ASSERT_EQ(reg.joints.size(), 10u);
  EXPECT_EQ(reg.joints.front().name, "left_hip1_joint");
  params["motors"]["left_knee_joint"]["motor_id"] = 6;  // duplicate of right_hip1
  EXPECT_THROW(robstride::parse_registry(copy), std::invalid_argument);
}

}  // namespace

// Driver-side motion guards; same cases as test_motor_configuration.MotionSafetyLimitTest.
namespace {
robstride::MotorSettings guarded(const std::string& extra = "{}") {
  YAML::Node cfg = YAML::Load("{model: rs-03, min_position: -1, max_position: 1, kp: 100, "
                              "kd: 1, max_torque_nm: 40}");
  for (const auto& kv : YAML::Load(extra)) cfg[kv.first.as<std::string>()] = kv.second;
  return robstride::settings_from_config(cfg, YAML::Load("{MAX_POSITION_ERROR_RAD: 0.3}"));
}
robstride::MotorCommandValues target(double position, double kp = 100, double ff = 0) {
  robstride::MotorCommandValues c;
  c.position = position;
  c.torque = ff;
  c.kp = kp;
  c.kd = 1;
  return c;
}
}  // namespace

TEST(RobstrideGuards, TargetStaysNearMeasuredPositionWithoutTorqueBudget) {
  robstride::AnkleState state;
  EXPECT_NEAR(guarded().command(target(0.9), state, true, 0.0).position, 0.3, 1e-12);
  const auto c = guarded().command(target(-0.9, 200, 10), state, true, 0.0);
  EXPECT_DOUBLE_EQ(c.kp, 100);
  EXPECT_NEAR(c.position, -0.3, 1e-12);  // MAX_POSITION_ERROR_RAD, not a torque budget
  EXPECT_NEAR(guarded("{max_torque_nm: 5}").command(target(-0.9), state, true, 0.0).position, -0.3,
              1e-12);
  const auto probe = guarded().command(target(0.9, 100, 5), state, true, std::nullopt);
  EXPECT_DOUBLE_EQ(probe.kp, 0.0);  // no feedback: zero-gain, zero-torque probe
  EXPECT_DOUBLE_EQ(probe.kd, 0.0);
  EXPECT_DOUBLE_EQ(probe.torque, 0.0);
  EXPECT_NO_THROW(guarded().command(target(0.1), state, false, std::nullopt));
}

TEST(RobstrideGuards, FeedbackJumpAndRegistryLimits) {
  const auto s = guarded();
  EXPECT_NO_THROW(s.check_feedback_step(0.0, 1.0, 0.1, 1.002));
  EXPECT_THROW(s.check_feedback_step(0.0, 1.0, 0.8, 1.002), std::runtime_error);
  EXPECT_NO_THROW(s.check_feedback_step(0.0, 1.0, 0.8, 1.5));
  EXPECT_THROW(guarded("{model: rs-02, max_torque_nm: 20}"), std::invalid_argument);
  EXPECT_THROW(guarded("{hard_min_position: -0.9}"), std::invalid_argument);
  EXPECT_NO_THROW(guarded("{hard_min_position: -1.2, hard_max_position: null}"));
}

TEST(RobstrideGuards, CanTimeoutSecondsBecomeManualTicks) {
  const auto none = YAML::Load("{}");
  const auto w = robstride::parse_firmware_writes(YAML::Load("{}"),
                                                  YAML::Load("{FIRMWARE_CAN_TIMEOUT_S: 1.0}"));
  ASSERT_EQ(w.size(), 1u);
  EXPECT_EQ(w[0].index, 0x7028);
  EXPECT_FALSE(w[0].is_float);
  EXPECT_DOUBLE_EQ(w[0].value, 20000);  // 20000 = 1 s
  EXPECT_DOUBLE_EQ(
      robstride::parse_firmware_writes(YAML::Load("{firmware_can_timeout_s: 0.75}"), none)[0].value,
      15000);
  EXPECT_TRUE(robstride::parse_firmware_writes(YAML::Load("{firmware_can_timeout_s: null}"),
                                               YAML::Load("{FIRMWARE_CAN_TIMEOUT_S: 1.0}"))
                  .empty());
  for (const char* bad : {"{firmware_can_timeout_s: 0}", "{firmware_can_timeout_s: 5.5}",
                          "{firmware_can_timeout_s: true}", "{firmware_can_timeout_s: '1'}"}) {
    EXPECT_THROW(robstride::parse_firmware_writes(YAML::Load(bad), none), std::invalid_argument)
        << bad;
  }
  EXPECT_EQ(hex(robstride::write_uint32(0xFD, 1, 0x7028, 20000).data), "28700000204e0000");
}

TEST(RobstrideCodec, ParameterReadRequestAndReply) {
  const auto req = robstride::read_parameter(0xFD, 1, 0x7028);
  EXPECT_EQ(req.id, (17u << 24) | (0xFDu << 8) | 1u);
  EXPECT_EQ(hex(req.data), "2870000000000000");
  const auto reply = unhex("28700000204e0000");
  robstride::Feedback fb;
  const auto lookup = [](int) { return robstride::profile_for("rs-04"); };
  EXPECT_EQ(robstride::decode((17u << 24) | (1u << 8) | 0xFDu, true, false, false, 8, reply.data(),
                              0xFD, lookup, fb),
            robstride::Decoded::kParameter);
  EXPECT_EQ(fb.motor_id, 1);
  EXPECT_EQ(fb.param_index, 0x7028);
  EXPECT_EQ(fb.param_value, 20000u);
  EXPECT_EQ(robstride::decode((17u << 24) | (1u << 8) | 0x01u, true, false, false, 8, reply.data(),
                              0xFD, lookup, fb),
            robstride::Decoded::kIgnored);  // another host
}
