// ROS 2 RobStride CAN backend in C++: drop-in replacement for python_can_node.
//
// Same node contract (topics, services, parameters, watchdog semantics) and the
// same audited private-protocol frames (robstride_codec.hpp, checked bit for bit
// against can_transport.py). Differences from python_can_node:
//   - TX, RX and ROS callbacks run truly in parallel (no GIL); SocketCAN is used
//     directly, so TX never waits behind ROS work or logging.
//   - The registry is fully validated (including the protocol profile) before
//     any socket opens; an interface that cannot be opened stops start-up
//     instead of leaving joints without a bus.
//   - Repeated lifecycle submissions and stale-command rejections log at DEBUG /
//     throttled WARN, never once per frame.
//
// Contract kept from python_can_node:
//   sub  motor_commands  (MotorCommand)  per-joint mode/pos/vel/torque/kp/kd
//   sub  /safety/estop   (Bool, reliable + transient_local) latched stop
//   pub  joint_states    (JointState)    registry order, fresh samples only
//   pub  motor_status    (String JSON)   {"<index>": {temperature, torque}}
//   pub  motor_telemetry (String JSON)   {schema_version: 1, motors: [...]}
//   pub  can_timing      (String JSON)   1 Hz TX/RX counters
//   srv  motor_backend/get_feedback       cached raw motor-coordinate sample
//   srv  motor_backend/execute_operation  zero admission; always rejected
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <limits>
#include <map>
#include <memory>
#include <motor_control_interfaces/msg/motor_command.hpp>
#include <motor_control_interfaces/srv/execute_motor_operation.hpp>
#include <motor_control_interfaces/srv/get_motor_feedback.hpp>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <set>
#include <sstream>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "motor_control_hybrid/robstride_codec.hpp"
#include "motor_control_hybrid/robstride_registry.hpp"

namespace {
using MotorCommand = motor_control_interfaces::msg::MotorCommand;
using ExecuteMotorOperation = motor_control_interfaces::srv::ExecuteMotorOperation;
using GetMotorFeedback = motor_control_interfaces::srv::GetMotorFeedback;

constexpr double kInf = std::numeric_limits<double>::infinity();

// time.monotonic() / time.time() equivalents, in seconds.
double mono() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
double wall() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string json_number(double v) {
  if (!std::isfinite(v)) {
    return "null";  // JSON has no NaN/Inf
  }
  char buf[64];
  const auto res = std::to_chars(buf, buf + sizeof(buf), v);
  return std::string(buf, res.ptr);
}

std::string json_escape(const std::string& s) {
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
    }
    if (static_cast<unsigned char>(c) >= 0x20) {
      out += c;
    }
  }
  return out;
}

std::string hex(const uint8_t* data, size_t n) {
  static const char* digits = "0123456789abcdef";
  std::string out;
  for (size_t i = 0; i < n; ++i) {
    out += digits[data[i] >> 4];
    out += digits[data[i] & 15];
  }
  return out;
}

struct Command {
  std::string joint;
  uint8_t mode{MotorCommand::MODE_VELOCITY};
  double received_at{0.0};
  double deadline{0.0};
  robstride::MotorCommandValues values;
};

// Port of can_transport.LatestCommandMailbox (insertion order, <= 2 per joint).
class LatestCommandMailbox {
 public:
  void put(const std::vector<Command>& commands) {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& command : commands) {
      auto& slot = find_or_add(command.joint);
      if (command.mode == MotorCommand::MODE_ENABLE || command.mode == MotorCommand::MODE_DISABLE) {
        slot.lifecycle = command;
        slot.motion.reset();
      } else if (!slot.lifecycle || slot.lifecycle->mode != MotorCommand::MODE_DISABLE) {
        slot.motion = command;
      }
    }
  }

  std::optional<Command> get() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (slots_.empty()) {
      return std::nullopt;
    }
    auto& slot = slots_.front().second;
    std::optional<Command> out;
    if (slot.lifecycle) {
      out = std::move(slot.lifecycle);
      slot.lifecycle.reset();
    } else {
      out = std::move(slot.motion);
      slot.motion.reset();
    }
    if (!slot.lifecycle && !slot.motion) {
      slots_.erase(slots_.begin());
    }
    return out;
  }

  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    slots_.clear();
  }

  size_t size() {
    std::lock_guard<std::mutex> lock(mutex_);
    size_t n = 0;
    for (const auto& entry : slots_) {
      n += entry.second.lifecycle.has_value() + entry.second.motion.has_value();
    }
    return n;
  }

 private:
  struct Slot {
    std::optional<Command> lifecycle;
    std::optional<Command> motion;
  };

  Slot& find_or_add(const std::string& joint) {
    for (auto& entry : slots_) {
      if (entry.first == joint) {
        return entry.second;
      }
    }
    slots_.emplace_back(joint, Slot{});
    return slots_.back().second;
  }

  std::mutex mutex_;
  std::vector<std::pair<std::string, Slot>> slots_;
};

// One SocketCAN raw socket per interface: TX nonblocking, RX by poll().
struct Bus {
  std::string channel;
  int fd{-1};
  int host_id{255};
  std::map<int, robstride::Profile> profiles;  // device id -> audited profile
  std::map<int, std::string> joint_by_device;
  std::mutex lock;  // python bus_locks: serialize multi-frame sequences
  std::atomic<uint64_t> tx_frames{0};
  std::atomic<uint64_t> tx_errors{0};

  void open(int host) {
    host_id = host;
    fd = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW);
    if (fd < 0) {
      throw std::runtime_error(channel + ": socket(): " + std::strerror(errno));
    }
    // Receive only status (2), fault (21) and parameter-read (17) frames for this host.
    can_filter filters[3];
    const uint32_t kinds[3] = {robstride::kOperationStatus, robstride::kFaultReport,
                               robstride::kReadParameter};
    for (int i = 0; i < 3; ++i) {
      const uint32_t kind = kinds[i];
      filters[i].can_id = ((kind << 24) | static_cast<uint32_t>(host)) | CAN_EFF_FLAG;
      filters[i].can_mask = ((31u << 24) | 255u) | CAN_EFF_FLAG;  // as python-can
    }
    // python-can defaults: error frames delivered, own messages not looped back.
    const can_err_mask_t err_mask = CAN_ERR_MASK;
    const int own = 0;
    const int one = 1;
    if (setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0 ||
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0 ||
        setsockopt(fd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, &own, sizeof(own)) < 0 ||
        setsockopt(fd, SOL_SOCKET, SO_TIMESTAMPNS, &one, sizeof(one)) < 0) {
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error(channel + ": setsockopt(): " + err);
    }
    ifreq ifr{};
    std::strncpy(ifr.ifr_name, channel.c_str(), IFNAMSIZ - 1);
    if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error(channel + ": interface not found: " + err);
    }
    sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      const std::string err = std::strerror(errno);
      close();
      throw std::runtime_error(channel + ": bind(): " + err);
    }
  }

  void close() {
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }

  // Zero-wait submit: backpressure raises immediately, like send(timeout=0).
  void send(const robstride::Frame& f) {
    can_frame frame{};
    frame.can_id = f.id | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    std::memcpy(frame.data, f.data.data(), 8);
    const ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
      tx_errors++;
      throw std::runtime_error(channel + ": CAN send failed: " +
                               (n < 0 ? std::strerror(errno) : "short write"));
    }
    tx_frames++;
  }
};

struct JointRuntime {
  robstride::JointConfig cfg;
  size_t index{0};
  Bus* bus{nullptr};
  robstride::Profile profile{};
  std::mutex ankle_lock;  // RX feedback writes, TX command reads the guesses
  robstride::AnkleState ankle;
};

struct Sample {  // joint coordinates, for joint_states / motor_status
  double position{0}, velocity{0}, torque{0}, temperature{0};
  double received_at{0};
};

struct RawSample {  // motor coordinates, for get_feedback
  robstride::Feedback fb;
  double received_at{0};
  double source_stamp{0};
};

// threading.Event equivalent for the TX wake-up.
class WakeEvent {
 public:
  void set() {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      flag_ = true;
    }
    cv_.notify_all();
  }
  void clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    flag_ = false;
  }
  void wait(double seconds) {
    std::unique_lock<std::mutex> lock(mutex_);
    cv_.wait_for(lock, std::chrono::duration<double>(std::max(0.0, seconds)),
                 [this] { return flag_; });
  }

 private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool flag_{false};
};

}  // namespace

class RobstrideCanNode : public rclcpp::Node {
 public:
  RobstrideCanNode() : Node("robstride_can_node") {
    declare_parameter<std::string>("motor_config_file", "");
    declare_parameter<bool>("can_rt_enabled", false);
    declare_parameter<int>("can_tx_rt_priority", 20);
    declare_parameter<int>("can_rx_rt_priority", 21);
    declare_parameter<double>("publish_rate_hz", 50.0);
    // Legacy launch compatibility only: RX continuously consumes frames.
    declare_parameter<double>("feedback_poll_hz", 50.0);
    declare_parameter<bool>("feedback_poll_when_idle", true);
    declare_parameter<double>("feedback_timeout_s", 0.1);
    declare_parameter<double>("command_timeout_s", 0.5);

    can_rt_enabled_ = get_parameter("can_rt_enabled").as_bool();
    can_tx_rt_priority_ = static_cast<int>(get_parameter("can_tx_rt_priority").as_int());
    can_rx_rt_priority_ = static_cast<int>(get_parameter("can_rx_rt_priority").as_int());
    for (const int priority : {can_tx_rt_priority_, can_rx_rt_priority_}) {
      if (priority < 1 || priority > 99) {
        throw std::invalid_argument("CAN RT priorities must be between 1 and 99");
      }
    }
    command_timeout_s_ = get_parameter("command_timeout_s").as_double();
    if (!std::isfinite(command_timeout_s_) || command_timeout_s_ <= 0) {
      throw std::invalid_argument("command_timeout_s must be finite and positive");
    }
    feedback_timeout_s_ = get_parameter("feedback_timeout_s").as_double();
    if (!(feedback_timeout_s_ > 0 && feedback_timeout_s_ <= 1.0)) {
      throw std::invalid_argument("feedback_timeout_s must be in (0, 1]");
    }
    const double publish_rate = get_parameter("publish_rate_hz").as_double();
    if (!std::isfinite(publish_rate) || publish_rate <= 0) {
      throw std::invalid_argument("publish_rate_hz must be finite and positive");
    }
    tx_period_s_ = 1.0 / publish_rate;

    load_and_connect(get_parameter("motor_config_file").as_string());
    for (const auto& j : joints_) {
      inhibited_since_[j->cfg.name] = -kInf;
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    motor_status_pub_ = create_publisher<std_msgs::msg::String>("motor_status", 10);
    telemetry_pub_ = create_publisher<std_msgs::msg::String>("motor_telemetry", 10);
    timing_pub_ = create_publisher<std_msgs::msg::String>("can_timing", 1);
    timing_timer_ = create_wall_timer(std::chrono::seconds(1), [this] { publish_timing(); });
    cmd_sub_ = create_subscription<MotorCommand>(
        "motor_commands", 10, [this](MotorCommand::SharedPtr msg) { cmd_callback(*msg); });
    feedback_service_ = create_service<GetMotorFeedback>(
        "motor_backend/get_feedback",
        [this](const std::shared_ptr<GetMotorFeedback::Request> req,
               std::shared_ptr<GetMotorFeedback::Response> res) { get_feedback(*req, *res); });
    operation_service_ = create_service<ExecuteMotorOperation>(
        "motor_backend/execute_operation",
        [this](const std::shared_ptr<ExecuteMotorOperation::Request> req,
               std::shared_ptr<ExecuteMotorOperation::Response> res) { operation(*req, *res); });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/safety/estop", rclcpp::QoS(1).reliable().transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr msg) { estop_callback(msg->data); });
    publish_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(tx_period_s_)),
        [this] { publish_states(); });

    running_ = true;
    for (auto& entry : buses_) {
      Bus* bus = entry.second.get();
      rx_threads_.emplace_back([this, bus] { rx_loop(*bus); });
    }
    tx_thread_ = std::thread([this] { tx_loop(); });

    RCLCPP_INFO(get_logger(), "RobStride C++ CAN node started with %zu motors on %zu CAN interface(s)",
                joints_.size(), buses_.size());
  }

  ~RobstrideCanNode() override {
    RCLCPP_INFO(get_logger(), "Shutting down RobStride C++ CAN node...");
    running_ = false;
    tx_wakeup_.set();
    // Join the owners before sending final disables and closing their sockets.
    if (tx_thread_.joinable()) {
      tx_thread_.join();
    }
    for (auto& t : rx_threads_) {
      if (t.joinable()) {
        t.join();
      }
    }
    for (const auto& j : joints_) {
      try {
        std::lock_guard<std::mutex> lock(j->bus->lock);
        j->bus->send(robstride::disable_frame(j->bus->host_id, j->cfg.motor_id));
      } catch (const std::exception&) {
      }
    }
    for (auto& entry : buses_) {
      entry.second->close();
    }
  }

 private:
  // ---------------------------------------------------------------- config
  void load_and_connect(const std::string& path) {
    // Refuse to come up half-alive: a node with no motors still answers on
    // /motor_commands, reports empty feedback and looks healthy to operators.
    if (path.empty()) {
      throw std::invalid_argument(
          "motor_config_file is required: refusing to start a CAN node with no motors. "
          "Pass the hardware registry (e.g. config/motors.yaml), or run fake_motor_node "
          "when there is no CAN hardware.");
    }
    RCLCPP_INFO(get_logger(), "Loading motor config from: %s", path.c_str());
    const robstride::Registry reg = robstride::parse_registry(YAML::LoadFile(path));
    max_vel_rad_s_ = reg.max_vel_rad_s;
    for (const auto& cfg : reg.joints) {
      for (const auto& w : cfg.firmware_writes) {
        if (w.index == robstride::kCanTimeoutIndex &&
            w.value / robstride::kCanTimeoutTicksPerS <= command_timeout_s_) {
          throw std::invalid_argument(cfg.name + ": firmware_can_timeout_s must exceed "
                                      "command_timeout_s; the host watchdog acts first");
        }
      }
    }
    for (const auto& cfg : reg.joints) {
      auto& bus = buses_[cfg.can_interface];
      if (!bus) {
        bus = std::make_unique<Bus>();
        bus->channel = cfg.can_interface;
        bus->host_id = cfg.master_id;
      }
      auto j = std::make_unique<JointRuntime>();
      j->cfg = cfg;
      j->index = joints_.size();
      j->bus = bus.get();
      j->profile = *robstride::profile_for(cfg.model);
      bus->profiles[cfg.motor_id] = j->profile;
      bus->joint_by_device[cfg.motor_id] = cfg.name;
      joint_by_name_[cfg.name] = j.get();
      RCLCPP_INFO(get_logger(), "Configured motor[%zu]: %s on %s, motor_id=%d, model=%s", j->index,
                  cfg.name.c_str(), cfg.can_interface.c_str(), cfg.motor_id, cfg.model.c_str());
      joints_.push_back(std::move(j));
    }
    for (auto& entry : buses_) {
      entry.second->open(entry.second->host_id);
      RCLCPP_INFO(get_logger(), "Opened SocketCAN %s with %zu motor(s), host id %d",
                  entry.first.c_str(), entry.second->profiles.size(), entry.second->host_id);
    }
  }

  void configure_scheduling(const std::string& role, int priority) {
    const long tid = static_cast<long>(gettid());
    if (!can_rt_enabled_) {
      RCLCPP_INFO(get_logger(), "CAN %s TID=%ld: existing scheduling", role.c_str(), tid);
      return;
    }
    sched_param param{};
    param.sched_priority = priority;
    const int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
    if (rc == 0) {
      RCLCPP_INFO(get_logger(), "CAN %s TID=%ld: SCHED_RR priority=%d", role.c_str(), tid, priority);
    } else {
      RCLCPP_WARN(get_logger(),
                  "CAN %s TID=%ld: cannot set SCHED_RR priority=%d: %s; continuing with existing "
                  "scheduling",
                  role.c_str(), tid, priority, std::strerror(rc));
    }
  }

  void latch_transport_fault() {
    if (!transport_fault_.exchange(true)) {
      estop_disable_pending_ = true;
      tx_wakeup_.set();
    }
  }

  // ---------------------------------------------------------------- RX
  void update_state(JointRuntime& j, double pos, double vel, double tq, double temp,
                    double received_at) {
    {
      std::lock_guard<std::mutex> lock(j.ankle_lock);
      j.cfg.settings.feedback(pos, vel, tq, j.ankle, mono());
    }
    const double now = mono();
    const double source_stamp = now_ros() - std::max(0.0, now - received_at);
    std::lock_guard<std::mutex> lock(state_lock_);
    telemetry_stamps_[j.cfg.name] = source_stamp;
    state_buffer_[j.cfg.name] = Sample{pos, vel, tq, temp, received_at};
  }

  void rx_loop(Bus& bus) {
    configure_scheduling("RX " + bus.channel, can_rx_rt_priority_);
    const auto lookup = [&bus](int device) -> std::optional<robstride::Profile> {
      const auto it = bus.profiles.find(device);
      if (it == bus.profiles.end()) {
        return std::nullopt;
      }
      return it->second;
    };
    while (running_ && rclcpp::ok()) {
      try {
        pollfd pfd{bus.fd, POLLIN, 0};
        const int ready = ::poll(&pfd, 1, 10);  // bounded idle wait, like recv(timeout=0.01)
        if (ready <= 0) {
          if (ready < 0 && errno != EINTR) {
            throw std::runtime_error(std::string("poll(): ") + std::strerror(errno));
          }
          continue;
        }
        can_frame frame{};
        char control[CMSG_SPACE(sizeof(timespec))];
        iovec iov{&frame, sizeof(frame)};
        msghdr msg{};
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);
        const ssize_t n = ::recvmsg(bus.fd, &msg, MSG_DONTWAIT);
        if (n < 0) {
          if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            continue;
          }
          throw std::runtime_error(std::string("recvmsg(): ") + std::strerror(errno));
        }
        double stamp = 0.0;  // kernel receive time (wall clock), as python-can
        for (cmsghdr* c = CMSG_FIRSTHDR(&msg); c != nullptr; c = CMSG_NXTHDR(&msg, c)) {
          if (c->cmsg_level == SOL_SOCKET && c->cmsg_type == SO_TIMESTAMPNS) {
            timespec ts{};
            std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
            stamp = ts.tv_sec + ts.tv_nsec * 1e-9;
          }
        }
        const bool error = (frame.can_id & CAN_ERR_FLAG) != 0;
        const bool extended = (frame.can_id & CAN_EFF_FLAG) != 0;
        const bool remote = (frame.can_id & CAN_RTR_FLAG) != 0;
        const uint32_t id = frame.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
        robstride::Feedback fb;
        const auto res = robstride::decode(id, extended, remote, error, frame.can_dlc, frame.data,
                                           bus.host_id, lookup, fb);
        if (res == robstride::Decoded::kErrorFrame) {
          throw std::runtime_error("SocketCAN error frame");
        }
        if (res == robstride::Decoded::kFault) {
          throw std::runtime_error("motor_" + std::to_string(fb.motor_id) +
                                   ": motor fault report " + hex(frame.data, 8));
        }
        if (res == robstride::Decoded::kIgnored) {
          rx_ignored_++;
          continue;
        }
        if (res == robstride::Decoded::kParameter) {
          const std::string& joint = bus.joint_by_device.at(fb.motor_id);
          {
            std::lock_guard<std::mutex> lock(state_lock_);
            param_readback_[{joint, fb.param_index}] = {fb.param_value, mono()};
          }
          tx_wakeup_.set();
          continue;
        }
        rx_frames_++;
        const double age = stamp > 0 ? std::max(0.0, wall() - stamp) : 0.0;
        JointRuntime& j = *joint_by_name_.at(bus.joint_by_device.at(fb.motor_id));
        if (age <= feedback_timeout_s_) {
          std::optional<RawSample> previous;
          const double received = mono() - age;
          {
            std::lock_guard<std::mutex> lock(state_lock_);
            const auto it = raw_feedback_.find(j.cfg.name);
            if (it != raw_feedback_.end()) previous = it->second;
            raw_feedback_[j.cfg.name] = RawSample{fb, received, now_ros() - age};
          }
          // Throws (latching the transport fault) on an impossible step.
          if (previous) {
            j.cfg.settings.check_feedback_step(previous->fb.position, previous->received_at,
                                               fb.position, received);
          }
        }
        if (fb.fault_bits) {
          char bits[8];
          std::snprintf(bits, sizeof(bits), "%02x", fb.fault_bits);
          throw std::runtime_error("motor_" + std::to_string(fb.motor_id) +
                                   ": status fault bits 0x" + bits);
        }
        if (age > feedback_timeout_s_) {
          continue;
        }
        if (awaiting_enable_ack_) {
          {
            std::lock_guard<std::mutex> lock(enable_report_lock_);
            enable_reports_[j.cfg.name] = {mono() - age, fb.mode};
          }
          tx_wakeup_.set();
        }
        update_state(j, fb.position, fb.velocity, fb.torque, fb.temperature, mono() - age);
      } catch (const std::exception& exc) {
        // A local transport/motor fault cannot be cleared by a healthy supervisor
        // publishing false; restart after diagnosis to reset.
        latch_transport_fault();
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "CAN RX fault on %s: %s",
                              bus.channel.c_str(), exc.what());
        std::this_thread::sleep_for(std::chrono::milliseconds(10));  // RX error backoff only
      }
    }
  }

  // ---------------------------------------------------------------- TX
  void tx_loop() {
    configure_scheduling("TX", can_tx_rt_priority_);
    RCLCPP_INFO(get_logger(), "CAN TX thread started; feedback never blocks TX");
    while (running_ && rclcpp::ok()) {
      const double begin = mono();
      tx_wakeup_.clear();
      expire_commands();
      drain_command_queue();
      process_firmware_readback();
      process_enable_reports();
      if (estop_active_ || transport_fault_) {
        enforce_estop();
      } else {
        std::vector<Command> active;
        {
          std::lock_guard<std::mutex> lock(active_command_lock_);
          for (const auto& entry : active_commands_) {
            active.push_back(entry.second);
          }
        }
        for (const auto& cmd : active) {
          send_active_command(cmd);
        }
      }
      const double end = mono();
      {
        std::lock_guard<std::mutex> lock(timing_lock_);
        tx_cycles_++;
        tx_max_work_ms_ = std::max(tx_max_work_ms_, (end - begin) * 1000.0);
      }
      // Wait only the remainder of the period, or wake immediately on a new
      // command, stop request, or an earlier per-joint deadline.
      double wake_at = begin + tx_period_s_;
      for (const auto& entry : command_deadlines_) {
        wake_at = std::min(wake_at, entry.second);
      }
      tx_wakeup_.wait(wake_at - end);
    }
  }

  void send(JointRuntime& j, const robstride::Frame& f) { j.bus->send(f); }

  void set_mode_raw(JointRuntime& j, int mode) {
    send(j, robstride::write_int8(j.bus->host_id, j.cfg.motor_id, robstride::kParamMode, mode));
  }

  void write_float(JointRuntime& j, uint16_t param, double value) {
    send(j, robstride::write_float(j.bus->host_id, j.cfg.motor_id, param, value));
  }

  void log_submission(const std::string& joint, const std::string& state) {
    auto& last = submitted_state_[joint];
    std::string label = state;
    label[0] = static_cast<char>(std::toupper(label[0]));
    if (last == state) {
      RCLCPP_DEBUG(get_logger(), "%s resubmitted for %s", label.c_str(), joint.c_str());
      return;
    }
    last = state;
    RCLCPP_INFO(get_logger(), "%s submitted for %s", label.c_str(), joint.c_str());
  }

  // Write (when `write`) and request readback of the firmware parameters; the
  // bus lock is held by the caller. Returns whether there are any.
  bool request_firmware_parameters(JointRuntime& j, bool write) {
    const int host = j.bus->host_id, id = j.cfg.motor_id;
    for (const auto& w : j.cfg.firmware_writes) {
      if (write) {
        send(j, w.is_float ? robstride::write_float(host, id, w.index, w.value)
                           : robstride::write_uint32(host, id, w.index,
                                                     static_cast<uint32_t>(w.value)));
      }
      send(j, robstride::read_parameter(host, id, w.index));
    }
    return !j.cfg.firmware_writes.empty();
  }

  // python_can_node._process_firmware_readback: send the deferred ENABLE once
  // every parameter reads back as written; re-request every 50 ms.
  void process_firmware_readback() {
    const double now = mono();
    for (auto it = firmware_pending_.begin(); it != firmware_pending_.end();) {
      if (!enable_pending_.count(it->first)) {
        it = firmware_pending_.erase(it);
        continue;
      }
      JointRuntime& j = *joint_by_name_.at(it->first);
      bool verified = true;
      for (const auto& w : j.cfg.firmware_writes) {
        std::optional<std::pair<uint32_t, double>> reply;
        {
          std::lock_guard<std::mutex> lock(state_lock_);
          const auto found = param_readback_.find({it->first, w.index});
          if (found != param_readback_.end()) reply = found->second;
        }
        uint32_t expected = static_cast<uint32_t>(w.value);
        if (w.is_float) {
          const float f = static_cast<float>(w.value);
          std::memcpy(&expected, &f, sizeof(expected));
        }
        if (!reply || reply->second < it->second.first) {
          verified = false;
        } else if (reply->first != expected) {
          verified = false;
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                "%s: parameter 0x%04x reads back %u, wrote %u; "
                                "motor stays disabled",
                                it->first.c_str(), w.index, reply->first, expected);
        }
      }
      if (verified) {
        // Arm before I/O: the Motor-mode ack must be newer than this ENABLE.
        enable_pending_[it->first] = mono();
      }
      try {
        std::lock_guard<std::mutex> lock(j.bus->lock);
        if (verified) {
          send(j, robstride::enable_frame(j.bus->host_id, j.cfg.motor_id));
        } else if (now - it->second.second >= 0.05) {
          it->second.second = now;
          request_firmware_parameters(j, false);
        }
      } catch (const std::exception& exc) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                              "%s: firmware readback I/O failed: %s", it->first.c_str(),
                              exc.what());
        ++it;
        continue;
      }
      if (verified) {
        RCLCPP_INFO(get_logger(), "%s: firmware parameters verified; enable sent",
                    it->first.c_str());
        it = firmware_pending_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void process_enable_reports() {
    // Complete enables on fresh healthy Motor-mode status. Type-2 status mode 2
    // means Motor/run, distinct from run_mode parameter 2 (velocity control).
    std::map<std::string, std::pair<double, int>> reports;
    {
      std::lock_guard<std::mutex> lock(enable_report_lock_);
      reports.swap(enable_reports_);
    }
    for (const auto& entry : reports) {
      if (firmware_pending_.count(entry.first)) {
        continue;  // our ENABLE frame has not been sent yet; ignore stale Motor mode
      }
      const auto pending = enable_pending_.find(entry.first);
      const auto deadline = command_deadlines_.find(entry.first);
      if (pending != enable_pending_.end() && entry.second.first >= pending->second &&
          entry.second.second == 2 &&
          mono() < (deadline == command_deadlines_.end() ? 0.0 : deadline->second)) {
        enable_pending_.erase(pending);
        inhibited_since_.erase(entry.first);
        enabled_at_[entry.first] = entry.second.first;
      }
    }
    if (enable_pending_.empty()) {
      awaiting_enable_ack_ = false;
    }
  }

  void enforce_estop() {
    if (!estop_disable_pending_) {
      return;
    }
    command_deadlines_.clear();
    enable_pending_.clear();
    firmware_pending_.clear();
    enabled_at_.clear();
    awaiting_enable_ack_ = false;
    const double now = mono();
    for (const auto& j : joints_) {
      inhibited_since_[j->cfg.name] = now;
    }
    {
      std::lock_guard<std::mutex> lock(active_command_lock_);
      active_commands_.clear();
    }
    bool all_disabled = true;
    for (const auto& j : joints_) {
      try {
        std::lock_guard<std::mutex> lock(j->bus->lock);
        send(*j, robstride::disable_frame(j->bus->host_id, j->cfg.motor_id));
        current_mode_[j->cfg.name].reset();
      } catch (const std::exception& exc) {
        all_disabled = false;
        RCLCPP_ERROR(get_logger(), "ESTOP disable failed for %s; retrying: %s",
                     j->cfg.name.c_str(), exc.what());
      }
    }
    if (all_disabled) {
      estop_disable_pending_ = false;
      RCLCPP_FATAL(get_logger(), "ESTOP disable requests submitted; verify physical stop via feedback");
    }
  }

  void expire_commands() {
    // Stop expired joints and retry failed disables on every worker pass. A
    // fresh explicit ENABLE is required to recover.
    const double now = mono();
    for (auto it = command_deadlines_.begin(); it != command_deadlines_.end();) {
      if (now >= it->second) {
        const std::string joint = it->first;
        it = command_deadlines_.erase(it);
        enable_pending_.erase(joint);
        firmware_pending_.erase(joint);
        enabled_at_.erase(joint);
        inhibited_since_[joint] = now;
        {
          std::lock_guard<std::mutex> lock(active_command_lock_);
          active_commands_.erase(joint);
        }
        pending_disable_.insert(joint);
        RCLCPP_ERROR(get_logger(), "Command timeout for %s; explicit enable required", joint.c_str());
      } else {
        ++it;
      }
    }
    if (enable_pending_.empty()) {
      awaiting_enable_ack_ = false;
    }
    for (auto it = pending_disable_.begin(); it != pending_disable_.end();) {
      JointRuntime& j = *joint_by_name_.at(*it);
      try {
        std::lock_guard<std::mutex> lock(j.bus->lock);
        send(j, robstride::disable_frame(j.bus->host_id, j.cfg.motor_id));
        current_mode_[j.cfg.name].reset();
        it = pending_disable_.erase(it);
      } catch (const std::exception& exc) {
        RCLCPP_ERROR(get_logger(), "Watchdog disable failed for %s; retrying: %s",
                     j.cfg.name.c_str(), exc.what());
        ++it;
      }
    }
  }

  void drain_command_queue() {
    // Bound work even when producers continuously update targets.
    for (size_t n = 0; n < 2 * joints_.size(); ++n) {
      expire_commands();
      std::optional<Command> next = command_queue_.get();
      if (!next) {
        return;
      }
      Command cmd = std::move(*next);
      const auto found = joint_by_name_.find(cmd.joint);
      if (found == joint_by_name_.end()) {
        RCLCPP_WARN(get_logger(), "Unknown joint: %s", cmd.joint.c_str());
        continue;
      }
      JointRuntime& j = *found->second;
      const uint8_t mode = cmd.mode;
      // Queue residence consumes the original lifetime. A delayed ENABLE must
      // not restart a joint after the watchdog has already stopped it.
      if (mode != MotorCommand::MODE_DISABLE) {
        if (estop_active_ || transport_fault_ || mono() >= cmd.deadline) {
          continue;
        }
        if (mode == MotorCommand::MODE_ENABLE) {
          if (pending_disable_.count(cmd.joint)) {
            continue;
          }
          const auto inhibited = inhibited_since_.find(cmd.joint);
          if (cmd.received_at <= (inhibited == inhibited_since_.end() ? -kInf : inhibited->second)) {
            continue;
          }
        } else if (inhibited_since_.count(cmd.joint) && !enable_pending_.count(cmd.joint)) {
          continue;
        }
      }
      try {
        if (mode == MotorCommand::MODE_ENABLE) {
          // Arm before I/O: an enable may reach hardware even when its
          // acknowledgement fails. Never retain an older target.
          {
            std::lock_guard<std::mutex> lock(active_command_lock_);
            active_commands_.erase(cmd.joint);
          }
          command_deadlines_[cmd.joint] = cmd.deadline;
          enable_pending_[cmd.joint] = mono();
          inhibited_since_.emplace(cmd.joint, cmd.received_at);  // setdefault
          awaiting_enable_ack_ = true;
          {
            std::lock_guard<std::mutex> lock(j.bus->lock);
            set_mode_raw(j, 0);
            // Arm before I/O: a readback reply may arrive before the send returns.
            if (!j.cfg.firmware_writes.empty()) {
              const double now = mono();
              firmware_pending_[cmd.joint] = {now, now};  // ENABLE deferred until readback
              request_firmware_parameters(j, true);
            } else {
              send(j, robstride::enable_frame(j.bus->host_id, j.cfg.motor_id));
            }
            current_mode_[cmd.joint].reset();
          }
          pending_disable_.erase(cmd.joint);
          command_deadlines_[cmd.joint] = cmd.deadline;
          log_submission(cmd.joint, "enable");
          continue;
        }
        if (mode == MotorCommand::MODE_DISABLE) {
          {
            std::lock_guard<std::mutex> lock(active_command_lock_);
            active_commands_.erase(cmd.joint);
          }
          command_deadlines_.erase(cmd.joint);
          enable_pending_.erase(cmd.joint);
          firmware_pending_.erase(cmd.joint);
          enabled_at_.erase(cmd.joint);
          inhibited_since_[cmd.joint] = mono();
          pending_disable_.insert(cmd.joint);
          {
            std::lock_guard<std::mutex> lock(j.bus->lock);
            send(j, robstride::disable_frame(j.bus->host_id, j.cfg.motor_id));
            current_mode_[cmd.joint].reset();
          }
          pending_disable_.erase(cmd.joint);
          log_submission(cmd.joint, "disable");
          continue;
        }
        submitted_state_[cmd.joint] = "active";
        std::optional<Command> previous;
        {
          std::lock_guard<std::mutex> lock(active_command_lock_);
          const auto it = active_commands_.find(cmd.joint);
          if (it != active_commands_.end()) {
            previous = it->second;
          }
          active_commands_[cmd.joint] = cmd;
        }
        if (!enable_pending_.count(cmd.joint)) {
          command_deadlines_[cmd.joint] = cmd.deadline;
        }
        if (mode == MotorCommand::MODE_VELOCITY &&
            (!previous || previous->mode != MotorCommand::MODE_VELOCITY ||
             previous->values.acceleration != cmd.values.acceleration)) {
          current_mode_[cmd.joint].reset();
        }
        RCLCPP_DEBUG(get_logger(), "Active command for %s: mode=%d, pos=%.4f, vel=%.4f, kp=%.2f, kd=%.2f",
                     cmd.joint.c_str(), mode, cmd.values.position, cmd.values.velocity,
                     cmd.values.kp, cmd.values.kd);
      } catch (const std::exception& exc) {
        if (mode == MotorCommand::MODE_ENABLE) {
          command_deadlines_.erase(cmd.joint);
          enable_pending_.erase(cmd.joint);
          inhibited_since_[cmd.joint] = mono();
          pending_disable_.insert(cmd.joint);
        }
        RCLCPP_ERROR(get_logger(), "Failed to apply command for %s: %s", cmd.joint.c_str(),
                     exc.what());
      }
    }
  }

  void send_active_command(const Command& active) {
    // Host scheduling may exhaust the deadline after the active snapshot.
    expire_commands();
    if (estop_active_ || transport_fault_ || inhibited_since_.count(active.joint)) {
      return;
    }
    const auto found = joint_by_name_.find(active.joint);
    if (found == joint_by_name_.end()) {
      return;
    }
    JointRuntime& j = *found->second;
    try {
      std::optional<double> measured;
      std::optional<RawSample> latest;
      {
        std::lock_guard<std::mutex> state(state_lock_);
        const auto it = raw_feedback_.find(active.joint);
        const double age = it == raw_feedback_.end() ? -1.0 : mono() - it->second.received_at;
        if (age >= 0.0 && age <= feedback_timeout_s_) measured = it->second.fb.position;
        if (it != raw_feedback_.end()) latest = it->second;
      }
      // Status left Motor/run without our DISABLE (motor CAN watchdog, undervoltage,
      // fault): stop everything. Velocity mode rewrites run_mode, so MIT joints only.
      const auto enabled = enabled_at_.find(active.joint);
      const auto mode_it = current_mode_.find(active.joint);
      const bool velocity_mode = mode_it != current_mode_.end() && mode_it->second == 2;
      if (latest && enabled != enabled_at_.end() && latest->received_at > enabled->second &&
          latest->fb.mode != 2 && !velocity_mode) {
        latch_transport_fault();
        throw std::runtime_error(active.joint + ": motor left run mode (status mode " +
                                 std::to_string(latest->fb.mode) + ") without a host DISABLE");
      }
      const bool position_mode = active.mode == MotorCommand::MODE_POSITION ||
                                 active.mode == MotorCommand::MODE_MOTION;
      robstride::MotorCommandValues c;
      {
        std::lock_guard<std::mutex> lock(j.ankle_lock);
        c = j.cfg.settings.command(active.values, j.ankle, position_mode, measured);
      }
      std::lock_guard<std::mutex> lock(j.bus->lock);
      auto& mode_now = current_mode_[active.joint];
      if (active.mode == MotorCommand::MODE_VELOCITY) {
        if (mode_now != 2) {
          send(j, robstride::write_int8(j.bus->host_id, j.cfg.motor_id, robstride::kParamMode, 2));
          write_float(j, robstride::kParamCurrentLimit, 5.0);
          write_float(j, robstride::kParamVelAccelerationTarget, c.acceleration);
          write_float(j, robstride::kParamVelocityTarget, 0.0);
          mode_now = 2;
        }
        write_float(j, robstride::kParamVelocityTarget,
                    std::max(-max_vel_rad_s_, std::min(max_vel_rad_s_, c.velocity)));
      } else if (active.mode == MotorCommand::MODE_POSITION) {
        if (mode_now != 0) {
          set_mode_raw(j, 0);
          mode_now = 0;
        }
        // MIT velocity is feedforward, not a position speed limit.
        send(j, robstride::operation_frame(j.profile, j.cfg.motor_id, c.position, c.kp, c.kd, 0.0,
                                           0.0));
      } else if (active.mode == MotorCommand::MODE_MOTION) {
        if (mode_now != 0) {
          set_mode_raw(j, 0);
          mode_now = 0;
        }
        const double vel = std::max(-max_vel_rad_s_, std::min(max_vel_rad_s_, c.velocity));
        send(j, robstride::operation_frame(j.profile, j.cfg.motor_id, c.position, c.kp, c.kd, vel,
                                           c.torque));
      }
    } catch (const std::exception& exc) {
      {
        std::lock_guard<std::mutex> lock(timing_lock_);
        tx_errors_++;
      }
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "Failed to send active command to %s: %s", active.joint.c_str(),
                            exc.what());
    }
  }

  // ---------------------------------------------------------------- ROS side
  double now_ros() { return get_clock()->now().nanoseconds() * 1e-9; }

  void estop_callback(bool asserted) {
    if (asserted) {
      const bool newly = !estop_active_.exchange(true);
      estop_disable_pending_ = true;
      tx_wakeup_.set();
      {
        std::lock_guard<std::mutex> lock(active_command_lock_);
        active_commands_.clear();
      }
      command_queue_.clear();
      if (newly) {
        RCLCPP_FATAL(get_logger(), "Safety estop asserted; rejecting commands and disabling motors");
      }
    } else if (estop_active_.exchange(false)) {
      RCLCPP_WARN(get_logger(), "Safety estop reset; motors remain disabled until explicitly enabled");
    }
  }

  void cmd_callback(const MotorCommand& msg) {
    if (estop_active_ || transport_fault_ || msg.joint_name.empty()) {
      return;
    }
    const double received_at = mono();
    const double now = now_ros();
    const double stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    const double age = now - stamp;
    bool source_valid = msg.header.stamp.sec >= 0 && msg.header.stamp.nanosec < 1000000000u &&
                        stamp > 0 && -0.01 <= age && age < command_timeout_s_;
    // The source age and local queue time both count against the lifetime.
    double deadline = received_at + command_timeout_s_ - std::max(0.0, age);
    const double expiry = msg.valid_until.sec + msg.valid_until.nanosec * 1e-9;
    if (expiry != 0) {
      source_valid = source_valid && msg.valid_until.sec >= 0 &&
                     msg.valid_until.nanosec < 1000000000u && expiry > now && expiry > stamp;
      deadline = std::min(deadline, received_at + expiry - now);
    }
    std::vector<Command> commands;
    for (size_t i = 0; i < msg.joint_name.size(); ++i) {
      const auto found = joint_by_name_.find(msg.joint_name[i]);
      if (found == joint_by_name_.end()) {
        continue;
      }
      uint8_t mode = MotorCommand::MODE_VELOCITY;
      if (msg.mode.size() == 1) {
        mode = msg.mode[0];
      } else if (i < msg.mode.size()) {
        mode = msg.mode[i];
      }
      if (mode != MotorCommand::MODE_DISABLE && !source_valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Rejecting stale or invalid command timestamp for %s",
                             msg.joint_name[i].c_str());
        continue;
      }
      const auto& settings = found->second->cfg.settings;
      const auto at = [i](const std::vector<double>& v, double fallback) {
        return i < v.size() ? v[i] : fallback;
      };
      Command cmd;
      cmd.joint = msg.joint_name[i];
      cmd.mode = mode;
      cmd.received_at = received_at;
      cmd.deadline = deadline;
      cmd.values.position = at(msg.position, 0.0);
      cmd.values.velocity = at(msg.velocity, 0.0);
      cmd.values.acceleration = at(msg.acceleration, 0.0);
      cmd.values.torque = at(msg.torque, 0.0);
      cmd.values.kp = at(msg.kp, settings.kp);
      cmd.values.kd = at(msg.kd, settings.kd);
      commands.push_back(std::move(cmd));
    }
    if (!commands.empty()) {
      command_queue_.put(commands);
      tx_wakeup_.set();
    }
  }

  // Status-frame fault bits of the latest raw sample (0 = none); state_lock_ held.
  int raw_fault_bits(const std::string& joint) const {
    const auto it = raw_feedback_.find(joint);
    return it == raw_feedback_.end() ? 0 : it->second.fb.fault_bits;
  }

  void get_feedback(const GetMotorFeedback::Request& req, GetMotorFeedback::Response& res) {
    // Bounded cache lookup: service callbacks never touch the CAN socket.
    auto& fb = res.feedback;
    fb.joint_name = req.joint_name;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    fb.age_s = fb.position_rad = fb.velocity_radps = fb.torque_nm = fb.temperature_c = nan;
    const auto found = joint_by_name_.find(req.joint_name);
    if (found == joint_by_name_.end()) {
      res.message = "Unknown joint";
      return;
    }
    const auto& cfg = found->second->cfg;
    fb.can_interface = cfg.can_interface;
    fb.motor_id = static_cast<uint8_t>(cfg.motor_id);
    fb.model = cfg.model;
    std::optional<RawSample> cached;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      const auto it = raw_feedback_.find(req.joint_name);
      if (it != raw_feedback_.end()) {
        cached = it->second;
      }
    }
    if (!cached) {
      res.message = "No raw type-2 feedback received";
      return;
    }
    fb.available = true;
    fb.age_s = mono() - cached->received_at;
    fb.fresh = 0 <= fb.age_s && fb.age_s <= feedback_timeout_s_;
    const int64_t source_ns = std::max<int64_t>(0, static_cast<int64_t>(cached->source_stamp * 1e9));
    fb.stamp.sec = static_cast<int32_t>(source_ns / 1000000000);
    fb.stamp.nanosec = static_cast<uint32_t>(source_ns % 1000000000);
    fb.position_rad = cached->fb.position;
    fb.velocity_radps = cached->fb.velocity;
    fb.torque_nm = cached->fb.torque;
    fb.temperature_c = cached->fb.temperature;
    fb.status_mode = static_cast<uint8_t>(cached->fb.mode);
    fb.fault_bits = static_cast<uint8_t>(cached->fb.fault_bits);
    res.success = fb.fresh && !cached->fb.fault_bits && !transport_fault_;
    res.message = transport_fault_      ? "CAN transport fault latched"
                  : !fb.fresh           ? "Stale raw feedback"
                  : cached->fb.fault_bits ? "Motor reports fault bits"
                                          : "Fresh raw feedback";
  }

  void operation(const ExecuteMotorOperation::Request& req, ExecuteMotorOperation::Response& res) {
    // Typed zero admission is reserved; never offer an unguarded type-6 send.
    res.request_id = req.request_id;
    res.accepted = false;
    res.state = ExecuteMotorOperation::Response::REJECTED;
    if (req.request_id.empty() || req.request_id.size() > 128) {
      res.message = "request_id must contain 1–128 characters";
    } else if (!joint_by_name_.count(req.joint_name)) {
      res.message = "Unknown joint";
    } else if (req.operation != ExecuteMotorOperation::Request::SET_MECHANICAL_ZERO) {
      res.message = "Unsupported operation; use typed ROS constants, not CAN frame numbers";
    } else if (estop_active_ || transport_fault_) {
      res.message = "Safety estop or CAN transport fault is active";
    } else {
      res.message =
          "Not ready: control_core pause/exclusive lease, verified firmware/run_mode profile and "
          "zero readback transaction are not implemented; no CAN command sent";
    }
  }

  void publish_states() {
    sensor_msgs::msg::JointState msg;
    const rclcpp::Time now_stamp = get_clock()->now();
    double oldest_age = 0.0;
    std::ostringstream status, telemetry;
    status << "{";
    telemetry << "{\"schema_version\": 1, \"motors\": [";
    bool first_status = true, first_telemetry = true;
    {
      std::lock_guard<std::mutex> lock(state_lock_);
      const double now = mono();
      for (const auto& j : joints_) {
        const auto it = state_buffer_.find(j->cfg.name);
        if (it == state_buffer_.end() || now - it->second.received_at > feedback_timeout_s_) {
          continue;
        }
        const Sample& s = it->second;
        oldest_age = std::max(oldest_age, now - s.received_at);
        msg.name.push_back(j->cfg.name);
        msg.position.push_back(s.position);
        msg.velocity.push_back(s.velocity);
        msg.effort.push_back(s.torque);
        status << (first_status ? "" : ", ") << "\"" << j->index
               << "\": {\"temperature\": " << json_number(s.temperature)
               << ", \"torque\": " << json_number(s.torque) << "}";
        first_status = false;
        telemetry << (first_telemetry ? "" : ", ") << "{\"joint_name\": \"" << json_escape(j->cfg.name)
                  << "\", \"can_interface\": \"" << json_escape(j->cfg.can_interface)
                  << "\", \"motor_id\": " << j->cfg.motor_id << ", \"model\": \""
                  << json_escape(j->cfg.model) << "\", \"source_stamp\": "
                  << json_number(telemetry_stamps_[j->cfg.name])
                  << ", \"temperature_c\": " << json_number(s.temperature)
                  << ", \"fault_bits\": " << raw_fault_bits(j->cfg.name) << "}";
        first_telemetry = false;
      }
    }
    if (!msg.name.empty()) {
      // JointState has one stamp: conservatively use the oldest included receipt.
      const int64_t stamp_ns =
          std::max<int64_t>(0, now_stamp.nanoseconds() - static_cast<int64_t>(oldest_age * 1e9));
      msg.header.stamp.sec = static_cast<int32_t>(stamp_ns / 1000000000);
      msg.header.stamp.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000);
      joint_state_pub_->publish(msg);
    }
    status << "}";
    telemetry << "]}";
    std_msgs::msg::String status_msg;
    status_msg.data = status.str();
    motor_status_pub_->publish(status_msg);
    std_msgs::msg::String telemetry_msg;
    telemetry_msg.data = telemetry.str();
    telemetry_pub_->publish(telemetry_msg);
  }

  void publish_timing() {
    std::ostringstream os;
    {
      std::lock_guard<std::mutex> lock(timing_lock_);
      os << "{\"tx_cycles\": " << tx_cycles_ << ", \"tx_max_work_ms\": " << json_number(tx_max_work_ms_)
         << ", \"tx_errors\": " << tx_errors_;
    }
    os << ", \"rx_frames\": " << rx_frames_.load() << ", \"rx_ignored\": " << rx_ignored_.load()
       << ", \"pending_commands\": " << command_queue_.size()
       << ", \"transport_fault\": " << (transport_fault_ ? "true" : "false")
       << ", \"repeat_hz\": " << json_number(1.0 / tx_period_s_) << ", \"interfaces\": {";
    bool first = true;
    for (const auto& entry : buses_) {
      os << (first ? "" : ", ") << "\"" << json_escape(entry.first)
         << "\": {\"tx_submitted\": " << entry.second->tx_frames.load()
         << ", \"tx_errors\": " << entry.second->tx_errors.load() << "}";
      first = false;
    }
    os << "}, \"backend\": \"cpp\"}";
    std_msgs::msg::String msg;
    msg.data = os.str();
    timing_pub_->publish(msg);
  }

  // ---------------------------------------------------------------- state
  bool can_rt_enabled_{false};
  int can_tx_rt_priority_{20};
  int can_rx_rt_priority_{21};
  double command_timeout_s_{0.5};
  double feedback_timeout_s_{0.1};
  double tx_period_s_{0.02};
  double max_vel_rad_s_{4.5};

  std::map<std::string, std::unique_ptr<Bus>> buses_;
  std::vector<std::unique_ptr<JointRuntime>> joints_;  // registry order
  std::unordered_map<std::string, JointRuntime*> joint_by_name_;

  std::atomic<bool> running_{false};
  std::atomic<bool> estop_active_{false};
  std::atomic<bool> estop_disable_pending_{false};
  std::atomic<bool> transport_fault_{false};
  std::atomic<bool> awaiting_enable_ack_{false};
  WakeEvent tx_wakeup_;

  LatestCommandMailbox command_queue_;
  std::mutex active_command_lock_;
  std::map<std::string, Command> active_commands_;

  // TX-thread-owned watchdog structures (as in python_can_node).
  std::map<std::string, double> command_deadlines_;
  std::map<std::string, double> inhibited_since_;
  std::set<std::string> pending_disable_;
  std::map<std::string, double> enable_pending_;
  // Verified ENABLE (python_can_node.firmware_pending): joint -> (submitted, last request).
  std::map<std::string, std::pair<double, double>> firmware_pending_;
  std::map<std::string, double> enabled_at_;  // joint -> Motor-mode ack receipt
  std::map<std::string, std::string> submitted_state_;
  std::map<std::string, std::optional<int>> current_mode_;

  std::mutex enable_report_lock_;
  std::map<std::string, std::pair<double, int>> enable_reports_;

  std::mutex state_lock_;
  std::map<std::string, Sample> state_buffer_;
  std::map<std::string, double> telemetry_stamps_;
  std::map<std::string, RawSample> raw_feedback_;
  // (joint, index) -> (type-17 value, host monotonic receipt).
  std::map<std::pair<std::string, int>, std::pair<uint32_t, double>> param_readback_;

  std::mutex timing_lock_;
  uint64_t tx_cycles_{0};
  double tx_max_work_ms_{0.0};
  uint64_t tx_errors_{0};
  std::atomic<uint64_t> rx_frames_{0};
  std::atomic<uint64_t> rx_ignored_{0};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr telemetry_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::Subscription<MotorCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Service<GetMotorFeedback>::SharedPtr feedback_service_;
  rclcpp::Service<ExecuteMotorOperation>::SharedPtr operation_service_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr timing_timer_;

  std::thread tx_thread_;
  std::vector<std::thread> rx_threads_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    auto node = std::make_shared<RobstrideCanNode>();
    rclcpp::spin(node);
  } catch (const std::exception& exc) {
    RCLCPP_FATAL(rclcpp::get_logger("robstride_can_node"), "%s", exc.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
