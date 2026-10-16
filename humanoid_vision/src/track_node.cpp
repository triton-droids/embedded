/**
 * @file track_node.cpp
 * @brief Convert a detected pixel center into a single head-joint angle command.
 *
 * The visual loop runs on the ROS host; the DS3225 closes its position loop
 * internally. CameraInfo and target centers must describe the same image frame.
 * No depth or IMU orientation is required. Output angles are radians and depend
 * on the pulse-to-angle calibration in the MCU firmware.
 */
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/float32.hpp>

namespace humanoid_vision {

/**
 * @brief Single-axis visual P controller with bounded command dynamics.
 *
 * A target callback updates angular error; an independent wall timer integrates
 * a rate-limited velocity into a position target. Thus a 30 Hz detector can
 * feed a 50 Hz command stream without pretending to provide 50 Hz measurements.
 * The angle is a command estimate, not encoder feedback. Losing freshness stops
 * publication so the MCU watchdog releases the output instead of chasing an
 * old error. Only one instance should publish to a given servo command topic.
 */
class TrackNode : public rclcpp::Node {
public:
  /** @brief Validate parameters and wire subscriptions, publisher, and timer.
   * @throws std::invalid_argument For non-finite or unsupported control settings.
   * No position command is sent until both intrinsics and a fresh target exist.
   */
  TrackNode() : Node("track_node") {
    axis_ = declare_parameter<std::string>("axis", "yaw");
    direction_ = declare_parameter<double>("direction", -1.0);
    kp_ = declare_parameter<double>("kp", 1.0);
    deadband_ = declare_parameter<double>("deadband_px", 8.0);
    min_angle_ = declare_parameter<double>("min_angle_rad", -0.6);
    max_angle_ = declare_parameter<double>("max_angle_rad", 0.6);
    angle_ = declare_parameter<double>("initial_angle_rad", 0.0);
    max_speed_ = declare_parameter<double>("max_speed_rad_s", 0.3);
    max_acceleration_ = declare_parameter<double>("max_accel_rad_s2", 0.8);
    timeout_ = declare_parameter<double>("target_timeout_s", 0.15);
    const double rate = declare_parameter<double>("control_rate_hz", 50.0);
    const std::string target_topic = declare_parameter<std::string>(
        "target_topic", "/vision/target_center");
    const std::string info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/camera/camera/color/camera_info");
    const std::string command_topic = declare_parameter<std::string>(
        "command_topic", "/head/servo_target_rad");

    // Reject configurations outside the firmware's current [-1, 1] rad mapping.
    for (double v : {direction_, kp_, deadband_, min_angle_, max_angle_, angle_,
                     max_speed_, max_acceleration_, timeout_, rate}) {
      if (!std::isfinite(v)) throw std::invalid_argument("Non-finite track parameter");
    }
    if ((axis_ != "yaw" && axis_ != "pitch") ||
        (direction_ != -1.0 && direction_ != 1.0) || kp_ < 0.0 ||
        deadband_ < 0.0 || min_angle_ >= max_angle_ || min_angle_ < -1.0 ||
        max_angle_ > 1.0 || angle_ < min_angle_ || angle_ > max_angle_ ||
        max_speed_ <= 0.0 || max_acceleration_ <= 0.0 || timeout_ <= 0.0 ||
        rate < 10.0 || rate > 200.0) {
      throw std::invalid_argument("Invalid track parameters; MCU angle range is [-1, 1] rad");
    }

    // Keep only the newest absolute target; queued historical commands add lag.
    command_publisher_ = create_publisher<std_msgs::msg::Float32>(
        command_topic, rclcpp::QoS(1).reliable().durability_volatile());
    info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        info_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
          camera_info_ = msg;
        });
    target_subscription_ = create_subscription<geometry_msgs::msg::PointStamped>(
        target_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
          receive_target(*msg);
        });
    last_tick_ = Clock::now();
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                              [this]() { control_tick(); });
    RCLCPP_INFO(get_logger(),
        "Single-axis %s tracker -> %s; direction=%+.0f (verify on hardware). "
        "No depth required; no commands until fresh target + CameraInfo.",
        axis_.c_str(), command_topic.c_str(), direction_);
  }

private:
  // Monotonic time measures scheduling/receipt intervals; ROS time checks stamps.
  using Clock = std::chrono::steady_clock;

  /** @brief Accept nonzero ROS stamps no older than timeout_, with 50 ms skew.
   * @param stamp Original color-image acquisition stamp, not detection completion.
   * All producers must use the same ROS time base as this node.
   */
  bool fresh_stamp(const builtin_interfaces::msg::Time &stamp) {
    if (stamp.sec == 0 && stamp.nanosec == 0) return false;
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= -0.05 && age <= timeout_;
  }

  /** @brief Clear usable target/velocity while retaining the last angle estimate. */
  void invalidate() {
    target_valid_ = false;
    speed_ = 0.0;
  }

  /** @brief Validate a pixel-center observation and update the held angular error.
   * @param msg x/y are color-image pixels; z is unused. Header identifies the
   * optical frame and preserves image acquisition time. Invalid input disables
   * tracking until a later valid message; CameraInfo may be cached between frames.
   */
  void receive_target(const geometry_msgs::msg::PointStamped &msg) {
    // Require calibrated intrinsics for the same color image frame. Detections
    // retain the image stamp, so delayed/queued detections cannot drive motion.
    if (!camera_info_ || !fresh_stamp(msg.header.stamp) ||
        msg.header.frame_id.empty() ||
        msg.header.frame_id != camera_info_->header.frame_id) {
      invalidate();
      return;
    }
    const auto &info = *camera_info_;
    if (info.width == 0 || info.height == 0 || !std::isfinite(msg.point.x) ||
        !std::isfinite(msg.point.y) || msg.point.x < 0.0 || msg.point.y < 0.0 ||
        msg.point.x >= info.width || msg.point.y >= info.height) {
      invalidate();
      return;
    }
    const bool yaw = axis_ == "yaw";
    const double focal = info.k[yaw ? 0 : 4];
    const double principal = info.k[yaw ? 2 : 5];
    if (!std::isfinite(focal) || focal <= 0.0 || !std::isfinite(principal)) {
      invalidate();
      return;
    }
    const double pixel = yaw ? msg.point.x : msg.point.y;
    const double center = 0.5 * (yaw ? info.width : info.height);
    // Aim for image center, even when principal point differs slightly.
    // Subtract the center ray, not merely the principal point, so the visible
    // image midpoint is the setpoint even for slightly off-center calibration.
    error_ = std::abs(pixel - center) <= deadband_ ? 0.0 :
        std::atan2(pixel - principal, focal) - std::atan2(center - principal, focal);
    target_stamp_ = msg.header.stamp;
    received_at_ = Clock::now();
    target_valid_ = true;
  }

  /** @brief Integrate one bounded-velocity command or stop on expired input.
   * Reuses the latest validated error between camera frames (zero-order hold).
   * The deadband requests zero velocity; acceleration limiting decelerates the
   * joint command rather than resetting its position to the nominal center.
   */
  void control_tick() {
    const auto current = Clock::now();
    const double dt = std::chrono::duration<double>(current - last_tick_).count();
    last_tick_ = current;
    // A stalled executor must not integrate a large jump on recovery.
    if (!target_valid_ || dt <= 0.0 || dt > timeout_ ||
        std::chrono::duration<double>(current - received_at_).count() > timeout_ ||
        !fresh_stamp(target_stamp_)) {
      if (tracking_) RCLCPP_INFO(get_logger(), "Target lost/stale: stop commands; MCU watchdog releases servo");
      tracking_ = false;
      invalidate();
      return;
    }
    if (!tracking_) RCLCPP_INFO(get_logger(), "Tracking target (%s)", axis_.c_str());
    tracking_ = true;
    // direction_ represents the measured camera-to-servo sign; kp_ has units 1/s.
    const double wanted_speed = std::clamp(direction_ * kp_ * error_,
                                            -max_speed_, max_speed_);
    const double step = max_acceleration_ * dt;
    speed_ += std::clamp(wanted_speed - speed_, -step, step);
    angle_ = std::clamp(angle_ + speed_ * dt, min_angle_, max_angle_);
    // Do not accumulate outward speed against the mechanical command limits.
    if ((angle_ >= max_angle_ && speed_ > 0.0) ||
        (angle_ <= min_angle_ && speed_ < 0.0)) speed_ = 0.0;
    std_msgs::msg::Float32 command;
    command.data = static_cast<float>(angle_);
    command_publisher_->publish(command);
  }

  // Configuration and dynamic state: radians, seconds, rad/s, and rad/s^2.
  std::string axis_;
  double direction_, kp_, deadband_, min_angle_, max_angle_, angle_;
  double max_speed_, max_acceleration_, timeout_;
  double error_ = 0.0, speed_ = 0.0;
  bool target_valid_ = false, tracking_ = false;
  Clock::time_point last_tick_, received_at_;
  builtin_interfaces::msg::Time target_stamp_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr command_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace humanoid_vision

/** @brief Run the controller in a single-threaded executor (callbacks share state). */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_vision::TrackNode>());
  rclcpp::shutdown();
  return 0;
}
