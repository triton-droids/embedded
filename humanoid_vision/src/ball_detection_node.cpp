/**
 * @file ball_detection_node.cpp
 * @brief Detect one image target and publish its 2D center and optional 3D point.
 *
 * The default model detects faces as a bench substitute for a football model.
 * Pixel centers always retain the source color-image stamp for tracker freshness
 * checks. Depth is optional and never gates the visual-servo center publisher.
 *
 * Outputs, all stamped with the color image that produced them:
 *  - /vision/target_center (PointStamped): target center, pixels; z unused.
 *  - /vision/target_track (vision_msgs/Detection2D): center and width/height in
 *    pixels, id = track id, results[0] = class id and score. score is 0 when the
 *    target was not detected in this frame and the box is a short prediction.
 *  - /vision/target_velocity (Vector3Stamped): center velocity, pixels/second,
 *    in the image plane (includes head rotation); z unused.
 */
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "humanoid_vision/cv_bridge_compat.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/detection_postprocess.hpp"
#include "humanoid_vision/target_3d_estimator.hpp"
#include "humanoid_vision/target_tracker.hpp"
#include "humanoid_vision/types.hpp"

namespace humanoid_vision {

/** @brief Host-side inference node with single-target lock-on.
 * Color callbacks perform inference synchronously. The latest depth/intrinsics
 * are cached independently; there is currently no timestamp synchronizer for
 * 3D output, so callers must not assume exact color/depth temporal alignment.
 * With track_target (default) a TargetTracker associates detections across
 * frames, smooths the box, and bridges short gaps; with it disabled each frame
 * independently publishes its highest-confidence detection, unfiltered.
 */
class BallDetectionNode : public rclcpp::Node {
public:
  /** @brief Load the configured model and create camera subscriptions/outputs.
   * TensorRT builds default to a serialized engine; CPU builds use ONNX.
   * Exceptions from model loading propagate to fail startup visibly.
   */
  BallDetectionNode() : Node("ball_detection_node") {
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("humanoid_vision");

    const std::string color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/camera/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    const std::string camera_info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/camera/camera/color/camera_info");
#ifdef HUMANOID_VISION_HAS_TENSORRT
    constexpr bool default_use_cuda = true;
#else
    constexpr bool default_use_cuda = false;
#endif
    const bool use_cuda = declare_parameter<bool>("use_cuda", default_use_cuda);
    const std::string model_path = declare_parameter<std::string>(
        "model_path", package_share + (use_cuda ?
        "/models/yolov8n-face-fp16.engine" :
        "/models/yolov8n-face-lindevs-opencv45.onnx"));

    show_image_ = declare_parameter<bool>("show_image", false);

    YoloDetector::Config config;
    config.input_width = declare_parameter<int>("input_width", 640);
    config.input_height = declare_parameter<int>("input_height", 640);
    config.confidence_threshold = static_cast<float>(
        declare_parameter<double>("confidence_threshold", 0.25));
    config.nms_threshold =
        static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    config.use_cuda = use_cuda;
    // -1 keeps any class (single-class face test model); 0 selects only the
    // ball class of the multi-class td-perception model.
    target_class_id_ = static_cast<int>(declare_parameter<int>("target_class_id", -1));

    // Use the face model to test the ball-position pipeline for now.
    detector_ = std::make_unique<YoloDetector>(model_path, config);

    if (declare_parameter<bool>("track_target", true)) {
      TrackerConfig tracking;
      tracking.alpha = static_cast<float>(declare_parameter<double>("tracker.alpha", tracking.alpha));
      tracking.beta = static_cast<float>(declare_parameter<double>("tracker.beta", tracking.beta));
      tracking.size_alpha = static_cast<float>(
          declare_parameter<double>("tracker.size_alpha", tracking.size_alpha));
      tracking.gate_min_px = static_cast<float>(
          declare_parameter<double>("tracker.gate_min_px", tracking.gate_min_px));
      tracking.gate_size_factor = static_cast<float>(
          declare_parameter<double>("tracker.gate_size_factor", tracking.gate_size_factor));
      tracking.confirm_frames = static_cast<int>(
          declare_parameter<int>("tracker.confirm_frames", tracking.confirm_frames));
      tracking.max_coast_s = declare_parameter<double>("tracker.max_coast_s", tracking.max_coast_s);
      tracker_ = std::make_unique<TargetTracker>(tracking);
    }

    // Depth-two sensor QoS favors the freshest frame over an inference backlog.
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS().keep_last(2),
        std::bind(&BallDetectionNode::image_callback, this,
                  std::placeholders::_1));

    // Keep the latest aligned depth frame and camera intrinsics.
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS().keep_last(2),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_depth_ = message;
        });

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, rclcpp::SensorDataQoS().keep_last(2),
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
              latest_camera_info_ = message;
            });

    // The 3D point is in the camera optical frame (meters), never body coordinates.
    position_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/ball_3d_position", 2);
    center_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/target_center", rclcpp::SensorDataQoS().keep_last(2));
    track_publisher_ = create_publisher<vision_msgs::msg::Detection2D>(
        "/vision/target_track", rclcpp::SensorDataQoS().keep_last(2));
    velocity_publisher_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
        "/vision/target_velocity", rclcpp::SensorDataQoS().keep_last(2));

    // Report throughput independently of target visibility; no detection is not
    // the same failure as no incoming camera frames.
    last_image_time_ = now();
    metrics_time_ = std::chrono::steady_clock::now();
    image_watchdog_ = create_wall_timer(std::chrono::seconds(5), [this, color_topic]() {
      const auto current = std::chrono::steady_clock::now();
      const double seconds = std::chrono::duration<double>(current - metrics_time_).count();
      if (processed_frames_ > 0) {
        RCLCPP_INFO(get_logger(), "Processing %.1f FPS; detection %.1f ms/frame; last balls: %zu; "
                    "track id: %d (-1 = none)",
                    processed_frames_ / seconds, detection_ms_ / processed_frames_,
                    last_ball_count_, last_track_id_);
      }
      metrics_time_ = current;
      processed_frames_ = 0;
      detection_ms_ = 0;
      if ((now() - last_image_time_).seconds() > 5.0) {
        RCLCPP_WARN(get_logger(), "No recent color image on %s; check camera driver", color_topic.c_str());
      }
    });
    RCLCPP_INFO(get_logger(), "Ball-position test node started; backend: %s; color topic: %s; "
                "target class: %d (-1 = any); frame-to-frame tracking: %s",
                use_cuda ? "TensorRT CUDA FP16" : "OpenCV CPU", color_topic.c_str(),
                target_class_id_, tracker_ ? "on" : "off");
  }

  /** @brief Release the optional OpenCV GUI window when the node exits. */
  ~BallDetectionNode() override {
    if (show_image_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  /** @brief Infer one color frame, select a box, and publish/annotate the result.
   * @param message Color image with acquisition timestamp and optical frame ID.
   * No detection means no center message: the downstream freshness watchdog
   * expires naturally. A missing/invalid depth only suppresses the 3D output.
   */
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message) {
    last_image_time_ = now();
    const auto started = std::chrono::steady_clock::now();
    std::vector<Detection2D> detections = detector_->detect(message);
    if (target_class_id_ >= 0) {
      // Other classes (goalposts, robots, field marks) must never become the target.
      detections.erase(
          std::remove_if(detections.begin(), detections.end(),
                         [this](const Detection2D &detection) {
                           return detection.class_id != target_class_id_;
                         }),
          detections.end());
    }
    detection_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    ++processed_frames_;
    last_ball_count_ = detections.size();

    const std::optional<Target> target = select_target(detections, *message);
    last_track_id_ = target ? target->track_id : -1;

    if (!target) {
      if (show_image_) {
        const auto image_cv = cv_bridge::toCvShare(
            message, sensor_msgs::image_encodings::BGR8);
        cv::Mat annotated = image_cv->image.clone();
        cv::putText(annotated, "No ball detected", cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::imshow(window_name_, annotated);
        cv::waitKey(1);
      }
      return;
    }

    // Pixel center for visual servoing. Publish even when depth is unavailable.
    geometry_msgs::msg::PointStamped center;
    center.header = message->header;
    center.point.x = target->center.x;
    center.point.y = target->center.y;
    center_publisher_->publish(center);

    vision_msgs::msg::Detection2D track;
    track.header = message->header;
    track.id = std::to_string(target->track_id);
    track.bbox.center.position.x = target->center.x;
    track.bbox.center.position.y = target->center.y;
    track.bbox.size_x = target->box.width;
    track.bbox.size_y = target->box.height;
    vision_msgs::msg::ObjectHypothesisWithPose hypothesis;
    hypothesis.hypothesis.class_id = std::to_string(target->class_id);
    hypothesis.hypothesis.score = target->measured ? target->confidence : 0.0;
    track.results.push_back(hypothesis);
    track_publisher_->publish(track);

    geometry_msgs::msg::Vector3Stamped velocity;
    velocity.header = message->header;
    velocity.vector.x = target->velocity.x;
    velocity.vector.y = target->velocity.y;
    velocity_publisher_->publish(velocity);

    // Cached depth must already be aligned to color pixels. This convenience
    // path does not transform coordinates or enforce timestamp synchronization.
    // A predicted (unmeasured) box is not sampled for depth.
    if (target->measured && latest_depth_ && latest_camera_info_) {
      const auto position =
          project_2d_to_3d(target->box, latest_depth_, latest_camera_info_);

      if (position) {
        geometry_msgs::msg::PointStamped point_message;
        point_message.header = latest_depth_->header;
        point_message.point.x = position->x();
        point_message.point.y = position->y();
        point_message.point.z = position->z();
        position_publisher_->publish(point_message);
      }
    }

    if (!show_image_) {
      return;
    }

    const cv_bridge::CvImageConstPtr image_cv =
        cv_bridge::toCvShare(message, sensor_msgs::image_encodings::BGR8);
    cv::Mat annotated = image_cv->image.clone();

    cv::rectangle(annotated, target->box, cv::Scalar(0, 255, 0), 2);
    const cv::Point target_pixel(static_cast<int>(center.point.x),
                                 static_cast<int>(center.point.y));
    const cv::Point image_center(annotated.cols / 2, annotated.rows / 2);
    cv::drawMarker(annotated, image_center, cv::Scalar(255, 255, 0));
    cv::circle(annotated, target_pixel, 4, cv::Scalar(0, 0, 255), -1);
    cv::line(annotated, image_center, target_pixel, cv::Scalar(0, 255, 255));

    std::ostringstream label;
    label << "id " << target->track_id << ' ';
    if (target->measured) {
      label << std::fixed << std::setprecision(2) << target->confidence;
    } else {
      label << "predicted";
    }

    const int text_y = std::max(18, static_cast<int>(target->box.y) - 6);
    cv::putText(annotated, label.str(),
                cv::Point(static_cast<int>(target->box.x), text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

    cv::imshow(window_name_, annotated);
    cv::waitKey(1);
  }

  /** @brief One published target in original color-image pixels. */
  struct Target {
    cv::Rect2f box;
    cv::Point2f center;
    cv::Point2f velocity;  ///< Pixels/second; zero when tracking is disabled.
    int track_id;          ///< -1 when tracking is disabled.
    int class_id;
    float confidence;
    bool measured;         ///< False for a coasting prediction.
  };

  /** @brief Choose this frame's target: tracker state, or the best detection.
   * The tracker runs on the image acquisition stamp so inference jitter does not
   * enter its dt. A coasting prediction may leave the image; its center is
   * clamped inside so consumers that range-check pixels still accept it.
   */
  std::optional<Target> select_target(const std::vector<Detection2D> &detections,
                                      const sensor_msgs::msg::Image &image) {
    if (!tracker_) {
      if (detections.empty()) return std::nullopt;
      const auto best = std::max_element(
          detections.begin(), detections.end(),
          [](const Detection2D &a, const Detection2D &b) { return a.confidence < b.confidence; });
      return Target{best->box,
                    {best->box.x + best->box.width * 0.5F, best->box.y + best->box.height * 0.5F},
                    {0.0F, 0.0F}, -1, best->class_id, best->confidence, true};
    }
    const rclcpp::Time stamp(image.header.stamp, get_clock()->get_clock_type());
    const double time_s = stamp.nanoseconds() != 0 ? stamp.seconds() : now().seconds();
    const std::optional<TrackOutput> track = tracker_->update(detections, time_s);
    if (!track) return std::nullopt;
    const TargetState &state = track->state;
    const cv::Point2f center(
        std::clamp(state.center.x, 0.0F, std::max(0.0F, static_cast<float>(image.width) - 1.0F)),
        std::clamp(state.center.y, 0.0F, std::max(0.0F, static_cast<float>(image.height) - 1.0F)));
    const cv::Rect2f box(center.x - state.size.width * 0.5F, center.y - state.size.height * 0.5F,
                         state.size.width, state.size.height);
    return Target{box, center, state.velocity, track->track_id, track->class_id,
                  track->confidence, track->measured};
  }

  static constexpr const char *window_name_ = "3D Target Test";

  // Monotonic metrics timing is separate from ROS image-age diagnostics.
  std::chrono::steady_clock::time_point metrics_time_;
  size_t processed_frames_ = 0;
  size_t last_ball_count_ = 0;
  int target_class_id_ = -1;
  int last_track_id_ = -1;
  std::unique_ptr<TargetTracker> tracker_;
  double detection_ms_ = 0;
  rclcpp::Time last_image_time_{0, 0, RCL_ROS_TIME};
  rclcpp::TimerBase::SharedPtr image_watchdog_;
  bool show_image_ = false;
  std::unique_ptr<YoloDetector> detector_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr
      camera_info_subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
      center_publisher_;
  rclcpp::Publisher<vision_msgs::msg::Detection2D>::SharedPtr track_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr velocity_publisher_;
};

} // namespace humanoid_vision

/** @brief Run detection with serialized callbacks so cached camera state is safe. */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_vision::BallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
