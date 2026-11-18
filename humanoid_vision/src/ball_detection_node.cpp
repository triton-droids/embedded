/**
 * @file ball_detection_node.cpp
 * @brief Detect one image target and publish its 2D center and optional 3D point.
 *
 * The default model detects faces as a bench substitute for a football model.
 * Pixel centers always retain the source color-image stamp for tracker freshness
 * checks. Depth is optional and never gates the visual-servo center publisher.
 */
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "humanoid_vision/cv_bridge_compat.hpp"
#include <geometry_msgs/msg/point_stamped.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "humanoid_vision/detection_postprocess.hpp"
#include "humanoid_vision/target_3d_estimator.hpp"
#include "humanoid_vision/types.hpp"

namespace humanoid_vision {

/** @brief Host-side inference node with highest-confidence target selection.
 * Color callbacks perform inference synchronously. The latest depth/intrinsics
 * are cached independently; there is currently no timestamp synchronizer for
 * 3D output, so callers must not assume exact color/depth temporal alignment.
 * Selection is per frame and does not maintain an object identity across frames.
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

    // Use the face model to test the ball-position pipeline for now.
    detector_ = std::make_unique<YoloDetector>(model_path, config);

    // Depth-one sensor QoS favors the freshest frame over an inference backlog.
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS().keep_last(1),
        std::bind(&BallDetectionNode::image_callback, this,
                  std::placeholders::_1));

    // Keep the latest aligned depth frame and camera intrinsics.
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS().keep_last(1),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_depth_ = message;
        });

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, rclcpp::SensorDataQoS().keep_last(1),
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
              latest_camera_info_ = message;
            });

    // The 3D point is in the camera optical frame (meters), never body coordinates.
    position_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/ball_3d_position", 10);
    center_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/target_center", rclcpp::SensorDataQoS().keep_last(1));

    // Report throughput independently of target visibility; no detection is not
    // the same failure as no incoming camera frames.
    last_image_time_ = now();
    metrics_time_ = std::chrono::steady_clock::now();
    image_watchdog_ = create_wall_timer(std::chrono::seconds(5), [this, color_topic]() {
      const auto current = std::chrono::steady_clock::now();
      const double seconds = std::chrono::duration<double>(current - metrics_time_).count();
      if (processed_frames_ > 0) {
        RCLCPP_INFO(get_logger(), "Processing %.1f FPS; detection %.1f ms/frame; last faces: %zu",
                    processed_frames_ / seconds, detection_ms_ / processed_frames_, last_face_count_);
      }
      metrics_time_ = current;
      processed_frames_ = 0;
      detection_ms_ = 0;
      if ((now() - last_image_time_).seconds() > 5.0) {
        RCLCPP_WARN(get_logger(), "No recent color image on %s; check camera driver", color_topic.c_str());
      }
    });
    RCLCPP_INFO(get_logger(), "Ball-position test node started; backend: %s; color topic: %s",
                use_cuda ? "TensorRT CUDA FP16" : "OpenCV CPU", color_topic.c_str());
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
    const std::vector<Detection2D> detections = detector_->detect(message);
    detection_ms_ += std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
    ++processed_frames_;
    last_face_count_ = detections.size();

    if (detections.empty()) {
      if (show_image_) {
        const auto image_cv = cv_bridge::toCvShare(
            message, sensor_msgs::image_encodings::BGR8);
        cv::Mat annotated = image_cv->image.clone();
        cv::putText(annotated, "No face detected", cv::Point(10, 25),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
        cv::imshow(window_name_, annotated);
        cv::waitKey(1);
      }
      return;
    }

    // Select one target for the RoboCup chase controller.
    const auto target =
        std::max_element(detections.begin(), detections.end(),
                         [](const Detection2D &left, const Detection2D &right) {
                           return left.confidence < right.confidence;
                         });

    // Pixel center for visual servoing. Publish even when depth is unavailable.
    geometry_msgs::msg::PointStamped center;
    center.header = message->header;
    center.point.x = target->box.x + target->box.width * 0.5;
    center.point.y = target->box.y + target->box.height * 0.5;
    center_publisher_->publish(center);

    // Cached depth must already be aligned to color pixels. This convenience
    // path does not transform coordinates or enforce timestamp synchronization.
    if (latest_depth_ && latest_camera_info_) {
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
    label << "target " << std::fixed << std::setprecision(2)
          << target->confidence;

    const int text_y = std::max(18, static_cast<int>(target->box.y) - 6);
    cv::putText(annotated, label.str(),
                cv::Point(static_cast<int>(target->box.x), text_y),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);

    cv::imshow(window_name_, annotated);
    cv::waitKey(1);
  }

  static constexpr const char *window_name_ = "3D Target Test";

  // Monotonic metrics timing is separate from ROS image-age diagnostics.
  std::chrono::steady_clock::time_point metrics_time_;
  size_t processed_frames_ = 0;
  size_t last_face_count_ = 0;
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
};

} // namespace humanoid_vision

/** @brief Run detection with serialized callbacks so cached camera state is safe. */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_vision::BallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
