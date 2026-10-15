#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cv_bridge/cv_bridge.hpp>
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

class BallDetectionNode : public rclcpp::Node {
public:
  BallDetectionNode() : Node("ball_detection_node") {
    const std::string package_share =
        ament_index_cpp::get_package_share_directory("humanoid_vision");

    const std::string color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/camera/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    const std::string camera_info_topic = declare_parameter<std::string>(
        "camera_info_topic", "/camera/camera/color/camera_info");
    const std::string model_path = declare_parameter<std::string>(
        "model_path", package_share + "/models/yolov8n-face-lindevs.onnx");

    show_image_ = declare_parameter<bool>("show_image", false);

    YoloDetector::Config config;
    config.input_width = declare_parameter<int>("input_width", 640);
    config.input_height = declare_parameter<int>("input_height", 640);
    config.confidence_threshold = static_cast<float>(
        declare_parameter<double>("confidence_threshold", 0.25));
    config.nms_threshold =
        static_cast<float>(declare_parameter<double>("nms_threshold", 0.45));
    config.use_cuda = declare_parameter<bool>("use_cuda", false);

    // Use the face model to test the ball-position pipeline for now.
    detector_ = std::make_unique<YoloDetector>(model_path, config);

    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic, rclcpp::SensorDataQoS(),
        std::bind(&BallDetectionNode::image_callback, this,
                  std::placeholders::_1));

    // Keep the latest aligned depth frame and camera intrinsics.
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_depth_ = message;
        });

    camera_info_subscription_ =
        create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic, rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
              latest_camera_info_ = message;
            });

    position_publisher_ = create_publisher<geometry_msgs::msg::PointStamped>(
        "/vision/ball_3d_position", 10);

    RCLCPP_INFO(get_logger(), "Ball-position test node started");
  }

  ~BallDetectionNode() override {
    if (show_image_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr message) {
    const std::vector<Detection2D> detections = detector_->detect(message);

    if (detections.empty()) {
      return;
    }

    // Select one target for the RoboCup chase controller.
    const auto target =
        std::max_element(detections.begin(), detections.end(),
                         [](const Detection2D &left, const Detection2D &right) {
                           return left.confidence < right.confidence;
                         });

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
};

} // namespace humanoid_vision

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_vision::BallDetectionNode>());
  rclcpp::shutdown();
  return 0;
}
