/** @file image_viewer_node.cpp
 * @brief Display latest RGB/aligned-depth frames for manual camera diagnostics.
 * The viewer is not a synchronized measurement source or a control component.
 */
#include <memory>
#include <string>

#include "humanoid_vision/cv_bridge_compat.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace humanoid_vision {

/** @brief Cache the latest image on each topic and show a side-by-side preview. */
class ImageViewerNode : public rclcpp::Node {
public:
  /** @brief Subscribe to configured image topics; display can be disabled headlessly. */
  ImageViewerNode() : Node("image_viewer_node") {
    const std::string color_topic = declare_parameter<std::string>(
        "color_topic", "/camera/camera/color/image_raw");
    const std::string depth_topic = declare_parameter<std::string>(
        "depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    display_ = declare_parameter<bool>("display", true);

    const auto sensor_qos = rclcpp::SensorDataQoS();

    // Keep the latest color frame for side-by-side display.
    color_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        color_topic, sensor_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_color_ = message;
          display_images();
        });

    // Keep the latest aligned depth frame.
    depth_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic, sensor_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr message) {
          latest_depth_ = message;
          display_images();
        });

    RCLCPP_INFO(get_logger(), "RGB and depth viewer node started");
  }

  /** @brief Destroy the GUI window only when GUI display was enabled. */
  ~ImageViewerNode() override {
    if (display_) {
      cv::destroyWindow(window_name_);
    }
  }

private:
  /** @brief Render when both latest frames exist; ignore unsupported depth encodings.
   * 16UC1 is interpreted in millimeters and 32FC1 in meters. Color-map scaling
   * is for display only; it must not be reused as a metric depth conversion.
   */
  void display_images() {
    if (!display_ || !latest_color_ || !latest_depth_) {
      return;
    }

    const cv_bridge::CvImageConstPtr color_cv =
        cv_bridge::toCvShare(latest_color_, sensor_msgs::image_encodings::BGR8);
    const cv_bridge::CvImageConstPtr depth_cv =
        cv_bridge::toCvShare(latest_depth_, latest_depth_->encoding);

    cv::Mat depth_for_display;
    cv::Mat invalid_mask;

    // Convert millimeters or meters to an 8-bit depth image.
    if (latest_depth_->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      cv::convertScaleAbs(depth_cv->image, depth_for_display, 0.03);
      invalid_mask = depth_cv->image == 0;
    } else if (latest_depth_->encoding ==
               sensor_msgs::image_encodings::TYPE_32FC1) {
      cv::Mat finite_depth = depth_cv->image.clone();
      cv::patchNaNs(finite_depth, 0.0);
      cv::convertScaleAbs(finite_depth, depth_for_display, 51.0);
      invalid_mask = finite_depth <= 0.0F;
    } else {
      return;
    }

    // Colorize depth and keep pixels identified by the encoding-specific mask black.
    cv::Mat depth_color;
    cv::applyColorMap(depth_for_display, depth_color, cv::COLORMAP_JET);
    depth_color.setTo(cv::Scalar(0, 0, 0), invalid_mask);

    cv::Mat resized_depth;
    cv::resize(depth_color, resized_depth, color_cv->image.size(), 0.0, 0.0,
               cv::INTER_NEAREST);

    // Show color and aligned depth in one window.
    cv::Mat combined;
    cv::hconcat(color_cv->image, resized_depth, combined);
    cv::imshow(window_name_, combined);
    cv::waitKey(1);
  }

  static constexpr const char *window_name_ = "RGB and Depth Images";

  bool display_ = true;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_depth_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
};

} // namespace humanoid_vision

/** @brief Run the diagnostic viewer and process GUI events from image callbacks. */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<humanoid_vision::ImageViewerNode>());
  rclcpp::shutdown();
  return 0;
}
