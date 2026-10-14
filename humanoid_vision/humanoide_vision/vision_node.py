"""Display RealSense color and aligned depth images side by side."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2


class VisionNode(Node):
    """Subscribe to camera streams and display their latest frames."""

    def __init__(self):
        """Initialize subscriptions and image buffers."""
        super().__init__('vision_node')
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None

        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info('Vision node has been started.')

    def image_callback(self, msg):
        """Store the latest color image in BGR format."""
        self.rgb_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='bgr8'
        )
        self.display_images()

    def depth_callback(self, msg):
        """Store the latest depth image without changing its encoding."""
        self.depth_image = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )
        self.display_images()

    def camera_info_callback(self, msg):
        """Store the latest color camera calibration."""
        self.camera_info = msg

    def display_images(self):
        """Display the latest frames, marking invalid depth pixels black."""
        if self.rgb_image is None or self.depth_image is None:
            return
        depth_color = cv2.applyColorMap(
            cv2.convertScaleAbs(self.depth_image, alpha=0.03),
            cv2.COLORMAP_JET,
        )
        depth_color[self.depth_image == 0] = (0, 0, 0)

        height, width = self.rgb_image.shape[:2]
        depth_color_resized = cv2.resize(
            depth_color, (width, height),
            interpolation=cv2.INTER_NEAREST,
        )
        combined = np.hstack((self.rgb_image, depth_color_resized))
        cv2.imshow('RGB and Depth Images', combined)
        cv2.waitKey(1)


def main(args=None):
    """Run the vision node and clean up its windows on exit."""
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
