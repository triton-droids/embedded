# Humanoid Vision: Current Instructions

## Scope

This document describes the vision code that currently exists in this
repository. It covers the C++ ROS 2 package, the temporary face model, aligned
depth sampling, the published 3D target point, and the environment needed to
build and inspect the package.

The current node is a pipeline test. It detects a face as a temporary target so
that image input, YOLO post-processing, depth lookup, and 3D publication can be
tested before a RoboCup ball model is installed.

## Current Package Layout

```text
humanoid_vision/
├── algorithm/
│   ├── include/humanoid_vision/
│   │   ├── types.hpp
│   │   ├── depth_sampler.hpp
│   │   ├── deprojection.hpp
│   │   ├── detection_postprocess.hpp
│   │   ├── target_3d_estimator.hpp
│   │   ├── target_filter.hpp
│   │   └── target_tracker.hpp
│   ├── src/
│   │   ├── depth_sampler.cpp
│   │   ├── deprojection.cpp
│   │   ├── detection_postprocess.cpp
│   │   ├── target_3d_estimator.cpp
│   │   ├── target_filter.cpp
│   │   └── target_tracker.cpp
│   └── test/
├── models/
│   └── yolov8n-face-lindevs.onnx
├── src/
│   ├── ball_detection_node.cpp
│   └── image_viewer_node.cpp
├── CMakeLists.txt
└── package.xml
```

`depth_sampler`, `deprojection`, `target_filter`, and `target_tracker` are
currently placeholders. The implemented detection and 3D-estimation code is in
`detection_postprocess` and `target_3d_estimator`.

## Environment

Use the repository environment script from the repository root:

```bash
cd /home/rcli/Github/Humanoid_ros2/embedded
ROS_DISTRO=jazzy source scripts/setup_rosenv.sh
```

Use `ROS_DISTRO=humble` on an Ubuntu 22.04/Humble machine. The script sources
ROS, creates or activates `rosenv`, and installs the Python dependencies used by
the motor-control stack.

The vision runtime is C++. Its OpenCV, Eigen, `cv_bridge`, and ROS message
dependencies come from the system/ROS installation, not from `rosenv`. Installing
`opencv-python` inside `rosenv` does not change the OpenCV version linked into
`ball_detection_node`.

Build only the vision package with:

```bash
colcon build --packages-select humanoid_vision --symlink-install
source install/setup.bash
```

## Required Camera Topics

The default configuration expects a RealSense color stream with depth aligned
to color:

```text
/camera/camera/color/image_raw
/camera/camera/aligned_depth_to_color/image_raw
/camera/camera/color/camera_info
```

The aligned depth image and color `CameraInfo` must describe the same pixel
grid. Do not pair an unaligned depth image with color-camera intrinsics.

Check the streams before starting detection:

```bash
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw

ros2 topic echo \
  /camera/camera/aligned_depth_to_color/image_raw \
  --once --field header.frame_id
```

For a common D400-series configuration, 640 x 480 at 30 Hz is a useful target.
An observed detection rate near 13 Hz can be acceptable for an initial chase
test, but the raw camera rate and the detection-output rate must be measured
separately.

## Run the Current Test Node

```bash
ros2 run humanoid_vision ball_detection_node --ros-args \
  -p show_image:=true \
  -p use_cuda:=false
```

The node parameters are:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `color_topic` | `/camera/camera/color/image_raw` | Color input |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | Aligned depth input |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Color intrinsics |
| `model_path` | Installed face ONNX model | Temporary detector |
| `input_width` | `640` | YOLO input width |
| `input_height` | `640` | YOLO input height |
| `confidence_threshold` | `0.25` | Detection threshold |
| `nms_threshold` | `0.45` | NMS overlap threshold |
| `use_cuda` | `false` | Request OpenCV DNN CUDA |
| `show_image` | `false` | Display the annotated frame |

The node selects the highest-confidence detection and publishes:

```text
/vision/ball_3d_position
geometry_msgs/msg/PointStamped
```

The topic name describes the future ball interface even though the current
model detects faces.

Inspect the result with:

```bash
ros2 topic hz /vision/ball_3d_position
ros2 topic echo /vision/ball_3d_position
```

## Depth Estimation

Both the native RealSense overload and the ROS image overload use the median of
valid depth samples in a 7 x 7 window around the detection center. Invalid,
non-finite, and non-positive samples are excluded before `std::nth_element`
selects the median.

The ROS overload supports:

```text
16UC1: depth in millimetres, converted to metres
32FC1: depth already in metres
```

The deprojected point has camera optical-frame coordinates:

```text
X: right
Y: down
Z: forward
```

The published header is copied from the aligned depth image. A controller must
transform this `PointStamped` into `base_link` before using it for robot motion.

## Coordinate-Frame Check

ROS robot frames and camera optical frames use different axis conventions:

```text
camera optical: X right, Y down, Z forward
base_link:      X forward, Y left, Z up
```

A future moving head requires this connected TF tree:

```text
base_link
└── neck_yaw_link
    └── neck_pitch_link
        └── camera_link
            └── camera_color_optical_frame
```

For a fixed camera, a calibrated static transform is sufficient. For a moving
head, real yaw and pitch encoder feedback must drive the dynamic transforms.

## CUDA and Model Compatibility

`use_cuda:=true` selects:

```text
OpenCV DNN CUDA backend
OpenCV DNN CUDA FP16 target
```

This works only when the C++ OpenCV library was built with CUDA, cuDNN, and
`OPENCV_DNN_CUDA`. Confirm the linked OpenCV build with:

```bash
opencv_version --verbose | grep -E "OpenCV|NVIDIA CUDA|cuDNN"
```

The currently installed system OpenCV 4.6 build does not contain the CUDA
modules and cannot parse the temporary face model. The same model was validated
with OpenCV 4.10, but a Python OpenCV 4.10 wheel does not repair the C++ node.
The C++ runtime must link against a compatible OpenCV build.

## Current Verification Level

- The `humanoid_vision` package compiles successfully.
- The face ONNX model was validated separately with OpenCV 4.10.
- The installed C++ OpenCV 4.6 runtime is not compatible with that model.
- Live camera publication of the 3D point has not yet been verified.
- RGB/depth time synchronization, TF conversion, filtering, and tracking are
  not implemented yet.
