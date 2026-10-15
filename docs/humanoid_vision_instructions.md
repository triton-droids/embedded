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
│   ├── yolov8n-face-lindevs.onnx
│   └── yolov8n-face-lindevs-opencv45.onnx
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
cd /home/darin/Documents/Github/embedded
ROS_DISTRO=humble source scripts/setup_rosenv.sh
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

## Humble compatibility and headless camera check

The package supports both the Humble `cv_bridge.h` and Jazzy
`cv_bridge.hpp` headers through `humanoid_vision/cv_bridge_compat.hpp`.
The bundled face model remains the pipeline test model.

Run the CPU face regression test after building:

```bash
colcon build --packages-select humanoid_vision --symlink-install --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/humanoid_vision --output-on-failure
```

Start a D435 with aligned depth (640 x 480 at 15 FPS is supported on the
USB 2 connection checked on this machine):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true align_depth.enable:=true \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x480x15
```

In a second sourced terminal, run detection without a display:

```bash
ros2 run humanoid_vision ball_detection_node --ros-args \
  -p show_image:=false -p use_cuda:=false
```

USB device access and ROS networking must be available to the process.
A restricted sandbox can fail to initialize the udev monitor even when the
camera is connected and usable from a host terminal.

The package requires OpenCV 4.5.x to match Humble's `cv_bridge`. On this
Jetson, Ubuntu 4.5.4 development/runtime packages are extracted under
`.deps/opencv45` (ignored by Git), while the system Jetson SDK stays installed.
CMake automatically selects this local SDK before loading `cv_bridge`.
To recreate it and rebuild:

```bash
bash scripts/setup_opencv45.sh
colcon build --packages-select humanoid_vision --symlink-install \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON
ctest --test-dir build/humanoid_vision --output-on-failure
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

With `show_image:=true`, the window displays received frames even when no
face is detected. The terminal reports detection status every five seconds;
if no recent color frame arrives, it warns to check the camera driver.
Window display requires a graphical session; use `show_image:=false` over a
headless connection.

The node parameters are:

| Parameter | Default | Purpose |
| --- | --- | --- |
| `color_topic` | `/camera/camera/color/image_raw` | Color input |
| `depth_topic` | `/camera/camera/aligned_depth_to_color/image_raw` | Aligned depth input |
| `camera_info_topic` | `/camera/camera/color/camera_info` | Color intrinsics |
| `model_path` | Installed OpenCV 4.5 face ONNX model | Temporary detector |
| `input_width` | `640` | YOLO input width |
| `input_height` | `640` | YOLO input height |
| `confidence_threshold` | `0.25` | Detection threshold |
| `nms_threshold` | `0.45` | NMS overlap threshold |
| `use_cuda` | `true` on CUDA builds | TensorRT FP16 engine; false selects OpenCV CPU |
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

## CUDA / TensorRT on Jetson

CUDA inference uses TensorRT 10 FP16 while ROS image conversion and
post-processing continue to use OpenCV 4.5. No CUDA OpenCV build is required.
A CUDA-enabled build defaults to `use_cuda:=true`. CPU-only builds default to
false. CUDA initialization or inference failures are reported; there is no
silent CPU fallback.

Build a device-specific engine and install it with:

```bash
bash scripts/build_vision_cuda.sh
```

The script uses the original ONNX model, TensorRT optimization level 0 for
a quick first build, and the installed `trtexec`. Set `TRT_OPTIMIZATION_LEVEL`
to request another builder level. It writes
`.deps/tensorrt/yolov8n-face-fp16.engine`, and installs it into the package's
model directory. Rebuild the engine after changing GPU, TensorRT version, or
model. CUDA runtime and TensorRT development packages must already be installed.
The backend accepts a fixed FP32 input `[1,3,640,640]` and FP32 YOLO output;
TensorRT uses FP16 internally.

With the camera driver running, start the detector:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run humanoid_vision ball_detection_node --ros-args \
  -p use_cuda:=true -p show_image:=false
```

Use `show_image:=true` from a graphical desktop to display the preview.
Startup must report `backend: TensorRT CUDA FP16`. Every five seconds the
node reports processed FPS and detection time (including image preparation
and post-processing). Input subscriptions keep only the latest frame to
avoid accumulating old camera images.

For CPU fallback explicitly set `use_cuda:=false`; the default model path
then selects the OpenCV-compatible ONNX model. An explicit `model_path`
must point to an engine for CUDA or ONNX for CPU.

Run reference-image checks and a warm benchmark on the host:

```bash
build/humanoid_vision/test_yolo_face \
  .deps/tensorrt/yolov8n-face-fp16.engine \
  humanoid_vision/algorithm/test/data/lena.jpg /tmp/face-cuda.jpg \
  --cuda --reference-face --benchmark
```

Validated on the Orin Nano Super with TensorRT 10.3 / CUDA 12.6:

- Reference face: CUDA confidence 0.833496, box `(212,186,144,203)`;
  CPU confidence 0.833417 with the same box.
- Warm reference-image CUDA detection: about 32 ms per frame, including
  preprocessing and post-processing (30 measured iterations).
- Live CUDA node: 14.6–14.9 FPS with approximately 30 ms detection time,
  using the existing 15 FPS camera input. Preview was disabled for this test.
- The formal workspace is `/home/darin/Documents/Github/embedded`; source
  `/opt/ros/humble/setup.bash` and its `install/setup.bash` before launching.

Camera FPS caps live processing FPS regardless of GPU speed. The tested
USB 2 connection uses 640 x 480 at 15 FPS. Check the actual color topic rate;
use a USB 3 connection for reliable higher-resolution/higher-rate RGB-D.

## Model compatibility and verification

The default model is `yolov8n-face-lindevs-opencv45.onnx`. It retains the
original face weights and `[1, 5, 8400]` output, with fixed 640 x 640 input.
The original model is retained as the conversion source. To regenerate:

```bash
python3 -m pip install --target .deps/model-tools 'onnx==1.16.2' 'numpy<2'
PYTHONPATH=.deps/model-tools python3 scripts/convert_face_opencv45.py
```

The conversion changes Split inputs to opset-12 attributes, removes default
Reshape attributes, provides explicit Resize ROI tensors, expands stride
constants, and replaces constant-first subtraction with Neg plus Add.
These rewrites accommodate OpenCV 4.5's importer and broadcasting behavior.
The regression checks both confidence and box coordinates on the bundled
reference image, rather than merely checking that a face was found.

RGB/depth time synchronization, TF conversion, filtering, and tracking are
not implemented yet. The current model detects faces for pipeline testing;
the 3D output retains the future ball interface name.
