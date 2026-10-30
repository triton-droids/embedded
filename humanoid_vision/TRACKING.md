# Head tracking in simulation

The active tracker consumes simulated camera intrinsics and detected target
pixels, then publishes a yaw/pitch command to the simulated head servo.

```bash
ros2 launch humanoid_sim sim.launch.py enable_vision:=true
```

The simulator renders the camera and supplies the head backend. For testing
only the tracker, publish target/camera-info fixtures and run:

```bash
ros2 launch humanoid_vision head_tracking.launch.py start_camera:=false start_agent:=false start_detector:=false
```

The existing launch source and defaults are unchanged. Its standalone defaults
request a physical camera and Agent; disable both with the flags above for
simulated inputs. The MuJoCo launch already passes these two overrides, so its
vision composition does not start physical devices. Request aligned depth from the simulator
with `enable_depth:=true` on `humanoid_sim`, not on the tracking launch.

| Interface | Purpose |
|---|---|
| `/camera/camera/color/image_raw` | Simulated color image for detection. |
| `/camera/camera/color/camera_info` | Camera intrinsics. |
| `/vision/target_center` | Target pixel center. |
| `/head/servo_target_rad` | Rate-limited head target consumed by `sim_head_mcu`. |

Use `--show-args` to inspect gains, direction, angle/rate limits, target timeout,
model path and topic overrides. `use_cuda:=false` selects CPU detection.
A full detector build needs the configured OpenCV 4.5.x SDK; `TRACK_ONLY=ON`
builds just the tracker and requires `start_detector:=false` at launch.

The ROS fixture exercise is `algorithm/test/test_track_node.py`; it needs a
sourced tracker install and DDS. Model conversion and algorithm details remain
in [the vision implementation guide](../docs/humanoid_vision_instructions.md).
The original physical-device instructions are retained in
[hardware/docs](../hardware/docs/head_tracking_original.md).
