# Football + two-axis tracking validation — 2026-10-31

The scene now contains a black/white football and a physical pan/tilt D435
assembly. It runs the newer `td-perception.onnx` YOLO11n ball detector on CPU.
This replaces the earlier orange-ball HSV demonstration as the default path.
No physical hardware was operated, and locomotion chase remains unimplemented.

## Source and geometry

- CAD: `/home/rcli/Github/td_perception_camera`, remote
  `triton-droids/td_perception`, branch `p-33-ball-tracker`, commit
  `d75a50b8adb356a025fea7d9511fa55630d16477`. The 12 CAD STL assets match the
  archive's assembly tessellation exactly; provenance is in `assets/SOURCE.json`.
- Football visuals are procedural truncated-icosahedron meshes: 12 black
  pentagons and 20 white hexagons. Collision is a radius 0.10 m sphere, mass
  0.20 kg. The pitch is green; the football is still a free physical body.
- Model: the supplied checkout's seven-class `td-perception.onnx`; only class
  0 (ball) drives tracking. File hash and metadata license are preserved in
  `assets/MODEL_SOURCE.json`. No detector consumes simulator ball ground truth.

## Verified behavior

- Scene compiles in MuJoCo 3.3.7 with 12 actuators: ten leg, yaw and pitch.
- The actual football model detects the initial rendered green-pitch image at
  about 0.90 confidence. It failed on the earlier checkerboard floor; changing
  the scene to a football pitch resolved that test. This does not establish
  general real-world or background-independent detector accuracy.
- Physical ball contact, optical TF under positive/negative yaw and pitch,
  pitch drive in both directions, release and reset pass source-tree tests.
- Existing sensor/world/launch regressions and RGB-D/watchdog tests pass.
  The suite contains 72 checks, including the latest-frame inference and per-frame confidence regressions.
- CPU inference measured about 120–135 ms per frame on this host under load.
  Running inference directly in ROS image callbacks accumulated stale frames.
  Inference now runs in a separate worker with one replaceable pending sample;
  intermediate frames are dropped and the command watchdog keeps running.
  Camera acquisition remains 15 Hz; inference rate is not claimed to be 15 Hz.
  Intermittent source-age stops are still observed under CPU load; the node
  sends zero until a fresh observation arrives. This is not a real-time guarantee.
  Source/receipt checks remain active; the initial 0.35 s budget was later made configurable for CPU inference, as detailed below.
- Both axes reuse the existing `humanoid_vision/track_node`. The simulation
  pitch sign is +1 because positive rotation about the joint -X axis looks down. This sign was
  verified by actual measured pitch, not inferred from the hardware linkage.

## Runtime acceptance

The test repeatedly positioned the football at low/high/low heights
(0.11 / 0.65 / 0.11 m) to keep it at each test height against gravity.
Detection and tracking still used rendered RGB/depth only. It checked actual
`/head/joint_states` feedback, not merely servo commands, then moved the ball
behind the camera and returned it.

The first successful headless run measured pitch offsets approximately
`[-0.083, -0.372, -0.138]` rad for low/high/low, with peak football confidences
`[0.915, 0.925, 0.912]`, 29 lost/zero status frames and successful reacquisition.
The prior two attempts failed due to stale-frame accumulation; those were not
counted as successful runs. The latest-frame worker corrected that failure.
Before the heading correction, the 15 Hz GUI-camera run measured pitch offsets
`[-0.086, -0.368, -0.135]` rad, confidences `[0.915, 0.928, 0.917]`,
28 lost/zero frames and successful reacquisition. Results are stored in
`football_pitch_smoke.json`.
The separate horizontal regression also passed with the football model:
60 left tracking status samples, 79 right samples, 29 lost/zero samples,
yaw command range -0.175 to +0.103 rad. Its result is `runtime_smoke.json`.

## Limits

The robot remains held and its leg motors disabled by default. Existing C++
forwarding, policy, supervisor and Web monitor remain in the launch. The bundled
56-input tracking ONNX has no base velocity input, so pursuit requests still
have no locomotion consumer. No walking/release/fall trials establish chasing.

Mount adapter, masses/inertias, ideal intrinsics and 0.03 Nm pitch gearbox
friction are simulation assumptions. Pitch starts holding neutral (30 degrees
down) before first acquisition. Once commands begin, its 250 ms command
watchdog releases drive on timeout; simulated passive gearbox friction then
resists sag. This is not physical servo holding or watchdog validation.
The newer C++ vision package has not been copied wholesale or validated as a
Jazzy detector build; its ONNX is reused by this package's CPU runtime.

## Camera-facing correction and live depth

The earlier -Y camera direction was opposite the saved robot's +Y toes.
The deterministic toe-direction regression failed before the fix. The complete
CAD assembly, optical camera and ball spawn now face +Y. Pitch's joint axis is
-X so positive pitch continues to look down. Camera TF explicitly uses the
scene's +Y-forward base axes; the existing leg policy observation frame is
unchanged. Other legacy camera fixtures retain their configured default.

`runtime_depth_smoke.py` verified matching live RGB, depth, CameraInfo and
football-point stamps. The depth image is 424x240 `16UC1`, millimetres,
`camera_color_optical_frame`. At sampled football pixel (187,96), image depth
was 1.857 m and reported optical Z was 1.858 m. This is rendered simulation
depth, not physical D435 sensor validation. See `depth_smoke.json`.

The first orientation-correction follow-up runs did not pass the tracking
acceptance test: CPU inference under the current GUI/ROS load measured
0.46–0.48 s, exceeding the old 0.35 s visual TTL even with a latest-frame
worker. Lowering camera capture alone did not resolve it. The simulation
launch now exposes a 1.0 s visual image/receipt budget consistently to all
three visual controllers. The standalone detector keeps its 0.35 s default;
leg motor deadlines and the independent servo watchdog are unchanged. This
is an explicit longer visual stopping latency, not a real-time performance
claim. The final post-correction runtime result is recorded separately below.

Final post-correction acceptance passed with the full C++/safety/Web/policy
launch, camera 6 Hz and visual TTL 1.0 s: measured pitch low/high/low was
`[-0.114, -0.386, -0.211]` rad, confidences `[0.907, 0.921, 0.918]`,
17 lost/zero status samples and successful reacquisition. The horizontal
regression passed with 46 left and 47 right tracking samples, 17 lost/zero
samples, and yaw target range -0.179 to +0.132 rad. The final matched depth
sample measured 1.881 m in the image versus 1.883 m optical point Z.
The three JSON result artifacts now describe this +Y-forward, 1.0 s budget
configuration. The GUI is running on domain 61 and Web port 8081. The held
robot's leg-policy process still reported control-budget/feedback timeouts
under this load; this acceptance establishes camera tracking/depth only, not
healthy walking or locomotion control.
# A* global planner addition

2026-10-31: 11 new tests passed (10 ROS-independent algorithm cases plus one
actual DDS map/goal/odometry-to-Path round trip in isolated ROS domain 173).
Verified optimal open-grid diagonal length, wall detour, unreachable goal,
corner-cut rejection, radius-dependent passage closure, boundary clearance,
unknown/occupied endpoints, invalid geometry and search-budget exhaustion.
The ROS test also verifies empty-path invalidation on frame mismatch, rotated
map origin, and stale source odometry. No motor commands or physical hardware
were used. This validates path generation; autonomous walking is not established.
