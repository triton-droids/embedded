#!/usr/bin/env bash
# VIO + Isaac YOLO chase along a route, recorded. Usage: vio_chase.sh NAME SECONDS LEGS
# ISAAC=split runs cuVSLAM and YOLO as two containers (default: one, run_isaac_perception.sh).
# DETECTOR=builtin: no Isaac YOLO; cuVSLAM alone and ball_chase_vision's own TensorRT detector.
# VIO=0: no stereo, cuVSLAM or localizer (Isaac YOLO only); the route starts without waiting.
# Logs /sim/status, /walk/status and tegrastats from before the simulator starts (sim_status.log,
# walk_status.log, tegra_full.log). Timing diagnostics go to sim.log: walking-controller cycles or
# gaps over 40 ms ('Slow:'), the simulator's per-second maxima ('DIAG', HUMANOID_SIM_DIAG) and
# cpp_control_node's command ages ('CPPDIAG', HUMANOID_CPP_DIAG).
set -u
name=$1; secs=$2; legs=$3
R=~/Documents/Github/embedded-vio; out=~/vio_runs/$name; mkdir -p $out
cd $R; export ROS_DISTRO=humble ROS_DOMAIN_ID=57
detector_args="vision_detector:=isaac vision_target_class:=0"
vio_args="enable_vio:=true"; route_args=""
if [ "${VIO:-1}" = 0 ]; then
  vio_args="share_state:=true"; route_args="-p wait_localization:=false"
  echo "Loaded node '/visual_slam_node' (not used)" > $out/cuvslam.log
  if [ "${DETECTOR:-isaac}" = builtin ]; then
    detector_args=""; echo "Loaded node '/yolov8_decoder' (not used)" > $out/yolo.log
  else
    setsid scripts/run_isaac_yolo.sh > $out/yolo.log 2>&1 < /dev/null &
  fi
elif [ "${DETECTOR:-isaac}" = builtin ]; then
  detector_args=""
  setsid scripts/run_cuvslam.sh > $out/cuvslam.log 2>&1 < /dev/null &
  echo "Loaded node '/yolov8_decoder' (not used)" > $out/yolo.log
elif [ "${ISAAC:-combined}" = split ]; then
  setsid scripts/run_cuvslam.sh > $out/cuvslam.log 2>&1 < /dev/null &
  setsid scripts/run_isaac_yolo.sh > $out/yolo.log 2>&1 < /dev/null &
else
  setsid scripts/run_isaac_perception.sh > $out/yolo.log 2>&1 < /dev/null &
  ln -sf yolo.log $out/cuvslam.log
fi
for i in $(seq 90); do grep -q "Loaded node '/visual_slam_node'" $out/cuvslam.log && grep -q "Loaded node '/yolov8_decoder'" $out/yolo.log && break; sleep 2; done
set +u; source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$R/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_sim_humble.xml
ros2 topic echo /sim/status std_msgs/msg/String --field data > $out/sim_status.log 2>/dev/null &
ros2 topic echo /walk/status std_msgs/msg/String --field data > $out/walk_status.log 2>/dev/null &
tegrastats --interval 1000 > $out/tegra_full.log 2>/dev/null &
sleep 5
HUMANOID_SIM_DIAG=1 HUMANOID_CPP_DIAG=1 setsid scripts/ball_chase_walk.sh scene:=hard walk_slow_log_ms:=40 $vio_args $detector_args \
  > $out/sim.log 2>&1 < /dev/null &
sim=$!
for i in $(seq 120); do pgrep -f "[b]all_chase_vision" >/dev/null && break; sleep 1; done; sleep 5
for p in $(pgrep -f "[b]all_chase_vision|[v]io_localizer"); do renice -n 10 -p $p >/dev/null; done
ros2 run humanoid_localization route_driver --ros-args -p legs:=$legs $route_args > $out/route.log 2>&1 &
python3 ~/vio_runs/record_chase.py $secs $out/chase.npz > $out/record.log 2>&1 &
rec=$!
( sleep 3; for p in $(pgrep -f "[r]oute_driver|[r]ecord_chase|[m]otor_web_node|[t]opic echo /(sim|walk)/status"); do renice -n 19 -p $p >/dev/null; done ) &
for i in $(seq 120); do grep -q "Gantry released\|aborted" $out/sim.log && break; sleep 2; done
( sleep 30; top -b -n 3 -d 5 -o %CPU > $out/top.txt; free -m > $out/free.txt;
  tegrastats --interval 1000 > $out/tegra.txt 2>/dev/null & t=$!; sleep 5; kill $t ) &
wait $rec
timeout 5 ros2 topic echo --once /localization/status --field data > $out/status_end.txt 2>&1
timeout 4 ros2 topic echo --once /walk/status --field data > $out/walk_status.txt 2>&1
pkill -f "[r]oute_driver|[t]opic echo /(sim|walk)/status" 2>/dev/null; pkill tegrastats 2>/dev/null
docker stop humanoid_cuvslam humanoid_isaac_yolo humanoid_isaac_perception > /dev/null 2>&1
kill -INT -$sim; sleep 15; kill -9 -$sim 2>/dev/null
echo done
