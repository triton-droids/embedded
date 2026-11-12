#!/usr/bin/env bash
# VIO + Isaac YOLO chase along a route, recorded. Usage: vio_chase.sh NAME SECONDS LEGS
set -u
name=$1; secs=$2; legs=$3
R=~/Documents/Github/embedded-vio; out=~/vio_runs/$name; mkdir -p $out
cd $R; export ROS_DISTRO=humble ROS_DOMAIN_ID=57
setsid scripts/run_cuvslam.sh > $out/cuvslam.log 2>&1 < /dev/null &
setsid scripts/run_isaac_yolo.sh > $out/yolo.log 2>&1 < /dev/null &
for i in $(seq 90); do grep -q "Loaded node" $out/cuvslam.log && grep -q "Loaded node '/yolov8_decoder'" $out/yolo.log && break; sleep 2; done
setsid scripts/ball_chase_walk.sh scene:=hard enable_vio:=true vision_detector:=isaac vision_target_class:=0 \
  > $out/sim.log 2>&1 < /dev/null &
sim=$!
set +u; source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$R/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_sim_humble.xml
for i in $(seq 120); do pgrep -f "[v]io_localizer" >/dev/null && break; sleep 1; done
for p in $(pgrep -f "[b]all_chase_vision|[v]io_localizer"); do renice -n 10 -p $p >/dev/null; done
ros2 run humanoid_localization route_driver --ros-args -p legs:=$legs > $out/route.log 2>&1 &
python3 ~/vio_runs/record_chase.py $secs $out/chase.npz > $out/record.log 2>&1 &
rec=$!
( sleep 3; for p in $(pgrep -f "[r]oute_driver|[r]ecord_chase|[m]otor_web_node"); do renice -n 19 -p $p >/dev/null; done ) &
timeout 1 true
for i in $(seq 120); do grep -q "Gantry released" $out/sim.log && break; sleep 2; done
wait $rec
timeout 5 ros2 topic echo --once /localization/status --field data > $out/status_end.txt 2>&1
timeout 4 ros2 topic echo --once /walk/status --field data > $out/walk_status.txt 2>&1
pkill -f "[r]oute_driver" 2>/dev/null
docker stop humanoid_cuvslam humanoid_isaac_yolo > /dev/null 2>&1
kill -INT -$sim; sleep 15; kill -9 -$sim 2>/dev/null
echo done
