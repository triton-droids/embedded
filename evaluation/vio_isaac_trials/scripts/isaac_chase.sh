#!/usr/bin/env bash
# Chase with Isaac detections. Usage: isaac_chase.sh NAME CLASS SECONDS
set -u
name=$1; cls=$2; secs=$3
R=~/Documents/Github/embedded-vio; out=~/vio_runs/$name; mkdir -p $out
cd $R; export ROS_DISTRO=humble ROS_DOMAIN_ID=58
setsid scripts/run_isaac_yolo.sh > $out/yolo.log 2>&1 < /dev/null &
for i in $(seq 60); do grep -q "Loaded node '/yolov8_decoder'" $out/yolo.log && break; sleep 2; done
setsid scripts/ball_chase_walk.sh scene:=hard vision_detector:=isaac vision_target_class:=$cls share_state:=true > $out/sim.log 2>&1 < /dev/null &
sim=$!
set +u; source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$R/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_sim_humble.xml
for i in $(seq 120); do grep -q "Gantry released" $out/sim.log && break; sleep 2; done
if [ -n "${BALL_XY:-}" ]; then  # Move the ball, e.g. BALL_XY=0.8,1.0
  ros2 topic pub --once /ball_chase/throw_position geometry_msgs/msg/PointStamped \
    "{header: {frame_id: world}, point: {x: ${BALL_XY%,*}, y: ${BALL_XY#*,}, z: 0.11}}" > /dev/null 2>&1
fi
timeout 5 ros2 topic echo --once /sim/ground_truth/odom --field pose.pose.position > $out/start_pos.txt 2>&1
[ -n "${RECORD:-}" ] && { python3 ~/vio_runs/record_chase.py $secs $out/chase.npz > $out/record.log 2>&1 & rec=$!; }
sleep $secs
[ -n "${RECORD:-}" ] && wait $rec
timeout 5 ros2 topic echo --once /sim/ground_truth/odom --field pose.pose.position > $out/end_pos.txt 2>&1
timeout 5 ros2 topic echo --once /walk/status --field data > $out/walk_status.txt 2>&1
docker stop humanoid_isaac_yolo > /dev/null 2>&1
kill -INT -$sim; sleep 15; kill -9 -$sim 2>/dev/null
echo done
