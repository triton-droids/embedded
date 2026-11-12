#!/usr/bin/env bash
# One VIO trial: cuVSLAM first, then sim (hard scene, stereo, localizer), evaluation from before release.
# Usage: [ROUTE_LEGS=16] [RECORD_S=290] trial.sh NAME EVAL_S [launch args]
set -u
name=$1; eval_s=$2; shift 2
R=~/Documents/Github/embedded-vio; out=~/vio_runs/$name; mkdir -p $out
cd $R; export ROS_DISTRO=humble ROS_DOMAIN_ID=57
setsid scripts/run_cuvslam.sh > $out/cuvslam.log 2>&1 < /dev/null &
for i in $(seq 60); do grep -q "Loaded node" $out/cuvslam.log && break; sleep 2; done
setsid scripts/ball_chase_walk.sh scene:=hard enable_vio:=true "$@" > $out/sim.log 2>&1 < /dev/null &
sim=$!
set +u; source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$R/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_sim_humble.xml
for i in $(seq 120); do pgrep -f "[v]io_localizer" >/dev/null && break; sleep 1; done
for p in $(pgrep -f "[b]all_chase_vision|[v]io_localizer"); do renice -n 10 -p $p >/dev/null; done
ros2 run humanoid_localization localization_eval --ros-args -p duration_s:=$eval_s -p csv:=$out/errors.csv > $out/eval.json 2> $out/eval.err &
ev=$!
[ -n "${ROUTE_LEGS:-}" ] && { ros2 run humanoid_localization route_driver --ros-args -p legs:=$ROUTE_LEGS > $out/route.log 2>&1 & }
[ -n "${RECORD_S:-}" ] && { python3 ~/vio_runs/record_route.py $RECORD_S $out/record.npz > $out/record.log 2>&1 & }
( sleep 3; for p in $(pgrep -f "[r]oute_driver|[l]ocalization_eval|[r]ecord_route|[m]otor_web_node"); do renice -n 19 -p $p >/dev/null; done ) &
for i in $(seq 120); do grep -q "Gantry released" $out/sim.log && break; sleep 2; done
timeout 5 ros2 topic echo --once /localization/status --field data > $out/status_start.txt 2>&1
wait $ev
timeout 5 ros2 topic echo --once /localization/status --field data > $out/status_end.txt 2>&1
timeout 4 ros2 topic echo --once /walk/status --field data > $out/walk_status.txt 2>&1
pkill -f "[r]oute_driver" 2>/dev/null
docker stop humanoid_cuvslam > /dev/null 2>&1
kill -INT -$sim; sleep 15; kill -9 -$sim 2>/dev/null
echo done
