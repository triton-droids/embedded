#!/usr/bin/env bash
# Isaac YOLO vs ball_chase_vision on the hard scene. Usage: yolo_trial.sh NAME SECONDS
set -u
name=$1; secs=$2
R=~/Documents/Github/embedded-vio; out=~/vio_runs/$name; mkdir -p $out
cd $R; export ROS_DISTRO=humble ROS_DOMAIN_ID=58
setsid scripts/ball_chase_walk.sh scene:=hard > $out/sim.log 2>&1 < /dev/null &
sim=$!
setsid scripts/run_isaac_yolo.sh > $out/yolo.log 2>&1 < /dev/null &
set +u; source scripts/activate_rosenv.sh >/dev/null 2>&1; source install/setup.bash; set -u
export ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$R/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_sim_humble.xml
for i in $(seq 120); do grep -q "Gantry released" $out/sim.log && break; sleep 2; done
sleep 5
( sleep 20; top -b -n 3 -d 5 -o %CPU > $out/top.txt ) &
python3 ~/vio_runs/compare_yolo.py $secs > $out/compare.json 2> $out/compare.err
tegrastats --interval 1000 > $out/tegra.txt 2>/dev/null & tp=$!; sleep 3; kill $tp 2>/dev/null
docker stop humanoid_isaac_yolo > /dev/null 2>&1
kill -INT -$sim; sleep 15; kill -9 -$sim 2>/dev/null
echo done
