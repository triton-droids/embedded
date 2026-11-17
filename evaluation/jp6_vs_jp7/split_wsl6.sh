#!/usr/bin/env bash
# Simulator host (WSL, Humble distro on port 22) for the JetPack 6 runs.
# Same steps as the JetPack 7 batch (~/split_wsl.sh in the Jazzy distro): sim +
# route_driver + localization_eval (estimate) + localization_eval (leg /odom).
# $1=RTF $2=TAG $3="extra launch args" $4="route_driver args" $5=sim|sim_control
RTF=${1:-1.0}; TAG=${2:-run}; EXTRA=${3:-}; ROUTE_ARGS=${4:-}; SPLIT=${5:-sim}
O=~/split_test/$TAG; mkdir -p $O
cd "${REPO:-$HOME/ws/sim_experimental_cyclonedds}" || exit 1
LOCAL=${LOCAL_IP:-192.168.50.1}; PEER=${PEER_IP:-192.168.50.3}
export ROS_DISTRO=humble ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-12} DDS_LOCAL_IP=$LOCAL DDS_PEER_IP=$PEER
# Render with Mesa llvmpipe like the Jazzy distro of the JetPack 7 batch. The
# Humble distro's Mesa 23.2 picks d3d12 by default: 9 fps at 640x480 vs 61 fps
# for llvmpipe, which left the stereo pair at 6.7 Hz instead of 15 Hz (sim time).
export LIBGL_ALWAYS_SOFTWARE=${LIBGL_ALWAYS_SOFTWARE-1}
{ date -Is; git log --oneline -1; echo "RTF=$RTF EXTRA=$EXTRA ROUTE=$ROUTE_ARGS SPLIT=$SPLIT CONTROL_CPUS=${CONTROL_CPUS:-} SIM_CPUS=${SIM_CPUS:-} LIBGL_ALWAYS_SOFTWARE=$LIBGL_ALWAYS_SOFTWARE"; } > $O/meta.txt
setsid env CONTROL_CPUS=${CONTROL_CPUS:-} SIM_CPUS=${SIM_CPUS:-} ${LP_NUM_THREADS:+LP_NUM_THREADS=$LP_NUM_THREADS} bash scripts/ball_chase_split.sh $SPLIT $LOCAL $PEER scene:=hard enable_vio:=true \
  enable_astar:=true real_time_factor:=$RTF $EXTRA > $O/sim.log 2>&1 < /dev/null &
SIM=$!
source scripts/activate_rosenv.sh >/dev/null
export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_split.xml"
# route_driver parks the ball before the robot stack releases the gantry.
setsid ros2 run humanoid_localization route_driver --ros-args -p use_sim_time:=true -p legs:=16 $ROUTE_ARGS > $O/route.log 2>&1 < /dev/null &
ROUTE=$!
setsid ros2 run humanoid_localization localization_eval --ros-args -p use_sim_time:=true -p csv:=$O/loc.csv > $O/eval.log 2>&1 < /dev/null &
EVAL=$!
setsid ros2 run humanoid_localization localization_eval --ros-args -r __node:=leg_eval -p use_sim_time:=true \
  -p estimate_topic:=/odom -p csv:=$O/leg.csv > $O/eval_leg.log 2>&1 < /dev/null &
EVAL2=$!
( sleep 150; python3 ~/rtf_test/clk.py > $O/rtf.txt 2>&1 ) &
END=done
for _ in $(seq 240); do
  kill -0 $ROUTE 2>/dev/null || break; kill -0 $SIM 2>/dev/null || { END=sim_died; break; }
  grep -q 'Physics paused' $O/sim.log && { END=paused; break; }
  # route_driver logs every leg (~10 s); 120 s of silence means the run is stuck.
  if [ $(( $(date +%s) - $(stat -c %Y $O/route.log) )) -gt 120 ]; then
    END=stuck
    for n in 1 2; do P=$(pgrep -f '[m]ujoco_sim' | head -1); echo "== sample $n $(date +%T.%N)" >> $O/threads.txt
      for t in /proc/$P/task/*; do echo "$(basename $t) $(cat $t/comm) $(awk '{print $3, $14+$15}' $t/stat) $(cat $t/wchan)"; done >> $O/threads.txt; sleep 2; done
    top -b -n1 -w 150 | sed -n 7,14p >> $O/threads.txt
    pkill -USR1 -f '[m]ujoco_sim'; pkill -USR1 -f '[s]im_imu_publisher'; pkill -USR1 -f '[s]im_stereo_publisher'; sleep 3; break
  fi
  sleep 10
done
sleep 15
kill -INT -- -$EVAL -$EVAL2 2>/dev/null; sleep 8
echo "END=$END" > $O/end.txt
kill -INT -- -$ROUTE 2>/dev/null; kill -INT -- -$SIM 2>/dev/null; sleep 10; kill -9 -- -$SIM -$ROUTE -$EVAL -$EVAL2 2>/dev/null
echo DONE > $O/done
