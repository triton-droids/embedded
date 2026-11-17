#!/usr/bin/env bash
# On WSL: one ab_vio-style run; after the robot starts walking, sample stereo stamps on both hosts.
TAG=${1:-j6_diag_vio1}; D=~/ws/sim_experimental_cyclonedds/evaluation/jp6_vs_jp7; O=~/jp6_diag/$TAG; mkdir -p $O
setsid bash $D/run_one.sh 0.5 $TAG "odometry:=leg" "" > $O/run.log 2>&1 < /dev/null &
until grep -q "Leg 2/16" ~/split_test/$TAG/route.log 2>/dev/null; do sleep 5; done
ENV_W='cd ~/ws/sim_experimental_cyclonedds && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.1 DDS_PEER_IP=192.168.50.3 CYCLONEDDS_URI=file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_split.xml'
ENV_J='cd ~/Documents/Github/embedded-jp6-split && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=12 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.3 DDS_PEER_IP=192.168.50.1 CYCLONEDDS_URI=file://$PWD/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml'
( bash -c "$ENV_W && python3 $D/stamp_probe.py 30" > $O/probe_wsl.txt 2>&1 ) &
ssh -o BatchMode=yes td@192.168.50.3 "$ENV_J && python3 evaluation/jp6_vs_jp7/stamp_probe.py 30" > $O/probe_jet.txt 2>&1 &
sleep 10
top -b -n1 -w 160 | sed -n 1,16p > $O/top_wsl.txt
ssh -o BatchMode=yes td@192.168.50.3 'top -b -n1 -w 160 | sed -n 1,16p' > $O/top_jet.txt
wait %2 %3 2>/dev/null; sleep 25
echo PROBED > $O/probed
