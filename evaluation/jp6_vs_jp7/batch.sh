#!/usr/bin/env bash
# JetPack 6 batch matching the JetPack 7 comparison runs (all 0.5x, hard scene,
# 16-leg route, A*, odometry:=leg). Groups (JetPack 7 tag -> JetPack 6 tag):
#   base    base01-10    control on Jetson, no VIO          -> j6_base01..
#   ab_vio  ab_vio01-10  control on Jetson, cuVSLAM + IMU   -> j6_ab_vio01..
#   ab_vo   ab_vo01-10   control on Jetson, stereo VO only  -> j6_ab_vo01..
#   c_vio   c_vio01-10   control on WSL (sim_control/perception), VIO -> j6_c_vio01..
#   c_vo    c_vo01-10    control on WSL, VO                 -> j6_c_vo01..
# Rounds are interleaved (one run of every group per round), so stopping early
# still leaves a balanced comparison. About 12 min per run.
#   bash batch.sh                    # 10 rounds, all groups (~10 h)
#   N=3 RUN_GROUPS="ab_vio ab_vo" bash batch.sh
#   START=4 bash batch.sh            # resume from round 4
# Stop after the current run: touch /tmp/jp6_batch.stop
D=$(cd -- "$(dirname -- "$0")" && pwd)
N=${N:-10}; START=${START:-1}; RUN_GROUPS=${RUN_GROUPS:-"base ab_vio ab_vo c_vio c_vo"}
rm -f /tmp/jp6_batch.stop
for i in $(seq -w "$START" "$N"); do
  i=$(printf %02d $((10#$i)))
  for g in $RUN_GROUPS; do
    [ -f /tmp/jp6_batch.stop ] && { echo "stopped before j6_$g$i"; exit 0; }
    tag=j6_$g$i
    echo "$(date +%T) start $tag"
    case $g in
      base)   bash "$D/run_one.sh" 0.5 $tag "odometry:=leg enable_vio:=false" none "-p wait_localization:=false" ;;
      ab_vio) bash "$D/run_one.sh" 0.5 $tag "odometry:=leg" "" ;;
      ab_vo)  bash "$D/run_one.sh" 0.5 $tag "odometry:=leg" "imu_fusion:=false" ;;
      c_vio)  bash "$D/run_one.sh" 0.5 $tag "odometry:=leg" "" "" ctl ;;
      c_vo)   bash "$D/run_one.sh" 0.5 $tag "odometry:=leg" "imu_fusion:=false" "" ctl ;;
      *) echo "unknown group $g" >&2; exit 2 ;;
    esac
  done
done
echo "$(date +%T) batch done"
