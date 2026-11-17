#!/usr/bin/env bash
# Robot host (Jetson, JetPack 6 / Humble): robot stack + cuVSLAM (Isaac ROS 3.2
# via scripts/run_cuvslam.sh -> run_cuvslam_jp6.sh). Same steps as the JetPack 7
# batch (~/split_jet.sh there). Stops when $O/stop appears.
# $1=RTF $2=TAG $3="extra launch args" $4="cuvslam args" or none $5=robot|perception
RTF=${1:-1.0}; TAG=${2:-run}; EXTRA=${3:-}; CUV=${4:-}; SPLIT=${5:-robot}
O=~/split_test/$TAG; mkdir -p $O
cd "${REPO:-$HOME/Documents/Github/embedded-jp6-split}" || exit 1
LOCAL=${LOCAL_IP:-192.168.50.3}; PEER=${PEER_IP:-192.168.50.1}
export ROS_DISTRO=humble ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-12} DDS_LOCAL_IP=$LOCAL DDS_PEER_IP=$PEER
{ date -Is; git log --oneline -1; git status -s | grep -v '^??'; head -1 /etc/nv_tegra_release; nvpmodel -q 2>/dev/null | head -1
  echo "RTF=$RTF EXTRA=$EXTRA CUV=$CUV SPLIT=$SPLIT"; } > $O/meta.txt
setsid bash scripts/ball_chase_split.sh $SPLIT $LOCAL $PEER scene:=hard enable_vio:=true \
  enable_astar:=true real_time_factor:=$RTF $EXTRA > $O/robot.log 2>&1 < /dev/null &
R=$!
( source scripts/activate_rosenv.sh >/dev/null; export ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file://$PWD/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml; exec setsid python3 ~/disable_probe.py > $O/probe.log 2>&1 < /dev/null ) &
( setsid tegrastats --interval 5000 > $O/tegra.log 2>&1 < /dev/null ) &
[ "$CUV" = none ] || setsid bash scripts/run_cuvslam.sh $CUV > $O/cuvslam.log 2>&1 < /dev/null &
until [ -f $O/stop ]; do sleep 5; done
docker stop humanoid_cuvslam >/dev/null 2>&1
pkill -f '[d]isable_probe.py'; pkill -f '[t]egrastats --interval 5000'; kill -INT -- -$R 2>/dev/null; sleep 10; kill -9 -- -$R 2>/dev/null
echo DONE > $O/done
