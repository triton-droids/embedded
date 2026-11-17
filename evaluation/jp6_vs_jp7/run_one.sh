#!/usr/bin/env bash
# One JetPack 6 split run, driven from this laptop (same protocol as the JetPack 7
# batch). $1=RTF $2=TAG $3="extra launch args (both hosts)" $4="cuvslam args"|none
# $5="route_driver args" $6=ctl (control loop on WSL) or empty (control on Jetson)
RTF=$1; TAG=$2; EXTRA=${3:-}; CUV=${4:-}; ROUTE_ARGS=${5:-}; MODE=${6:-}
WSL=${WSL:-rcli@192.168.1.129}; JET=${JET:-td@100.71.186.19}
SSH="ssh -o ConnectTimeout=10 -o ServerAliveInterval=15"
if [ "$MODE" = ctl ]; then
  WSL_ENV="CONTROL_CPUS=0-3 SIM_CPUS=4-15 LP_NUM_THREADS=6"; WSL_ROLE=sim_control; JET_ROLE=perception
else
  WSL_ENV=""; WSL_ROLE=sim; JET_ROLE=robot
fi
$SSH $WSL "test -e ~/split_test/$TAG" && { echo "$TAG exists on WSL; pick another tag" >&2; exit 1; }
$SSH $WSL "$WSL_ENV setsid nohup bash ~/split_wsl6.sh $RTF $TAG '$EXTRA' '$ROUTE_ARGS' $WSL_ROLE > /dev/null 2>&1 < /dev/null &"
sleep 15
$SSH $JET "setsid nohup bash ~/split_jet6.sh $RTF $TAG '$EXTRA' '$CUV' $JET_ROLE > /dev/null 2>&1 < /dev/null &"
until $SSH $WSL "test -f ~/split_test/$TAG/done" 2>/dev/null; do sleep 20; done
$SSH $JET "touch ~/split_test/$TAG/stop"
until $SSH $JET "test -f ~/split_test/$TAG/done" 2>/dev/null; do sleep 5; done
$SSH $WSL "python3 ~/split_summary.py $TAG"
