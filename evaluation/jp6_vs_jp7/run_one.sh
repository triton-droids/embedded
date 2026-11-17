#!/usr/bin/env bash
# One JetPack 6 split run (same protocol as the JetPack 7 batch), driven from the
# WSL host itself or from the laptop (see hosts.sh).
# $1=RTF $2=TAG $3="extra launch args (both hosts)" $4="cuvslam args"|none
# $5="route_driver args" $6=ctl (control loop on WSL) or empty (control on Jetson)
RTF=$1; TAG=$2; EXTRA=${3:-}; CUV=${4:-}; ROUTE_ARGS=${5:-}; MODE=${6:-}
source "$(dirname -- "$0")/hosts.sh"
if [ "$MODE" = ctl ]; then
  WSL_ENV="CONTROL_CPUS=0-3 SIM_CPUS=4-15 LP_NUM_THREADS=6"; WSL_ROLE=sim_control; JET_ROLE=perception
else
  WSL_ENV=""; WSL_ROLE=sim; JET_ROLE=robot
fi
on_wsl "test -e ~/split_test/$TAG" && { echo "$TAG exists on WSL; pick another tag" >&2; exit 1; }
on_wsl "$WSL_ENV setsid nohup bash ~/split_wsl6.sh $RTF $TAG '$EXTRA' '$ROUTE_ARGS' $WSL_ROLE > /dev/null 2>&1 < /dev/null &"
sleep 15
on_jet "setsid nohup bash ~/split_jet6.sh $RTF $TAG '$EXTRA' '$CUV' $JET_ROLE > /dev/null 2>&1 < /dev/null &"
until on_wsl "test -f ~/split_test/$TAG/done" 2>/dev/null; do sleep 20; done
# Retry: the stop file must reach the Jetson even after a brief link drop.
until on_jet "touch ~/split_test/$TAG/stop" 2>/dev/null; do sleep 10; done
until on_jet "test -f ~/split_test/$TAG/done" 2>/dev/null; do sleep 5; done
on_wsl "python3 ~/split_summary.py $TAG"
