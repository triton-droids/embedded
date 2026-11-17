#!/usr/bin/env bash
# Checks before the JetPack 6 batch; run on this laptop after jetson_link_setup.sh.
# Exit status 0 only if every check passes.
WSL=${WSL:-rcli@192.168.1.129}; JET=${JET:-td@100.71.186.19}
SSH="ssh -o ConnectTimeout=10"
fail=0
ok()  { printf '  ok    %s\n' "$*"; }
bad() { printf '  FAIL  %s\n' "$*"; fail=1; }
check() { local what=$1; shift; if out=$("$@" 2>&1); then ok "$what${out:+: $out}"; else bad "$what${out:+: $out}"; fi; }

echo "== WSL ($WSL, Humble)"
check "reachable" $SSH $WSL 'hostname'
check "eth1 192.168.50.1" $SSH $WSL 'ip -4 -br addr | grep -q "192.168.50.1/24"'
check "ping Jetson 192.168.50.3" $SSH $WSL 'ping -c 3 -W 2 192.168.50.3 | tail -1'
check "rmem_max >= 32 MB" $SSH $WSL '(( $(cat /proc/sys/net/core/rmem_max) >= 33554432 ))'
check "checkout" $SSH $WSL 'cd ~/ws/sim_experimental_cyclonedds && git log --oneline -1 && ls install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml >/dev/null'
check "runner files" $SSH $WSL 'ls ~/split_wsl6.sh ~/split_summary.py ~/rtf_test/clk.py >/dev/null'
check "no simulator running" $SSH $WSL '! pgrep -fa "[m]ujoco_sim|[r]os2 launch"'

echo "== Jetson ($JET, JetPack 6)"
check "reachable" $SSH $JET 'head -1 /etc/nv_tegra_release | cut -c1-40'
check "power mode MAXN_SUPER" $SSH $JET 'nvpmodel -q 2>/dev/null | grep -q MAXN_SUPER'
check "enP8p1s0 192.168.50.3" $SSH $JET 'ip -4 -br addr show enP8p1s0 | grep -q "192.168.50.3/24"'
check "ping WSL 192.168.50.1" $SSH $JET 'ping -c 3 -W 2 192.168.50.1 | tail -1'
check "rmem_max >= 32 MB" $SSH $JET '(( $(cat /proc/sys/net/core/rmem_max) >= 33554432 ))'
check "build finished" $SSH $JET 'grep -q "^EXIT=0" ~/jp6_build.log'
check "cuVSLAM 3.2 image" $SSH $JET 'cd ~/Documents/Github/embedded-jp6-split && scripts/run_cuvslam.sh check'
check "detector engines" $SSH $JET 'ls ~/Documents/Github/embedded-jp6-split/simulation/humanoid_ball_chase/assets/td-perception-fp16.engine >/dev/null'
check "runner files" $SSH $JET 'ls ~/split_jet6.sh ~/disable_probe.py >/dev/null'
check "nothing running" $SSH $JET '! pgrep -fa "[r]os2 launch|[m]ujoco_sim" && ! docker ps -q --filter name=humanoid_ | grep -q .'
check "free memory" $SSH $JET 'free -m | awk "/Mem:/ {print \$7\" MB available\"; exit !(\$7 > 4000)}"'

echo "== DDS across the link (5 s)"
check "WSL participant seen from Jetson" $SSH $JET 'cd ~/Documents/Github/embedded-jp6-split && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.3 DDS_PEER_IP=192.168.50.1 CYCLONEDDS_URI=file://$PWD/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml && (timeout 25 ros2 topic pub -r 5 /jp6_preflight std_msgs/msg/String "{data: jet}" >/dev/null 2>&1 &) && sleep 1'
check "  ...topic echoed on WSL" $SSH $WSL 'cd ~/ws/sim_experimental_cyclonedds && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.1 DDS_PEER_IP=192.168.50.3 CYCLONEDDS_URI=file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_split.xml && timeout 15 ros2 topic echo --once /jp6_preflight std_msgs/msg/String | head -1'

[ $fail = 0 ] && echo "All checks passed." || echo "Some checks failed."
exit $fail
