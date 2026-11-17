#!/usr/bin/env bash
# Checks before the JetPack 6 batch, after jetson_link_setup.sh. Run it where the
# batch will run: on the WSL host (Jetson over the link) or on the laptop.
# Exit status 0 only if every check passes.
source "$(dirname -- "$0")/hosts.sh"
fail=0
ok()  { printf '  ok    %s\n' "$*"; }
bad() { printf '  FAIL  %s\n' "$*"; fail=1; }
check() { local what=$1; shift; if out=$("$@" 2>&1); then ok "$what${out:+: $out}"; else bad "$what${out:+: $out}"; fi; }

echo "== WSL ($WSL, Humble)"
check "reachable" on_wsl 'hostname'
[ "$WSL" = local ] && check "WSL distro is Humble" on_wsl 'test -d /opt/ros/humble'
check "eth1 192.168.50.1" on_wsl 'ip -4 -br addr | grep -q "192.168.50.1/24"'
check "ping Jetson 192.168.50.3" on_wsl 'ping -c 3 -W 2 192.168.50.3 | tail -1'
check "rmem_max >= 32 MB" on_wsl '(( $(cat /proc/sys/net/core/rmem_max) >= 33554432 ))'
check "checkout" on_wsl 'cd ~/ws/sim_experimental_cyclonedds && grep -q LIBGL_ALWAYS_SOFTWARE evaluation/jp6_vs_jp7/split_wsl6.sh && git log --oneline -1 && ls install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml >/dev/null'
check "runner files" on_wsl 'ls ~/split_summary.py ~/rtf_test/clk.py >/dev/null'
check "no simulator running" on_wsl '! pgrep -fa "[m]ujoco_sim|[r]os2 launch"'

echo "== Jetson ($JET, JetPack 6)"
check "reachable" on_jet 'head -1 /etc/nv_tegra_release | cut -c1-40'
check "power mode MAXN_SUPER" on_jet 'nvpmodel -q 2>/dev/null | grep -q MAXN_SUPER'
check "enP8p1s0 192.168.50.3" on_jet 'ip -4 -br addr show enP8p1s0 | grep -q "192.168.50.3/24"'
check "ping WSL 192.168.50.1" on_jet 'ping -c 3 -W 2 192.168.50.1 | tail -1'
check "rmem_max >= 32 MB" on_jet '(( $(cat /proc/sys/net/core/rmem_max) >= 33554432 ))'
check "build finished" on_jet 'grep -q "^EXIT=0" ~/jp6_build.log'
check "cuVSLAM 3.2 image" on_jet 'cd ~/Documents/Github/embedded-jp6-split && scripts/run_cuvslam.sh check'
check "detector engines" on_jet 'ls ~/Documents/Github/embedded-jp6-split/simulation/humanoid_ball_chase/assets/td-perception-fp16.engine >/dev/null'
check "runner files" on_jet 'ls ~/disable_probe.py >/dev/null'
check "nothing running" on_jet '! pgrep -fa "[r]os2 launch|[m]ujoco_sim" && ! docker ps -q --filter name=humanoid_ | grep -q .'
check "free memory" on_jet 'free -m | awk "/Mem:/ {print \$7\" MB available\"; exit !(\$7 > 4000)}"'

echo "== DDS across the link (5 s)"
check "WSL participant seen from Jetson" on_jet 'cd ~/Documents/Github/embedded-jp6-split && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.3 DDS_PEER_IP=192.168.50.1 CYCLONEDDS_URI=file://$PWD/install/humanoid_sim/share/humanoid_sim/config/cyclonedds_split.xml && (timeout 25 ros2 topic pub -r 5 /jp6_preflight std_msgs/msg/String "{data: jet}" >/dev/null 2>&1 < /dev/null &) && sleep 1'
check "  ...topic echoed on WSL" on_wsl 'cd ~/ws/sim_experimental_cyclonedds && export ROS_DISTRO=humble && source scripts/activate_rosenv.sh >/dev/null && export ROS_DOMAIN_ID=99 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp DDS_LOCAL_IP=192.168.50.1 DDS_PEER_IP=192.168.50.3 CYCLONEDDS_URI=file://$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/cyclonedds_split.xml && timeout 15 ros2 topic echo --no-daemon --once /jp6_preflight std_msgs/msg/String | head -1'

[ $fail = 0 ] && echo "All checks passed." || echo "Some checks failed."
exit $fail
