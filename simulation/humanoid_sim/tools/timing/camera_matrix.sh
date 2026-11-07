#!/usr/bin/env bash
# Timing of the feedback streams with and without the head camera.
#
#   simulation/humanoid_sim/tools/timing/camera_matrix.sh            # experimental launch
#   LAUNCH=sim simulation/humanoid_sim/tools/timing/camera_matrix.sh # original, Fast DDS
#   WALK=1 ...  also two auto_start runs (gantry released, robot walks)
#
# Held on the gantry (falls cannot confound timing), each configuration runs
# WARMUP_S (15) then timing_probe.py for SECONDS_PER_RUN (30):
#   A  no camera     B  640x480 @ 15 Hz     C  B + depth     D  320x240 @ 5 Hz
# Note: the probe itself joins the graph at WARMUP_S; with Fast DDS that join
# alone can cause one ~330 ms stall inside the window.
# Results: $OUT_ROOT/matrix_<launch>_<host>_<time>/summary.md
source "$(dirname -- "${BASH_SOURCE[0]}")/common.sh"
timing_setup || exit 1
SECONDS_PER_RUN="${SECONDS_PER_RUN:-30}"
WARMUP_S="${WARMUP_S:-15}"
DOMAIN="${DOMAIN:-81}"
out="$(new_out_dir matrix)"
write_env "$out/environment.txt"
cat "$out/environment.txt"

share_cfg="$(ros2 pkg prefix humanoid_sim)/share/humanoid_sim/config/sim.yaml"
lowres_cfg="$out/sim_camera_320x240_5hz.yaml"
sed -e 's/camera_width: 640/camera_width: 320/' -e 's/camera_height: 480/camera_height: 240/' \
    -e 's/rate_hz: 15.0/rate_hz: 5.0/' "$share_cfg" > "$lowres_cfg"

names=(); configs=()
add() { names+=("$1"); configs+=("$2"); }
add A_nocam             "enable_vision:=false"
add B_cam640_15hz       "enable_vision:=true start_detector:=false"
add C_cam640_15hz_depth "enable_vision:=true start_detector:=false enable_depth:=true"
add D_cam320_5hz        "enable_vision:=true start_detector:=false sim_config_file:=$lowres_cfg"

run_one() {  # name, launch arguments, auto_start
  local tag="$1_$([[ $3 == true ]] && echo walk || echo held)"
  echo; echo "=== $tag"
  stop_domain "$DOMAIN"
  # shellcheck disable=SC2086
  start_launch "$DOMAIN" "$out/$tag.launch.log" use_viewer:=false auto_start:="$3" $2
  sleep "$WARMUP_S"
  ROS_DOMAIN_ID="$DOMAIN" python3 "$TOOLS_DIR/timing_probe.py" --seconds "$SECONDS_PER_RUN" \
    --label "$tag" --out "$out/$tag.json" > "$out/$tag.probe.log" 2>&1 \
    || echo "probe failed: $out/$tag.probe.log"
  stop_domain "$DOMAIN"
  sleep 2
}

for i in "${!names[@]}"; do run_one "${names[$i]}" "${configs[$i]}" false; done
if [[ "${WALK:-0}" == 1 ]]; then
  run_one A_nocam "enable_vision:=false" true
  run_one B_cam640_15hz "enable_vision:=true start_detector:=false" true
fi
python3 "$TOOLS_DIR/summarize.py" "$out" | tee "$out/summary.md"
echo; echo "Results: $out"
