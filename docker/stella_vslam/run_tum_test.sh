#!/bin/bash
# Offline check on the Mac: stella_vslam on TUM freiburg1_xyz (same sequence as docker/orbslam3).
# Output in data/out/: map.msg, frame_trajectory.txt, keyframe_trajectory.txt, track_times.txt
#
# Uses run_tum_rgbd_slam (not run_image_slam) so poses carry the dataset's timestamps from rgb.txt.
# run_image_slam stamps frames with wall-clock time, which evo cannot match to groundtruth.txt.
#
# If evo is installed (pip install evo), the keyframe trajectory is also scored against the ground truth.
# Set EVO_APE to its path if it is not on PATH, e.g. EVO_APE=~/evo-venv/bin/evo_ape ./run_tum_test.sh
set -euo pipefail
cd "$(dirname "$0")"
SEQ=rgbd_dataset_freiburg1_xyz
mkdir -p data/out
if [ ! -d data/$SEQ/rgb ]; then
  curl -fL https://vision.in.tum.de/rgbd/dataset/freiburg1/$SEQ.tgz -o data/fr1_xyz.tgz
  tar xzf data/fr1_xyz.tgz -C data && rm data/fr1_xyz.tgz
fi

docker run --rm -v "$(pwd)/data:/data" stella-vslam:arm64 \
  /opt/stella_vslam_examples/build/run_tum_rgbd_slam \
    -v /opt/vocab/orb_vocab.fbow \
    -c /opt/stella_vslam/example/tum_rgbd/TUM_RGBD_mono_1.yaml \
    -d /data/$SEQ \
    --no-sleep --auto-term --viewer none \
    --eval-log-dir /data/out --map-db-out /data/out/map.msg

echo
echo "frames tracked:  $(wc -l < data/out/frame_trajectory.txt) / $(ls data/$SEQ/rgb | wc -l)"
echo "keyframes:       $(wc -l < data/out/keyframe_trajectory.txt)"
sort -n data/out/track_times.txt | awk '{a[NR]=$1} END {printf "tracking time:   median %.1f ms per frame\n", a[int(NR/2)+1]*1000}'

EVO_APE=${EVO_APE:-evo_ape}
if command -v "$EVO_APE" >/dev/null 2>&1; then
  # -a -s: Sim(3) alignment, needed because monocular SLAM has no metric scale
  rmse=$("$EVO_APE" tum data/$SEQ/groundtruth.txt data/out/keyframe_trajectory.txt -as --no_warnings \
    | awk '$1 == "rmse" {printf "%.1f", $2 * 1000}')
  echo "keyframe APE:    RMSE ${rmse} mm (Sim(3)-aligned)"
else
  echo "keyframe APE:    skipped (evo_ape not found; pip install evo, or set EVO_APE)"
fi
