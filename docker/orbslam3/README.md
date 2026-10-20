# ORB-SLAM3 (headless Docker)

Reproducible, headless ORB-SLAM3 build for the AlohaMini2 SLAM work (issue #39).
Build verified on arm64 (Apple Silicon) only.

- Base: Ubuntu 22.04 (works on arm64 and, untested so far, x86_64)
- OpenCV 4.5.4, Eigen 3.4, Pangolin v0.8
- ORB-SLAM3 pinned to commit `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`
- Patches applied at build time: `-march=native` removed, C++14, `-Werror` removed from Sophus, Pangolin viewer disabled in `mono_tum`

ORB-SLAM3 is GPLv3 (https://github.com/UZ-SLAMLab/ORB_SLAM3). This repo only contains the Dockerfile; the source is downloaded during the build.

## Build

```bash
docker build -t orbslam3 .
```

This takes 20+ minutes. To use more CPU cores (needs more Docker memory):

```bash
docker build --build-arg JOBS=8 -t orbslam3 .
```

## Run the TUM freiburg1_xyz test

```bash
mkdir -p data && cd data
curl -L -O https://vision.in.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.tgz
tar xzf rgbd_dataset_freiburg1_xyz.tgz
cd ..

docker run --rm -v "$(pwd)/data:/workspace/data" orbslam3 bash -c '
cd /workspace/data &&
/opt/ORB_SLAM3/Examples/Monocular/mono_tum \
  /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  /opt/ORB_SLAM3/Examples/Monocular/TUM1.yaml \
  /workspace/data/rgbd_dataset_freiburg1_xyz'
```

Expected: 798 images processed, `New Map created with 424 points`, and `data/KeyFrameTrajectory.txt` written (about 52 keyframes; the count can vary by a few between runs).

## Notes

- The container has no display and no USB camera access. The viewer is disabled; judge results from the saved trajectory files.
- Docker Desktop needs at least 4 GB of memory allocated.
- Only `mono_tum` is patched headless. Other examples still enable the viewer.
