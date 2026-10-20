FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive

# Parallel build jobs. Default 2 keeps peak memory low (needed on small Docker VMs).
# Override with: docker build --build-arg JOBS=8 -t orbslam3 .
ARG JOBS=2

# Pinned ORB-SLAM3 commit (reproducible builds)
ARG ORB_SLAM3_COMMIT=4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ca-certificates sudo wget unzip \
    libeigen3-dev libopencv-dev libboost-serialization-dev libssl-dev \
    python3 python3-numpy \
    libgl1-mesa-dev libglew-dev libepoxy-dev libegl1-mesa-dev \
    libxkbcommon-dev libwayland-dev wayland-protocols ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Pangolin v0.8
WORKDIR /opt
RUN git clone --recursive https://github.com/stevenlovegrove/Pangolin.git \
    && cd Pangolin && git checkout v0.8 && git submodule update --init --recursive
WORKDIR /opt/Pangolin
RUN cmake -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_PANGOLIN_PYTHON=OFF \
    && cmake --build build -j${JOBS} && cmake --install build && ldconfig

# ORB-SLAM3 (pinned commit)
WORKDIR /opt
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git \
    && git -C ORB_SLAM3 checkout ${ORB_SLAM3_COMMIT}
WORKDIR /opt/ORB_SLAM3

# Portability / toolchain patches
RUN sed -i 's/-march=native//g' CMakeLists.txt Thirdparty/DBoW2/CMakeLists.txt Thirdparty/g2o/CMakeLists.txt \
    && sed -i 's/-std=c++11/-std=c++14/g' CMakeLists.txt Thirdparty/Sophus/CMakeLists.txt \
    && sed -i 's/ -Werror//' Thirdparty/Sophus/CMakeLists.txt

# Headless: disable the Pangolin viewer in the TUM monocular example.
# The grep -q fails the build if the patch did not apply.
RUN sed -i -E 's/(System::MONOCULAR,[[:space:]]*)true/\1false/' Examples/Monocular/mono_tum.cc \
    && grep -q 'System::MONOCULAR,false' Examples/Monocular/mono_tum.cc

RUN chmod +x build.sh && sed -i "s/make -j[0-9]*/make -j${JOBS}/" build.sh && ./build.sh

WORKDIR /workspace
CMD ["/bin/bash"]
