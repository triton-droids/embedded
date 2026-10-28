# ROS 2 Humble / Ubuntu 22.04 development image, matching the robot's Jetson.
FROM osrf/ros:humble-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=1000

ENV DEBIAN_FRONTEND=noninteractive

# Dependencies for every package in this repo. rosdep runs again in
# post-create.sh as a safety net for anything added later.
# libopencv-dev is 4.5.4 on 22.04, which humanoid_vision requires.
RUN apt-get update && apt-get install -y --no-install-recommends \
      sudo git can-utils clangd jq \
      python3-venv python3-pip \
      python3-colcon-common-extensions python3-rosdep \
      libeigen3-dev libopencv-dev \
      ros-humble-moveit \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher-gui ros-humble-xacro \
      ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# ENCOS motor driver for motor_control_hybrid's encos_can_node. Only the CAN
# plugin is built, which keeps the GPL-3.0 EtherCAT plugin out. Installs to
# /usr/local, where the driver looks for its plugins by default.
ARG ENCOS_DRIVER_VERSION=v4.0.0
COPY zstd-shim/zstdConfig.cmake /tmp/zstd-shim/
RUN apt-get update && apt-get install -y --no-install-recommends \
      libcap-dev libcap2-bin libzstd-dev libspdlog-dev libfmt-dev \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 --branch "${ENCOS_DRIVER_VERSION}" \
      https://github.com/EncosTech/encos_driver.git /tmp/encos_driver \
    && cmake -S /tmp/encos_driver -B /tmp/encos_driver/build \
      -DCMAKE_BUILD_TYPE=Release -DENCOS_PLUGINS_LIST="can" \
      -Dzstd_DIR=/tmp/zstd-shim \
    && cmake --build /tmp/encos_driver/build -j"$(nproc)" \
    && cmake --install /tmp/encos_driver/build \
    && ldconfig \
    && rm -rf /tmp/encos_driver /tmp/zstd-shim

# Non-root user; devcontainers remaps the UID/GID to the host user on start.
RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
