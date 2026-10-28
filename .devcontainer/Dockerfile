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
      sudo git can-utils \
      python3-venv python3-pip \
      python3-colcon-common-extensions python3-rosdep \
      libeigen3-dev libopencv-dev \
      ros-humble-moveit \
      ros-humble-ros2-control ros-humble-ros2-controllers \
      ros-humble-joint-state-publisher-gui ros-humble-xacro \
      ros-humble-realsense2-camera \
    && rm -rf /var/lib/apt/lists/*

# Non-root user; devcontainers remaps the UID/GID to the host user on start.
RUN groupadd --gid "${USER_GID}" "${USERNAME}" \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" -m -s /bin/bash "${USERNAME}" \
    && usermod -aG dialout,video "${USERNAME}" \
    && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > "/etc/sudoers.d/${USERNAME}" \
    && chmod 0440 "/etc/sudoers.d/${USERNAME}"

USER ${USERNAME}
