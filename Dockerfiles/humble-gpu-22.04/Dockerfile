# Use the NVIDIA CUDA runtime base image
FROM nvidia/cuda:12.6.3-runtime-ubuntu22.04

WORKDIR /tdroids

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    neovim \
    vim \
    nano \
    wget \
    git \
    python3-pip \
    lsb-release \
    gnupg2 \
    build-essential \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Everything below is from ROS2 documentation
RUN add-apt-repository universe

# Get ROS 2 repository and its key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble desktop and dev tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

# Sourced container so no need to source once in
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose common ROS 2 ports (for example, DDS communication)
EXPOSE 11311 7400 7410 7420

CMD ["bash"]

