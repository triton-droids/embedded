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
    wget \
    git \
    python3-pip \
    lsb-release \
    gnupg2 \
    build-essential \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Everything below is from ROS2 Installation Documentation
RUN add-apt-repository universe

# ROS 2 repository and its key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# ROS2 Development tools
RUN apt-get update && apt-get install -y ros-dev-tools \
    && apt-get upgrade -y

# Install ROS 2 Iron desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Expose common ROS 2 ports
EXPOSE 11311 7400 7410 7420

# Default command
CMD ["bash"]

