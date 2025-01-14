# Use the NVIDIA CUDA runtime base image
FROM nvidia/cuda:12.6.3-runtime-ubuntu22.04

# Set the working directory
WORKDIR /tdroids

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=iron

# Install basic dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    wget \
    git \
    python3-pip \
    lsb-release \
    gnupg2 \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages using pip
RUN pip3 install --no-cache-dir \
    vcstool \
    colcon-common-extensions \
    flake8-docstrings \
    pytest-cov \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures

# Add the ROS 2 repository and its key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Update and install ROS 2 Iron desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-iron-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup in the bashrc for interactive use
RUN echo "source /opt/ros/iron/setup.bash" >> ~/.bashrc

# Expose common ROS 2 ports (for example, DDS communication)
EXPOSE 11311 7400 7410 7420

# Default command
CMD ["bash"]


