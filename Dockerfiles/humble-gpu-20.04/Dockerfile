# HUMBLE DOES NOT HAVE A PACKAGE IN UBUNTU 20.04. THIS DOCKERFILE IS MEANT TO SERVE AS BASEPLATE. YOU CAN COMPILE FROM SCRATCH IF YOU INSIST ON USING HUMBLE ON 20.04. PLEASE MESSAGE ME IF YOU HAVE INQUIRIES AT @github/kevshin2002 OR @discord/realkevinshin

# https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html# FOR SOURCE

# Use the NVIDIA CUDA runtime base image
FROM nvidia/cuda:12.6.3-runtime-ubuntu20.04

# Set the working directory
WORKDIR /tdroids

# Set environment variables for non-interactive installations
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

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
    && rm -rf /var/lib/apt/lists/*

# Add the ROS 2 repository and its key 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Update and install ROS 2 Humble desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# Source ROS 2 setup in the bashrc for interactive use
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Expose common ROS 2 ports (for example, DDS communication)
EXPOSE 11311 7400 7410 7420

# Default command
CMD ["bash"]


