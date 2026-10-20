# DRAFT replica of the AlohaMini2 Raspberry Pi software environment.
# Status: built from the documented LeRobot-on-Pi install pattern, NOT yet
# verified against the real Pi. Run pi_inventory.sh on the Pi and reconcile
# every ARG below with its output before treating this as a replica.
#
# Build (arm64, matches the Pi):
#   docker build -t alohamini-pi .
# Run (shell):
#   docker run -it --rm alohamini-pi

FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive

# --- Values to confirm against the Pi ---------------------------------------
# LeRobot source. The AlohaMini fork is assumed; confirm with `git remote -v`
# in the Pi's lerobot checkout. Upstream alternative:
#   https://github.com/huggingface/lerobot.git
ARG LEROBOT_REPO=https://github.com/liyitenga/lerobot_alohamini.git
# Pinned to the fork's main as of 2026-10-09 (the commit the draft was verified with).
# Replace with the Pi's `git rev-parse HEAD` from pi_inventory.txt. Empty = default branch.
ARG LEROBOT_REF=ec65318f4cc3d830242f6e987d43f46e6b61f0a5
# Python version of the Pi's lerobot environment.
ARG PYTHON_VERSION=3.12
# Set to 0 if the Pi does not have ROS2 installed.
ARG INSTALL_ROS=1
ARG ROS_DISTRO=jazzy
# -----------------------------------------------------------------------------

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git pkg-config ca-certificates curl wget sudo \
    usbutils v4l-utils iputils-ping openssh-client \
    libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# ROS2 (optional)
RUN if [ "${INSTALL_ROS}" = "1" ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
      && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
      && apt-get update \
      && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions \
      && rm -rf /var/lib/apt/lists/* ; \
    fi

# Miniforge (conda) -- LeRobot's Pi install guide uses a conda env named "lerobot"
RUN curl -sSL -o /tmp/miniforge.sh \
      "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-$(uname -m).sh" \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh
ENV PATH=/opt/conda/bin:$PATH

RUN conda create -y -n lerobot python=${PYTHON_VERSION} \
    && conda install -y -n lerobot -c conda-forge ffmpeg \
    && conda clean -afy
# Make `python`/`pip` resolve to the lerobot env even in non-interactive commands
ENV PATH=/opt/conda/envs/lerobot/bin:$PATH

# LeRobot with Feetech servo support, plus ZeroMQ (used by the host/client scripts)
WORKDIR /root
RUN git clone ${LEROBOT_REPO} lerobot \
    && if [ -n "${LEROBOT_REF}" ]; then git -C lerobot checkout ${LEROBOT_REF}; fi
SHELL ["/bin/bash", "-c"]
RUN source /opt/conda/etc/profile.d/conda.sh && conda activate lerobot \
    && cd lerobot && pip install -e ".[feetech]" pyzmq

# Activate the env and source ROS in every interactive shell
RUN echo 'source /opt/conda/etc/profile.d/conda.sh && conda activate lerobot' >> /root/.bashrc \
    && if [ "${INSTALL_ROS}" = "1" ]; then echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc; fi

WORKDIR /root/lerobot
CMD ["/bin/bash"]
