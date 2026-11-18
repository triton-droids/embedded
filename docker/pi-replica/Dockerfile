# Replica of the AlohaMini2 Raspberry Pi software environment (issue #41).
# Matched to pi_inventory.txt taken 2026-11-09 from host "tdroids-desktop":
#   Ubuntu 24.04.4 LTS arm64, Raspberry Pi 4 Model B Rev 1.5, 3.7 GiB RAM, kernel 6.8.0-1065-raspi
#   user tdroids (uid 1000), Miniforge at ~/miniforge3 with base Python 3.13.13, conda env "so101"
#   NOT on the Pi: ROS2, a "lerobot" conda env, any LeRobot checkout, Docker, v4l-utils
#
# What a container cannot replicate: the raspi kernel, GNOME desktop, systemd services (tailscaled, ssh, ...)
# and the USB devices. See README.md.
#
# Build (arm64, matches the Pi):   docker build -t alohamini-pi .
# Run (shell, as tdroids):         docker run -it --rm alohamini-pi

# noble-20260905 is the newest Ubuntu image still at 24.04.4 (20260917 is 24.04.5); the Pi reports 24.04.4.
FROM ubuntu:noble-20260905
ENV DEBIAN_FRONTEND=noninteractive

# --- Matched to the Pi --------------------------------------------------------
ARG PI_USER=tdroids
ARG PI_UID=1000
# Miniforge base env: the Pi reports Python 3.13.13 (standard GIL build, not free-threaded).
ARG BASE_PYTHON=3.13.13
# --- Unknown: not captured by the 2026-11-09 inventory --------------------------
# Python version and packages of the Pi's "so101" env. Re-run pi_inventory.sh (it now exports every
# conda env) and set these. Empty SO101_PYTHON = create no so101 env.
ARG SO101_PYTHON=
# --- Not on the Pi; opt-in for planned work --------------------------------------
ARG INSTALL_ROS=0
ARG ROS_DISTRO=jazzy
ARG INSTALL_LEROBOT=0
ARG LEROBOT_REPO=https://github.com/liyitenga/lerobot_alohamini.git
ARG LEROBOT_REF=ec65318f4cc3d830242f6e987d43f46e6b61f0a5
ARG LEROBOT_PYTHON=3.12
# Debugging tools the Pi does NOT have (v4l-utils, build-essential, ...). 1 = install them.
ARG DEV_TOOLS=0
# ------------------------------------------------------------------------------

# Command-line packages from the Pi's `apt-mark showmanual` (desktop/GNOME/snap/kernel packages left out).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl wget git sudo less vim-tiny file lsof psmisc strace rsync time \
    iproute2 iputils-ping netcat-openbsd bind9-dnsutils openssh-client \
    usbutils pciutils lshw dmidecode udev kmod locales tzdata xz-utils \
    python3 python3-pip python3-venv python3-tk \
    && if [ "${DEV_TOOLS}" = "1" ]; then \
         apt-get install -y --no-install-recommends build-essential cmake pkg-config v4l-utils; fi \
    && rm -rf /var/lib/apt/lists/*

# User tdroids, uid 1000, same groups as on the Pi. ubuntu:24.04 ships a uid-1000 "ubuntu" user; remove it.
# Passwordless sudo is a container convenience; the Pi asks for a password.
# The user is created first so its own group also gets gid 1000; the desktop-only groups get system gids.
RUN userdel -r ubuntu 2>/dev/null || true \
    && groupadd -g ${PI_UID} ${PI_USER} \
    && for g in lpadmin sambashare; do groupadd -f -r "$g"; done \
    && useradd -m -u ${PI_UID} -g ${PI_UID} -s /bin/bash -G adm,dialout,cdrom,sudo,dip,plugdev,users,lpadmin,sambashare ${PI_USER} \
    && echo "${PI_USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${PI_USER}

# ROS2 (opt-in; NOT on the Pi as of 2026-11-09)
RUN if [ "${INSTALL_ROS}" = "1" ]; then \
      curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
      && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
      && apt-get update \
      && apt-get install -y --no-install-recommends ros-${ROS_DISTRO}-ros-base python3-colcon-common-extensions \
      && rm -rf /var/lib/apt/lists/* ; \
    fi

USER ${PI_USER}
WORKDIR /home/${PI_USER}
SHELL ["/bin/bash", "-c"]
ENV CONDA_DIR=/home/${PI_USER}/miniforge3

# Miniforge in the same place as on the Pi, base Python pinned to the Pi's minor version
RUN curl -sSL -o /tmp/miniforge.sh \
      "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-$(uname -m).sh" \
    && bash /tmp/miniforge.sh -b -p "$CONDA_DIR" && rm /tmp/miniforge.sh \
    && "$CONDA_DIR/bin/conda" install -y -n base "python=${BASE_PYTHON}=*_cp313" \
    && "$CONDA_DIR/bin/python" --version | grep -qx "Python ${BASE_PYTHON}" \
    && "$CONDA_DIR/bin/conda" clean -afy
ENV PATH=/home/${PI_USER}/miniforge3/bin:$PATH

# so101 env: exists on the Pi, contents unknown, so only created when SO101_PYTHON is set
RUN if [ -n "${SO101_PYTHON}" ]; then conda create -y -n so101 "python=${SO101_PYTHON}" && conda clean -afy; fi

# LeRobot (opt-in; NOT on the Pi as of 2026-11-09). Kept in its own env so it cannot be mistaken for so101.
RUN if [ "${INSTALL_LEROBOT}" = "1" ]; then \
      conda create -y -n lerobot "python=${LEROBOT_PYTHON}" \
      && conda install -y -n lerobot -c conda-forge ffmpeg \
      && git clone ${LEROBOT_REPO} lerobot \
      && if [ -n "${LEROBOT_REF}" ]; then git -C lerobot checkout ${LEROBOT_REF}; fi \
      && conda run -n lerobot pip install -e "./lerobot[feetech]" pyzmq \
      && conda clean -afy ; \
    fi

# .bashrc as on the Pi: conda + mamba init blocks, then the MiniAloha shortcuts
# (ALOHA_WEB_HOST is the Pi's own Tailscale address).
RUN conda init bash >/dev/null \
    && (mamba shell init --shell bash --root-prefix "$CONDA_DIR" >/dev/null 2>&1 || true) \
    && printf '\n# MiniAloha shortcuts\nexport ALOHA_WEB_HOST=100.113.252.70\nexport ALOHA_WEB_PORT=8000\n' >> ~/.bashrc \
    && if [ "${INSTALL_ROS}" = "1" ]; then echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc; fi

CMD ["/bin/bash", "-l"]
