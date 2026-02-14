# Setup a Raspberry Pi with Ubuntu 22.04

> Tested on a Raspberry Pi 4 running Ubuntu 22.04 LTS (Server/Desktop).  
> This tutorial was written and updated on **2026-02-14**.

This document is a step-by-step guide for configuring a Raspberry Pi as the **host device** to control a humanoid robot.

It covers:
- Essential system tools
- Programming environment setup
- Optional network settings

By following this guide, you will install:
- **Python 3.10** and the Python libraries used in this project
- **gcc/g++ 11** (Ubuntu 22.04 default toolchain)
- **Rust (latest stable)** via `rustup`
- **ROS 2 Humble**

---

## 0. Prerequisites

- Ubuntu **22.04 LTS** (Server or Desktop) is installed on the Raspberry Pi.
  - Other Linux versions may work but are **not tested**.
- You can access a terminal on the Pi via **SSH**, **wired network**, or **local GUI**.
- You have `sudo` privileges on the Pi.

---

## 1. System Configuration

### 1.1 Update system packages

```bash
sudo apt update
sudo apt -y upgrade
sudo reboot
```

After reboot:

```bash
sudo apt update
```

### 1.2 Optional setup

Set timezone (example: Los Angeles):

```bash
sudo timedatectl set-timezone America/Los_Angeles
timedatectl
```

Set hostname:

```bash
sudo hostnamectl set-hostname tdroids-pi4
hostname
```

### 1.3 Enable SSH

If not yet enabled, install and start the SSH server:

```bash
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
sudo systemctl status ssh --no-pager
```

(Optional) check that port 22 is listening:

```bash
sudo ss -lntp | grep ':22'
```

---

## 2. Install Essential Tools

```bash
sudo apt install -y \
  git curl wget ca-certificates \
  htop tmux unzip zip \
  net-tools iputils-ping
```

---

## 3. Install Programming Environments

### 3.1 Python

Ubuntu 22.04 ships with Python 3.10 by default. Verify:

```bash
python3 --version
python3.10 --version
```

Install pip, venv, and headers:

```bash
sudo apt install -y python3-pip python3-venv python3.10-dev
```

Install commonly used libraries (edit as needed):

```bash
python3 -m pip install --user --upgrade pip
python3 -m pip install --user \
  pyserial \
  python-can
```

Install RoboStride SDK:

```bash
python3 -m pip install --user robostride
```

> Tip: If `pip install robostride` fails, the package may not be on PyPI.  
> In that case, install from your Git repo or local source instead.

### 3.2 C/C++ Toolchain

Ubuntu 22.04 default toolchain is gcc/g++ 11.

Install:

```bash
sudo apt install -y build-essential cmake
```

Verify:

```bash
gcc --version
g++ --version
cmake --version
```

### 3.3 Rust

> As of 2026-02-14, Rust is not required yet, but is planned for future development.

Install Rust via rustup:

```bash
sudo apt install -y curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

Reload shell environment:

```bash
source "$HOME/.cargo/env"
```

Verify:

```bash
rustc --version
cargo --version
rustup --version
```

Update Rust (if needed):

```bash
rustup update
```

---

## 4. ROS 2 Installation (Humble)

This project is only tested on **ROS 2 Humble** (Ubuntu 22.04).

### 4.1 Set locale

```bash
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 4.2 Add ROS 2 apt repository

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt install -y curl ca-certificates gnupg lsb-release
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update:

```bash
sudo apt update
```

### 4.3 Install ROS 2 Humble

Minimal install:

```bash
sudo apt install -y ros-humble-ros-base
```

Desktop install (includes GUI tools like RViz):

```bash
sudo apt install -y ros-humble-desktop
```

> Note: Desktop install is unnecessary if you do not need GUI tools.  
> If you need GUI apps over SSH, see **X11 Forwarding** below.

### 4.4 Setup environment

Add to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify:

```bash
ros2 --version
ros2 topic list
```

Install common build tools:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
```

If you keep encountering issues during ROS 2 installation, you can try FishROS (optional):

```bash
wget http://fishros.com/install -O fishros && . fishros
```

---

## 5. Optional

### 5.1 VPN for collaboration (Tailscale)

If you want to remotely access the Raspberry Pi, you can configure a VPN such as Tailscale:

```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
```

Login:

```bash
sudo tailscale up
```

Check status:

```bash
tailscale status
tailscale ip -4
```

### 5.2 X11 Forwarding

Use this if you want to run GUI apps over SSH.

#### 5.2.1 Install X11 utilities on the Pi

```bash
sudo apt install -y xauth x11-apps
```

#### 5.2.2 Linux

X server is usually available on Linux desktop environments.

SSH with X11 forwarding:

```bash
ssh -X user@<host>
# or (more permissive)
ssh -Y user@<host>
```

Test:

```bash
echo $DISPLAY
xclock
```

#### 5.2.3 macOS

You need an X server such as **XQuartz**.

After installing XQuartz, connect with:

```bash
ssh -Y user@<host>
```

Test:

```bash
echo $DISPLAY
xclock
```

#### 5.2.4 Windows

Recommended: **MobaXterm**.

- Start X server: `X11 -> Start X server`
- Create SSH session
- In **Advanced SSH settings**, enable **X11-forwarding**

Test:

```bash
echo $DISPLAY
xclock
```

#### 5.2.5 Debug

If X11 forwarding is not working, check:

```bash
sudo sshd -T | grep -i x11
```

You should see `x11forwarding yes`. Otherwise, in `/etc/ssh/sshd_config`, enable:

```conf
X11Forwarding yes
X11UseLocalhost yes
```

Then restart SSH:

```bash
sudo systemctl restart ssh
```
