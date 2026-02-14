# Setup a Raspberry Pi with Ubuntu 22.04

> Tested on a Raspberry Pi 4 running Ubuntu 22.04 LTS (Server/Desktop).
> This tutorial is written and updated on 2026/02/14.

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
After reboot
```bash
sudo apt update
```
### 1.2 Optional setup

Set timezone:
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
If not yet done, install and enable SSH server:
```bash
sudo apt install -y openssh-server
sudo systemctl enable --now ssh
sudo systemctl status ssh --no-pager
```
## 2 Install Essential Tools
```bash
sudo apt install -y \
  git curl wget ca-certificates \
  htop tmux unzip zip \
  net-tools iputils-ping
```
## 3. Install Programming Environments
### 3.1 Python
Ubuntu 22.04 ships with Python 3.10 by default. To verify, run:
```bash
python3 --version
python3.10 --version
```
Install pip, venv and headers
```bash
sudo apt install -y python3-pip python3-venv python3.10-dev
```
Install commonly used libraries (edit as needed):
```bash
python3 -m pip install --user \
  pyserial \
  python-can
```
Install Robstride SDK
```bash
python3 -m pip install robstride
```

Install other libraries if needed.

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
Up until the tutorial's latest update, rust is not yet used by planned to be used in this project.

Install Rust via rustup
```bash
sudo apt install -y build-essential curl
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
If needed, update rust:
```bash
rustup update
```
## 4. ROS2 Installation
This project is only tested on ROS2 Humble. 

Other versions of ROS2 will also work but are not tested.

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

### 4.3 Install ROS2 Humble
For minimal install:
```bash
sudo apt install -y ros-humble-ros-base
```

For desktop install:
```bash
sudo apt install -y ros-humble-ros-base
```
**Note that desktop install is unnecessary and using GUI tools require Xforwarding which is discussed later.**

### 4.4 Setup environment
Add to ```~/.bashrc```
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Verify
```bash
ros2 --version
ros2 topic list
```
Install common build tools
```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init || true
rosdep update
```
If you kept on encountering issue during installation of ros2, try
```bash
wget http://fishros.com/install -O fishros && . fishros
```
## 5. Optional
### 5.1 Setup VPN for collaboration
If you want to remotely access Rawsberry Pi, you can try to configure a VPN on the device such as Tailscale which is used in the tutorial
```bash
curl -fsSL https://tailscale.com/install.sh | sh
sudo systemctl enable --now tailscaled
```
Login
```bash
sudo tailscale up
```
Check status
```bash
tailscale status
tailscale ip -4
```
### 5.2 X11 Forwarding
Use this if you want to run GUI apps。

#### 5.2.1 Install X11 utilities on the Pi
```bash
sudo apt install -y xauth x11-apps
```

#### 5.2.2 Linux
X server is usually pre-installed in a Linux OS. Install one if not.

SSH with Xforwarding:
```bash
ssh -X user@<host>
# or
ssh -Y user@<host>
```
Test
```bash
echo $DISPLAY
xclock
```
#### 5.2.3 macOS
You can use terminal to use X11 forwarding. However, installing a X server such as XQuartz is needed.

After you install it,
```bash
ssh -Y user@<host>
```
Test
```bash
echo $DISPLAY
xclock
```
#### 5.2.4 Windows
It is recommended to use MobaXterm.
After installing and opening MobaXterm：
- Click Session on the top left corner
- Type in ip and username
- In advanced SSH settings, check Enable X11—forwarding.

Test
```bash
echo $DISPLAY
xclock
```

#### 5.2.5 Debug
If it is not working, try following steps:
```bash
sudo sshd -T | grep -i x11
```
You should be able to see ```x11forwarding yes```. Otherwise, in ``` /etc/ssh/sshd_config ```, uncomment
```bash
X11Forwarding yes
X11UseLocalhost yes
```
Then 
```bash
sudo systemctl restart ssh
```
