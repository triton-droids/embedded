
## Table of Contents
- [Dockerfiles](#dockerfiles)
- [Using Docker Images](#using-docker-images)
- - [MacOS](#macos)
- - [Windows](#windows)
- - [Linux](#linux)
- [Mounting Directories[(#mounting-directories)
- [X11 GUI Forwarding](#x11-gui-forwarding)


## Dockerfiles
All Dockerfiles are available within /Dockerfiles and as of 1/15/25 contains:
```txt
foxy-gpu-20.04
iron-gpu-22.04
humble-gpu-20.04
humble-gpu-22.04
jazzy-gpu-24.04
```

Note: humble-gpu-20.04 is not available currently as there wasn't a prebuilt binary package for it. It is possible though to install it from source if our organization needs it.

If you want to learn Docker and Dockerfiles, please reference their documentation online.

## Using Docker Images
As of 1/15/25, there are only GPU images and these instructions are explictly for them.

### MacOS
I have no knowledge over the MacOS and Apple's ecosystem. If you want to use MacOS to develop, this is a great opportunity for you. If you do succeed, please update this README with the instructions for all future MacOS users.

### Windows
If you are on Windows, you will have to use WSL2 as GPU support is only available on Docker Desktop. As such, there is nothing necessary for you to set up as long as you have Docker Desktop installed. Simply launch your WSL2 and utilize the Docker CLI to launch a container using an image.

### Linux
If you are on Linux (only tested on Arch Linux), you will need to have the **NVIDIA Container Toolkit** installed. This is assuming that you have docker installed. If you don't, you can follow the instructions in this order. It's important you get used to reading documentation.

1. [NVIDIA GPU Drivers](https://docs.nvidia.com/datacenter/tesla/driver-installation-guide/index.html)
2. [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installation)

You can then run via CLI:
```bash
docker run --gpus all -it <image_name> /bin/bash
```

### Mounting Directories
The containers provides all the necessary dependencies to develop code for Triton Droids. As such, it's in our best interests to isolate our code repository from our image as we'll have to constantly update our image and is a bad practice.

The best practice is to mount our folders to wherever we need it within our container. This'll allow us to work in the container with the folders we need and also save them. This means that all of our changes will be saved. 

The standard usage for mounting a folder is as follows:
```bash
docker run -it 
           -v /path/to/local:/path/to/remote \
           <image_name> /bin/bash
```

### X11 GUI Forwarding
We need to forward our GUI to be displayed on our host machine. In simpler terms, since we are working inside a container (an isolated environment), all the graphical instances will not be displayed unless we set it up. I am still figuring this out and will update when needed.
