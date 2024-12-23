# Base image
FROM osrf/ros:humble-desktop

# Set working directory for inside container
WORKDIR /workspace

# Install dependencies
RUN apt-get update && apt-get install -y \
        python3-pip \
        nano \
        vim \
        && rm -rf /var/lib/apt/lists/*
        # - "update": Updates package metadata
        # - "install": Installs specified packages
        # - "rm -rf": Remove cached package lists (package metadata) to reduce image size

# Set default command for container
CMD ["bash"]