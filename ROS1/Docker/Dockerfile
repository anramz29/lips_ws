# Use the official ROS Noetic base image
FROM osrf/ros:noetic-desktop-full

# Install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    sudo \
    wget \
    curl \
    nano \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies: rmw_fastrtps_cpp, rviz, and rosdep
RUN apt-get update && apt-get install -y \
    ros-noetic-rviz \
    ros-noetic-move-base-msgs \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install NVIDIA dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 \
    ca-certificates \
    && curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub | apt-key add - \
    && echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64 /" > /etc/apt/sources.list.d/cuda.list \
    && apt-get update && apt-get install -y --no-install-recommends \
    cuda-toolkit-11-4 \
    cuda-drivers \
    libnvidia-compute-470 \
    nvidia-container-toolkit \
    libnvidia-gl-470 \
    && rm -rf /var/lib/apt/lists/*

# Set CUDA environment variables
ENV PATH /usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/cuda/lib64:${LD_LIBRARY_PATH}

# Set up a non-root user for safe operations
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the environment to use the user and home directory
USER $USERNAME
WORKDIR /home/$USERNAME

# Source ROS Noetic setup file on container startup
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/$USERNAME/.bashrc

# Label the image as NVIDIA compatible
LABEL com.nvidia.volumes.needed="nvidia_driver"

CMD ["/bin/bash"]