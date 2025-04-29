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
    ros-noetic-move-base-msgs
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

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

CMD ["/bin/bash"]
