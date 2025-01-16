FROM --platform=linux/arm64 osrf/ros:noetic-desktop-full

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Copy the script as root and set permissions
COPY /xslocobot_remote_install.sh /home/ros/xslocobot_remote_install.sh
RUN chmod +x /home/ros/xslocobot_remote_install.sh
RUN apt-get update \ 
    && apt-get install -y \ 
    nano \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# Install debconf-utils to preconfigure keyboard settings
RUN apt-get update && apt-get install -y \
  debconf-utils \
  locales

# Preconfigure keyboard settings
RUN echo "keyboard-configuration keyboard-configuration/layoutcode select us" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/xkb-keymap select us" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/model select Generic 105-key PC (intl)" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/layout select English (US)" | debconf-set-selections && \
    echo "keyboard-configuration keyboard-configuration/variant select English (US)" | debconf-set-selections

# Set the locale
RUN locale-gen en_US.UTF-8 && \
    update-locale LANG=en_US.UTF-8

# create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Setup sudo for the new user
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(\root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \ 
    && rm -rf /var/lib/apt/lists/* 

# Create the interbotix_ws directory with the appropriate permissions
RUN mkdir -p /interbotix_ws && chown ros:ros /interbotix_ws

# Create .bashrc file and give permissions to the ros user
RUN touch /home/ros/.bashrc && chown ros:ros /home/ros/.bashrc

# Change ownership of the script to ros
COPY /xslocobot_remote_install.sh /home/ros/xslocobot_remote_install.sh
RUN chown ros:ros /home/ros/xslocobot_remote_install.sh

# Run the script as the ros user
RUN /home/ros/xslocobot_remote_install.sh -d noetic -b create3 -r locobot4 -i 10.253.20.224 -n -p /interbotix_ws

# Source the .bashrc file to apply changes
RUN echo "source /home/ros/.bashrc" >> /home/ros/.bashrc

CMD ["/bin/bash"]
