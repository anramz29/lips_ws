# Locobot ROS Docker Setup Guide

This guide will help you set up a Docker environment for working with Locobots using ROS.

## Prerequisites

- Docker installed on your system
- Basic knowledge of Docker and ROS
- Access to a Locobot

## Setup Instructions

### 1. Build the Docker Image

Build the Docker image with a meaningful name:

```bash
docker build -t locobot_image .
```

**Important for non-Linux systems (e.g., macOS with Apple Silicon):**
If you're using a non-Linux system with Docker, specify the platform explicitly:

```bash
docker build --platform linux/amd64 -t locobot_image .
```

### 2. Network Configuration Requirements

#### Critical: Host File Configuration

Proper network configuration through the `/etc/hosts` file is **essential** for this Docker container to function correctly. Without these configurations, communication between ROS nodes may fail and services might be unreachable.

##### Why This Matters
- Enables proper hostname resolution between the container and host
- Facilitates ROS node discovery across the network
- Prevents communication errors with ROS master
- Ensures sensor data flows correctly through the system

##### Required Configuration

1. Edit your host machine's `/etc/hosts` file:
   ```bash
   sudo nano /etc/hosts
   ```

2. Add the following entries:
   ```
   127.0.0.1       localhost
   127.0.1.1       <your_hostname>
   # Container hostnames
   172.17.0.2      locobot-container
   # Add any additional networked devices here
   192.168.1.X     robot-base
   ```

3. Save the file and verify connectivity:
   ```bash
   ping locobot-container
   ```

> ⚠️ **WARNING**: Skipping this step will result in ROS communication failures and prevent the system from operating correctly.

### 3. Run the Docker Container with a Mounted Volume

Run the container and mount a folder from your host system to the container. This allows you to persist data between container restarts:

```bash
docker run -it --name locobot_container \
  --net=host \
  -v $(pwd):/home/rosuser/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --privileged \
  locobot_image
```

#### Explanation of the Command:

- `--net=host`: Shares the host's network stack with the container.
- `-v $(pwd):/home/rosuser/workspace`: Mounts your current directory into the container.
- `-v /tmp/.X11-unix:/tmp/.X11-unix`: Mounts the host's X11 socket for GUI applications.
- `-e DISPLAY=$DISPLAY`: Sets the DISPLAY environment variable for GUI applications.
- `--privileged`: Grants the container extended privileges for hardware access.

### 4. Download Locobot Remote Script

Once you are in the container, download the remote setup for the Locobots:

1. Follow the steps described in [Trossen Robotics Documentation](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/software_setup.html#remote-install)

2. Ensure that the ROS middleware configuration is correct as specified in the [RMW Configuration Guide](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/rmw_configuration.html#remote-computer)

### 5. Switch Between Locobots

A `switch_locobots.sh` script has been provided to easily switch between different Locobot units. Simply run:

```bash
switch_robot locobot<robot_num>
```

Where `<robot_num>` is the number of the Locobot you want to connect to.

## Troubleshooting Network Issues

If you encounter communication errors like `unable to contact ROS master` or `failed to connect to robot`, verify your host file configuration first as it's the most common source of issues.

## Additional Resources

- [Official ROS Documentation](http://wiki.ros.org/)
- [Locobot Documentation](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/)
