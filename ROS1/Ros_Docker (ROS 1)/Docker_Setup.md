# Build the Docker Image
Build the Docker image with a meaningful name:

```bash
docker build -t <image_name> .
```

# Run the Docker Container with a Mounted Volume
Run the container and mount a folder from your host system to the container. This allows you to persist data (e.g., downloaded .sh scripts) between container restarts.

```bash
docker run -it
  --net=host \
  -v $(pwd):/home/rosuser/workspace \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  --privileged \
  --name <container_name> \
  <image_name>
```

Explanation of the Command:

```bash
--net=host:
```

- Shares the host's network stack with the container.

```bash
-v $(pwd):/home/rosuser/workspace:
```
- Mounts your current directory ($(pwd)) into /home/rosuser/workspace inside the container.

```bash
-v /tmp/.X11-unix:/tmp/.X11-unix:
```

- Mounts the host's X11 socket into the container.

```bash
-e DISPLAY=$DISPLAY: 
```
Sets the DISPLAY environment variable in the container to use the host's display.


Any changes in this folder will persist on your host system.
```bash
--privileged:
```
Grants the container extended privileges (useful if the .sh script requires root-like access).

# Download locobot_remote script

Once you are in the container download the remote setup for the locobots and follow steps described in this [link](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/ros_interface/ros2/software_setup.html#remote-install)

Finally ensure that the ROS middleware configuration is correct as seen in this [link](https://docs.trossenrobotics.com/interbotix_xslocobots_docs/getting_started/rmw_configuration.html#remote-computer)