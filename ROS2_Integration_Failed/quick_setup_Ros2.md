Here’s an adjusted version of your setup instructions updated to use ROS2 commands while keeping the structure consistent:

---

### Locobot Quick Set Up (ROS2)

### 1) Turn on Locobot

### 2) SSH into the Locobot

To SSH into the `locobot#` server, use the following command in your terminal:

```bash
ssh -X locobot@locobot#
```
- Replace `#` with your assigned Locobot number. If successful, the terminal will prompt you to enter your password.

### 3) Run Docker on the Remote PC

#### Once you have already created the Docker container, open a new terminal and input:

```bash
docker exec -it <container_id_or_name> /bin/bash
```

A common issue between sessions is the Docker container stopping, as indicated by this message:

```bash
Error response from daemon: container 6012bd1026c14b0d7afcf70b919033f0ec720e73a4bdf90509b68cb9f41e834a is not running
```

If this happens, start the container with the following command:

```bash
docker start <container_id_or_name>
```

Then run the `docker exec` command above.

This should be applicable for both Server and PC configurations.

### 4) Launch the Control Script in the Locobot

Make sure you still have an SSH terminal open and input the following:

```bash
ros2 launch interbotix_xslocobot_control xslocobot_control.launch.py robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true use_base_odom_tf:=true
```

### 5) On Your Terminal Where Docker is Executed

Run the following command to launch the remote visualization:

```bash
ros2 launch interbotix_xslocobot_descriptions remote_view.launch.py
```

If you encounter issue please refer to the [troubleshooting](troubleshooting.md) documentation
---
