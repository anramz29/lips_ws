### Locobot Quick Set Up

### 1) Turn on Locobot

### 2) SSH into the locobot

To SSH into the `locobot#` server, use the following command in your terminal:

```bash
ssh -X locobot@locobot#
```
- Switch "#" with your given locobot #, if sucessful the terminal will prompt you to give your password

### 3) Now run docker on remote PC 

#### Once you have already created the docker container open a new termianl and input:

```bash
docker exec -it <container_id_or_name> /bin/bash
```

A common issue seen between sessions is that the docker container stops indicated by this message: 

```bash
Error response from daemon: container 6012bd1026c14b0d7afcf70b919033f0ec720e73a4bdf90509b68cb9f41e834a is not running
```

If this happens you will start the container with this command:

```bash
docker start <container_id_or_name>
```

Then run the docker exec command above



This should be applicable for both Server and PC configurations

### 4) Lanuch the Control Script in the Locobot, make sure you still have SSH'ed terminal Open and input

```bash
roslaunch interbotix_xslocobot_control xslocobot_control.launch robot_model:=locobot_wx250s use_base:=true use_camera:=true use_lidar:=true
```

### 5) Now on your terminal where you have executed docker 

```bash
roslaunch interbotix_xslocobot_descriptions remote_view.launch
```

