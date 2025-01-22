## Setting up Dockerfile for remove devlopment

In this repository we have a directory called Docker info, This contains everything we need to create a dockerfile. To start download Ros_Docker then in a new terminal

### 1) Naviagate to the directory containing the dockerfile, usually this means... 

```bash
cd Ros_Docker
```

### 2) build a docker image, replace <image_name> with your desired image name

```bash
docker build \
  --build-arg ROBOT_TYPE=<locobot_hostname> \
  --build-arg ROBOT_IP=<locobot_ip> \
  -t <image_name> .
```

### 3) Create a Container from the Image

#### The first time you create a container run this script:
```bash 
xhost +
```
then..
```
docker run -it \
  --net=host \
  -e DISPLAY \
  -e XAUTHORITY \
  -v $XAUTHORITY:$XAUTHORITY \
  --name <container_name> \
  <image_name>
```
#### Once you have already created the docker container use:

```bash
xhost + && docker exec -it <container_name> <command>
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

### 4) To verify that you are in the dockerfile and everything works you shoud see the bash prompt

```bash
root@<computer_hostname>:/# 
```

### Congrats you just set up the container on the Server! 
