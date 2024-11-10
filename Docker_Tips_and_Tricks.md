



## Docker Troubleshooting and basic Commands 

Below are some useful Docker commands and tips to help you manage Docker containers, images, and other resources effectively.

### 1) Start a Stopped Container



If you have a container that has been stopped but not removed, you can **restart** it using the following command:


```bash
docker start <container_id_or_name>
```

### 2) Stop a Running Container
To stop a running container, you can use:

```bash
docker stop <container_id_or_name>
```

### 3) Remove a Stopped Container
To remove a stopped container:

```bash
docker rm <container_id_or_name>
```

If you want to remove all stopped containers at once:

```bash
docker container prune
```

### 4) List All Running Containers
To list only the currently running containers:

```bash
docker ps
```

To list all containers, including those that are stopped:

```bash
docker ps -a
```

### 5) View Logs of a Container
To view the logs of a container (useful for debugging):

```bash
docker logs <container_id_or_name>
```

To follow the logs in real-time:

```bash
docker logs -f <container_id_or_name>
```

### 6) Execute a Command Inside a Running Container
To run a command or open a shell inside a running container:

```bash
docker exec -it <container_id_or_name> /bin/bash
```

For containers that don’t have Bash, try `sh` instead:

```bash
docker exec -it <container_id_or_name> /bin/sh
```

## Ros Specific Docker Resources

1) Articulated Robotics is a great channel that provides some of the best info on how to use docker. Below is his playlist covering the usage of docker for ROS

- https://www.youtube.com/playlist?list=PLunhqkrRNRhaqt0UfFxxC_oj7jscss2qe

