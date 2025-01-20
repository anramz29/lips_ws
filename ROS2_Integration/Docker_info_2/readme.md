# Build the Docker Image
Build the Docker image with a meaningful name:

```bash
docker build -t <image_name> .
```

# Run the Docker Container with a Mounted Volume
Run the container and mount a folder from your host system to the container. This allows you to persist data (e.g., downloaded .sh scripts) between container restarts.

```bash
docker run -it --name <container_name> \
  -v $(pwd):/home/rosuser/workspace \
  --privileged \
  <image_name>
```

Explanation of the Command:

-v $(pwd):/home/rosuser/workspace:
Mounts your current directory ($(pwd)) into /home/rosuser/workspace inside the container.

Any changes in this folder will persist on your host system.

--privileged:
Grants the container extended privileges (useful if the .sh script requires root-like access).

