## SSH into `lipsserver1`

To SSH into the `lipsserver1` server, use the following command in your terminal:

```bash
ssh -X username@lipsserver1
```
- Switch "username" with your given username, if sucessful the terminal will prompt you to give your password

## Setting up Dockerfile

In this repository we have a directory called Ros_Docker, This contains everything we need to create a dockerfile. To start download Ros_Docker then in a new terminal

### 1) Naviagate to the directory containing the dockerfile, usually this means... 

```bash
cd Ros_Docker
```

### 2) build a docker image, replace <image_name> with your desired image name

```bash
docker build -it <image_name> 
```

### 3) Create a Container from the Image

#### The first time you create a container run this script:
```bash 
docker run -it \
  --net=host \
  -e DISPLAY \
  -e XAUTHORITY \
  -v $XAUTHORITY:$XAUTHORITY \
  <image_name>
```

#### Once you have already created the docker container use:

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

### 4) To verify that you are in the dockerfile and everything works you shoud see the bash prompt

```bash
root@lipsserver1:/# 
```

### Congrats you just set up the container on the Server! 


## Vs Code 



### SSH Extension
- We use Microsoft's VSCode and its Remote - SSH extension in tandem with a devlopment container for simple remote development on the LoCoBot.

- In VSCode, press F1 and run the Remote-SSH: Open SSH Host... command and select the Add New SSH Host option. Enter the same locobot@locobot(1,2,3,4) combination you used when opening the SSH connection between your remote computer and the LoCoBot like ssh locobot@locobot(1,2,3,4) If prompted, enter the password locobot

- Once connected, use File > Open Folder, and select the directory you wish to operate in, i.e. the ~/interbotix_ws directory if using the ROS Interface.

#### **Important Note about Display Forwarding**:
- As of the time of writing, configuring display forwarding through the Remote-SSH extension can be challenging. To bypass this issue, make sure you run display forwarding programs directly from the terminal (SSH) on the LoCoBot, not from within VSCode.
---

## **Using Docker in VSCode**

If you have a Docker container already running on the LoCoBot, you can use the Docker extension in VSCode to interact with it.

#### **Steps to Use Docker Extension in VSCode**:

1. **Install the Docker Extension** (if not already installed):
   - Open **VSCode** and go to the **Extensions** view (click the Extensions icon in the Activity Bar on the side of the window).
   - Search for "Docker" and click **Install** on the extension provided by Microsoft.

2. **Connect to Docker**:
   - Once the Docker extension is installed, you should see a **Docker** icon in the left sidebar of VSCode.
   - Click on the **Docker** icon to open the Docker view, which will display your containers, images, volumes, and networks.

3. **Run or Attach to a Running Container**:
   - In the **Docker** view, you should see a list of your **running containers**.
   - Right-click on the container you want to work with, and you can either:
     - **Attach to the Container**: This opens a terminal inside the container so you can run commands directly inside it.
     - **Open in VSCode**: If your container has a development environment set up (e.g., with a mounted workspace), you can open the project inside VSCode from within the container.

4. **Run a Command in the Container**:
   - If you just want to run a command in the container, right-click on the container name and select **Attach Shell**. This opens an interactive shell session in the container, where you can run any commands.

#### **Example Workflow with Docker**:
- If you're working in the `~/interbotix_ws` directory, you can mount this folder into your container and use VSCode to edit files while the container is running.

---