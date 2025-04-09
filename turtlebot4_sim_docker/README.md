# TurtleBot4 Simulation Environment

This folder provides a pre-built Docker image for running a ROS2 Humble-based simulation environment with TurtleBot4 support and Gazebo. The image comes pre-installed with all necessary ROS2 and TurtleBot4 packages, and it automatically sets up custom simulation files (world and models).

## Prerequisites

- **Docker**: Install Docker onto system
  - Set up Docker's apt repository:
  ```
    # Add Docker's official GPG key:
  sudo apt-get update
  sudo apt-get install ca-certificates curl
  sudo install -m 0755 -d /etc/apt/keyrings
  sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
  sudo chmod a+r /etc/apt/keyrings/docker.asc
  
  # Add the repository to Apt sources:
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
    $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
    sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
  sudo apt-get update
  ```
  - Install Docker for Linux:
  ```
  sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
  ```
  - Verify successful install:
  ```
  sudo docker run hello-world
  ```

- **X Server**: An X server must be running (this is typically set up by default on most Linux distributions).
- **Pre-Built Image**: The pre-built docker image has been provided in this repository, so you do not need to build it yourself.
- **Install VS Code**
  - Update Packages:
  ```
  sudo apt update
  sudo apt install software-properties-common apt-transport-https wget
  ```
  - Import Microsoft GPG key:
  ```
  wget -q https://packages.microsoft.com/keys/microsoft.asc -O- | sudo apt-key add -
  ```
  - Enable VS Code Repository:
  ```
  sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main"
  ```
  - Install VS Code
  ```
  sudo apt install code
  ```

## Video Walkthrough
Find a walkthrough of the setup process here: [https://www.youtube.com/watch?v=OJLpkSIJko8](https://www.youtube.com/watch?v=OJLpkSIJko8).

## Directory Structure

```
.
├── simulation_files/
│   ├── tic_field_v1.world  # Custom Gazebo world file
│   ├── models/             # Folder containing the simulation models
│   └── install_sim_files.sh  # Script that installs the simulation files inside the container
└── run.sh  # Wrapper script to run the container with proper settings
```

## Running the Container

To start the simulation environment, follow these steps:

### 1. Download the Pre-Built Docker Image

The Dockerfile has been provided for you already. You can build it using the following command while within this folder (use: "cd turtlebot4_sim_docker" to navigate into this folder from your main repository):
```
cd TIC_Software_Challenge_S25/turtlebot4_sim_docker
```
```bash
sudo docker build -t turtlebot4_test:latest .
```

### 2. Run the Provided Wrapper Script

The `run_docker.sh` script handles all the necessary configuration for display, audio, and mounting your simulation files. 

You will need to update the paths within that file, open it up in VSCode and make the changes to lines 11 and 12 as specified in the comments of the file.

Once this is done, run the file using the following command:

```bash
sudo ./run_docker.sh
```

If this command fails with the message "Permissions denied", run the following commands to update the permissions of the files you want to run.

```bash
cd ../simulation_files
chmod +x install_sim_files.sh
cd -
chmod +x run_docker.sh
```

The script will:

- Allow local connections to the X server.
- Mount your host’s X11 socket and audio devices.
- Mount your local `Documents` directory and the `simulation_files` folder.
- Automatically execute the simulation installation script (`install_sim_files.sh`).
- Drop you into an interactive bash shell inside the container.

## Important Note
Do not run `run_docker.sh` in a new terminal while a container is already running. This will create a new container instance, causing ROS2 nodes to be unable to communicate with each other. Instead, use the following commands to open another terminal inside the same running container:

```bash
docker ps   # Find the Container ID or NAME
docker exec -it <ContainerID_or_Name> bash
```
## Using the Simulation

Once inside the container, you can launch the simulation with the following command:

```bash
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

- The Gazebo client (GUI) will appear on your host machine.
- The ROS2 environment is already set up, including sourcing `/opt/ros/humble/setup.bash` and setting the TurtleBot model to `burger`.

## Simulation Files Installation

The `install_sim_files.sh` script (located in the `simulation_files` folder) performs the following tasks:

- Copies `tic_field_v1.world` to the appropriate Gazebo worlds directory.
- Copies all folders from the `models/` directory into the container’s `/root/.gazebo/models/` directory.

### `install_sim_files.sh` Content

```bash
#!/usr/bin/env bash
set -e

# Change to the script's directory
cd "$(dirname "$0")"

# Copy the world file to the TurtleBot3 Gazebo worlds folder
cp tic_field_v1.world /opt/ros/humble/share/turtlebot3_gazebo/worlds

# Ensure the .gazebo/models directory exists in /root
mkdir -p /root/.gazebo/models

# Copy all models into /root/.gazebo/models/
cp -R models/* /root/.gazebo/models/

echo "Simulation files installed successfully!"
```

## Final Notes

This container runs as the root user; therefore, environment paths such as the home directory are set to `/root`. Ensure your necessary files are accessible within this environment.
