# TurtleBot4 Simulation Environment

This repository provides a pre-built Docker image for running a ROS2 Humble-based simulation environment with TurtleBot4 support and Gazebo. The image comes pre-installed with all necessary ROS2 and TurtleBot4 packages, and it automatically sets up custom simulation files (world and models).

## Prerequisites

- **Docker**: Make sure Docker is installed on your system.
- **X Server**: An X server must be running (this is typically set up by default on most Linux distributions).
- **Pre-Built Image**: You will receive the pre-built Docker image, so you do not need to build it yourself.

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

You will receive instructions on how to load/pull the image. For example, if using a tarball, load it with:

```bash
docker load -i turtlebot4-image.tar
```

### 2. Run the Provided Wrapper Script

The `run_docker.sh` script handles all the necessary configuration for display, audio, and mounting your simulation files. To run the container, simply execute:

```bash
./run_docker.sh
```

The script will:

- Allow local connections to the X server.
- Mount your host’s X11 socket and audio devices.
- Mount your local `Documents` directory and the `simulation_files` folder.
- Automatically execute the simulation installation script (`install_sim_files.sh`).
- Drop you into an interactive bash shell inside the container.

### `run_docker.sh` Content

```bash
#!/bin/bash
# Allow local root connections to the X server (for GUI apps like Gazebo)
xhost +local:root

# Make sure you change the paths to where you want to work and where your folders are. As an example, we have added our folder in home/ideasclinic/Documents/challenge_toyota/ . 
docker run -it --rm --net=host \
  --name challenge \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/snd:/dev/snd \
  -v /home/ideasclinic/Documents:/home/ideasclinic/Documents \
  -v /home/ideasclinic/Documents/challenge_toyota/TMMC-Working/simulation_files:/simulation_files \
  -w /home/ideasclinic/Documents \
  turtlebot4-image \
  bash -c "cd /simulation_files && ./install_sim_files.sh && bash"
```
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
