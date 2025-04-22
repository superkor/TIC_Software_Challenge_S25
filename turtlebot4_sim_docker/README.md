# TurtleBot4 Programming Environment

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
- **X Server**: An X server must be running (this is typically set up by default on most Linux distributions).
- **Pre-Built Image**: The pre-built docker image has been provided in this repository, so ensure that this repository has already been cloned onto your VM.

## Running the Container

To start the simulation environment, follow these steps:

### 1. Set up the Docker

The Dockerfile has been provided for you already. You can run it through vs Code following these steps:

- Install Dev Containers within VSCode extensions
  - Find it in the extensions section on the left of your VSCode

- Go into terminal and type:
  ```bash
  sudo usermod -aG docker $USER
  ```
- Restart the VM by clicking player in the top left then power.

- Once the VM has restarted open the terminal and type groups making sure docker is there:
  ```bash
  groups
  ```

- Then in the same terminal type:
  ```bash
  xhost +local:root
  ```
### 2. Run the Docker through vs code

- Open the repository folder in VSCode

- Press ctrl + shift + p and type in the search bar "Dev Containers: Rebuild and Reopen in Container: 

- Now your code and terminal commands within vs code will run through the docker.

- If you ever want to exit vs code from the docker type ctrl + shift + p and type in the search bar "Dev Containers: Reopen Folder Locally"

## Running the Simulation

Once the sim files are installed, you can launch the simulation with the following commands. Navigate to the `simulation_files` folder by doing:

```bash
cd simulation_files
```
Run the simulation file with:

```bash
ros2 launch turtlebot_tic_world.launch.py
```
- The Gazebo client (GUI) will appear on your host machine.
- The ROS2 environment is already set up, including sourcing `/opt/ros/humble/setup.bash` and setting the TurtleBot model to `waffle_pi`.

## Connecting to a physical TurtleBot

To run the script to connect to the turtlebot, make sure you are inside the docker container as described in steps 1 and 2. Then navigate to the robohub folder with:

```bash
cd robohub/rendezvous_vpn
```

Then connect to the robot by running the following script, replacing the <NUMBER_OF_TURTLEBOT> with the number on the sticker of the turtlebot you wish to use:
```bash
sudo ./vpn.sh <NUMBER_OF_TURTLEBOT>
```

Follow the instructions in the command prompt, it will ask you to enter your UWaterloo WatID and Password. You will know it was successful once you see:
```
Tunnel ready
Stopping this process will terminate the VPN
Don't forget to restart the ros2 daemon if necessary
```

## Controlling the TurtleBot with Code
To control your turtlebot in either the simulation or using a physical robot with a python file, simply open up the docker container command line (see the note above if you are doing this with the sim) and enter:

```bash
python3 name_of_file.py
```

### Important: All Python files provided will have a line "TMMC_Wrapper.is_sim = True". If you wish to connect to a physical turtlebot, you must first set this variable to False.

To test for the first time, we have provided you with solution-joystick.py in root of this repository. Try it out! If it runs successfully, you should be able to control your turtlebot using the WASD keys on your keyboard.
