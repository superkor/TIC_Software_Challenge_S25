# Use the official ROS2 Humble image as a base
FROM ros:humble

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Update apt and install required packages including python3-pip for pip3 commands
RUN apt-get update && apt-get install -y \
    git \
    net-tools \
    iputils-ping \
    python3-pip \
    ros-humble-turtlebot4-desktop \
    ros-humble-turtlebot3* \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-irobot-create-msgs \
    openvpn \
 && rm -rf /var/lib/apt/lists/*

# Install additional Python packages using pip3
RUN pip3 install eigenpy pynput ultralytics apriltag

# Create a workspace directory and clone the TurtleBot4 repository if not already present
RUN mkdir -p /root/robohub && \
    if [ ! -d "/root/robohub/turtlebot4" ]; then \
       git clone https://git.uwaterloo.ca/robohub/turtlebot4.git /root/robohub/turtlebot4; \
    fi

# Copy Fast DDS configuration file
RUN cp /root/robohub/turtlebot4/configs/.fastdds.xml /root/

# Append environment configuration to the root user's .bashrc if not already present.
RUN grep -qxF "source /opt/ros/humble/setup.bash" /root/.bashrc || \
    echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    grep -qxF "export TURTLEBOT3_MODEL=waffle_pi" /root/.bashrc || \
    echo "export TURTLEBOT3_MODEL=waffle_pi" >> /root/.bashrc

# Set the default command to run bash so that when you launch the container, you get a shell with the proper environment
CMD [ "bash" ]
