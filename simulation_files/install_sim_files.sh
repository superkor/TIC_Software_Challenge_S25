#!/usr/bin/env bash
set -e

# Move to the script's directory so that relative paths work
cd "$(dirname "$0")"

# Copy the world file to the TurtleBot3 Gazebo worlds folder
cp tic_field_v1.world /opt/ros/humble/share/turtlebot3_gazebo/worlds

# Ensure the .gazebo/models directory exists under root’s home
mkdir -p /root/.gazebo/models

# Copy all models into /root/.gazebo/models/
cp -R models/* /root/.gazebo/models/

echo "Simulation files installed successfully!"

