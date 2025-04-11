from TMMC_Wrapper import Constants, Robot, Battery, Camera, IMU, Lidar, Logging, Control
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

# Set to True for simulation mode (or hardware, as needed)
Constants.is_SIM = True
if not Constants.is_SIM:
    Constants.use_hardware()

# Set to the level of which challenge you are attempting
challengeLevel = 1

# Initialize ROS2 if not already running
if not rclpy.ok():
    rclpy.init()
robot = Robot()

if challengeLevel == 1 or challengeLevel == 2:
    Control.start_keyboard_control(robot)

# Declare your own functions here:

#----------------------------------------------------

try:
    if challengeLevel == 1:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 1
            # It is recommended you use functions for aspects of the challenge that will be resused in later challenges
            # For example, create a function that will detect if the robot is too close to a wall

    if challengeLevel == 2:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2
            
    if challengeLevel == 3:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 3

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 5
            
    if challengeLevel == 6:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 6


except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    robot.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
