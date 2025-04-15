from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO

challengeLevel = 1
Constants.is_SIM = False

if not rclpy.ok():
    rclpy.init()


if not Constants.is_SIM:
    Robot.use_hardware()

if not "robot" in globals():
    robot = Robot()

control = Control(robot)
battery = Battery(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

if challengeLevel == 1 or challengeLevel == 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)



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
