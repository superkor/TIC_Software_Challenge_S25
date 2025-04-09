import TMMC_Wrapper
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO  # For ML-based stop sign detection

# Initialize ROS 2 if not already running
if not rclpy.ok():
    rclpy.init()

# Set simulation mode (or hardware, as needed)
TMMC_Wrapper.is_SIM = False
if not TMMC_Wrapper.is_SIM:
    TMMC_Wrapper.use_hardware()

# Create the robot instance if not already defined
if "robot" not in globals():
    robot = TMMC_Wrapper.Robot()

# Start keyboard control (this launches a separate listener thread)
robot.start_keyboard_control()

try:
    while rclpy.ok():
        rclpy.spin_once(robot, timeout_sec=0.1)
        time.sleep
        
except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    robot.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
