#start with imports, ie: import the wrapper
from TMMC_Wrapper import Constants, Robot, Control
import rclpy
import numpy as np
import math

#start ros
if not rclpy.ok():
    rclpy.init()

Constants.is_SIM = True

if not "robot" in globals():
    robot = Robot()

if not Constants.is_SIM:
    #specify hardware api
    robot.use_hardware()

#debug messaging 
print("running main")

#start processes
Control.start_keyboard_control(robot)   #this one is just pure keyboard control

rclpy.spin_once(robot, timeout_sec=0.1)

#run the keyboard control functions
try:
    print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
    while True: 
        rclpy.spin_once(robot, timeout_sec=0.1)
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    Control.stop_keyboard_control(robot)
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

