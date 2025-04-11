#start with imports, ie: import the wrapper
from TMMC_Wrapper import Robot, Control
import rclpy
import numpy as np
import math

#start ros
if not rclpy.ok():
    rclpy.init()

Robot.initialize(is_SIM=True) #initialize the robot

if not Robot.is_SIM:
    #specify hardware api
    Robot.use_hardware()

#debug messaging 
print("running main")

#start processes
Control.start_keyboard_control()   #this one is just pure keyboard control

rclpy.spin_once(Robot, timeout_sec=0.1)

#run the keyboard control functions
try:
    print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
    while True: 
        rclpy.spin_once(Robot, timeout_sec=0.1)
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    Control.stop_keyboard_control()
    Robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

