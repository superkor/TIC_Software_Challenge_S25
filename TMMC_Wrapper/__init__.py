import numpy as np
import os
import subprocess

# include all files in this folder
from .Battery import Battery
from .Camera import Camera
from .IMU import IMU
from .Lidar import Lidar
from .Logging import Logging
from .Control import Move
from .Robot import Robot

class Constants:
    #---constants---
    CONST_speed_control = 1 #set this to 1 for full speed, 0.5 for half speed
    DEBUG = False #set to false to disable terminal printing of some functions
    is_SIM = False #to disable some functions that can not be used on the sim

    @staticmethod
    def use_hardware():
        global is_SIM
        if not is_SIM:
            # import ROS settings for working locally or with the robot (equivalent of ros_local/ros_robot in the shell)
            env_file = ".env_ros_robot"
            os.environ.update(dict([l.strip().split("=") for l in filter(lambda x: len(x.strip())>0,open(env_file).readlines())]))
            try:
                output = subprocess.check_output("ip addr show",shell=True)
                import re
                robot_id = int(re.search(r"tap[0-9]\.([0-9]+)@tap",output.decode()).groups()[0])
            except Exception as ex:
                raise Exception("VPN does not seem to be running, did you start it?:",ex)
            print("You are connected to uwbot-{:02d}".format(robot_id))
            try:
                subprocess.check_call("ping -c 1 -w 10 192.168.186.3",shell=True,stdout=subprocess.DEVNULL)
            except Exception as ex:
                raise Exception("Could not ping robot (192.168.186.3)")
            print("Robot is reachable")
            try:
                subprocess.check_call("ros2 topic echo --once /ip",shell=True,stdout=subprocess.DEVNULL)
            except Exception as ex:
                print("ros2 topic echo --once /ip failed. Proceed with caution.")
            print("ros2 topic subscription working. Everything is working as expected.")
    
__all__ = [
    "Constants",
    "Battery",
    "Camera",
    "Control",
    "IMU",
    "Lidar",
    "Logging",
    "Robot"
]