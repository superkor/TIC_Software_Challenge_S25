# include all files in this folder
from .Battery import Battery
from .Camera import Camera
from .Constants import Constants
from .IMU import IMU
from .Lidar import Lidar
from .Logging import Logging
from .Control import Control
from .Robot import Robot
import rclpy


control = None
robot = None

def initialize():
    global control
    global robot
    robot = Robot()
    control = Control(robot)
    
__all__ = [
    "initialize",
    "Constants",
    "Battery",
    "Camera",
    "control",
    "IMU",
    "Lidar",
    "Logging",
    "robot"
]