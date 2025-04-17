from .Battery import Battery
from .Camera import Camera
from .IMU import IMU
from .Lidar import Lidar
from .Logging import Logging
from .Control import Control
from .Robot import Robot

control = None
battery = None 
camera = None
imu = None
lidar = None
logging = None

def initializeModules(robot):
    global control, battery, camera, imu, lidar, logging
    control = Control(robot)
    battery = Battery(robot)
    camera = Camera(robot)
    imu = IMU(robot)
    lidar = Lidar(robot)
    logging = Logging(robot)
  
__all__ = [
    "Battery",
    "Camera",
    "Control",
    "IMU",
    "Lidar",
    "Logging",
    "Robot",
    "initializeModules",
    "control",
    "battery",
    "camera",
    "imu",
    "lidar",
    "logging"
]
