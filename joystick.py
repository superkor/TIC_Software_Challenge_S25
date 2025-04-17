#start with imports, ie: import the wrapper
from TMMC_Wrapper import *
import rclpy

#start ros
if not rclpy.ok():
    rclpy.init()

if not "robot" in globals():
    robot = Robot(IS_SIM = False)

control = Control(robot)
battery = Battery(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

#debug messaging 
print("running main")

#start processes
control.start_keyboard_control()   #this one is just pure keyboard control

rclpy.spin_once(robot, timeout_sec=0.1)

#run the keyboard control functions
try:
    print("Listening for keyboard events. Press keys to test, Ctrl C to exit")
    while True: 
        rclpy.spin_once(robot, timeout_sec=0.1)
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
    control.stop_keyboard_control()
finally:
    #when exiting program, run the kill processes
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
