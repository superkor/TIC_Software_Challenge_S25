
from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
import threading

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 5

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra print statements, False otherwise
Debug = True

# Initialization    
if not "robot" in globals():
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug)
    
control = Control(robot)
camera = Camera(robot)
imu = IMU(robot)
logging = Logging(robot)
lidar = Lidar(robot)

DEFAULT_APRIL_TAG = {
            "id": -1,
            "bearing": 99999,
            "range": 999999,
            "elevation": 9999999
        }

current_april_tag_info = DEFAULT_APRIL_TAG
# Precomputed turn angles at the 
# detect april tag
# WE DONT KNOW IF THE APRIL TAG NUMBERS ARE CORREECT
ignored_april_tags = [2, 8]
april_to_turn = {
    1: 90,
    3: -305,
    4: 90,
    5: 245,
    6: -270,
    7: -270,
    9: 45,
}

# april_to_err_offset = {
#     1: 0,
#     3: -75,
#     4: 0,
#     5: 0,
#     6: 50,
#     7: 50,
#     9: 0,
# }

def handle_april_tag():
    """
    This is ran on a thread
    """

    global current_april_tag_info
    while True:
        #This is blocking 
        # print("Thread: getting image")
        # camera_image = camera.checkImage()
        cv_im = camera.rosImg_to_cv2()
        # camera.checkImageRelease()

        # Get april tage pose
        if (cv_im is not None):
            tags: list [tuple [int, float, float, float]] = camera.estimate_apriltag_pose(cv_im)
                
        # get april tag and make a control decision
        current_april_tag_info = DEFAULT_APRIL_TAG
                
        for tag in tags:
            print(f"tag: {tag}")
            id: int = tag[0]
            range: float = tag[1]
            bearing: float = tag[2]
            elevation: float = tag[3]
                    
            if bearing < current_april_tag_info["bearing"]:
                #get the closest april tag from the camera baesd on bearing
                current_april_tag_info["id"] = id
                current_april_tag_info["range"] = range
                current_april_tag_info["bearing"] = bearing
                current_april_tag_info["elevation"] = elevation

def get_april_tag_and_reset() -> dict:
    """
    This returns the april tag detected by the camera and resets the april tag
    """

    global current_april_tag_info
    res = current_april_tag_info
    current_april_tag_info = DEFAULT_APRIL_TAG
    return res


if challengeLevel <= 2:
    control.start_keyboard_control()
    rclpy.spin_once(robot, timeout_sec=0.1)

try:
    if challengeLevel == 0:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Challenge 0 is pure keyboard control, you do not need to change this it is just for your own testing

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
            # Write your solution here for challenge level 3 (or 3.5)

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        thread = threading.Thread(target=handle_april_tag)
        thread.start()

        while rclpy.ok():
            # Main control loop

            current_april_tag = get_april_tag_and_reset()

            if current_april_tag["id"] == -1:
                continue #don't perform on the same april tag (before a new camera detects an april tag)
            init_imu_msg = imu.checkImu()

            print("Check imu")
            control.rotate(current_april_tag["bearing"], 1)

            while True:
                print("In while")
                #wait until the robot has rotated to the desired bearing
                imu_msg = imu.checkImu() #waits until a message from the IMU is received
                cur_angle = imu.rotation_angle(imu_msg.orientation)

                if imu.has_rotation_occurred(init_imu_msg.orientation, imu_msg.orientation, current_april_tag["bearing"]):
                    break
            
            # velocity 
            # We assume that the velocity is a known quatity? yes
            velocity = 0.2
            
            # straight_distance = hypothenuse*cos(\theta)*cos(\phi)
            distance_to_travel = np.cos(current_april_tag["elevation"]) * current_april_tag["range"] * np.cos(current_april_tag["bearing"])
            print(f"distance to travel: {distance_to_travel}")

            # v = d/t
            time_to_travel = distance_to_travel / velocity   

            control.set_cmd_vel(velocity, 0, 1) #only go forward with `velocity` for 1 second (with 0 angular vel)
            # Compute distance we travelled for at a fixed velocity
            distance_travelled_this_step = velocity*(time_to_travel - 1) #only went 1 second; subtract from total time to travel

            # Compute the new distance to travel
            distance_to_travel = distance_to_travel - distance_travelled_this_step

except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    control.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()

def grass():
    print("Touch grass")