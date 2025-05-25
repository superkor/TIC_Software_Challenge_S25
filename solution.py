
from TMMC_Wrapper import *
import rclpy
import numpy as np
import math
import time
from ultralytics import YOLO
import threading

# Variable for controlling which level of the challenge to test -- set to 0 for pure keyboard control
challengeLevel = 2

# Set to True if you want to run the simulation, False if you want to run on the real robot
is_SIM = True

# Set to True if you want to run in debug mode with extra b statements, False otherwise
Debug = False


# Initialization    
if not "robot" in globals():
    #todo change back to 0.15
    robot = Robot(IS_SIM=is_SIM, DEBUG=Debug, CONST_speed_control=0.15)
    
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

    #This is blocking 
    print("Thread: getting image")
    # camera_image = camera.checkImage()
    cv_im = camera.rosImg_to_cv2()
    # camera.checkImageRelease()

    # Get april tage pose
    if (cv_im is not None):
        tags: list [tuple [int, float, float, float]] = camera.estimate_apriltag_pose(cv_im)
            
    # get april tag and make a control decision
    current_april_tag_info = DEFAULT_APRIL_TAG

    print("Len of tags: ", len(tags))
            
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
            current_april_tag_info["bearing"] = 2*np.pi*(bearing/360)
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
            lidar_scan = lidar.checkScan()
            
            min_dist, min_dist_angle = lidar.detect_obstacle_in_cone(lidar_scan, 0.3, 0, 15)
            
            print(min_dist)
            
            if (min_dist != -1 or min_dist_angle != -1):
                control.stop_keyboard_input()
                control.set_cmd_vel(-0.1, 0, 0.5)
                control.start_keyboard_input()
        
    if challengeLevel == 2:
        ignore_stop_signs = False
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 2

            cv_im = camera.rosImg_to_cv2()
            camera.checkImageRelease()
            has_stop, x1, x2, y1, y2 = camera.ML_predict_stop_sign(cv_im)
            handle_april_tag()
            
            if (current_april_tag_info["id"] == 2 and has_stop or ignore_stop_signs):
                tag_2_offset = 3
                rotation_offset_tag_2 = 20
                distance_to_stop_sign = np.cos(current_april_tag_info["bearing"]) * current_april_tag_info["range"] + tag_2_offset
               
                # Drive forward until stop sign is reached
                control.stop_keyboard_control()
                print("Desired bearing: ", current_april_tag_info["bearing"])
                print()
                control.rotate(np.abs(current_april_tag_info["bearing"]*180/np.pi - rotation_offset_tag_2), -1 if np.abs(current_april_tag_info["bearing"]*180/np.pi - rotation_offset_tag_2) > 0 else 1)
                control.set_cmd_vel(0.10, 0.0, distance_to_stop_sign)
                
                # Stop, then prroceed
                control.set_cmd_vel(0.0, 0.0, 3.0)
                control.start_keyboard_control()
                ignore_stop_signs = True 
            elif (current_april_tag_info["id"] == 3 and has_stop or ignore_stop_signs):
                tag_3_offset = 0
                distance_to_stop_sign = np.cos(current_april_tag_info["bearing"]) * current_april_tag_info["range"] + tag_3_offset
                # Drive forward until stop sign is reached
                control.stop_keyboard_control()
                print("Desired bearing 2: ", current_april_tag_info["bearing"])
                control.rotate(np.abs(current_april_tag_info["bearing"])*180/np.pi, -1 if current_april_tag_info["bearing"] > 0 else 1)
                control.set_cmd_vel(0.10, 0.0, distance_to_stop_sign*3)
                
                # Stop, then proceed
                control.set_cmd_vel(0.0, 0.0, 3.0)
                control.set_cmd_vel(0.15, 0.0, 4.0)
                control.start_keyboard_control()
                ignore_stop_signs = True
            continue
        
        
            # todo
            cv_im = robot.rosImg_to_cv2()
            stop_sign = has_stop
            if stop_sign is True and stop_sign_prev is False:
                robot.send_cmd_vel(0.0, 0.0)
                time.sleep(2)
                stop_sign_prev = True
                print("Stopped because of sign")
                while(stop_sign is True):
                    robot.send_cmd_vel(0.25, 0.0)
                    cv_im = robot.rosImg_to_cv2()
                    has_stop, x1, x2, y1, y2 = robot.ML_predict_stop_sign(model, cv_im)
                    stop_sign = has_stop
            else:
                stop_sign_prev = False
        
        
            
    if challengeLevel == 3:
        while rclpy.ok():
            # 1. Check camera for April tags
            cv_camera_image = camera.rosImg_to_cv2()
            tags = camera.estimate_apriltag_pose(cv_camera_image)

            # 2. Find angle to April tag
            if tags:
                # tags is a list of (tag_id, range, bearing, elevation)
                tag_id, range_to_tag, bearing, elevation = tags[0]
                # bearing is the angle from the camera’s forward axis to the tag, in degrees

                # 3. Turn robot by angle to April tag (bearing)
                # The rotate function expects degrees and a direction (1 or -1)
                direction = 1 if bearing > 0 else -1
                control.rotate(abs(bearing), direction)

                # 4. Once robot is aligned to april tag, calculate distance to go forward
                # range_to_tag is the straight-line distance to the tag

                # 5. Move forward in increments (e.g., 0.1 meters at a time)
                increment = 0.1
                num_steps = int(range_to_tag // increment)
                for _ in range(num_steps):
                    control.set_cmd_vel(increment, 0, 1)  # Move forward by increment (meters) for 1 second
                    # You may want to add a delay here if needed (e.g., time.sleep(1))

                # Move any remaining distance
                remaining = range_to_tag - (num_steps * increment)
                if remaining > 0:
                    control.set_cmd_vel(remaining, 0, 1)
            else:
                print("No AprilTag detected.")

    if challengeLevel == 4:
        while rclpy.ok():
            rclpy.spin_once(robot, timeout_sec=0.1)
            time.sleep(0.1)
            # Write your solution here for challenge level 4

    if challengeLevel == 5:
        while rclpy.ok():
            handle_april_tag()
            rclpy.spin_once(robot, timeout_sec=0.1)
            # Main control loop

            current_april_tag = get_april_tag_and_reset()

            if current_april_tag["id"] == -1:
                continue #don't perform on the same april tag (before a new camera detects an april tag)
            init_imu_msg = imu.checkImu()

            control.rotate(current_april_tag["bearing"], 1)

            while True:
                #wait until the robot has rotated to the desired bearing
                imu_msg = imu.checkImu() #waits until a message from the IMU is received
                cur_angle = imu.rotation_angle(imu_msg.orientation)

                # print("Current april tag: ", current_april_tag["bearing"])
                # print("init imu msg: ", init_imu_msg.orientation)
                # print("imu msg: ", imu_msg.orientation)

                if imu.has_rotation_occurred(init_imu_msg.orientation, imu_msg.orientation, current_april_tag["bearing"]):
                    break

            print("Finished turning")
            
            # velocity 
            # We assume that the velocity is a known quatity? yes
            velocity = 0.2
            
            # straight_distance = hypothenuse*cos(\theta)*cos(\phi)
            distance_to_travel = np.cos(current_april_tag["elevation"]) * current_april_tag["range"] * np.cos(current_april_tag["bearing"])
            # print(f"distance to travel: {distance_to_travel}")

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
