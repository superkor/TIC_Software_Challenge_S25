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

# Load the pre-trained stop sign detection model (adjust the path as needed)
stop_sign_model = YOLO('yolov8n.pt')

no_stop_detection = True
stop_wait_done = True
STOP_WAIT_TIME = 3
stop_trigger_timer = 0
direction = 1


def approach_april_tag(tag_id, desired_distance, forward_speed=1.0,stop_sign_model=None, STOP_WAIT_TIME=3.0):
    
    img = robot.rosImg_to_cv2()
    detections = robot.detect_april_tag_from_img(img)

    if tag_id in detections:
        distance, angles = detections[tag_id]

        if distance > desired_distance:
            robot.send_cmd_vel(forward_speed, 0.0)
        else:
            robot.send_cmd_vel(0.0, 0.0)
    else:
        robot.send_cmd_vel(0.0,0.0)

def rotate_until_april_tag(tag_id, rotation_speed=0.75, direction=1):
    while rclpy.ok():
        robot.send_cmd_vel(0.0, direction * rotation_speed)
        img = robot.rosImg_to_cv2()
        detections = robot.detect_april_tag_from_img(img)
        if tag_id in detections:
            distance, angles = detections[tag_id]
            if(angles[2] < 1 and angles[2] > -1):
                robot.send_cmd_vel(0.0,0.0)
                break
        rclpy.spin_once(robot, timeout_sec=0.1)
        time.sleep(0.01)


#level 1
def no_wall_contact():
    global direction
    scan = robot.checkScan()
        
    # --- Check for front obstacles and rear obstacles---
    fwd_distance, fwd_angle = robot.detect_obstacle_in_cone(scan, 0.3, 0, 45)

        
    if fwd_distance != -1:
        robot.stop_keyboard_input()
        if fwd_angle > 2:
            robot.rotate(fwd_angle, 1)
            direction = -1
        elif fwd_angle < 2:
            robot.rotate(abs(fwd_angle), -1)
            direction = 1
                
        scan = robot.checkScan()
        back_distance, back_angle = robot.detect_obstacle_in_cone(scan, 0.25, 180, 40)

        if back_distance == -1:
            robot.set_cmd_vel(-0.2, 0.0, 0.75)
            robot.rotate(fwd_angle, direction)

        robot.start_keyboard_input()

#level 2
def stop_sign_detection():
    img = robot.rosImg_to_cv2()
    stop_detected, x1, y1, x2, y2 = TMMC_Wrapper.Robot.ML_predict_stop_sign(stop_sign_model, img)
    global no_stop_detection
    global stop_wait_done
    global stop_trigger_timer
        
    if stop_detected and no_stop_detection:
        stop_trigger_timer = time.time()
        robot.stop_keyboard_input()
        no_stop_detection = False
        stop_wait_done = False
    elif no_stop_detection and stop_wait_done:
        stop_trigger_timer = time.time()
    elif not stop_detected:
        no_stop_detection = True
        
    if time.time() - stop_trigger_timer > STOP_WAIT_TIME:
        robot.start_keyboard_input()
        stop_wait_done = True
        stop_trigger_timer = time.time()

try:
    while rclpy.ok():
        #stop_sign_detection()
        #no_wall_contact()
        img = robot.rosImg_to_cv2()
        detections = robot.detect_april_tag_from_img(img)
        print(detections)
        rclpy.spin_once(robot, timeout_sec=0.1)
        time.sleep
        
except KeyboardInterrupt:
    print("Keyboard interrupt received. Stopping...")

finally:
    robot.stop_keyboard_control()
    robot.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
