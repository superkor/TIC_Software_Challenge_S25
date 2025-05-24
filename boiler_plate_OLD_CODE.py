#Start with imports, ie: import the wrapper
#import other libraries as needed
import TMMC_Wrapper
import rclpy
import numpy as np
import math
import random
from ultralytics import YOLO
import time

#Start ros with initializing the rclpy object
if not rclpy.ok():
    rclpy.init()

TMMC_Wrapper.is_SIM = True
if not TMMC_Wrapper.is_SIM:
    #Specify hardware api
    TMMC_Wrapper.use_hardware()
    
if not "robot" in globals():
    robot = TMMC_Wrapper.Robot()

#Debug messaging 
print("running main")

#start processes
#add starter functions here
# robot.dock()
# robot.undock()
kbd_control = True
robot.start_keyboard_control()
# Load a pretrained model (COCO dataset)
model = YOLO('yolov8n.pt')
stop_sign = False
stop_sign_prev = False

#rclpy,spin_once is a function that updates the ros topics once
rclpy.spin_once(robot, timeout_sec=0.1)

# aligns to april tag, drives towards it, performs predefined movements
should_turn = False
CAMERA_WIDTH_PIXELS = 640
last_seen_april_tag = -1
def perform_april_tag_detection(tag_dict):
    global should_turn
    global last_seen_april_tag
    global CAMERA_WIDTH_PIXELS
    if len(tag_dict) > 0:
        print("exist april tag")
        # Find the tag closest to the center of the screen
        min_offset = list([0, 9999])
        for key, val in tag_dict.items():
            if key in ignored_april_tags:
                continue
            err = (CAMERA_WIDTH_PIXELS / 2 + april_to_err_offset.get(key)) - val[1]
            if math.fabs(err) < math.fabs(min_offset[1]):
                min_offset = list([key, err])

        # Move towards the april tag if within sight and in reasonable position
        if math.fabs(min_offset[1]) < 200:
            should_turn = True
            last_seen_april_tag = min_offset[0]
            print("tag: " + str(last_seen_april_tag))
            robot.send_cmd_vel(0.7, min_offset[1] * 0.002)
    elif should_turn:  # Reached end of hallway
        should_turn = False
        robot.send_cmd_vel(0.0, 0.0)
        robot.set_cmd_vel(0.1, 0, 2.75)  # move a little forward
        robot.set_cmd_vel(0, 0, 1, stop=True)
        if last_seen_april_tag > 0 and last_seen_april_tag not in ignored_april_tags:  # do the turn at the april tag if we have one
            robot.turn_to_angle(math.fabs(april_to_turn.get(last_seen_april_tag)))

# detect april tag
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

april_to_err_offset = {
    1: 0,
    3: -75,
    4: 0,
    5: 0,
    6: 50,
    7: 50,
    9: 0,
}
def get_april_tag(cv_im):
    global ignored_april_tags
    global april_to_err_offset
    global april_to_turn
    #print("got tag " + str(next(iter(robot.detect_april_tag_from_img(robot.checkImage()).items()))))
    tag_dict = robot.detect_april_tag_from_img(cv_im)
    if (len(tag_dict) > 0 or should_turn):
        should_detect = should_turn
        for key, val in tag_dict.items():
            if key not in ignored_april_tags:
                should_detect = True
                break
        if should_detect:
            print("April tag detected. Running april tag routine")
            perform_april_tag_detection(tag_dict)
    print("exiting april tag routine")
    

#run control functions on loop
try:
    print("Entering the robot loop which cycles until the srcipt is stopped")
    while True:
        
        # camera data
        robot.checkImageRelease()
        cv_im = robot.rosImg_to_cv2()
        if TMMC_Wrapper.autonomous_enabled:
            get_april_tag(cv_im)

        #rclpy,spin_once is a function that updates the ros topics once
        rclpy.spin_once(robot, timeout_sec=0.1)

        #Add looping functionality here
        lidar_scan = robot.checkScan()
        front_range = lidar_scan.ranges[0:20] + lidar_scan.ranges[340:359]
        back_range = lidar_scan.ranges[170:190]
        #print(min(front_range))
        #print(min(back_range))
        
        # robot.set_cmd_vel(0.25, 0.0, 0.5)
        # robot.move_forward()

        # if min(back_range) < 0.5 and min(front_range) < 0.5:
        #     turn_right = bool(random.getrandbits(1))
        #     while min(front_range) < 0.6:
        #         robot.send_cmd_vel(0.0, 0.1)
        #         if turn_right:
        #             robot.turn_right()
        #         else:
        #             robot.turn_left()
        #         lidar_scan = robot.checkScan()
        #         front_range = lidar_scan.ranges[0:20] + lidar_scan.ranges[340:359]

        if TMMC_Wrapper.autonomous_enabled:
            if min(front_range) < 0.5:
                robot.send_cmd_vel(-0.05, 0.0)
                # robot.stop_keyboard_control()
                lidar_scan = robot.checkScan()    
                # scan left
                left_max = max(lidar_scan.ranges[90:135])
                #print("left_max： " + str(left_max))
                
                
                # scan right
                right_max = max(lidar_scan.ranges[270:315])
                
                #print("right_max： " + str(right_max))
                # compare left with right
                turn_right = left_max < right_max

                while min(front_range) < 0.5:
                    if TMMC_Wrapper.autonomous_enabled is False:
                        break
                    robot.send_cmd_vel(0.0, 0.05)

                    if turn_right:
                        #print("I am turning right!")
                        robot.turn_right()
                    else:
                        robot.turn_left()
                        #print("I am turning left!")
                    lidar_scan = robot.checkScan()
                    front_range = lidar_scan.ranges[0:10] + lidar_scan.ranges[350:359]
            # elif min(back_range) < 0.6:
            #     turn_right = bool(random.getrandbits(1))
            #     while min(front_range) < 0.5:
            #         robot.send_cmd_vel(0.0, 0.1)
            #         if turn_right:
            #             robot.turn_right()
            #         else:
            #             robot.turn_left()
            #         lidar_scan = robot.checkScan()
            #         front_range = lidar_scan.ranges[0:10] + lidar_scan.ranges[350:359]
            else:
                # robot.start_keyboard_control()
                # kbd_control = True
                robot.send_cmd_vel(0.25, 0.0)
            # # print(len(scan_ranges))
            # if (not kbd_control):
            #     robot.move_forward()

            # get stop sign data
            cv_im = robot.rosImg_to_cv2()
            has_stop, x1, y1, x2, y2 = robot.ML_predict_stop_sign(model, cv_im)

            stop_sign_area = (x2-x1) * (y2-y1)

            focal_length = 4.81
            real_height = 100
            image_height = 480
            sensor_height = 10
            object_width_pixels = abs(x2-x1)
            
            real_stop_sign = abs(x2 - x1) > 80
            
            if (object_width_pixels == 0) or (not real_stop_sign):
                distance_to_stop_sign_mm = -1
            else:
                distance_to_stop_sign_mm = (focal_length * real_height * image_height)/(object_width_pixels * sensor_height)

            #print(f"stop sign area is {stop_sign_area}")\
            print(f"y2: {y2}, y1: {y1}, x2: {x2}, x1: {x1}")
            print(f"stop sign distance is {distance_to_stop_sign_mm/100} meters")
            
            distance_to_stop_sign_m = distance_to_stop_sign_mm / 1000

            # robot sees a stop sign
            if has_stop is True and real_stop_sign:
                print("object_width_pixels: ", str(object_width_pixels))  
                if (object_width_pixels > 90):
                    # Turn left a little bit to correct itself
                    print("Adjusting by turning left a little bit")
                    robot.send_cmd_vel(0.0, 0.2)
                    robot.turn_left()
                elif (object_width_pixels < 85):
                    # Turn right a little bit to correct itself
                    print("Adjusting by turning right a little bit")
                    robot.send_cmd_vel(0.0, 0.2)
                    robot.turn_right()
                    
                startTime = -1
                currentTime = -1
                timeElapsed_seconds = -1
                
                robot.send_cmd_vel(0.25, 0.0)
                
                # move forward until distance is 0
                print("Moving forward until distance is 0")
                while distance_to_stop_sign_m > 0:   
                    timeElapsed_seconds = 0.1 # seconds
                    currentSpeed = 0.25 * 0.3 # m/s
                    distance_traveled_m = timeElapsed_seconds * currentSpeed 
                    distance_to_stop_sign_m -= distance_traveled_m
                    time.sleep(0.1)

                        # reset the startTime
                    # if (startTime == -1):
                    #     startTime = time.time()
                    # else:
                    #     currentTime = time.time()
                    #     timeElapsed_seconds = currentTime - startTime # seconds
                    #     currentSpeed = 0.25 * 0.3 # m/s
                    #     distance_traveled_m = timeElapsed_seconds * currentSpeed 
                    #     distance_to_stop_sign_m -= distance_traveled_m
                    #     startTime = time.time() 
                    #     time.sleep(0.1)

                    #     # reset the startTime
                    
                
                # stop the robot
                robot.send_cmd_vel(0.0, 0.0)
                time.sleep(2)
                print("Stopped because of sign")

                #while True:
                #    pass # robot should do nothing

                #while(stop_sign is True):
                #    robot.send_cmd_vel(0.25, 0.0)
                #    cv_im = robot.rosImg_to_cv2()
                #    has_stop, x1, x2, y1, y2 = robot.ML_predict_stop_sign(model, cv_im)
                #    stop_sign = has_stop      
        else:
            if min(front_range) < 0.3:
                print("WALL DETECTED IN FRONT, backing up")
                robot.send_cmd_vel(-0.05, 0.0)
                robot.stop_keyboard_control()
                time.sleep(1)
            elif min(back_range) < 0.3:
                print("WALL DETECTED BEHIND, moving forward")
                robot.send_cmd_vel(0.05, 0.0)
                robot.stop_keyboard_control()
                time.sleep(1)
            #print("stopsign: " + str(has_stop))
            robot.start_keyboard_control()
                
        # elif
except KeyboardInterrupt:
    print("keyboard interrupt receieved.Stopping...")
finally:
    #when exiting program, run the kill processes
    #add functionality to ending processes here
    robot.destroy_node()
    rclpy.shutdown()