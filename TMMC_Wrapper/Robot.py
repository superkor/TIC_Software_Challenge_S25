from .Constants import Constants
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import BatteryState
from rclpy.action import ActionClient
import tf2_ros
import rclpy
from irobot_create_msgs.action import Dock,Undock
from irobot_create_msgs.srv import ResetPose
from geometry_msgs.msg import Twist
import numpy as np
import os
import subprocess
import time
import rclpy.qos
from copy import copy

class Robot(Node):
    def __init__(self):
        super().__init__('notebook_wrapper')
        # Create custom qos profile to make subscribers time out faster once notebook
        import rclpy.qos
        from copy import copy
        qos_profile_sensor_data = copy(rclpy.qos.qos_profile_sensor_data)
        qos_policy = copy(rclpy.qos.qos_profile_sensor_data)
        #qos_policy.liveliness = rclpy.qos.LivelinessPolicy.MANUAL_BY_TOPIC
        #qos_policy.liveliness_lease_duration = rclpy.time.Duration(seconds=10)

        self.last_scan_msg = None
        self.last_imu_msg = None

        self.scan_future = rclpy.Future()
        self.scan_subscription = self.create_subscription(LaserScan,'/scan',self.scan_listener_callback,qos_policy)
        self.scan_subscription  # prevent unused variable warning
        
        self.imu_future = rclpy.Future()
        self.imu_subscription = self.create_subscription(Imu,'/imu',self.imu_listener_callback,qos_profile_sensor_data)
        self.imu_subscription  # prevent unused variable warning
        
        self.image_future = rclpy.Future()
        if Constants.is_SIM:
            self.image_subscription = self.create_subscription(Image,'/camera/image_raw',self.image_listener_callback,qos_profile_sensor_data)
        else:
            self.image_subscription = self.create_subscription(Image,'/oakd/rgb/preview/image_raw',self.image_listener_callback,qos_profile_sensor_data)
        self.image_subscription  # prevent unused variable warning

        self.camera_info_future = rclpy.Future()
        if Constants.is_SIM:
            self.camera_info_subscription = self.create_subscription(CameraInfo,'/camera/camera_info',self.camera_info_listener_callback,qos_profile_sensor_data)
        else:
            self.camera_info_subscription = self.create_subscription(CameraInfo,'oakd/rgb/preview/camera_info',self.camera_info_listener_callback,qos_profile_sensor_data)
        self.camera_info_subscription  # prevent unused variable warning
    
        self.battery_state_future = rclpy.Future()
        self.battery_state_subscription = self.create_subscription(BatteryState,'/battery_state',self.battery_state_listener_callback,qos_profile_sensor_data)
        self.battery_state_subscription  # prevent unused variable warning

        if (not(Constants.is_SIM)): 
            self.dock_client = ActionClient(self, Dock, '/dock')
            self.undock_client = ActionClient(self, Undock, '/undock')
            self.dock_client.wait_for_server()
            self.undock_client.wait_for_server()
        
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tf_listener = tf2_ros.transform_listener.TransformListener(self.tf_buffer, self)
        self.logging_topics = ["/tf","/tf_static","/scan","/odom"]
        
        self._reset_pose_client = self.create_client(ResetPose, '/reset_pose')

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.keyboard_listener = None #temp placeholder for the keyboard listener

        self.input = True
        self.k = None

    def use_hardware(self):
        if not Constants.is_SIM:
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
 
    def spin_until_future_completed(self, future):
        rclpy.spin_until_future_complete(self,future)
        return future.result()

    # Callback Functions
    
    def scan_listener_callback(self, msg):
        self.last_scan_msg = msg
        if Constants.DEBUG == True:
            print(f"Laserscan data recieved: Range - {msg.ranges[:5]}")
        self.scan_future.set_result(msg)
        self.scan_future.done()
        
    def imu_listener_callback(self, msg):
        self.last_imu_msg = msg
        if Constants.DEBUG == True:
            print(f"IMU Data recieved: orientation - {msg.orientation}")
        self.imu_future.set_result(msg)
        self.imu_future.done()
        
    def image_listener_callback(self, msg):
        self.last_image_msg = msg
        self.image_future.set_result(msg)
        self.image_future.done()

    def camera_info_listener_callback(self, msg):
        self.last_camera_info_msg = msg
        if self.k is None:
            self.k = np.array(msg.k).reshape((3, 3))
        self.camera_info_future.set_result(msg)
        self.camera_info_future.done()
    
    def battery_state_listener_callback(self, msg):
        self.last_battery_state_msg = msg
        self.battery_state_future.set_result(msg)
        self.battery_state_future.done()

    def cmd_vel_timer_callback(robot):
        if robot.cmd_vel_terminate:
            robot.cmd_vel_future.set_result(None)
            robot.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if robot.end_time<time.time():
            robot.cmd_vel_terminate = True
        if robot.cmd_vel_terminate and robot.cmd_vel_stop:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(robot.velocity_x)
            msg.angular.z = float(robot.velocity_phi)
        robot.cmd_vel_publisher.publish(msg)
