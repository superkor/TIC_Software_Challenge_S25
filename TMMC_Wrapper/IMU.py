import math
import rclpy
from geometry_msgs.msg import Quaternion
import numpy as np

class IMU:
    def __init__(self, robot):
        self.robot = robot

    def checkImu(self):
        self.robot.imu_future = rclpy.Future()
        self.robot.spin_until_future_completed(self.robot.imu_future)
        return self.robot.last_imu_msg

    def rotation_angle(self, q):
        # Extract the angle of rotation from the quaternion
        # w = cos(theta/2)
        w_clamped = max(-1.0, min(1.0, q.w))
        return 2 * math.acos(w_clamped)

    def conjugate_q(self, q):  
        return Quaternion(w=q.w, x=-q.x, y=-q.y, z=-q.z)

    def quaternion_multiply(self, q1, q2):
        # Quaternion multiplication q1 * q2
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z
        return Quaternion(
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2,
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2,
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2,
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    )

    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        if yaw<0:
            yaw += math.pi*2

        return roll, pitch, yaw

    def has_rotation_occurred(self, orientation1, orientation2, desired_rotation_angle):
        
        q1_inv = self.conjugate_q(orientation1)
        q_rel = self.quaternion_multiply(orientation2, q1_inv)
        rotation_angle = self.rotation_angle(q_rel)

        #print(f"Original q: {round(orientation1.w,2), round(orientation1.x,2), round(orientation1.y,2), round(orientation1.z,2)} and current q: {round(orientation2.w,2), round(orientation2.x,2), round(orientation2.y,2), round(orientation2.z,2)}")
        print("current rotation: ", math.degrees(rotation_angle))

        #is desired angle met
        return math.isclose(rotation_angle, desired_rotation_angle, abs_tol=0.01)  # Adjust tolerance as needed
