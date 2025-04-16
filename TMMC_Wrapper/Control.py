from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener

class Control:
    def __init__(self, robot):
        self.robot = robot
        self.imu = IMU(self.robot)

    def set_cmd_vel(self, velocity_x, velocity_phi, duration):
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.robot.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self.robot,self.cmd_vel_future)  

    def cmd_vel_timer_callback(self):
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.robot.cmd_vel_publisher.publish(msg)
        
    def undock(self):
        # does not wait until finished
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()
        
    def dock(self):
        if not self.robot.IS_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.robot.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self.robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self.robot,action_completed_future)
            return action_completed_future.result()

    def rotate(self, angle, direction):
        q_initial = self.imu.checkImu().orientation
        _, _, yaw_start = self.imu.euler_from_quaternion(q_initial)
        yaw_start_deg = math.degrees(yaw_start)
        
        def minimal_angle_diff(start, current):
            diff = (current - start + 180) % 360 - 180
            return abs(diff)
        
        current_diff = 0.0
        
        while current_diff < abs(angle):
            q_current = self.imu.checkImu().orientation
            _, _, yaw_current = self.imu.euler_from_quaternion(q_current)
            yaw_current_deg = math.degrees(yaw_current)
            current_diff = minimal_angle_diff(yaw_start_deg, yaw_current_deg)
            
            self.send_cmd_vel(0.0, direction * 0.75)
            rclpy.spin_once(self.robot, timeout_sec=0.1)
        
        self.send_cmd_vel(0.0, 0.0)
        if self.robot.DEBUG:
            print("turn complete")

        
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.robot.cmd_vel_publisher.publish(msg)

    def stop_keyboard_input(self):
        self.robot.input = False

    def start_keyboard_input(self):
        self.robot.input = True

    def start_keyboard_control(self):
        if self.robot.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            self.robot.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if self.robot.input:
                        self.send_cmd_vel(-0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if self.robot.input:
                        self.move_forward()
                elif 's' in pressed_keys:
                    if self.robot.input:
                        self.move_backward()
                elif 'a' in pressed_keys:
                    if self.robot.input:
                        self.turn_left()
                elif 'd' in pressed_keys:
                    if self.robot.input:
                        self.turn_right()
                else:
                    # If no keys are pressed, stop the movement.
                    self.send_cmd_vel(0.0, 0.0)

            def key_control_loop():
                while not self.robot.stop_event.is_set() and rclpy.ok():
                    update_command()
                    time.sleep(0.05)

            def on_press(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.add(key_char)

            def on_release(key):
                try:
                    key_char = key.char
                except AttributeError:
                    key_char = str(key)

                pressed_keys.discard(key_char)


            # Start the keyboard listener.
            self.robot.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.robot.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            self.robot.update_thread = threading.Thread(target=key_control_loop, daemon=True)
            self.robot.update_thread.start()
        else:
            print("Keyboard listener already running")

    def stop_keyboard_control(self):
        if self.robot.keyboard_listener is not None:
            self.robot.keyboard_listener.stop()
            self.robot.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

        if hasattr(self, 'stop_event'):
            self.robot.stop_event.set()
        if hasattr(self, 'update_thread'):
            self.robot.update_thread.join()

    def move_forward(self):
        self.send_cmd_vel(1.0*self.robot.CONST_speed_control, 0.0)

    def move_backward(self):
        self.send_cmd_vel(-1.0*self.robot.CONST_speed_control, 0.0)

    def turn_left(self):
        self.send_cmd_vel(0.0, 1.0*self.robot.CONST_speed_control)

    def turn_right(self):
        self.send_cmd_vel(0.0, -1.0*self.robot.CONST_speed_control)
