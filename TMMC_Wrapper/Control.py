from .Robot import Robot
from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener
from .Robot import Robot

class CONTROL:
    def giveRobot(self):
        return Robot
    def set_cmd_vel(self, velocity_x, velocity_phi, duration, stop=True):
        Robot.velocity_x = velocity_x
        Robot.velocity_phi = velocity_phi
        Robot.end_time = time.time() + duration
        Robot.cmd_vel_future = rclpy.Future()
        Robot.cmd_vel_stop = stop
        timer_period = 0.01  # seconds
        Robot.cmd_vel_terminate = False
        Robot.cmd_vel_timer = Robot.create_timer(timer_period, Robot.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(Robot,Robot.cmd_vel_future)  
        
    def undock(self):
        # does not wait until finished
        if not Robot.is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = Robot.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(Robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(Robot,action_completed_future)
            return action_completed_future.result()
        
    def dock(self):
        if not Robot.is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = Robot.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(Robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(Robot,action_completed_future)
            return action_completed_future.result()

    def rotate(self, angle, direction):
        '''Rotate by a certain angle and direction
            Params : angle in deg, direction 1 or -1
            Return : none
        '''
        #get the starting quaternion
        q1 = IMU.checkImu(Robot).orientation
        #get the yaw angle in rad from the quaternion
        _,_,yaw1 = IMU.euler_from_quaternion(q1)
        #Convert to deg
        yaw1 = math.degrees(yaw1)
        #start the second yaw at the start yaw
        yaw2 = yaw1
        #print(f"angle: {angle}")
        #print(f"yaw 1: {yaw1} yaw2: {yaw2}")
        #while the angle between the new yaw while rotating is not the desired angle rotate
        while abs(yaw2 - yaw1) <= abs(angle):
            #print(f"yaw 1: {yaw1} yaw2: {yaw2}")
            rclpy.spin_once(Robot, timeout_sec=0.1)
            q2 = IMU.checkImu(Robot).orientation
            _,_,yaw2 = IMU.euler_from_quaternion(q2)
            yaw2 = math.degrees(yaw2)
            self.send_cmd_vel(0.0,direction * 0.75)
        #set final vel = 0
        self.send_cmd_vel(0.0,0.0)
        if Robot.DEBUG:
            print("turn complete")
        
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        Robot.cmd_vel_publisher.publish(msg)

    def stop_keyboard_input(self):
        Robot.input = False

    def start_keyboard_input(self):
        Robot.input = True

    def start_keyboard_control(self):
        if Robot.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            Robot.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if Robot.input:
                        self.send_cmd_vel(0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if Robot.input:
                        self.send_cmd_vel(0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if Robot.input:
                        self.send_cmd_vel(-0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if Robot.input:
                        self.send_cmd_vel(-0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if Robot.input:
                        self.move_forward()
                elif 's' in pressed_keys:
                    if Robot.input:
                        self.move_backward()
                elif 'a' in pressed_keys:
                    if Robot.input:
                        self.turn_left()
                elif 'd' in pressed_keys:
                    if Robot.input:
                        self.turn_right()
                else:
                    # If no keys are pressed, stop the movement.
                    self.send_cmd_vel(0.0, 0.0)

            def key_self_loop():
                while not Robot.stop_event.is_set() and rclpy.ok():
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
            Robot.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            Robot.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            Robot.update_thread = threading.Thread(target=key_self_loop, daemon=True)
            Robot.update_thread.start()
        else:
            print("Keyboard listener already running")

    def stop_keyboard_control(self):
        if Robot.keyboard_listener is not None:
            Robot.keyboard_listener.stop()
            Robot.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

        if hasattr(self, 'stop_event'):
            Robot.stop_event.set()
        if hasattr(self, 'update_thread'):
            Robot.update_thread.join()

    def move_forward(self):
        self.send_cmd_vel(1.0*Robot.CONST_speed_control, 0.0)

    def move_backward(self):
        self.send_cmd_vel(-1.0*Robot.CONST_speed_control, 0.0)

    def turn_left(self):
        self.send_cmd_vel(0.0, 1.0*Robot.CONST_speed_control)

    def turn_right(self):
        self.send_cmd_vel(0.0, -1.0*Robot.CONST_speed_control)

Control = CONTROL()
