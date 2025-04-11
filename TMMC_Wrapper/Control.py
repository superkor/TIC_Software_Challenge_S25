from .Constants import Constants
from .IMU import IMU
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener

class Control:
    @staticmethod   
    def set_cmd_vel(robot, velocity_x, velocity_phi, duration, stop=True):
        robot.velocity_x = velocity_x
        robot.velocity_phi = velocity_phi
        robot.end_time = time.time() + duration
        robot.cmd_vel_future = rclpy.Future()
        robot.cmd_vel_stop = stop
        timer_period = 0.01  # seconds
        robot.cmd_vel_terminate = False
        robot.cmd_vel_timer = robot.create_timer(timer_period, robot.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(robot,robot.cmd_vel_future)  
        
    @staticmethod
    def undock(robot):
        # does not wait until finished
        if not Constants.is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = robot.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(robot,action_completed_future)
            return action_completed_future.result()
        
    @staticmethod
    def dock(robot):
        if not Constants.is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = robot.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(robot,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(robot,action_completed_future)
            return action_completed_future.result()

    @staticmethod
    def rotate(robot, angle, direction):
        '''Rotate by a certain angle and direction
            Params : angle in deg, direction 1 or -1
            Return : none
        '''
        #get the starting quaternion
        q1 = IMU.checkImu(robot).orientation
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
            rclpy.spin_once(robot, timeout_sec=0.1)
            q2 = IMU.checkImu(robot).orientation
            _,_,yaw2 = IMU.euler_from_quaternion(q2)
            yaw2 = math.degrees(yaw2)
            Control.send_cmd_vel(robot, 0.0,direction * 0.75)
        #set final vel = 0
        Control.send_cmd_vel(robot, 0.0,0.0)
        if Constants.DEBUG:
            print("turn complete")
        
    @staticmethod
    def send_cmd_vel(robot, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        robot.cmd_vel_publisher.publish(msg)

    @staticmethod
    def stop_keyboard_input(robot):
        robot.input = False

    @staticmethod
    def start_keyboard_input(robot):
        robot.input = True

    @staticmethod
    def start_keyboard_control(robot):
        if robot.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            robot.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if robot.input:
                        Control.send_cmd_vel(robot, 0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if robot.input:
                        Control.send_cmd_vel(robot, 0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if robot.input:
                        Control.send_cmd_vel(robot, -0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if robot.input:
                        Control.send_cmd_vel(robot, -0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if robot.input:
                        Control.move_forward(robot)
                elif 's' in pressed_keys:
                    if robot.input:
                        Control.move_backward(robot)
                elif 'a' in pressed_keys:
                    if robot.input:
                        Control.turn_left(robot)
                elif 'd' in pressed_keys:
                    if robot.input:
                        Control.turn_right(robot)
                else:
                    # If no keys are pressed, stop the movement.
                    Control.send_cmd_vel(robot, 0.0, 0.0)

            def key_control_loop():
                while not robot.stop_event.is_set():
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
            robot.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            robot.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            robot.update_thread = threading.Thread(target=key_control_loop, daemon=True)
            robot.update_thread.start()
        else:
            print("Keyboard listener already running")

    @staticmethod
    def stop_keyboard_control(robot):
        if robot.keyboard_listener is not None:
            robot.keyboard_listener.stop()
            robot.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

        if hasattr(robot, 'stop_event'):
            robot.stop_event.set()
        if hasattr(robot, 'update_thread'):
            robot.update_thread.join()

    @staticmethod
    def move_forward(robot):
        Control.send_cmd_vel(robot, 1.0*Constants.CONST_speed_control, 0.0)

    @staticmethod
    def move_backward(robot):
        Control.send_cmd_vel(robot, -1.0*Constants.CONST_speed_control, 0.0)

    @staticmethod
    def turn_left(robot):
        Control.send_cmd_vel(robot, 0.0, 1.0*Constants.CONST_speed_control)

    @staticmethod
    def turn_right(robot):
        Control.send_cmd_vel(robot, 0.0, -1.0*Constants.CONST_speed_control)
