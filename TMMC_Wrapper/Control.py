from TMMC_Wrapper import CONST_speed_control, is_SIM
from irobot_create_msgs.action import Dock,Undock
from geometry_msgs.msg import Twist
import time
import rclpy
import math
import threading
from pynput.keyboard import Listener

class Control:
    @staticmethod
    def cmd_vel_timer_callback(self):
        if self.cmd_vel_terminate:
            self.cmd_vel_future.set_result(None)
            self.cmd_vel_timer.cancel()
            return
        msg = Twist()
        if self.end_time<time.time():
            self.cmd_vel_terminate = True
        if self.cmd_vel_terminate and self.cmd_vel_stop:
            msg.linear.x = 0.
            msg.angular.z = 0.
        else:
            msg.linear.x = float(self.velocity_x)
            msg.angular.z = float(self.velocity_phi)
        self.cmd_vel_publisher.publish(msg)
         
    @staticmethod   
    def set_cmd_vel(self, velocity_x, velocity_phi, duration, stop=True):
        self.velocity_x = velocity_x
        self.velocity_phi = velocity_phi
        self.end_time = time.time() + duration
        self.cmd_vel_future = rclpy.Future()
        self.cmd_vel_stop = stop
        timer_period = 0.01  # seconds
        self.cmd_vel_terminate = False
        self.cmd_vel_timer = self.create_timer(timer_period, self.cmd_vel_timer_callback)
        rclpy.spin_until_future_complete(self,self.cmd_vel_future)  
        
    @staticmethod
    def spin_until_future_completed(self,future):
        rclpy.spin_until_future_complete(self,future)
        return future.result()

    @staticmethod
    def undock(self):
        # does not wait until finished
        global is_SIM
        if not is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.undock_client.send_goal_async(Undock.Goal())
            rclpy.spin_until_future_complete(self,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self,action_completed_future)
            return action_completed_future.result()
        
    @staticmethod
    def dock(self):
        global is_SIM
        if not is_SIM:
            action_completed_future = rclpy.Future()
            def result_cb(future):
                result = future.result().result
                action_completed_future.set_result(result)
                action_completed_future.done()
            goal_received_future = self.dock_client.send_goal_async(Dock.Goal())
            rclpy.spin_until_future_complete(self,goal_received_future)
            goal_handle = goal_received_future.result()
            if not goal_handle.accepted:
                raise Exception('Goal rejected')

            get_result_future = goal_handle.get_result_async()
            get_result_future.add_done_callback(result_cb)
            rclpy.spin_until_future_complete(self,action_completed_future)
            return action_completed_future.result()

    @staticmethod
    def rotate(self, angle, direction):
        '''Rotate by a certain angle and direction
            Params : angle in deg, direction 1 or -1
            Return : none
        '''
        #get the starting quaternion
        q1 = self.checkImu().orientation
        #get the yaw angle in rad from the quaternion
        _,_,yaw1 = self.euler_from_quaternion(q1)
        #Convert to deg
        yaw1 = math.degrees(yaw1)
        #start the second yaw at the start yaw
        yaw2 = yaw1
        #print(f"angle: {angle}")
        #print(f"yaw 1: {yaw1} yaw2: {yaw2}")
        #while the angle between the new yaw while rotating is not the desired angle rotate
        while abs(yaw2 - yaw1) <= abs(angle):
            #print(f"yaw 1: {yaw1} yaw2: {yaw2}")
            rclpy.spin_once(self, timeout_sec=0.1)
            q2 = self.checkImu().orientation
            _,_,yaw2 = self.euler_from_quaternion(q2)
            yaw2 = math.degrees(yaw2)
            self.send_cmd_vel(0.0,direction * 0.75)
        #set final vel = 0
        self.send_cmd_vel(0.0,0.0)
        print("turn complete")
        
    @staticmethod
    def send_cmd_vel(self, linear_x, angular_z):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_publisher.publish(msg)

    @staticmethod
    def stop_keyboard_input(self):
        self.input = False

    @staticmethod
    def start_keyboard_input(self):
        self.input = True

    @staticmethod
    def start_keyboard_control(self):
        if self.keyboard_listener is None:
            # This set holds keys that are currently pressed.
            pressed_keys = set()

            self.stop_event = threading.Event()

            def update_command():
                # Check for combined key movements first.
                if 'w' in pressed_keys and 'a' in pressed_keys:
                    if self.input:
                        self.send_cmd_vel(0.5, 1.0)
                    return
                elif 'w' in pressed_keys and 'd' in pressed_keys:
                    if self.input:
                        self.send_cmd_vel(0.5, -1.0)
                    return
                elif 's' in pressed_keys and 'a' in pressed_keys:
                    if self.input:
                        self.send_cmd_vel(-0.5, 1.0)
                    return
                elif 's' in pressed_keys and 'd' in pressed_keys:
                    if self.input:
                        self.send_cmd_vel(-0.5, -1.0)
                    return

                # Process individual keys.
                if 'w' in pressed_keys:
                    if self.input:
                        self.move_forward()
                elif 's' in pressed_keys:
                    if self.input:
                        self.move_backward()
                elif 'a' in pressed_keys:
                    if self.input:
                        self.turn_left()
                elif 'd' in pressed_keys:
                    if self.input:
                        self.turn_right()
                else:
                    # If no keys are pressed, stop the movement.
                    self.send_cmd_vel(0.0, 0.0)

            def key_control_loop():
                while not self.stop_event.is_set():
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
            self.keyboard_listener = Listener(on_press=on_press, on_release=on_release)
            self.keyboard_listener.start()
            
            # Start the continuous update thread. Make sure it is a daemon thread so it doesn't block shutdown.
            self.update_thread = threading.Thread(target=key_control_loop, daemon=True)
            self.update_thread.start()
        else:
            print("Keyboard listener already running")

    @staticmethod
    def stop_keyboard_control(self):
        if self.keyboard_listener is not None:
            self.keyboard_listener.stop()
            self.keyboard_listener = None
            print("Keyb list stopped")
        else: 
            print("Keyb list is not running")

        if hasattr(self, 'stop_event'):
            self.stop_event.set()
        if hasattr(self, 'update_thread'):
            self.update_thread.join()

    @staticmethod
    def move_forward(self):
        self.send_cmd_vel(1.0*CONST_speed_control, 0.0)

    @staticmethod
    def move_backward(self):
        self.send_cmd_vel(-1.0*CONST_speed_control, 0.0)

    @staticmethod
    def turn_left(self):
        self.send_cmd_vel(0.0, 1.0*CONST_speed_control)

    @staticmethod
    def turn_right(self):
        self.send_cmd_vel(0.0, -1.0*CONST_speed_control)
