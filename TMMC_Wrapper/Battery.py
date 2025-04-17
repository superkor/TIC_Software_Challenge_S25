import rclpy
from .Robot import Robot

class Battery:
    def __init__(self, robot : Robot):
        ''' Initializes the Battery object by storing a reference to the provided robot for later use. '''
        self.robot = robot

    def checkBattery(self) -> float:
        ''' Waits for the robot\'s battery state update to be completed and then returns the battery percentage from the latest message. '''
        self.robot.battery_state_future = rclpy.Future()
        self.robot.spin_until_future_completed(self.robot.battery_state_future)
        return self.robot.last_battery_state_msg.percentage

