import rclpy

class Battery:
    def __init__(self, robot):
        self.robot = robot

    def checkBattery(self):
        self.robot.battery_state_future = rclpy.Future()
        self.robot.spin_until_future_completed(self.robot.battery_state_future)
        return self.robot.last_battery_state_msg.percentage

