import rclpy

class Battery:
    @staticmethod
    def checkBattery(robot):
        robot.battery_state_future = rclpy.Future()
        robot.spin_until_future_completed(robot.battery_state_future)
        return robot.last_battery_state_msg.percentage
