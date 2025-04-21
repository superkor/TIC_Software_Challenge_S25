from .Robot import Robot
from sensor_msgs.msg import LaserScan

class Lidar:
    def __init__(self, robot : Robot):
        '''Initializes the Lidar object by storing a reference to the provided robot. '''
        self.robot = robot

    def checkScan(self) -> LaserScan:
        ''' Waits until a new scan message is received and returns that scan message. '''
        self.robot.spin_until_future_completed(self.robot.scan_future)
        return self.robot.last_scan_msg

    def detect_obstacle_in_cone(self, scan, distance, center, offset_angle) -> tuple[float, float]:
        ''' Analyzes the scan data within the cone to detect and return the distance and angle of the closest obstacle. '''
        if offset_angle < 0 or offset_angle > 180:
            raise ValueError("offset_angle must be between 0 and 180 degrees")
        
        if scan is None:
            print("Warning: No LIDAR Data")
            return -1,-1
                
        # Need to seperate the logic for simulation and real robot
        # because the angles are different in the two cases.
        # In simulation, the angles are in degrees and range from 0 to 360.
        # In the real robot, the angles are in degrees and range from 0 to 720.
        if self.robot.IS_SIM:
            # Find the relevent range
            while center >= 360:
                center = center - 360
            while center < 0:
                center = center + 360
            
            right = center - offset_angle
            left = center + offset_angle

            if left >= 360:
                left = left - 360
            if right < 0:
                right = right + 360

            # Get the smallest distance in the relevent range
            if right == left:
                relevent_range = scan.ranges[0:360]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
            elif right > left:
                relevent_1 = scan.ranges[0:left]
                relevent_2 = scan.ranges[right: 360]
                relevent_range = relevent_1 + relevent_2
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
            else:
                relevent_range = scan.ranges[right:left]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
            min_dist_angle = min_dist_index
            if min_dist_angle > 180:
                min_dist_angle = min_dist_angle - 360
        else:
            # Find the relevent range
            new_center = center + 180

            if new_center >= 720:
                new_center = 720 - new_center

            left = new_center + (offset_angle * 2)
            right = new_center - (offset_angle * 2)

            if left >= 720:
                left = left - 720

            if right < 0:
                right = right + 720
            
            # Get the smallest distance in the relevent range
            if right > left:
                relevent_1 = scan.ranges[0:(left + 1)]
                relevent_2 = scan.ranges[right: 720]
                relevent_range = relevent_1 + relevent_2

                #filtered = [x for x in relevent_range if x > 0.14]

                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)

                if min_dist_index > left:
                    min_dist_angle = (left - min_dist_index) / 2
                else:
                    min_dist_angle = min_dist_index / 2
            else:
                relevent_range = scan.ranges[right:(left + 1)]
                #filtered = [x for x in relevent_range if x > 0.14]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                min_dist_angle = (min_dist_index + right) / 2 - 90

        if min_dist <= distance:
            return min_dist, min_dist_angle
        
        # If no obstacle is detected, return -1 for both distance and angle
        return -1, -1
