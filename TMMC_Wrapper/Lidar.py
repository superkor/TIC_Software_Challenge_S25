import numpy as np

class Lidar:
    def __init__(self, robot):
        self.robot = robot

    def checkScan(self):
        self.robot.spin_until_future_completed(self.robot.scan_future)
        return self.robot.last_scan_msg

    def lidar_data_too_close(self, scan, th1, th2, min_dist):
        #returns points between angles th1, th2 that are closer tha min_dist
        if th2 < th1:
            temp = th1
            th1 = th2 
            th2 = temp 
        th1 = max(th1, scan.angle_min)
        th2 = min(th2, scan.angle_max)
        ind_start = int((th1-scan.angle_min)/scan.angle_increment)
        ind_end = int((th2-scan.angle_min)/scan.angle_increment)
        meas = scan.ranges[ind_start:ind_end]
        total = len(meas)
        meas = [m for m in meas if np.isfinite(m)]
        if len(meas) == 0:
            return 0.0
        num_too_close = 0.0
        for m in meas: 
            if m < min_dist: 
                print(f"m < min dist addition is {m}")
                num_too_close = num_too_close + 1

        return float(num_too_close) / total

    def detect_obstacle_in_cone(self, scan, distance, center, offset_angle):
        obstacle_dist = distance

        if scan is None:
            print("Warning: No LIDAR Data")
            return -1,-1
        

        if self.robot.is_SIM:
            left = center + offset_angle
            right = center - offset_angle

            if right < 0:
                right = right + 360
            if left >= 360:
                left = left - 360

            if right > left:
                relevent_1 = scan.ranges[0:(left + 1)]
                relevent_2 = scan.ranges[right: 360]
                relevent_range = relevent_1 + relevent_2

                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)

                if min_dist_index > left:
                    min_dist_angle = left - min_dist_index
                else:
                    min_dist_angle = min_dist_index
            else:
                relevent_range = scan.ranges[right:(left + 1)]
                min_dist = min(relevent_range)
                min_dist_index = relevent_range.index(min_dist)
                min_dist_angle = min_dist_index + right
        else:
            new_center = center + 180

            if new_center >= 720:
                new_center = 720 - new_center

            left = new_center + (offset_angle * 2)
            right = new_center - (offset_angle * 2)

            if left >= 720:
                left = left - 720

            if right < 0:
                right = right + 720

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

        if min_dist <= obstacle_dist:
            return min_dist, min_dist_angle
        
        return -1, -1


    def test_lidar_orientation(self):
        #---this was used to find the front heading of the robot, should not be used in solutions
        ranges = self.robot.last_scan_msg.ranges
        num_ranges = len(ranges)
        quarter_segment = num_ranges // 4
        degrees_per_range = 360 / num_ranges

        def index_range_for_segment(start_angle, end_angle):
            start_index = int(start_angle / degrees_per_range)
            end_index = int(end_angle / degrees_per_range)
            return ranges[start_index:end_index]
        
        def analyze_segment(segment):
            sorted_segment = sorted(segment)
            unique_distances = []
            last_added = None
            total = 0

            for distance in sorted_segment: 
                if (last_added is None or abs(distance - last_added) > 0.15) and distance > 0.23:
                    unique_distances.append(distance)
                    last_added = distance 
                    if len(unique_distances) >=5:
                        break
                total += distance
            average_distance = total / len(segment)
            return unique_distances, average_distance
        front_segment = index_range_for_segment(45, 90+45)
        right_segment = index_range_for_segment(90, 180)
        back_segment = index_range_for_segment(180, 270)
        left_segment = index_range_for_segment(270, 359)
        front = analyze_segment(front_segment)
        right = analyze_segment(right_segment)
        back = analyze_segment(back_segment)
        left = analyze_segment(left_segment)


        print(f"Front (Lidar left): {front} meters") #front is 45-135
