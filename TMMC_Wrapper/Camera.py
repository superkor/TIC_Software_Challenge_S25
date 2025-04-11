import cv2
import apriltag
from .Robot import Robot
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO
import rclpy
from .Control import Control

class Camera:
    @staticmethod
    def checkImage(robot):
        robot.image_future = rclpy.Future()
        robot.spin_until_future_completed( robot.image_future)
        return robot.last_image_msg

    @staticmethod
    def checkImageRelease(robot): #this one returns an actual image instead of all the data
        image = robot.checkImage()
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        cv2.imshow("image", img_3D)
        cv2.waitKey(10)

    @staticmethod
    def checkCamera(robot):
        robot.camera_info_future = rclpy.Future()
        robot.spin_until_future_completed(robot.camera_info_future)
        return robot.last_camera_info_msg 
        # ^ this might have more data, test this

    @staticmethod
    def estimate_apriltag_pose(robot, image):

        img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", img_gray)
        cv2.waitKey(1)  # Add a small delay so the window can update
        
        # Set up AprilTag detector options; adjust families as needed.
        options = apriltag.DetectorOptions(families="tag16h5, tag25h9")
        detector = apriltag.Detector(options)
        detections = detector.detect(img_gray)
        

        if len(detections) == 0 or robot.k is None:
            return None

        # For simplicity, use the first detected tag.
        detection = detections[0]
        tag_id = detection.tag_id
        
        # Extract the image coordinates of the tag's four corners.
        image_points = np.array(detection.corners, dtype=np.float32)
        
        # Define the tag's physical corner coordinates in its own coordinate system.
        # Here the tag is centered at (0,0,0) and lies on the XY plane.
        half_size = Robot.TAG_SIZE / 2.0
        object_points = np.array([
            [-half_size,  half_size, 0],
            [ half_size,  half_size, 0],
            [ half_size, -half_size, 0],
            [-half_size, -half_size, 0]
        ], dtype=np.float32)
        
        # Estimate the pose of the tag relative to the camera.
        ret, rvec, tvec = cv2.solvePnP(object_points, image_points, robot.k, None)
        if not ret:
            print("Pose estimation failed.")
            return None

        # Flatten tvec for simpler processing.
        tvec = tvec.flatten()
        
        # Compute the range (Euclidean distance).
        range_ = np.linalg.norm(tvec)
        
        # Calculate the bearing and elevation (in degrees) relative to the camera's forward axis.
        # For a typical camera coordinate system:
        #   - x axis: right, y axis: down, z axis: forward.
        bearing = np.degrees(np.arctan2(tvec[0], tvec[2]))
        elevation = np.degrees(np.arctan2(tvec[1], tvec[2]))
        
        return tag_id, range_, bearing, elevation

    @staticmethod
    def red_filter(img):
        """
        mask image for red only area, note that the red HSV bound values are tunable and should be adjusted base on evironment
        :param img: list RGB image array
        :return: list RGB image array of binary filtered image
        """
        # Colour Segmentation
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Define lower and upper bounds for red and brown hue
        lower_red_1 = np.array([-3, 100, 0])     # Lower bound for red hue (reddish)
        lower_red_2 = np.array([170, 70, 50])   # Lower bound for red hue (reddish)
        upper_red_1 = np.array([3, 255, 255])  # Upper bound for red hue (reddish)
        upper_red_2 = np.array([180, 255, 255]) # Upper bound for red hue (reddish)
        lower_brown = np.array([10, 60, 30])    # Lower bound for brown hue
        upper_brown = np.array([30, 255, 255])  # Upper bound for brown hue
        
        # Create masks for red and brown
        red_mask_1 = cv2.inRange(hsv_img, lower_red_1, upper_red_1)
        red_mask_2 = cv2.inRange(hsv_img, lower_red_2, upper_red_2)
        brown_mask = cv2.inRange(hsv_img, lower_brown, upper_brown)

        # Combine red masks
        red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)
        # Exclude brown by subtracting its mask from the red mask
        red_mask = cv2.subtract(red_mask, brown_mask)

        # Apply the red mask to the original image then convert to grayscale
        red_img = cv2.bitwise_and(img, img, mask=red_mask)
        gray = cv2.cvtColor(red_img, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 0)

        #get binary image with OTSU thresholding
        (T, threshInv) = cv2.threshold(blurred, 0, 255,
        cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        cv2.imshow("Threshold", threshInv)
        # print("[INFO] otsu's thresholding value: {}".format(T))

        #Morphological closing
        kernel_dim = (21,21)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_dim)
        filtered_img = cv2.morphologyEx(threshInv, cv2.MORPH_CLOSE, kernel)
        
        return filtered_img

    @staticmethod
    def add_contour(img):
        """
        apply contour detection to the red only masked image
        :param img: list image array
        :return: contoured img, max area and centroid(cy,cx)
        """
        max_area = 0    # stores the largest red area detected
        #edges = cv2.Canny(img, 100, 200)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


        #loop through and get the area of all contours
        areas_of_contours = [cv2.contourArea(contour) for contour in contours]
        contoured = np.zeros((img.shape[0], img.shape[1], 3), dtype = np.uint8)

        try:
            max_poly_indx = np.argmax(areas_of_contours)
            stop_sign = contours[max_poly_indx]
            #approximate contour into a simpler shape
            epsilon = 0.01 * cv2.arcLength(stop_sign, True)
            approx_polygon = cv2.approxPolyDP(stop_sign, epsilon, True)
            area = cv2.contourArea(approx_polygon)
            max_area = max(max_area, area)
            cv2.drawContours(contoured, [approx_polygon], -1, (0, 255, 0), 3)

            # compute the center of the contour
            M = cv2.moments(stop_sign)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.circle(contoured, (cX, cY), 2, (255, 255, 255), -1)
        except:
            x=1

        return contoured, max_area ,(cX,cY)

    @staticmethod
    def rosImg_to_cv2(robot):
        #returns np array for image to be used for open cv
        image = Camera.checkImage(robot)
        height = image.height
        width = image.width
        img_data = image.data
        img_3D = np.reshape(img_data, (height, width, 3))
        return img_3D

    @staticmethod
    def ML_predict_stop_sign(model, img):
        # height, width = image.shape[:2]
        # imgsz = (width, height)

        stop_sign_detected = False

        x1 = -1
        y1 = -1 
        x2 = -1 
        y2 = -1

        # Predict stop signs in image using model
        results = model.predict(img, classes=[11], conf=0.25, imgsz=640, max_det=1)
        
        # Results is a list containing the results object with all data
        results_obj = results[0]
        
        # Extract bounding boxes
        boxes = results_obj.boxes.xyxy

        try:
            for box in boxes:
                x1, y1, x2, y2 = map(int, box[:4])
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                stop_sign_detected = True
        except:
            stop_sign_detected = False

        cv2.imshow("Bounding Box", img)

        return stop_sign_detected, x1, y1, x2, y2   
