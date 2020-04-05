#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math
import time

from auto_rc_car_api.clients import CreateRCCar

TIME_END = 12
LEFT = 0.5
RIGHT = -0.5
STRAIGHT = 0

def rad_to_deg(rad):
    deg = rad * (180 / math.pi)
    return deg

def length_of_object(angle, sl1, sl2):
    sl1_sqrd = math.pow(sl1, 2)
    sl2_sqrd = math.pow(sl2, 2)

    length = math.sqrt(sl1_sqrd + sl2_sqrd - (2*sl1*sl2*math.cos(angle)))
    return length

def detect_edges(image):
    hsv_img = cv2.cvtColor(imgCV, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0,0,90])
    upper_white = np.array([255,255,255])

    img_mask = cv2.inRange(hsv_img, lower_white, upper_white)

    edges = cv2.Canny(img_mask, 200, 400)

    return edges


def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    polygon = np.array([[
        (0, height * 6 / 10),
        (width, height * 6 / 10),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    cropped_edges = cv2.bitwise_and(edges, mask)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8, maxLineGap=4)

    return line_segments


def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        print('Unable to detect line segments')
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print('skipping vertical line segment (slope=inf): {}'.format(line_segment))
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0: #adding slope of right lane to right_fit
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else: #adding slope of left lane to left_fit
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0: #checking to make sure that a left lane was detected
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0: #checking to make sure that a right lane was detected
        lane_lines.append(make_points(frame, right_fit_average))

    print('lane lines: {}'.format(lane_lines)) 

    return lane_lines

def make_points(frame, line): #Helper function for average_slope_intercept
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


def detect_lane(img):
    
    edges = detect_edges(img)
    cropped_edges = region_of_interest(edges)
    line_segments = detect_line_segments(cropped_edges)
    lane_lines = average_slope_intercept(img, line_segments)
    
    return lane_lines

def display_lines(img, lines, line_color=(0, 255, 0), line_width=2):
    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(img, 0.8, line_image, 1, 1)
    
    return line_image

if __name__ == "__main__":
    car = CreateRCCar()
   
    time_start = rospy.Time.now()   

    # lidar = car.get_latest_lidar()

    # lidar_values = list(lidar[0].ranges)

    increment = (math.pi / 720)
    ang_rad = -(math.pi / 720)

    # for value in lidar_values:
    #     ang_rad += increment
    #     if type(value) == float and value != float("inf"):
    #         dist_arr.append(value)
    #         angle_arr.append(ang_rad)


    # angle = rad_to_deg(angle_arr[-1] - angle_arr[0])

    mid = 360

    # length = length_of_object(angle, dist_arr[0], dist_arr[-1])


    # turn_right = car.send_control(10, 0.5)
    # turn_left = car.send_control(10, 0)

    # time_right = time.time() + 5

    img, dt = car.get_latest_camera_rgb()
    print(img)
    imgCV = car.camera_to_cv_image(img)
    cv2.imshow('pic', imgCV)
    cv2.waitKey(0)
    print(imgCV)
    print(imgCV.shape)
    print(type(imgCV))

    img_height = imgCV.shape[0]
    img_width = imgCV.shape[1]

    region_of_interest_vertices = [(0, img_height), (img_width / 2, img_height / 2), (img_width, img_height)]
    

    edges = detect_edges(imgCV)
    cv2.imshow('edges', edges)
    cv2.waitKey(0) 

    crop_img = region_of_interest(edges)
    cv2.imshow('cropped img', crop_img)
    cv2.waitKey(0)  

    lane_lines = detect_lane(imgCV)
    lane_lines_image = display_lines(imgCV, lane_lines)
    cv2.imshow("lane lines", lane_lines_image)
    cv2.waitKey(0) 

 
    # while(True):

    #     lidar = car.get_latest_lidar()

    #     list_of_ranges = list(lidar[0].ranges)

    #     dist_arr = []
    #     angle_arr = []

    #     for value in list_of_ranges:
    #         ang_rad += increment
    #         if type(value) == float and value != float("inf"):
    #             dist_arr.append(value)
    #             angle_arr.append(ang_rad)

    #     angle = angle_arr[-1] - angle_arr[0]

    #     length = length_of_object(angle, dist_arr[0], dist_arr[-1])

    #     car.send_control(10, STRAIGHT)

    #     if list_of_ranges[mid] < 1.5: 
    #         car.send_control(10, 0.5)


    #     print("center dist: {}       length: {}".format(list_of_ranges[mid], length))

    #     #print("First Dist: {}      Second Dist: {}      Angle: {}      Length: {}".format(dist_arr[0], dist_arr[-1], angle, length))

    #     # while(time.time() < time_right): 
    #     #     car.send_control(10, 0.5)
        
    #     # car.send_control(10, -0.5)

    #     if list_of_ranges[mid] < 0.5:
    #         car.send_control(0, 0)

    #     if (rospy.Time.now() - time_start).to_sec() > TIME_END:
    #         car.send_control(0,0)
    #         break