#!/usr/bin/env python2

import logging

import rospy
import numpy as np
import cv2
import math

import datetime
import sys

from auto_rc_car_api.clients import CreateRCCar



_SHOW_IMAGE = False
BLACK_MIN = np.array([0, 0, 0],np.uint8)
BLACK_MAX = np.array([180, 255, 90],np.uint8)
BASE_SPEED = 5
STEERING_OFFSET = 0
TIMEOUT = 15
MAX_ANGLE_DEV_1_LINE = 1
MAX_ANGLE_DEV_2_LINES = 6 
IMG_SCALE = 0.25
ROI = 0.5
K_DT = 2
STEER_SCALE = 26

class HandCodedLaneFollower():

    def __init__(self, car=None):
        logging.info('Creating a HandCodedLaneFollower...')
        #self.car = car
        self.curr_steering_angle = 90

    def follow_lane(self, frame):
        # Main entry point of the lane follower
        #show_image("orig", frame)

        lane_lines, frame = detect_lane(frame)
        final_frame, final_steering_angle = self.steer(frame, lane_lines)

        return final_frame, final_steering_angle

    def convert_to_VESC_steer(self, steering_angle):
	final_steering_angle = -((steering_angle - 90.0)/90.0)
	return final_steering_angle - STEERING_OFFSET #between -1 (right) and 1 (left)

    def steer(self, frame, lane_lines):
        logging.debug('steering...')
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
	    print("No lane lines detected, nothing to do.")
            return frame, self.convert_to_VESC_steer(self.curr_steering_angle)

        new_steering_angle = compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))
	print("Steering Angle in degrees: ", self.curr_steering_angle)
	final_steering_angle = self.convert_to_VESC_steer(self.curr_steering_angle)

        #if self.car is not None:
        #    self.car.front_wheels.turn(self.curr_steering_angle)
	
        curr_heading_image = display_heading_line(frame, self.curr_steering_angle)
        show_image("heading", curr_heading_image)

        return curr_heading_image, final_steering_angle
        
############################
# Frame processing steps
############################
def detect_lane(frame):
    logging.debug('detecting lane lines...')

    edges = detect_edges(frame)
    #show_image('edges', edges)

    cropped_edges = region_of_interest(edges)
    #show_image('edges cropped', cropped_edges)

    line_segments = detect_line_segments(cropped_edges)
    line_segment_image = display_lines(frame, line_segments)
    #show_image("line segments", line_segment_image)

    lane_lines = average_slope_intercept(frame, line_segments)
    lane_lines_image = display_lines(frame, lane_lines)
    #show_image("lane lines", lane_lines_image)

    return lane_lines, lane_lines_image


def detect_edges(frame):
    # filter for black lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #show_image("hsv", hsv)
    mask = cv2.inRange(hsv, BLACK_MIN, BLACK_MAX)
    #show_image("black mask", mask)

    # detect edges
    edges = cv2.Canny(mask, 200, 400)

    return edges
    

def region_of_interest(canny):
    height, width = canny.shape
    mask = np.zeros_like(canny)

    # only focus bottom ROI of the screen

    polygon = np.array([[
        (0, height * ROI),
        (width, height * ROI),
        (width, height),
        (0, height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255)
    #show_image("mask", mask)
    masked_image = cv2.bitwise_and(canny, mask)
    return masked_image
    
def detect_line_segments(cropped_edges):
    # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
    rho = 1  # precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # degree in radian, i.e. 1 degree
    min_threshold = 10  # minimal of votes
    line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                    maxLineGap=4)

    if line_segments is not None:
        for line_segment in line_segments:
            logging.debug('detected line_segment:')
            logging.debug("%s of length %s" % (line_segment, length_of_line_segment(line_segment[0])))

    return line_segments
    

def average_slope_intercept(frame, line_segments):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        logging.info('No line_segment segments detected')
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
                logging.info('skipping vertical line segment (slope=inf): %s' % line_segment)
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
    #print('lane lines: %s' % lane_lines)
    return lane_lines
    
def compute_steering_angle(frame, lane_lines):
    """ Find the steering angle based on lane line coordinate
        We assume that camera is calibrated to point to dead center
    """
    if len(lane_lines) == 0:
        logging.info('No lane lines detected, do nothing')
        return -90.0

    height, width, _ = frame.shape
    if len(lane_lines) == 1:
        logging.debug('Only detected one lane line, just follow it. %s' % lane_lines[0])
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
    else:
        _, _, left_x2, _ = lane_lines[1][0]
        _, _, right_x2, _ = lane_lines[0][0]
        camera_mid_offset_percent = 0.0 # 0.0 means car pointing to center, -0.03: car is centered to left, +0.03 means car pointing to right
	#print(lane_lines[0][0])
        mid = int(width / 2 * (1 + camera_mid_offset_percent))
        x_offset = (left_x2 + right_x2) / 2 - mid

    # find the steering angle, which is angle between navigation direction to end of center line
    y_offset = int(height / 2)

    angle_to_mid_radian = math.atan(float(x_offset) / float(y_offset))  # angle (in radian) to center vertical line
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  # angle (in degrees) to center vertical line
    steering_angle = angle_to_mid_deg + 90  # this is the steering angle needed by picar front wheel

    logging.debug('new steering angle: %s' % steering_angle)
    return steering_angle
    
def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=MAX_ANGLE_DEV_2_LINES, max_angle_deviation_one_lane=MAX_ANGLE_DEV_1_LINE):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1*ROI)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]


############################
# Utility Functions
############################
def display_lines(frame, lines, line_color=(0, 255, 0), line_width=10):
    line_image = np.zeros_like(frame)
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    # figure out the heading line from steering angle
    # heading line (x1,y1) is always center bottom of the screen
    # (x2, y2) requires a bit of trigonometry

    # Note: the steering angle of:
    # 0-89 degree: turn left
    # 90 degree: going straight
    # 91-180 degree: turn right 
    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image


def length_of_line_segment(line):
    x1, y1, x2, y2 = line
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def show_image(title, frame, show=_SHOW_IMAGE):
    if show:
        cv2.imshow(title, frame)


def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
    x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
    return [[x1, y1, x2, y2]]

def stabilize_steering_angle(curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=5, max_angle_deviation_one_lane=1):
    """
    Using last steering angle to stabilize the steering angle
    This can be improved to use last N angles, etc
    if new angle is too different from current angle, only turn by max_angle_deviation degrees
    """
    if num_of_lane_lines == 2 :
        # if both lane lines detected, then we can deviate more
        max_angle_deviation = max_angle_deviation_two_lines
    else :
        # if only one lane detected, don't deviate too much
        max_angle_deviation = max_angle_deviation_one_lane
    
    angle_deviation = new_steering_angle - curr_steering_angle
    if abs(angle_deviation) > max_angle_deviation:
        stabilized_steering_angle = int(curr_steering_angle
                                        + max_angle_deviation * angle_deviation / abs(angle_deviation))
    else:
        stabilized_steering_angle = new_steering_angle
    logging.info('Proposed angle: %s, stabilized angle: %s' % (new_steering_angle, stabilized_steering_angle))
    return stabilized_steering_angle


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    car = CreateRCCar()
    lane_follower = HandCodedLaneFollower()

    
    print("Starting")

    t_start = rospy.Time.now()
	
    while not rospy.is_shutdown():
        
        if (rospy.Time.now() - t_start).to_sec() > TIMEOUT:
            car.send_control(0,0)
            break

        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)
        lidar, dt2 = car.get_latest_lidar()
	
	
        dt_speed_scale = 1.0 / (1 + K_DT* dt)

        if img is None:
            car.software_sleep(0.25)
            continue
	
        rows, cols, _ = img.shape    

	img_size = img.shape

	img_scale = IMG_SCALE
	newX = int(img_size[1]*img_scale)
	newY = int(img_size[0]*img_scale)
	img_red = cv2.resize(img, (newX, newY))
	
        #c, c_mag = detect_sign(img_red)

        #c = 'x'
        #if c == 'r':
        #    light_speed_scale = 0.0
        #else:
        light_speed_scale = 1.0

	lidar_front_speed_scale = 1.0
        lidar_left_speed_scale = 1.0
        lidar_right_speed_scale = 1.0

        dist_front = -2
        dist_front_left = -2
        dist_front_right = -2
        dist_left = -2
        dist_right = -2

        try:
            sweep = 0.5
            front_limit = 0.75
            dist_front_left = lidar_data_too_close(lidar, -3.12, -3.12+sweep, front_limit)
            dist_front_right = lidar_data_too_close(lidar, 3.14-sweep, 3.14, front_limit)

            def dist_too_close(dist, close_limit):
                if dist > 0 and dist < close_limit:
                    return True
                return False

            if dist_front_left > 0.05 or dist_front_right > 0.05:
                lidar_front_speed_scale = 0.0
            
            dist_left = lidar_data_too_close(lidar, -3, -0.6, 0.25)
            if dist_left > 0.1:
                lidar_left_speed_scale = 0.5

            dist_right = lidar_data_too_close(lidar, 0.6, 3, 0.25)
            if dist_right > 0.1:
                lidar_right_speed_scale = 0.5

        except ZeroDivisionError as e:
            print(e)
        
        lidar_speed_scale = lidar_front_speed_scale * lidar_left_speed_scale * lidar_right_speed_scale
	#lidar_speed_scale = 1.0
	returned_img, control_ang = lane_follower.follow_lane(img_red)
        steer_speed_scale = 1.0 / (STEER_SCALE* control_ang*control_ang + 1.0)
        speed = BASE_SPEED * steer_speed_scale * dt_speed_scale * light_speed_scale * lidar_speed_scale

        if abs(speed) < 0.1:
            car.brake()
        else:
            car.send_control(speed, control_ang)
	
	print("Steering Angle: ", control_ang)
	print("Speed: ", speed)
	#print("dt: ", dt)
	#print("lidar_speed_scale: ", lidar_speed_scale)
	print("========================")
        if _SHOW_IMAGE:
	    cv2.waitKey(25)
	
    car.send_control(0, 0)



