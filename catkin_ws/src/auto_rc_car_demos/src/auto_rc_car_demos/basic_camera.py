import cv2
import math
import numpy as np
import logging


_SHOW_IMAGE = True
#BLACK_MIN = np.array([0, 0, 0],np.uint8)
#BLACK_MAX = np.array([180, 255, 90],np.uint8)
sensitivity = 40
lower_white = np.array([0,0,255-sensitivity])
upper_white = np.array([255,sensitivity,255])

WHITE_MIN = np.array(lower_white,np.uint8)
WHITE_MAX = np.array(upper_white,np.uint8)

#51,88,118 -> gazebo road line color HSV
lower_yellow = np.array([45,40, 40])
upper_yellow = np.array([55,80,80])

YELLOW_MIN = np.array(lower_yellow,np.uint8)
YELLOW_MAX = np.array(upper_yellow,np.uint8)

BASE_SPEED = 5
STEERING_OFFSET = 0
TIMEOUT = 15
MAX_ANGLE_DEV_1_LINE = 1
MAX_ANGLE_DEV_2_LINES = 4 
IMG_SCALE = 0.25
ROI = 0.75
K_DT = 2
STEER_SCALE = 26

class HandCodedLaneFollower():

    def __init__(self, car=None):
        logging.info('Creating a HandCodedLaneFollower...')
        #self.car = car
        self.curr_steering_angle = 90

    def follow_lane(self, frame):
        # Main entry point of the lane follower
        #self.show_image("orig", frame)

        lane_lines, frame = self.detect_lane(frame)
        final_frame, final_steering_angle, ang_deg = self.steer(frame, lane_lines)

        return final_frame, final_steering_angle, ang_deg

    def convert_to_api_steer(self, steering_angle):
        final_steering_angle = -((steering_angle - 90.0)/90.0)
        return final_steering_angle - STEERING_OFFSET #between -1 (right) and 1 (left)

    def steer(self, frame, lane_lines):
        logging.debug('steering...')
        if len(lane_lines) == 0:
            logging.error('No lane lines detected, nothing to do.')
            print("No lane lines detected, nothing to do.")
            return frame, self.convert_to_api_steer(self.curr_steering_angle)

        new_steering_angle = self.compute_steering_angle(frame, lane_lines)
        self.curr_steering_angle = self.stabilize_steering_angle(self.curr_steering_angle, new_steering_angle, len(lane_lines))
        # print("Steering Angle in degrees: ", self.curr_steering_angle)
        final_steering_angle = self.convert_to_api_steer(self.curr_steering_angle)

        #if self.car is not None:
        #    self.car.front_wheels.turn(self.curr_steering_angle)
	
        curr_heading_image = self.display_heading_line(frame, self.curr_steering_angle)
        self.show_image("heading", curr_heading_image)

        return curr_heading_image, final_steering_angle, self.curr_steering_angle
        
    ############################
    # Frame processing steps
    ############################
    def detect_lane(self, frame):
        logging.debug('detecting lane lines...')

        edges = self.detect_edges(frame)
        #show_image('edges', edges)

        cropped_edges = self.region_of_interest(edges)
        self.show_image('edges cropped', cropped_edges)

        line_segments = self.detect_line_segments(cropped_edges)
        line_segment_image = self.display_lines(frame, line_segments)
        #show_image("line segments", line_segment_image)

        lane_lines = self.average_slope_intercept(frame, line_segments)
        lane_lines_image = self.display_lines(frame, lane_lines)
        #show_image("lane lines", lane_lines_image)

        return lane_lines, lane_lines_image


    def detect_edges(self, frame):
        # filter for black lane lines
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #self.show_image("hsv", hsv)
        mask = cv2.inRange(hsv, WHITE_MIN, WHITE_MAX)
        #mask = cv2.inRange(hsv, YELLOW_MIN, YELLOW_MAX)
        self.show_image("white mask", mask)

        # detect edges
        edges = cv2.Canny(mask, 200, 400)

        return edges
    

    def region_of_interest(self, canny):
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
        #self.show_image("mask", mask)
        masked_image = cv2.bitwise_and(canny, mask)
        return masked_image
    
    def detect_line_segments(self, cropped_edges):
        # tuning min_threshold, minLineLength, maxLineGap is a trial and error process by hand
        rho = 1  # precision in pixel, i.e. 1 pixel
        angle = np.pi / 180  # degree in radian, i.e. 1 degree
        min_threshold = 10  # minimal of votes
        line_segments = cv2.HoughLinesP(cropped_edges, rho, angle, min_threshold, np.array([]), minLineLength=8,
                                        maxLineGap=4)

        if line_segments is not None:
            for line_segment in line_segments:
                logging.debug('detected line_segment:')
                logging.debug("%s of length %s" % (line_segment, self.length_of_line_segment(line_segment[0])))

        return line_segments
    

    def average_slope_intercept(self, frame, line_segments):
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
            lane_lines.append(self.make_points(frame, left_fit_average))

        right_fit_average = np.average(right_fit, axis=0)
        if len(right_fit) > 0:
            lane_lines.append(self.make_points(frame, right_fit_average))

        logging.debug('lane lines: %s' % lane_lines)  # [[[316, 720, 484, 432]], [[1009, 720, 718, 432]]]
        #print('lane lines: %s' % lane_lines)
        return lane_lines
    
    def compute_steering_angle(self, frame, lane_lines):
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
    
    def stabilize_steering_angle(self, curr_steering_angle, new_steering_angle, num_of_lane_lines, max_angle_deviation_two_lines=MAX_ANGLE_DEV_2_LINES, max_angle_deviation_one_lane=MAX_ANGLE_DEV_1_LINE):
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
    def display_lines(self, frame, lines, line_color=(0, 255, 0), line_width=10):
        line_image = np.zeros_like(frame)
        if lines is not None:
            for line in lines:
                for x1, y1, x2, y2 in line:
                    cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
        line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
        return line_image


    def display_heading_line(self, frame, steering_angle, line_color=(0, 0, 255), line_width=5, ):
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


    def length_of_line_segment(self, line):
        x1, y1, x2, y2 = line
        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


    def show_image(self, title, frame, show=_SHOW_IMAGE):
        if show:
            cv2.imshow(title, frame)
            cv2.waitKey(1)

    def make_points(self, frame, line):
        height, width, _ = frame.shape
        slope, intercept = line
        y1 = height  # bottom of the frame
        y2 = int(y1*ROI)  # make points from middle of the frame down

        # bound the coordinates within the frame
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]






class BasicCameraCalc:
    def __init__(self):
        pass

    def detect_sign(self, img):
        # Return:
        ## 'r'/'g'/'b'/'n' for color detected
        ## dominance of that color, or 0 if none
        rows, cols, _ = img.shape
        midr = rows/2
        midc = cols/2

        # Find circles in image
        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        grey = grey[0:midr, midc:]
        circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1, 25, param1=30, param2=18)

        if circles is None:
            return 'n', 0

        circles = np.round(circles[0, :]).astype("int")
        # loop over the (x, y) coordinates and radius of the circles
        for (x0, y0, r) in circles:
            if r < 8:
                print("too small")
                continue

            x = midc+x0
            y = midr+y0
            sr = int(1.0/math.sqrt(2.0) * r)

            # Bounding box coordinates
            i_low = x - sr
            i_high = x + sr
            if i_low < 0 or i_high >= cols:
                #print("i (%d, %d) out of range" % (i_low, i_high))
                continue

            j_low = y0 - sr
            j_high = y0 + sr
            if j_low < 0 or j_high >= rows:
                #print("j (%d, %d) out of range" % (j_low, j_high))
                continue

            #cv2.circle(img, (x , y0), r, (0, 255, 0), 4)
            #cv2.rectangle(img, (x - 5, y0 - 5), (x + 5, y0 + 5), (0, 128, 255), -1)

            # Sum up colored pixels in bounding box
            color_acc = [0,0,0]
            count = 0.0
            for i in range(i_low, i_high):
                for j in range(j_low, j_high):
                    color = img[j, i]
                    #print(color)
                    color_acc = color_acc + color
                    count = count + 1.0
            color_acc = color_acc / count
            #print("avg_color: {}".format(color_acc))

            b = color_acc[0]
            g = color_acc[1]
            r = color_acc[2]

            tot = float(b + g + r)

            thresh = 0.5
            r = r / tot
            g = g / tot

            if r > thresh:
                return 'r', r

            if g > thresh:
                return 'g', g

        return 'n', 0

    def get_black_centroid(self, img, scaled_by):
        rows, cols, _ = img.shape
        count = 0
        totX = 0
        
        threshold = 20
        num_thresh = 30 
        drop_bottom_rows = 20

        for y in range(int(rows/2.5), rows-drop_bottom_rows):
            for x in range(0, cols):
                b,g,r = img[y, x]
                
                if b < threshold and g < threshold and r < threshold:
                    count += 1
                    totX += x

        if totX < num_thresh:
            centroid_x = int(cols / 2)
        else:
            centroid_x = int(totX / (count + 1))
        
        cv2.line(img, (centroid_x, 0), (centroid_x, rows), (0, 0, 255), 2)
        return int(centroid_x / scaled_by)