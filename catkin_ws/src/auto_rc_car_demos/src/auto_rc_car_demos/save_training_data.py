#!/usr/bin/env python2
import cv2
import sys
from auto_rc_car_demos.basic_camera import HandCodedLaneFollower
import rospy
import rospkg

def save_image_and_steering_angle(video_file, output_path):
    lane_follower = HandCodedLaneFollower()
    cap = cv2.VideoCapture(video_file + '.avi')

    try:
        i = 4646
        while cap.isOpened():
            _, frame = cap.read()
            final_frame , control_ang, ang_deg = lane_follower.follow_lane(frame)
            cv2.imwrite(output_path + "%s_%03d_%03d.png" % ("video", i, ang_deg), frame)
            i += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    rospack = rospkg.RosPack()
    package_dir = rospack.get_path("auto_rc_car_demos")
    video_filepath = package_dir + "/training_data"
    print(video_filepath)

    save_image_and_steering_angle(video_filepath + "/video/car_video210719_182657", video_filepath + "/images/")