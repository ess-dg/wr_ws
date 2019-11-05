#!/usr/bin/env python
# _stream.py --- Process every frame without distorting it

import sys
import time
import glob

import numpy as np
from scipy.ndimage import filters

import cv2 as cv

import roslib
import rospy
import imutils
import matplotlib.pyplot as plt

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

# Houglines detection
minLineLength = 10 # pixels to make a line
maxLineGap = 30 # pixels to separate lines

# canny detection variables
lowTh = 50
HighTh = 200
len_th = [60, 80] # length to consider a line a blade 
dst_th = [35, 130] # distance from marker to qualify as blade
loc_th = 10 # distance from ideal situation to qualify as blade
slope_th = 0.5  # rads
px_to_mm = 0.1499  # pseudo empiric value

# HSV green mask parameters
sensitivity = 20
lower_green = np.array([60 - sensitivity, 50, 50])
upper_green = np.array([60 + sensitivity, 200, 200])

# Distortion parameters
font = cv.FONT_HERSHEY_SIMPLEX
chunk_factor = 3
image_chunks = []


class image_feature:

    def __init__(self):
        # topic where we publish
        self.ready = True
        self.camera_matrix = 0
        self.dist_coeff = 0
        self.image_pub = rospy.Publisher("/line_detection/image/compressed",
                                         CompressedImage, queue_size=1)

        self.subscriber = rospy.Subscriber("/raspicam_node/image/compressed",
                                           CompressedImage, self.frame_callback,  queue_size=10)

        self.info_sub = rospy.Subscriber("/raspicam_node/camera_info",
                                         CameraInfo, self.info_callback,  queue_size=1)

        self.template = cv.imread(
            '/home/multigrid/catkin_ws/src/line_detection/scripts/pattern.png')
        self.template = cv.cvtColor(self.template, cv.COLOR_BGR2GRAY)
        self.template = cv.Canny(self.template, lowTh, HighTh)
        self.han_logo = cv.imread(
            '/home/multigrid/catkin_ws/src/line_detection/scripts/han100.png')
        self.ess_logo = cv.imread(
            '/home/multigrid/catkin_ws/src/line_detection/scripts/ess100.png')
        self.blade_loc = [79.58, 146.29,  213,  279.71,
                          346.42, 413.13,  479.84,  546.55,
                          613.26, 679.97,  746.68,  813.39,
                          880.10, 946.81, 1013.52, 1080.23, 1146.94]

    def info_callback(self, ros_data):
        self.camera_matrix = np.reshape(ros_data.K, (3, 3))
        self.dist_coeff = ros_data.D
        self.info_sub.unregister()

    def frame_callback(self, ros_data):

        np_arr = np.fromstring(ros_data.data, np.uint8)
        image = cv.imdecode(np_arr, cv.IMREAD_COLOR)
        # Camera distortion correction
        h = np.size(image, 0)
        w = np.size(image, 1)
        K = self.camera_matrix
        d = self.dist_coeff
        newcamera, roi = cv.getOptimalNewCameraMatrix(K, d, (w, h), 0)
        image = cv.undistort(image, K, d, None, newcamera)
        gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        canny = cv.Canny(gray, lowTh, HighTh, apertureSize=3)
        image_chunks = np.vsplit(canny, chunk_factor)
        chunk_size = h / chunk_factor

        image_hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        green_mask = cv.inRange(image_hsv, lower_green, upper_green)
        canny_green = cv.Canny(green_mask, lowTh, HighTh, apertureSize=3)
        markers = cv.HoughCircles(canny_green, cv.HOUGH_GRADIENT, 1, 100,
                                  param1=30,
                                  param2=15,
                                  minRadius=27,
                                  maxRadius=33)
        if markers is not None:
            markers = np.uint16(np.around(markers))
            for mark in markers[0, :]:
                # draw the outer circle
                cv.circle(image, (mark[0], mark[1]), mark[2], (0, 255, 0), 2)
                # draw the center of the circle
                cv.circle(image, (mark[0], mark[1]), 2, (0, 0, 255), 3)
        for idx, chunk in enumerate(image_chunks):

            lines = cv.HoughLinesP(chunk, 1, np.pi/180,
                                   40, None, minLineLength, maxLineGap)

            valid_lines = []
            circles = cv.HoughCircles(chunk, cv.HOUGH_GRADIENT, 1, 100,
                                      param1=30,
                                      param2=15,
                                      minRadius=27,
                                      maxRadius=33)
            if circles is not None:
                circles = np.uint16(np.around(circles))
                if idx != len(image_chunks):
                    cv.line(image, (0, chunk_size*idx),
                            (w, chunk_size*idx), (0, 0, 0), 1, cv.LINE_AA)

                for i in circles[0, :]:
                    if lines is not None:
                        valid_lines = get_lines(i, lines, self.blade_loc)
                    for line in valid_lines:
                        cv.line(image, (line[0], line[1] + chunk_size*idx), (line[2],
                                                                             line[3] + chunk_size*idx), (0, 255, 0), 2, cv.LINE_AA)
                    cv.line(image, (0, i[1] - dst_th[0] + chunk_size*idx), (w,
                                                                            i[1] - dst_th[0] + chunk_size*idx), (255, 0, 0), 1, cv.LINE_AA)
                    cv.line(image, (0, i[1] - dst_th[1] + chunk_size*idx), (w,
                                                                            i[1] - dst_th[1] + chunk_size*idx), (255, 0, 0), 1, cv.LINE_AA)
                    for loc in self.blade_loc:
                        cv.line(image, (i[0] + int(loc), chunk_size*idx), (i[0] +
                                                                           int(loc), chunk_size*(idx+1)), (255, 0, 0), 1, cv.LINE_AA)
                        cv.line(image, (i[0] + int(loc) + loc_th, chunk_size*idx), (i[0] + int(
                            loc) + loc_th, chunk_size*(idx+1)), (100, 0, 100), 1, cv.LINE_AA)
                        cv.line(image, (i[0] + int(loc) - loc_th, chunk_size*idx), (i[0] + int(
                            loc) - loc_th, chunk_size*(idx+1)), (100, 0, 100), 1, cv.LINE_AA)
                    ## Used for calibration
                    # draw the outer circle
                    #cv.circle(image,(i[0],i[1] + chunk_size*idx),i[2],(0,255,0),2)
                    # draw the center of the circle
                    #cv.circle(image,(i[0],i[1] + chunk_size*idx),2,(0,0,255),3)

        cv.imshow('canny', canny)
        ## See the output of the algorithm
        # cv.imshow('green_canny', canny_green)
        # cv.imshow('green',green_mask)
        cv.waitKey(2)

        #### Create CompressedIamge ####
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
        self.image_pub.publish(msg)


def main(args):

    ic = image_feature()
    rospy.init_node('line_detection', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv.destroyAllWindows()


def match_template(template, gray):

    (tH, tW) = template.shape[:2]
    found = None

    for scale in np.linspace(0.2, 1.0, 20)[::-1]:
        # resize the image according to the scale, and keep track
        # of the ratio of the resizing
        resized = imutils.resize(gray, width=int(gray.shape[1] * scale))
        r = gray.shape[1] / float(resized.shape[1])
        # if the resized image is smaller than the template, then break
        # from the loop
        if resized.shape[0] < tH or resized.shape[1] < tW:
            break
        edged = cv.Canny(resized, lowTh, HighTh)
        result = cv.matchTemplate(edged, template, cv.TM_CCOEFF)
        (_, maxVal, _, maxLoc) = cv.minMaxLoc(result)
        # if we have found a new maximum correlation value, then update
        # the bookkeeping variable
        # print(found)
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r)
    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio
    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0] * r), int(maxLoc[1] * r))
    (endX, endY) = (int((maxLoc[0] + tW) * r), int((maxLoc[1] + tH) * r))
    return [(startX + endX)/2, (startY + endY)/2]


def get_lines(ctr, lines, blade_loc):
    valid_lines = []
    for element in lines:
        line = element[0]
        x1 = line[0]
        y1 = line[1]
        x2 = line[2]
        y2 = line[3]
        line_length = length(x1, x2, y1, y2)
        slope = slope_cal(x1, x2, y1, y2)
        # Check if the line is vertical and with the desired length
        # if line_length > 60 and line_length < 75:
        if slope == 1 and line_length > len_th[0] and line_length < len_th[1]:
            # discard the lines that dont start where the blade should start
            if ((ctr[1] - y1) > dst_th[0] and (ctr[1] - y1) < dst_th[1]) or ((ctr[1] - y2) > dst_th[0] and (ctr[1] - y2) < dst_th[1]):
                if y1 <= y2:
                    dist_to_ref = length(ctr[0], x2, ctr[1], y2)
                else:
                    dist_to_ref = length(ctr[0], x1, ctr[1], y1)
                for loc_idx, loc in enumerate(blade_loc):
                    if (dist_to_ref > (loc - loc_th)) and (dist_to_ref < (loc + loc_th)):
                        valid_lines.append(line)

    return valid_lines


def slope_cal(x0, x1, y0, y1):
    slope = np.arctan2((y1-y0), (x1-x0))
    if np.abs(slope - 0) < slope_th or np.abs(slope - np.pi) < slope_th:
        return 0 # horizontal
    if np.abs(slope - np.pi/2) < slope_th or np.abs(slope + np.pi/2) < slope_th:
        return 1 # vertical
    return 2 # nothing


def length(x0, x1, y0, y1):
    return np.sqrt(np.power(y1-y0, 2) + np.power(x1-x0, 2))


if __name__ == '__main__':
    main(sys.argv)
