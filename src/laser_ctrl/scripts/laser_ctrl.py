#!/usr/bin/env python

import sys, time
import numpy as np 
import cv2 as cv 
import roslib 
import rospy
from sensor_msgs.msg import Image,CompressedImage, CameraInfo
from std_msgs.msg import String
font = cv.FONT_HERSHEY_SIMPLEX


color_palette = {"red"    : (  0,  0,255),
	             "blue"   : (255,  0,  0),
	             "green"  : (  0,255,  0),
	             "purple" : (255,  0,255)}

class laser_ctrl:

	def __init__(self):
		self.location_pub  = rospy.Publisher("/laser_ctrl/img_feed/compressed", CompressedImage, queue_size = 1)
		self.laser_status  = rospy.Subscriber("/laser_ctrl/cmd",String, self.cmd_callback, queue_size = 10)
		self.image_sub     = rospy.Subscriber("/camera_array/cam0/image_raw/compressed",CompressedImage,self.image_callback, queue_size = 10)
		self.info_sub      = rospy.Subscriber("/camera_array/cam0/camera_info",
			CameraInfo, self.info_callback,  queue_size = 1)
		self.status        = False
		self.camera_matrix = 0 
		self.dist_coeff    = 0
		self.marker_color  = 0 

	def info_callback(self, ros_data):
		self.camera_matrix = np.reshape(ros_data.K,(3,3))
		self.dist_coeff = ros_data.D  
		self.info_sub.unregister()

	def cmd_callback(self,ros_data):
		msg = ros_data.data
		if   msg == 'f':#fire
			self.status = True
		elif msg == 's':#stop
			self.status = False

	def image_callback(self,ros_data):
		np_image = np.fromstring(ros_data.data, np.uint8)
		image = cv.imdecode(np_image, cv.IMREAD_COLOR)
		#image = CompressedImage(image)
		h = np.size(image,0)
		w = np.size(image,1)
		K = self.camera_matrix
		d = self.dist_coeff
		newcamera, roi = cv.getOptimalNewCameraMatrix(K,d,(w,h),0)
		image = cv.undistort(image, K,d, None, newcamera)

		if self.status:
			self.marker_color = color_palette['red']		
		else:
			self.marker_color = color_palette['green']
		#self.marker_color = color_palette['purple']
		cv.drawMarker(image,(w/2,h/2),self.marker_color,cv.MARKER_CROSS ,60,3,cv.LINE_AA)
		cv.circle(image,(w/2,h/2), 30,self.marker_color,3,cv.LINE_AA)	    
		msg = CompressedImage()
		msg.header.stamp = rospy.Time.now()
		msg.format = "jpeg"
		msg.data = np.array(cv.imencode('.jpg', image)[1]).tostring()
		self.location_pub.publish(msg)

def main(args):
    laser = laser_ctrl()
    rospy.init_node('laser_ctrl', anonymous=False)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
