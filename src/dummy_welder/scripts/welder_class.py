#!/usr/bin/env python
import sys, time
import roslib 
import rospy
import numpy as np
from   std_msgs.msg        import String
from   geometry_msgs.msg   import Twist
from   rospy.numpy_msg     import numpy_msg
from   rospy_tutorials.msg import Floats
import re


line_regex = re.compile('(\-?\d+\.\d+)') 


class welder:
	def __init__(self):
		# subscriber handlers
		self.mark_loc_sub			= rospy.Subscriber('line_detection/mark_location',String,self.mark_callback, queue_size = 10)
		self.blade_loc_sub          = rospy.Subscriber('/line_detection/blades_location'       ,String,         self.traj_clbk, queue_size  = 100,
																							    buff_size = 400)#, tcp_nodelay = True)
		self.cnc_pos_sub            = rospy.Subscriber('/cnc_interface/position', Twist,      self.cnc_pos_clbk, queue_size  = 10)
		self.laser_status_sub       = rospy.Subscriber('/laser_ctrl/status'     ,String, self.laser_status_clbk, queue_size  = 10)
		self.start_sub              = rospy.Subscriber('/welder/cmd'            ,String,          self.cmd_clbk, queue_size  = 10)
		# publisher handlers
		self.laser_pub              = rospy.Publisher('/laser_ctrl/cmd',     String, queue_size = 10)
		self.cnc_pub                = rospy.Publisher('/cnc_interface/cmd',   Twist, queue_size = 10)
		self.cnc_stop_pub	 		= rospy.Publisher('/cnc_interface/stop', String, queue_size =  1)
		self.line_query_pub         = rospy.Publisher('line_detection/query',String, queue_size = 10)
		self.welder_progress_pub	= rospy.Publisher('/welder/progress'    ,String, queue_size = 10)
		self.ROI_TO_PROCESS 		=  6
		self.LINES_PER_BLADE		= 16
		# reference points Template
		self.grid_refs      = [[440.00, 163.40, 30.00], # [0]  center point 0   [2]    
						       [440.00, 203.40, 30.00], # [1]  center point 0   [1]
						       [440.00, 243.40, 30.00], # [2]  center point 0   [0]
						       [440.00, 283.40, 30.00], # [3]  center point 1   [2]
						       [440.00, 323.40, 30.00], # [4]  center point 1   [1]
						       [440.00, 363.40, 30.00], # [5]  center point 1/2 [0][2]
						       [440.00, 403.40, 30.00], # [6]  center pointv 2   [1]
						       [440.00, 443.40, 30.00], # [7]  center point 2   [2]
						       [193.50, 442.30, 30.00], # [8]  center point 3   [0]
						       [193.50, 402.30, 30.00], # [9]  center point 3   [1]
						       [193.50, 362.30, 30.00], # [10] center point 3   [2]
						       [193.50, 322.30, 30.00], # [11] center point 4   [0]
						       [193.50, 282.30, 30.00], # [12] center point 4   [1]
						       [193.50, 242.30, 30.00], # [13] center point 4/5 [2][0]
						       [193.50, 202.30, 30.00], # [14] center point 5   [1]
							   [193.50, 162.90, 30.00]] # [15] center point 5   [2]

		self.mark_loc 		= []
		self.mark_count		= 0

		self.detect_roi     = [[362.00, 210.70, 30.00],
							   [362.00, 330.70, 30.00],
							   [362.00, 410.70, 30.00],
							   [115.00, 412.70, 30.00],
							   [115.00, 288.70, 30.00],
							   [115.00, 208.70, 30.00]]	
		#timestamp variables
		self.start_time 	= 0 
		self.detect_time	= 0
		self.detect_time_en	= True 
		self.welding_time	= 0
		self.overall_time	= 0

		self.cnc_pos        = [0.0, 0.0, 0.0]
		# boolean variables for fsm
		self.start          = False
		self.next_is_blade  = False
		self.laser_status   = False
		self.laser_cmd      = False
		self.laser_sync     = True
		self.pub_time  		= False
		# traj related variables
		self.last_point     = [0.0, 0.0, 0.0]
		self.received_traj  = []
		self.traj_completed = False
		self.traj_received  = False
		self.grid_count	    = 0
		self.weld_idx		= 0
		self.blade_count	= 0 
		self.detect_roi_idx	= 0
		self.traj 			= []

	def laser_status_clbk(self,ros_data):

		msg = ros_data.data
		if   msg == '1':#fire
			self.laser_status =  True
		elif msg == '0':#stop
			self.laser_status = False		

	def cmd_clbk(self,ros_data):

		char = ros_data.data
		if char == '1':
			self.start 		    = True
			self.stop 			= False
			self.detect_roi_idx     = 0
			self.detect_time_en = True
			self.traj_completed = False
			self.traj_received  = False

		elif char == '0':
			self.start 		    = False
			self.stop 			= True
			self.detect_roi_idx     = 0
			self.detect_time_en = False
			self.traj_completed = False
			self.traj_received  = False

	def cnc_pos_clbk(self,ros_data):

		pos = ros_data
		self.cnc_pos[0] = pos.linear.x
		self.cnc_pos[1] = pos.linear.y
		self.cnc_pos[2] = pos.linear.z


	def mark_callback(self, ros_data):

		msg = ros_data.data 
		marks = line_regex.findall(msg)
		received_marks = []
		received_marks.append(marks[:2])
		received_marks.append(marks[2:4])
		received_marks.append(marks[4:])
		for mark in received_marks:
			mark[1] = float('%.2f' %float(self.detect_roi[self.detect_roi_idx][1]  - float(mark[1])))
			mark[0] = float('%.2f' %float(self.detect_roi[self.detect_roi_idx][0]  + float(mark[0])))
			mark.append(self.detect_roi[self.detect_roi_idx][2]) 
		if self.detect_roi_idx <2:
			self.mark_loc.append(received_marks[2])
			self.mark_loc.append(received_marks[1])
			self.mark_loc.append(received_marks[0])
			self.mark_count += 3
		elif self.detect_roi_idx == 2:
			self.mark_loc.append(received_marks[1])
			self.mark_loc.append(received_marks[0])
			self.mark_count += 2
		elif self.detect_roi_idx < 5:
			self.mark_loc.append(received_marks[0])
			self.mark_loc.append(received_marks[1])
			self.mark_loc.append(received_marks[2])
			self.mark_count += 3
		elif self.detect_roi_idx == 5:
			self.mark_loc.append(received_marks[1])
			self.mark_loc.append(received_marks[2])
			self.mark_count += 2

		for mark in self.mark_loc:
			print mark
		print(self.mark_count)


	def traj_clbk(self,ros_data):

		msg    = ros_data.data
		points = []
		grid   = []
		parsed_msg = line_regex.findall(msg)
		points.append(parsed_msg[:2])
		points.append(parsed_msg[2:])
		self.received_traj.append(points)
		if len(self.received_traj) >= 48: #16 blades 3 times
			grids = np.reshape(np.array(self.received_traj),(3,16,2,2))
			np.array(grids).tolist()

			for grid_idx, grid in enumerate(grids):
				if not ((self.detect_roi_idx == 2 and grid_idx == 2) or (self.detect_roi_idx == 5 and grid_idx == 0)):
					self.traj.append(np.array(grid).tolist())
				else:
					print("Already stored grid, skip")

			self.received_traj = []
			print("Grids received: %d , detection idx: %d "%(len(self.traj),self.detect_roi_idx)) 
			print("Overall time: %.2f s , detection time: %.2f s"%((time.time() - self.start_time),
																   (time.time() - self.detect_time)))
			self.detect_roi_idx += 1
			self.traj_received = True
			if self.detect_roi_idx == self.ROI_TO_PROCESS:
				for grid_idx, grid in enumerate(self.traj): 
					for blade in grid:
						for point in blade:
							# Markers position obtained by CV coords
							point[0] = float(('%.2f'% (float(point[0]) + self.mark_loc[grid_idx][0])))
							point[1] = float(('%.2f'% (float(point[1]) + self.mark_loc[grid_idx][1])))
							point.append(float(self.mark_loc[self.detect_roi_idx][2]))
							#Markers position obtain by real world Coords							
							#point[0] = float(('%.2f'% (float(point[0]) + self.grid_refs[grid_idx][0])))
							#point[1] = float(('%.2f'% (float(point[1]) + self.grid_refs[grid_idx][1])))
							#point.append(self.grid_refs[self.detect_roi_idx][2])
				print("traj of %d grids %d blades %d points"%(len(self.traj),
															  len(self.traj[0]),
															  len(self.traj[0][0])))
	def make_twist(self,p):

		point = Twist()
		point.linear.x  = p[0]
		point.linear.y  = p[1]
		point.linear.z  = p[2]
		point.angular.x =  0.0
		point.angular.y =  0.0
		point.angular.z =  0.0
	
		return point

	def move_to(self, point):

		next_p =  self.make_twist(point)
		print('ROBOT_FSM: Moving to X: %.2f Y: %.2f Z: %.2f' %((next_p.linear.x), (next_p.linear.y), (next_p.linear.z)))
		self.cnc_pub.publish(next_p)

