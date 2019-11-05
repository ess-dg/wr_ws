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

from 		fsm import fsm, fsm_trans
from welder_fsm import welder_fsm_new, welder_states, robot
from  laser_fsm import  laser_fsm_new

def main(args):

	welder_robot_fsm       = welder_fsm_new()
	laser_fsm 		       =  laser_fsm_new()
	laser_status_query_pub = rospy.Publisher('/laser_ctrl/status_query', String, queue_size = 10)
	welder_status_pub 	   = rospy.Publisher('/welder/status'	       , String, queue_size = 10)
	welder_time_pub 	   = rospy.Publisher('/welder/time'	           , String, queue_size = 10)
	#welder_progress_pub	   = rospy.Publisher('/welder/progress'        , String, queue_size = 10)
	rospy.init_node('dummy_welder', anonymous = False)
	rate = rospy.Rate(100)

	while not rospy.is_shutdown():
		msg = 'q'
		welder_status_pub.publish(str(welder_robot_fsm.current_state))
		if robot.pub_time:
			welder_time = '%.2f'%( (time.time() - robot.start_time))
			welder_time_pub.publish(str(welder_time))

		laser_status_query_pub.publish(msg)
		laser_fsm.fire()
		welder_robot_fsm.fire()
		rate.sleep()

	rospy.spin()
#main

if __name__ == '__main__':
    main(sys.argv)

