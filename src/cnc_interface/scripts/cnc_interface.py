"""Creates Cnc object and creates ROS topics."""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

from cnc_class import Cnc

CNC_OBJ = Cnc()


def cmd_callback(msg):
    rospy.loginfo(rospy.get_name() + ": " + str(msg))
    #print(msg.linear.x, msg.linear.y, msg.linear.z)
    CNC_OBJ.move_to(msg.linear.x, msg.linear.y, msg.linear.z)

def stop_callback(msg):
    if msg.data == 's':
        CNC_OBJ.disable_stepper_motors()
    elif msg.data == 'f':
        CNC_OBJ.enable_stepper_motors()

def main():
    """Create ROS topics."""
    pos_pub = rospy.Publisher('/cnc_interface/position', Twist, queue_size=10)
    status_pub = rospy.Publisher('/cnc_interface/status', String, queue_size=10)
    rospy.Subscriber('cnc_interface/cmd', Twist, cmd_callback)
    rospy.Subscriber('cnc_interface/stop', String, stop_callback)

    rospy.init_node('cnc_interface', anonymous=True)

    port = rospy.get_param('cnc_interface/port')
    baudrate = rospy.get_param('cnc_interface/baudrate')
    acc = rospy.get_param('cnc_interface/acceleration')
    max_x = rospy.get_param('cnc_interface/x_max')
    max_y = rospy.get_param('cnc_interface/y_max')
    max_z = rospy.get_param('cnc_interface/x_max')
    default_speed = rospy.get_param('cnc_interface/default_speed')
    speed_x = rospy.get_param('cnc_interface/x_max_speed')
    speed_y = rospy.get_param('cnc_interface/y_max_speed')
    speed_z = rospy.get_param('cnc_interface/z_max_speed')
    steps_x = rospy.get_param('cnc_interface/x_steps_mm')
    steps_y = rospy.get_param('cnc_interface/y_steps_mm')
    steps_z = rospy.get_param('cnc_interface/z_steps_mm')

    CNC_OBJ.startup(port, baudrate, acc, max_x, max_y, max_z, default_speed, speed_x, speed_y,
                    speed_z, steps_x, steps_y, steps_z)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        status = CNC_OBJ.get_status()
        cnc_pose = CNC_OBJ.get_twist()
        ros_status = String(status)
        pos_pub.publish(cnc_pose)
        status_pub.publish(ros_status)
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
