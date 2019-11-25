#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():
    #init publisher
    cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
    rospy.sleep(1.5)
    twist = Twist()
    #go straight
    twist.linear.x = 1.0
    rospy.loginfo(twist)
    cmd_pub.publish(twist)
    rospy.sleep(5)
    #turn 360
    twist.linear.x = 0.0
    twist.angular.z = 2.0
    rospy.loginfo(twist)
    cmd_pub.publish(twist)
    rospy.sleep(3.65)
    #stop
    twist.angular.z = 0.0
    rospy.loginfo(twist)
    cmd_pub.publish(twist)
    rospy.sleep(1)

    pass

def main():
    
    try:
        rospy.init_node('motor')
	
        publisher_node()
	
	
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
