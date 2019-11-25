#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def publisher_node():


	#init publisher
	cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
	rospy.sleep(1)
	twist = Twist()

	#stop
	twist.linear.x = 0.0
	twist.angular.z = 0.0
	
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
