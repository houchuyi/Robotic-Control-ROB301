#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PIDControl():

	def __init__(self):
		self.intensity = 320
	def Control(self):


		#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		twist.linear.x = 0.1
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(30)
		self.cmd_pub.publish(twist)
		
		ku = 0.0025
		Tu = 125
		kp = 2*ku
		ki = 3*ku/Tu
		kd = ku*Tu/9
		interror = 0
		dererror = 0
		lasterror = 0
		t = 0
		land = 4

		while(t < 600):			
			
						
			if (land == 4 and abs(self.state - 0.61) <0.05) or (land == 3 and abs(self.state - 1.22) <0.05) or (land==2 and abs(self.state - 2.44) <0.05) or (land == 1 and abs(self.state - 3.05) <0.05):
				twist.linear.x = 0
				twist.angular.z = 0
				self.cmd_pub.publish(twist)
				rospy.sleep(1.5)
				twist.linear.x = 0.1
				twist.angular.z = 0
				self.cmd_pub.publish(twist)
				t = t+1
				rate.sleep()

			else:
				self.actual = self.intensity
				self.error = self.desired - self.actual
				interror += self.error
				dererror = self.error-lasterror
				if(ki*interror > 0.5):
					interror = 0.5/ki
				elif(ki*interror < -0.5):
					interror = -0.5/ki	

				self.correction = (self.error)*kp+ (ki*interror)+kd*dererror
				twist.angular.z = self.correction
				lasterror = self.error
				twist.linear.x = 0.1
				self.cmd_pub.publish(twist)	
				t = t+1
				rate.sleep()

			#rospy.loginfo(twist)
			
		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_pub.publish(twist)
		rospy.sleep(1)

		pass



	def callback(self,cam_data):

		self.intensity = int(cam_data.data)
		

		pass
	def stateCallback(self, state):
		self.state = float(state.data);

	def main(self):
	    
		try:
			rospy.init_node('PID')
			self.cam_sub = rospy.Subscriber('/color_mono', String,self.callback)		
			self.x_state = rospy.Subscriber('state',String, self.stateCallback)
			

		except rospy.ROSInterruptException:
			pass

if __name__ == '__main__':
	controller = PIDControl()
	controller.main()

	controller.Control()
	
