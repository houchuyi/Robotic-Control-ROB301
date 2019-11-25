#!/usr/bin/env python
import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PIDControl():

	def __init__(self):
		self.intensity = 320

	def bang_bang(self):


		#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(20)
		twist.linear.x = 0.03
		t = 0
		while(t < 1200):
			
			self.actual = self.intensity
			
			self.error = self.desired - self.actual
			#print(self.error)
			if self.error > 0 : self.correction = 0.2
			elif self.error < 0: self.correction = -0.2
			else: self.correction = 0.0
			twist.angular.z = self.correction
			self.cmd_pub.publish(twist)
			rate.sleep()
			t += 1
			rospy.loginfo(twist)

		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_pub.publish(twist)
		rospy.sleep(1)

		pass

	def PControl(self):


			#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(20)
		twist.linear.x = 0.06
		t = 0
		k = 0.015
		while(t < 800):
			
			self.actual = self.intensity
			
			
			self.error = self.desired - self.actual
			#print(self.error)
			if self.error > 0 : self.correction = 0.2*(self.error)*k
			elif self.error < 0: self.correction = 0.2*(self.error)*k
			else: self.correction = 0.0
			twist.angular.z = self.correction
			
			self.cmd_pub.publish(twist)
			rate.sleep()
			t += 1
			rospy.loginfo(twist)

		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_pub.publish(twist)
		rospy.sleep(1)

		pass

	

	def PIControl(self):


		#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(50)
		twist.linear.x = 0.1
		self.cmd_pub.publish(twist)
		t = 0
		kp = 0.0015
		ki = 0.00004
		interror = 0
		while(t < 1000):
			
			self.actual = self.intensity
			
			
			self.error = self.desired - self.actual
			interror += self.error
			#print(self.error)
			self.correction = (self.error)*kp+ (ki*interror)
			twist.angular.z = self.correction
			
			self.cmd_pub.publish(twist)
			rate.sleep()
			t += 1
			rospy.loginfo(twist)

		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_pub.publish(twist)
		rospy.sleep(1)

		pass

	def PID(self):


		#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(30)
		t = 0
		twist.linear.x = 0.1
		self.cmd_pub.publish(twist)
		

		ku = 0.0025
		Tu = 125
		kp = 1.5*ku
		ki = 2*ku/Tu
		kd = ku*Tu/9
		interror = 0
		dererror = 0
		lasterror = 0
		while(t<300):			

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
			t = t + 1
			self.cmd_pub.publish(twist)
			rate.sleep()
			


		twist.linear.x = 0.0
		twist.angular.z = 0.0
		self.cmd_pub.publish(twist)
		rospy.sleep(1)

		pass
	def FasterControl(self):


		#init publisher
		self.cmd_pub = rospy.Publisher('cmd_vel',Twist,queue_size=1)
		twist = Twist()
		rospy.sleep(1)
		self.desired = 320
		rate=rospy.Rate(20)
		twist.linear.x = 0.2
		self.cmd_pub.publish(twist)
		

		ku = 0.0025
		Tu = 62.5
		kp = 4*ku
		ki = 5*ku/Tu
		kd = ku*Tu/9
		interror = 0
		dererror = 0
		lasterror = 0
		while(1):			
			
			twist.linear.x = 0.2
			self.actual = self.intensity
			self.error = self.desired - self.actual
			interror += self.error
			dererror = self.error-lasterror
			if(ki*interror > 0.5):
				interror = 0.5/ki
			elif(ki*interror < -0.5):
				interror = -0.5/ki	

			self.correction = (self.error)*kp+ (ki*interror)+kd*dererror
			#if self.correction < 0 and abs(self.correction) >=1.82:						
			#	twist.angular.z = -1.82
			#elif self.correction > 0 and abs(self.correction) >=1.82:
			#	twist.angular.z = 1.82
			#else:
			twist.angular.z = self.correction
			lasterror = self.error
			
			if abs(twist.angular.z) > 1:
				
				twist.linear.x = 0.03
				
			self.cmd_pub.publish(twist)
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

	def main(self):
	    
		try:
			rospy.init_node('PID')
			self.cam_sub = rospy.Subscriber('/color_mono', String,self.callback)		

			#self.bang_bang()
			#self.PControl()
			#self.PIControl()
			#self.PIDControl()


		except rospy.ROSInterruptException:
			pass

if __name__ == '__main__':
	controller = PIDControl()
	controller.main()
	#controller.bang_bang()
	#controller.PControl()
	#controller.PIControl()
	#controller.PIDControl()
	controller.PID()
	#controller.raceslow()
