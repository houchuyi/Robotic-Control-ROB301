#!/usr/bin/env python

import rospy
import math
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import re
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


class BayesLoc:

    def __init__(self, color_map):
        self.colour_sub = rospy.Subscriber('mean_img_rgb', String, self.measurement_callback)
        self.line_idx_sub = rospy.Subscriber('line_idx', String, self.line_callback)
        self.cmd_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.color_map = color_map
        self.measured_rgb = np.array([0,0,0]) # updated with the measurement_callback
        self.line_idx = 0 # updated with the line_callback with the index of the detected black line.
	
	self.p = np.zeros((13,12))

    def Bayesian(self,clr):
	
	offices = np.array([1,3,1,2,0,1,3,2,0,0,3,2])	
	p_init = np.array([0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333,0.83333])
	state_model = np.array([[0.85, 0.05, 0.05],[0.10,0.90,0.10],[0.05,0.05,0.85]])
	measurement_model = np.array([[0.60,0.2,0.05,0.05],[0.2,0.6,0.05,0.05],[0.05,0.05,0.65,0.2],[0.05,0.05,0.15,0.6],[0.1,0.1,0.1,0.1]])
	
	self.p[0]= p_init
	idx = 2
	
	color = ['Blue', 'Green', 'Yellow', 'Orange']
	#once we get one measurement, we estimate and update:
	
	self.p[PID.visited][0] = self.p[PID.visited-1][1]*state_model[0][idx] + self.p[PID.visited-1][0]*state_model[1][idx] + self.p[PID.visited-1][11]*state_model[2][idx]
	self.p[PID.visited][11] = self.p[PID.visited-1][1]*state_model[0][idx] + self.p[PID.visited-1][11]*state_model[1][idx] + self.p[PID.visited-1][10]*state_model[2][idx]
	
	for i in range(1,11):
		self.p[PID.visited][i] = self.p[PID.visited-1][i+1]*state_model[0][idx] +self.p[PID.visited-1][i]*state_model[1][idx] + self.p[PID.visited-1][i-1]*state_model[2][idx]
	
	#color measurement update:
	for i in range(12):
		self.p[PID.visited][i] = measurement_model[clr][offices[i]]*self.p[PID.visited][i]

	self.p[PID.visited] = self.p[PID.visited]/sum(self.p[PID.visited])
	#print index + 1, color, prob
	prob = np.amax(self.p[PID.visited])		
	rvl = [color[clr],np.where(self.p[PID.visited] == prob)[0][0]+1,prob, PID.visited]
	print(rvl)
				

    def measurement_callback(self, msg):
        rgb = msg.data.replace('r:','').replace('b:','').replace('g:','').replace(' ','')
        r,g,b = rgb.split(',')
        r,g,b=(float(r), float(g),float(b))
        self.measured_rgb = np.array([r,g,b])
	
        
    def line_callback(self, data):
        index = int(data.data)
        self.line_idx = index

    def getColor(self): #Blue:0, Green:1, Yellow:2, Orange:3, Nothing:4
	
	if self.measured_rgb[0] < self.measured_rgb[1] and self.measured_rgb[2] < 10: 
		return 1
	elif self.measured_rgb[0] > self.measured_rgb[1] and self.measured_rgb[2] < 10 and self.measured_rgb[1] > 140:
		return 2
	
	elif self.measured_rgb[0] > 200 and self.measured_rgb[1] <140 and self.measured_rgb[2] < 10:
		return 3
	elif self.measured_rgb[2] > self.measured_rgb[1] and self.measured_rgb[2] > 130:
		return 0
	else:
		return 4


class PIDcontrol():

	def __init__(self):
		self.visited = 0
		self.twist = Twist()
		self.twist.linear.x = 0.05
		self.desired = 320
	
	def Control(self):
		ku = 0.0025
		kp = ku #2*ku
		kd = 0 #ku*Tu/9
		dererror = 0
		lasterror = 0
		clr = BL.getColor()		

	        if clr == 0 or clr == 1 or clr == 2 or clr == 3:
			#self.twist.linear.x = 0.05
			#self.twist.angular.z = 0
			#BL.cmd_pub.publish(self.twist)	
			#rospy.sleep(5)		
			#self.twist.linear.x = 0
			#self.twist.angular.z = 0
			#BL.cmd_pub.publish(self.twist)	
			#rospy.sleep(1.5)
			#if clr == 1 or clr == 2: rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
		
			self.visited = self.visited + 1
			BL.Bayesian(clr)
			self.twist.linear.x = 0.05
			self.twist.angular.z = 0.0
			BL.cmd_pub.publish(self.twist)
			rospy.sleep(6.3)

		elif BL.getColor() == 4 or BL.getColor() == None:
			self.actual = BL.line_idx
			self.error = self.desired - self.actual
			dererror = self.error-lasterror	

			self.correction = (self.error)*kp+kd*dererror
			self.twist.angular.z = self.correction
			self.twist.linear.x = 0.05
			lasterror = self.error
			BL.cmd_pub.publish(self.twist)
		

		pass


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
      
    color_map = [0,1,2,3] ### A sample map with 4 colours in a row
                 
    rospy.init_node('bayes_loc')
    BL=BayesLoc(color_map)
    rospy.sleep(0.5)
    rate=rospy.Rate(60)
    ### Initialize your PID controller here ( to merge with the bayes_loc node )
    PID = PIDcontrol()
    t = 0
    try:
        
        while (1):
            key = getKey()
            if (key == '\x03'): #1.22:bayesian.curPos >= 1.6 or
                rospy.loginfo('Finished!')
                break
            
	    if PID.visited == 12:
		break
	 
	    if t >3000:
		break

	    PID.Control()
	
	    t = t + 1
	    rate.sleep()
	    
	    #print(BL.getColor())
            #rospy.loginfo("Measurement: {}".format(BL.measured_rgb))
            #rospy.loginfo("Line index: {}".format(BL.line_idx))
                
#    except Exception as e:
#        print("comm failed:{}".format(e))

    finally:

            ### Stop the robot when code ends
        cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_publisher.publish(twist)







