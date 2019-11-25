#!/usr/bin/env python
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import matplotlib.pyplot as plt
import math
import numpy as np
import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios

e = """
Communications Failed
"""

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

class KalmanFilter(object):
    def __init__(self, h, d, x_0, Q, R, P_0):
        self.h = h
        self.d = d
	self.tic = 0
	self.toc = 0
        self.Q = Q
        self.R = R
        self.P = P_0
        self.x = x_0
	self.intensity = 320
        self.u = 0 # initialize the cmd_vel input
        self.phi = np.nan #initialize the measurement input
        
        self.state_pub = rospy.Publisher('state', String, queue_size = 1)
	
	self.A = 1
	self.D = np.array([0.6/((1.5-x_0)**2+0.6**2)])
	self.B = 1
	self.Q = Q
	self.R = R
	self.P = P_0
	self.x = np.array([x_0])
	self.xhat = np.array([x_0])
	self.k = 0
	self.W = 0
	self.S = 0
	self.zhat =0
	self.end = 0

    def cmd_callback(self, cmd_msg):
	#noisy vel
        self.u = cmd_msg.linear.x

    ## scall_callback updates self.phi with the most recent measurement of the tower.
    def scan_callback(self, data):
        self.phi = float(data.data)*math.pi/180

    ## call within run_kf to update the state with the measurement 
    def predict(self):
        #rospy.loginfo("TODO: update state via the motion model, and update the covariance with the process noise")
	self.xhat = np.append(self.xhat, np.array([self.xhat[self.k] * 1 + 1 * self.u/30]))#
	self.D = np.append(self.D, np.array([0.6/((1.5-self.xhat[self.k])**2+0.6**2)]))#
	self.zhat = np.arctan2(0.6,(1.5-self.xhat[self.k+1]))
	self.P = np.append(self.P, np.array([self.P[self.k]+self.Q]))#
	self.S = self.P[self.k+1]*self.D[self.k+1]**2+self.R
	self.W = self.P[self.k+1]*self.D[self.k+1]/self.S


        return

    ## call within run_kf to update the state with the measurement 
    def measurement_update(self):
        #rospy.loginfo("TODO: update state when a new measurement has arrived using this function")
	
	self.P[self.k+1] = self.P[self.k+1] - (self.W**2)*(self.S)
	self.xhat[self.k+1] = self.xhat[self.k] + self.u/30
	if np.isnan(self.phi):
		return	

	else:
		self.xhat[self.k+1] = self.xhat[self.k+1] + self.W[0]*(self.phi-self.zhat)

        return

    def run_kf(self):
        
        #rospy.loginfo("TODO: complete this function to update the state with current_input and current_measurement")
	
	self.predict()
	self.measurement_update()
	print(self.xhat[self.k+1])
	if self.xhat[self.k+1] >= 3.1:
		self.end = 1
        self.state_pub.publish(str(float(self.xhat[self.k+1])))
	self.k = self.k+1

    def plotHelper(self):
	return [self.k, self.xhat, self.P]


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node('Lab4')
    try:
        h = 0.6 #y distance to tower
        d = 1.5 #x distance to tower (from origin)  
        
        x_0 = 0 #initial state position
        
        Q = np.array([0.005]) #process noise covariance
        R = np.array([1]) #measurement noise covariance
        P_0 = np.array([0.01]) #initial state covariance 
        kf = KalmanFilter(h, d, x_0, Q, R, P_0)
		
        kf.scan_sub = rospy.Subscriber('scan_angle', String, kf.scan_callback, queue_size=1)
        kf.cmd_sub = rospy.Subscriber('cmd_vel_noisy', Twist, kf.cmd_callback)
        rospy.sleep(1)
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
	    if kf.end == 0:
		
            	kf.run_kf()
	    else:
	    	plt.plot(kf.P)
		plt.show()
	    	plt.plot(kf.xhat)
	    	plt.show()	
	    
	    rate.sleep()
	   
	    
    finally:
	
	
        rospy.loginfo("goodbye")

