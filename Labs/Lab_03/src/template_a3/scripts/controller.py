#!/usr/bin/env python
import rospy
import numpy as np
import math
import time
from scipy import optimize
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from quat2euler import quat2euler
from template_a3.msg import Trilateration

distanceLandmarkA	= 10.0
distanceLandmarkB	= 10.0
distanceLandmarkC	= 10.0
Landmarks			= {} 
Landmarks[0]		= [ 7.0,  7.0] 
Landmarks[1]		= [-7.0, -7.0]
Landmarks[2]		= [ 7.0, -7.0]
XY_Pres01			= [0.00, 0.00]
XY_NowAvg			= [0.00, 0.00]
XY_PrvAvg			= [0.00, 0.00]
XY_PoseSZ			= [0.00, 0.00]
XY_MASize			= deque([])
pose_z				= 0.00
Kp					= 0.40
Kp_w_z				= 0.15
d					= 0.00

def func(p, ancs, radii):
	global d
	d = 0.0
	for i in range(0,3):
		d += np.power(np.sqrt(np.power(p[0]-ancs[i][0],2) + np.power(p[1]-ancs[i][1],2) ) - radii[i], 2)
	return d

def waypoint(t):
	x = 5*math.cos(t*10*np.pi/180)
	y = 5*math.sin(t*10*np.pi/180)
	return [x,y]

def distance(p1, p2):
	return ((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)**(0.5)
	
def callback(data):
	global d, XY_Pres01, XY_NowAvg, XY_PrvAvg, XY_PoseSZ, XY_MASize, pose_z
	dist   = [data.landmarkA.distance, data.landmarkB.distance, data.landmarkC.distance]
	
	XY_Pres01 = XY_NowAvg
	XY_Pres01 = optimize.fmin_powell(func, XY_Pres01, args=(Landmarks, dist, ), xtol=0.000001, ftol=0.000001, disp=0)
	
	if(abs(d)<0.10):
		XY_PrvAvg = XY_NowAvg
		if(len(XY_MASize)==5):
			XY_MASize.popleft()
		XY_MASize.append(XY_Pres01)
		XY_NowAvg = [0.00, 0.00]
		for i in range(len(XY_MASize)):
			XY_NowAvg[0] = XY_NowAvg[0] + XY_MASize[i][0]
			XY_NowAvg[1] = XY_NowAvg[1] + XY_MASize[i][1]
		XY_NowAvg[0] = XY_NowAvg[0]/len(XY_MASize)
		XY_NowAvg[1] = XY_NowAvg[1]/len(XY_MASize)
		# if(distance(XY_NowAvg, XY_Pres01)>0.03):
		pose_z = math.atan2(XY_PrvAvg[1] - XY_NowAvg[1], XY_PrvAvg[0] - XY_NowAvg[0])
	
	## Compute position of robot from the distances to landmark

	## Compute the heading of the robot using last pose and current pose

def map_angle(theta):
	if(theta > np.pi):
		theta = -(2*np.pi-theta)
	elif(theta < -np.pi):
		theta = (2*np.pi+theta)
	return theta

def sat(x, Th):
	if x<-Th:
		return -Th
	if x > Th:
		return Th
	return x

## This is where we will calculate error and apply control signal
def control_loop():
	global pose_z, range_data
	t = 0
	rospy.init_node('controller_node')
	pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
	rospy.Subscriber('/trilateration_data', Trilateration, callback)
	
	## Setting the rate for loop execution
	rate = rospy.Rate(5) 
	
	## Twist values to move the robot
	d_err = 2.0
	rot   = False
	while not rospy.is_shutdown():
		[x_fin, y_fin]	= waypoint(t)
		x_err			= x_fin-XY_NowAvg[0]
		y_err			= y_fin-XY_NowAvg[1]
		d_err			= (x_err**2 + y_err**2)**0.5
		theta_ref		= math.atan2(y_err, x_err)
		theta_err		= map_angle(theta_ref - pose_z)
		velocity_msg	= Twist()

		
		if(abs(theta_err) > (2.0*np.pi/180)):
			if rot:
				velocity_msg.angular.z = -Kp_w_z*theta_err
			else:
				velocity_msg.linear.x  = 0.10
			rot = not rot
		else:
			velocity_msg.linear.x  = Kp*d_err

		# velocity_msg.angular.z = sat(Kp*theta_err, 0.15)
		# if(abs(velocity_msg.angular.z)==0.15):
		# 	velocity_msg.linear.x  = 0.00
		# 	velocity_msg.linear.z  = 0.50
		# else:
		# 	velocity_msg.linear.x  = sat(Kp*d_err*math.cos(theta_err), 0.20)

		# if rot:
		# 	velocity_msg.linear.x  = sat(Kp*d_err*math.cos(theta_err), 0.20)
		# 	velocity_msg.angular.z = 0.00
		# 	rot = False

		#### Compute robot pose in the callback (line 29)

		if d_err < 0.3:
			velocity_msg = Twist()
			t = t + 1
		
		print(str(t) + "," + str(round(XY_NowAvg[0],4)) + "," + str(round(XY_NowAvg[1],4)))
		pub.publish(velocity_msg)
		rate.sleep()

if __name__ == '__main__':
	try:
		time.sleep(5)
		control_loop()
	except rospy.ROSInterruptException:
		pass
