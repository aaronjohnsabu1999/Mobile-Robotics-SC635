#!/usr/bin/env python
import sys
import numpy as np
import rospy
import roslib
import logging
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

global th, N
N		= 16
Pi		= np.pi
th		= np.zeros(N)
x		= np.zeros(N)
y		= np.zeros(N)
t		= np.zeros(N)
K_p			= 1
dist_cutoff	= 1
DONE	= [False, False]

def bot_0_cb(data):
	global th	
	x[0] = data.pose.pose.position.x;
	y[0] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[0] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_1_cb(data):
	global th	
	x[1] = data.pose.pose.position.x;
	y[1] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[1] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_2_cb(data):
	global th	
	x[2] = data.pose.pose.position.x;
	y[2] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[2] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_3_cb(data):
	global th	
	x[3] = data.pose.pose.position.x;
	y[3] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[3] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_4_cb(data):
	global th	
	x[4] = data.pose.pose.position.x;
	y[4] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[4] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_5_cb(data):
	global th	
	x[5] = data.pose.pose.position.x;
	y[5] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[5] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_6_cb(data):
	global th	
	x[6] = data.pose.pose.position.x;
	y[6] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[6] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_7_cb(data):
	global th	
	x[7] = data.pose.pose.position.x;
	y[7] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[7] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_8_cb(data):
	global th	
	x[8] = data.pose.pose.position.x;
	y[8] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[8] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_9_cb(data):
	global th	
	x[9] = data.pose.pose.position.x;
	y[9] = data.pose.pose.position.y;
	q0   = data.pose.pose.orientation.w;
	q1   = data.pose.pose.orientation.x;
	q2   = data.pose.pose.orientation.y;
	q3   = data.pose.pose.orientation.z;
	th[9] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_10_cb(data):
	global th	
	x[10] = data.pose.pose.position.x;
	y[10] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[10] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));	

def bot_11_cb(data):
	global th	
	x[11] = data.pose.pose.position.x;
	y[11] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[11] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_12_cb(data):
	global th	
	x[12] = data.pose.pose.position.x;
	y[12] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[12] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_13_cb(data):
	global th	
	x[13] = data.pose.pose.position.x;
	y[13] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[13] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_14_cb(data):
	global th	
	x[14] = data.pose.pose.position.x;
	y[14] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[14] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def bot_15_cb(data):
	global th	
	x[15] = data.pose.pose.position.x;
	y[15] = data.pose.pose.position.y;
	q0    = data.pose.pose.orientation.w;
	q1    = data.pose.pose.orientation.x;
	q2    = data.pose.pose.orientation.y;
	q3    = data.pose.pose.orientation.z;
	th[15] = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));

def dist((x1,y1),(x2,y2)):
	d2 = (x2-x1)**2 + (y2-y1)**2
	return np.sqrt(d2)

def sat(val):
	out = val
	while(out < 0):
		out = out + 2*Pi
	while (out >= 2*Pi):
		out = out - 2*Pi
	return out

def main_node():
	global th, N, DONE

	pub_vel = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	pub_vel[0]  = rospy.Publisher('bot_0/cmd_vel',  Twist, queue_size=10)
	pub_vel[1]  = rospy.Publisher('bot_1/cmd_vel',  Twist, queue_size=10)
	pub_vel[2]  = rospy.Publisher('bot_2/cmd_vel',  Twist, queue_size=10)
	pub_vel[3]  = rospy.Publisher('bot_3/cmd_vel',  Twist, queue_size=10)
	pub_vel[4]  = rospy.Publisher('bot_4/cmd_vel',  Twist, queue_size=10)
	pub_vel[5]  = rospy.Publisher('bot_5/cmd_vel',  Twist, queue_size=10)
	pub_vel[6]  = rospy.Publisher('bot_6/cmd_vel',  Twist, queue_size=10)
	pub_vel[7]  = rospy.Publisher('bot_7/cmd_vel',  Twist, queue_size=10)
	pub_vel[8]  = rospy.Publisher('bot_8/cmd_vel',  Twist, queue_size=10)
	pub_vel[9]  = rospy.Publisher('bot_9/cmd_vel',  Twist, queue_size=10)
	pub_vel[10] = rospy.Publisher('bot_10/cmd_vel', Twist, queue_size=10)
	pub_vel[11] = rospy.Publisher('bot_11/cmd_vel', Twist, queue_size=10)
	pub_vel[12] = rospy.Publisher('bot_12/cmd_vel', Twist, queue_size=10)
	pub_vel[13] = rospy.Publisher('bot_13/cmd_vel', Twist, queue_size=10)
	pub_vel[14] = rospy.Publisher('bot_14/cmd_vel', Twist, queue_size=10)
	pub_vel[15] = rospy.Publisher('bot_15/cmd_vel', Twist, queue_size=10)

	rospy.Subscriber("bot_0/odom",  Odometry, bot_0_cb)
	rospy.Subscriber("bot_1/odom",  Odometry, bot_1_cb)
	rospy.Subscriber("bot_2/odom",  Odometry, bot_2_cb)
	rospy.Subscriber("bot_3/odom",  Odometry, bot_3_cb)
	rospy.Subscriber("bot_4/odom",  Odometry, bot_4_cb)
	rospy.Subscriber("bot_5/odom",  Odometry, bot_5_cb)
	rospy.Subscriber("bot_6/odom",  Odometry, bot_6_cb)
	rospy.Subscriber("bot_7/odom",  Odometry, bot_7_cb)
	rospy.Subscriber("bot_8/odom",  Odometry, bot_8_cb)
	rospy.Subscriber("bot_9/odom",  Odometry, bot_9_cb)
	rospy.Subscriber("bot_10/odom", Odometry, bot_10_cb)
	rospy.Subscriber("bot_11/odom", Odometry, bot_11_cb)
	rospy.Subscriber("bot_12/odom", Odometry, bot_12_cb)
	rospy.Subscriber("bot_13/odom", Odometry, bot_13_cb)
	rospy.Subscriber("bot_14/odom", Odometry, bot_14_cb)
	rospy.Subscriber("bot_15/odom", Odometry, bot_15_cb)
	
	rospy.init_node('swarm_ctrl', anonymous = True)

	rate = rospy.Rate(20)

	logging.basicConfig(filename = "IdealP1.log", format = '%(message)s', filemode = 'w')
	logger = logging.getLogger()
	logger.setLevel(logging.DEBUG)
	
	while not rospy.is_shutdown():
		for i in range(0,N):
			th_N_i	= 0
			N_i		= 0
			th_cutoff1	= 5.0*Pi/180
			th_cutoff2	= 8.0*Pi/180
			vel_cmd	= Twist()
			

			### PART 1 ###
			for j in range(0,N):
				N_i = N_i + 1
				th_N_i = th_N_i + th[j]
			##############

			### PART 2 ###
			# N_i = 2
			# if (not(i == 0) and not(i == N-1)):
			# 	th_N_i = th[i-1] + th[i+1]
			# elif (i == 0):
			# 	th_N_i = th[1] + th[N-1]
			# elif (i == N-1):
			# 	th_N_i = th[0] + th[N-2]
			##############

			### PART 4 ###
			# for j in range(0,N):
			# 	if (not(i == j) and (dist((x[i], y[i]),(x[j], y[j])) < dist_cutoff)):
			# 		N_i = N_i + 1
			# 		th_N_i = th_N_i + th[j]
			##############

			th_req = K_p*(th[i]+th_N_i)/(1+N_i)
			
			vel_cmd.linear.x  = 0.1
			if ((th[i] - th_req) > th_cutoff1):
				vel_cmd.angular.z = -(th[i] - th_req)
			else:
				vel_cmd.angular.z = 0.0
				t[i] = t[i] + 1
			
			check = True
			diff  = np.zeros(N)
			for j in range(0,N):
				diff[j] = round(abs(th[i] - th[j]) * 180.0/Pi, 3)
				if ((i != j) and (abs(th[i] - th[j]) > th_cutoff2)):
					check = False
			if (check == True):
				DONE[0]	= True
			
			logging.info(diff)

			if (DONE[0] == True):
				vel_cmd = Twist()
				for j in range(0,N):
					pub_vel[j].publish(vel_cmd)
				if (DONE[1] == False):
					logging.info(t)
					print("t = " + str(t))
					print("~~~~~~~~~~~~ DONE ~~~~~~~~~~~~")
				DONE[1] = True
			else:
				if (i == 0):
					print("diff = " + str(diff))
					print("")
			
			pub_vel[i].publish(vel_cmd)
		rate.sleep()

if __name__ == '__main__':
	main_node()
