#!/usr/bin/env python
import rospy
import numpy as np
from quat2euler import quat2euler
from nav_msgs.msg import Odometry
from math import atan2,tan,sin,cos,sqrt
from geometry_msgs.msg import Point, Twist

## Define global variables
pose_x = 0.0
pose_y = 0.0
pose_t = 0.0
a = 1
b = 2
t = []
X = []
Y = []
A = 3
B = 3
buff      = 10
cutoff    = 0.25
numPoints = 200

## This function will give access to robots current position in `pose` variable
def callback(data):
    global pose_x
    global pose_y
    global pose_t
    x    = data.pose.pose.orientation.x;
    y    = data.pose.pose.orientation.y;
    z    = data.pose.pose.orientation.z;
    w    = data.pose.pose.orientation.w;
    pose_x = data.pose.pose.position.x
    pose_y = data.pose.pose.position.y
    pose_t = quat2euler(x,y,z,w)[2]
    
def waypoint_gen():
    global t
    global X
    global Y
    t = []
    X = []
    Y = []
    for i in range(numPoints+buff):
        t.append(i*(2.0*math.pi)/(numPoints))
    for i in range(numPoints+buff):
        X.append(A*math.cos(a*t[i]))
        Y.append(B*math.sin(b*t[i]))

def Waypoints(t):
    return [A*cos(a*t),B*sin(b*t)]

def dist(point1,point2):
    return sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)

## This is where we will calculate error and apply the proportional control to compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    global pose
    global t
    global X
    global Y
    i     = 0
    goal = Point()
    waypoint_gen()

    rospy.init_node('turtlebot_trajectory_tracker')
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
    rospy.Subscriber('/odom', Odometry, callback)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        velocity_msg = Twist()
        goal.x  = X[i]
        goal.y  = Y[i]
        del_x   = goal.x - pose_x
        del_y   = goal.y - pose_y
        theta   = atan2(del_y, del_x)
        del_t   = theta  - pose_t

        error   = distance((pose_x,pose_y),(goal.x,goal.y))
        if(abs(del_t) > 0.22):
            velocity_msg.angular.z = 0.4*del_t
        else
            velocity_msg.linear.x = 0.8*error
    
        if(error < cutoff):
	    i = i + 1
        if(i == numPoints):
            i = 0
        pub.publish(velocity_msg)

        print(str(x)+" "+str(y))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
