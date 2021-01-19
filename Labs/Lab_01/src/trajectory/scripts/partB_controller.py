#!/usr/bin/env python
import math
import numpy
import rospy
from quat2euler import quat2euler
from nav_msgs.msg import Odometry
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

def E_pos(point):
    global pose_x
    global pose_y
    global pose_t
    del_x = point[0]-pose_x
    del_y = point[1]-pose_y
    dist = math.sqrt( (del_x**2) + (del_y**2) )
    return dist

def E_theta(point):
    global pose_x
    global pose_y
    global pose_t
    tan_val = (point[1]-pose_y)/(point[0]-pose_x)
    t_dif   = math.atan(tan_val)
    return t_dif

## This is where we will calculate error and apply the proportional control to compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    global pose_x
    global pose_y
    global pose_t
    global t
    global X
    global Y
    i     = 0
    waypoint_gen()

    rospy.init_node('turtlebot_trajectory_tracker')
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=5)
    rospy.Subscriber('/odom', Odometry, callback)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        del_pos = E_pos(X[i])
        del_ang = E_theta(X[i])
        print("E_pos   = " + str(del_pos) + ", E_theta = " + str(del_ang))
        i = i + 1
        if(i == numPoints):
            i = 0
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
