#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from quat2euler import quat2euler

import numpy
import math

## Define global variables
pose = [0,0,0]
range_data = [0,0,0]

def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    
def laser_callback(msg):
    global range_data
    data=msg.ranges    
    # range_data=[min(data[480:719]),min(data[240:479]),min(data[0:239])]
    range_data=[min(data[480:719]),min(data[200:529]),min(data[0:239])]
    
def Waypoints(t):
    x  = 0
    y  = 0
    return [x,y] 
    
def sat(val, lim):
    return max(min(val,lim),-lim)


## This is where we will calculate error and apply the proportional control to 
##  compute linear velocity and angular velocity for the turtlebot 
def control_loop():
    global pose, range_data
    
    rospy.init_node('differential_bot_con')
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.Subscriber('/odom', Odometry, callback)
    sub=rospy.Subscriber('/laser/scan',LaserScan,laser_callback)
    
    ## Setting the rate for loop execution
    rate = rospy.Rate(5) 
    
    # PID
    kp = 3; 
    ki = 0.0; 
    kd = 0.0;
    th_intg = 0;
    th_err = 0;
    th_err_prev = 0;
    # goal
    eps = 0.04
    xg = 10; yg = 0;
    # limits
    v_a_sat = 0.9;
    v_fwd = 0.25;
    sens_lim = 1;
    # brake
    brk_f = 0.7;
    
    ## Twist values to move the robot
    velocity_msg = Twist()
    velocity_msg.linear.x = v_fwd
    velocity_msg.angular.z = 0.0
    
    while not rospy.is_shutdown():
    
        #### Progress to robot 
        ####    if obstacle : execute_bug_manoeuver

        # GOAL and PID 
        dist_sq = (pose[0]-xg)*(pose[0]-xg)+(pose[1]-yg)*(pose[1]-yg) 
        th_g = math.atan((yg-pose[1])/(xg-pose[0]))
        th_err = -(pose[2]-th_g)
        #
        if dist_sq < eps:
            velocity_msg.linear.x = brk_f*velocity_msg.linear.x
            velocity_msg.angular.z = 0.0
            v_fwd = 0;
        else:
            velocity_msg.angular.z = sat(kp*th_err+ki*th_intg+kd*(th_err_prev-th_err), v_a_sat)
        
        th_intg = th_intg + th_err
        th_err_prev = th_err 

        # BUG 1
        if min(range_data)<sens_lim:   # Move fwd if obstacle on left or right (center doesn't matter)
            velocity_msg.angular.z = 0  # move straight
            th_intg = 0

        if range_data[1]<sens_lim:     # if obstacle at center and turn towards the free side
            velocity_msg.linear.x = brk_f*velocity_msg.linear.x # stop
            if range_data[0]>sens_lim:
                velocity_msg.angular.z = 1.6
            else:
                velocity_msg.angular.z = -1.6
        else:
            velocity_msg.linear.x = v_fwd

        print "Theta: %f" % pose[2]
        print "Heading: %f" % th_g
        print "Error: %f" % th_err
        print "IError: %f" % th_intg
        print "Range: %f" % range_data[0]
        print "Control Linr: %f" % velocity_msg.linear.x
        print "Control Angr: %f" % velocity_msg.angular.z
    
        pub.publish(velocity_msg)
        print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
