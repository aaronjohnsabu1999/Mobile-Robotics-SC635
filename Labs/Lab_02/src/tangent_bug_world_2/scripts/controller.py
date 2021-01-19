#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from quat2euler import quat2euler

import numpy as np
from math import sqrt
from math import atan2

## Define global variables
pose         = [0.0,0.0,0.0]
range_data   = []
obs_rot_done = 0
obs_mode     = 0
obs_pose     = 0
obs_cutoff   = 0.5
obstacle_LT  = 0
obstacle_MD  = 0
obstacle_RT  = 0
velocity_msg = Twist()

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
    range_data=[min(data[0:71]), min(data[72:647]), min(data[648:719])]
    obs_val_set()

def obs_val_set():
    global range_data, obstacle_LT, obstacle_MD, obstacle_RT
    obstacle_LT = 1 if (range_data[0]<obs_cutoff) else 0
    obstacle_MD = 1 if (range_data[1]<obs_cutoff) else 0
    obstacle_RT = 1 if (range_data[2]<obs_cutoff) else 0

def Waypoints(t):
    x  = 0
    y  = 0
    return [x,y]

def control_loop():
    global obs_mode, obs_pose, obs_cutoff, obs_rot_done
    global pose, range_data, obstacle_LT, obstacle_MD, obstacle_RT
    global velocity_msg
    Kp_w_z = 0.7
    x_fin = 5.0
    y_fin = 0.0
    t_cut = 2.0*np.pi/180

    rospy.init_node('differential_bot_con')
    pub  = rospy.Publisher('/bot_0/cmd_vel', Twist, queue_size=10)
    # pub1 = rospy.Publisher('/obstacle', Int8)
    rospy.Subscriber('/bot_0/odom', Odometry, callback)
    sub  = rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    rate = rospy.Rate(5) 
    
    while not rospy.is_shutdown():
        obstacle = obstacle_LT or obstacle_MD or obstacle_RT
        if (obstacle and (obs_mode == 0)):
            obs_mode = 1
        elif (obs_mode == 1):
            if (obstacle_MD == 0):
                obs_mode = 2
            else:
                velocity_msg.linear.x  =  0.00
                velocity_msg.angular.z =  0.08
        elif (obs_mode == 2):
            if (obstacle_MD):
                obs_mode = 1
            elif (obstacle_LT):
                velocity_msg.linear.x  =  0.12
                velocity_msg.angular.z =  0.01
            elif (obstacle_RT):
                velocity_msg.linear.x  =  0.12
                velocity_msg.angular.z = -0.01
            else:
                velocity_msg.linear.x  =  0.10
                obs_mode = 0
        else:
            if(abs(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))>t_cut):
                velocity_msg.linear.x  = 0.05
                velocity_msg.angular.z = -Kp_w_z*(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))
                nav_mode = 1
            else:
                velocity_msg.linear.x  = 0.25
                velocity_msg.angular.z = 0.00
                nav_mode = 0

        pub.publish(velocity_msg)
        print(str(pose[0]) + "," + str(pose[1]))
        if (sqrt((y_fin-pose[1])**2 + (x_fin-pose[0])**2) < 0.05):
            velocity_msg.linear.x  = 0.0
            velocity_msg.angular.z = 0.0
            pub.publish(velocity_msg)
            print("~~~ DONE DONE DONE ~~~")
            break
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
