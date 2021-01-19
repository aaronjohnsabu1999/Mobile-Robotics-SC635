#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from quat2euler import quat2euler

import numpy as np
from math import sqrt
from math import atan2

pose         = [0.0,0.0,0.0]
range_data   = []
velocity_msg = Twist()
x_fin  = 5.0
y_fin  = 0.0
Kp_w_z = 1.5
t_cut  = 2.0*np.pi/180

nav_mode     = 0
obstacle     = 0
obstacle_LT  = 0
obstacle_MD  = 0
obstacle_RT  = 0
obs_cutoff   = 0.5
obs_mode     = 0
obs_pose     = 0.0
obs_rot_done = 0

def straight_check():
    if (abs(pose[1])<0.1):                         # m-line specific to trajectory
        return 1
    return 0

def callback(data):
    global pose
    x    = data.pose.pose.orientation.x;
    y    = data.pose.pose.orientation.y;
    z    = data.pose.pose.orientation.z;
    w    = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    
def laser_callback(msg):
    global range_data, obstacle_LT, obstacle_MD, obstacle_RT
    data       = msg.ranges
    range_data = [min(data[0:71]), min(data[72:647]), min(data[648:719])]
    obstacle_LT = 1 if (range_data[0]<obs_cutoff) else 0
    obstacle_MD = 1 if (range_data[1]<obs_cutoff) else 0
    obstacle_RT = 1 if (range_data[2]<obs_cutoff) else 0

def control_loop():
    global pose, range_data, velocity_msg, x_fin, y_fin, K_p_w_z, t_cut
    global nav_mode, obstacle, obstacle_LT, obstacle_MD, obstacle_RT, obs_cutoff, obs_mode, obs_pose, obs_rot_done

    rospy.init_node('differential_bot_con')
    pub  = rospy.Publisher('/bot_0/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/bot_0/odom', Odometry, callback)
    sub  = rospy.Subscriber('/bot_0/laser/scan',LaserScan,laser_callback)
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        nav_mode = 0
        obstacle = obstacle_MD

        if(obs_mode == 0):
            if (obstacle):
                print(" :: 00 : 1 :: ")
                obs_mode = 1
                velocity_msg.linear.x  = 0.00
                velocity_msg.angular.z = 0.00
            elif (not(straight_check())):
                print(" :: 00 : 2 :: ")
                obs_mode = 50
                velocity_msg.linear.x  = 0.00
                velocity_msg.angular.z = 0.00
            elif (abs(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))>t_cut):
                print(" :: 00 : 3 :: ")
                velocity_msg.linear.x  = 0.05
                velocity_msg.angular.z = -Kp_w_z*(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))
                nav_mode = 1
            else:
                print(" :: 00 : 4 :: ")
                velocity_msg.linear.x  = 0.30
                velocity_msg.angular.z = 0.00
                nav_mode = 0

        elif (obs_mode == 1):
            if (not(obstacle)):
                if (straight_check()):
                    print(" :: 01 : 1 :: ")
                    obs_mode = 0
                else:
                    print(" :: 01 : 2 :: ")
                    obs_mode = 2
                velocity_msg.linear.x  = 0.25
            else:
                print(" :: 01 : 3 :: ")
                velocity_msg.linear.x  =  0.00
                velocity_msg.angular.z =  0.10

        elif (obs_mode == 2):
            if (obstacle):
                print(" :: 02 : 1 :: ")
                obs_mode = 1
            else:
                if (straight_check()):
                    print(" :: 02 : 2 :: ")
                    obs_mode = 0
                elif (obstacle_LT):
                    print(" :: 02 : 3 :: ")
                    velocity_msg.linear.x  =  0.18
                    velocity_msg.angular.z = -0.03
                elif (obstacle_RT):
                    print(" :: 02 : 4 :: ")
                    velocity_msg.linear.x  =  0.18
                    velocity_msg.angular.z =  0.03
                else:
                    print(" :: 02 : 5 :: ")
                    velocity_msg.linear.x  =  0.00
                    velocity_msg.angular.z =  0.00
                    obs_mode = 0

        elif (obs_mode == 50):
            if (straight_check()):
                print(" :: 50 : 1 :: ")
                obs_mode = 0
            elif (obstacle):
                print(" :: 50 : 2 :: ")
                obs_mode = 1
#            elif (abs(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))>t_cut):
#                print(" :: 50 : 3 :: ")
#                velocity_msg.linear.x  = 0.05
#                velocity_msg.angular.z = -Kp_w_z*(pose[2]-atan2(y_fin-pose[1], x_fin-pose[0]))
            elif (abs(abs(pose[2])-(np.pi/2))>t_cut):
                print(" :: 50 : 3 :: ")
                velocity_msg.linear.x  = 0.02
                velocity_msg.angular.z = -Kp_w_z*(pose[2] + ((np.pi/2)*pose[1]/abs(pose[1])))
            else:
                print(" :: 50 : 4 :: ")
                velocity_msg.linear.x  = 0.25
                velocity_msg.angular.z = 0.00

        pub.publish(velocity_msg)
        print("Obstacle   MODE   : " + str(obs_mode))
        print("Straight   CHECK  : " + str(straight_check()))
        print("Navigation MODE   : " + str(nav_mode))
        print("Obstacle   LEFT   : " + str(obstacle_LT))
        print("Obstacle   MID    : " + str(obstacle_MD))
        print("Obstacle   RIGHT  : " + str(obstacle_RT))
        print(str(pose[0]) + "," + str(pose[1]))
        print("")
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
