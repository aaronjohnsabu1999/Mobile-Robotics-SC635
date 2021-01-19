#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from quat2euler import quat2euler

from math import cos, sin, pi
import numpy as np

import matplotlib.pyplot as plt
from controller import K_samp, get_waypoint

plt.ion() 



LIM=10
fig, ax = plt.subplots(1, 1)
#ax.set_autoscaley_on(True)
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_xlim([-LIM/2, LIM/2])
ax.set_ylim([-LIM/3, LIM/3])
ax.grid()
ax.legend()
line_waypoints = [[3,0]]
line_poses = []

tracking_points = []

line_waypoints, = ax.plot([], [], 'g^', label="waypoint", ms=5)
line_poses2, = ax.plot([],[],'r', lw=3 , alpha=0.9 )
line_poses, = ax.plot([],[],'ro', label="robot", ms=15.0, alpha=0.8)
track, = ax.plot([],[],'b:', lw=2, alpha=0.65)
    

def pose_listener( data):
    global line_poses, line_poses2, line_poses2, X_track, Y_track, tracking_points
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    print("Pose : {:0.2f},{:0.2f},{:0.2f}".format(pose[0], pose[1], pose[2]))
    delta = 0.20
    line_poses.set_data([pose[0]], [pose[1]])

    dx = delta*cos(pose[2])
    dy = delta*sin(pose[2])

    line_poses2.set_data([pose[0], pose[0]+dx], [pose[1], pose[1]+dy])
    
    ## save the tracks to plot later
    tracking_points.append(pose)
    

def waypoint_listener( data):
    global line_waypoints
    waypoint = eval(data.data)
    print("The type of data :{}",type(waypoint))
    line_waypoints.set_data( waypoint[0], waypoint[1])
            
def process():
    
    rospy.init_node('plotting_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, pose_listener)
    rospy.Subscriber('/bot_0/waypoint', String, waypoint_listener)
    
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():
        fig.canvas.draw()
        fig.canvas.flush_events()
        
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Process started ")
        process()
    except rospy.ROSInterruptException:
        pass
    finally:
        #plt.plot(np.array(tracking_points)[:,0], np.array(tracking_points)[:,1], 'r:', alpha=0.7, label='trajectory')
        fig, ax = plt.subplots(1)
        Num_Pts = int(2*pi/K_samp)+1
        T = [i for i in range(Num_Pts)]
        W = [get_waypoint(t) for t in T]
        for w in W:
            ax.plot(w[0],w[1],'bs')
        ax.plot(w[0], w[1], 'bs', label="Waypoints")

        plt.xlabel('X (in meters)')
        plt.ylabel('Y (in meters)')
        plt.title('Robot Trajectory')
        plt.savefig('Robot_Trajectory.png')
