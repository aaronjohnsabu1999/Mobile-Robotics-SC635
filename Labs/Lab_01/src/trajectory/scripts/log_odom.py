#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from quat2euler import quat2euler

def callback(data):
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    print("Robot pose is : {}".format(pose))

def logger():
    rospy.init_node('Log_odom')
    rospy.Subscriber('/odom', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    logger()
