#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import numpy as np

from template_a4.msg import Trilateration, Landmark
from quat2euler import quat2euler

pose = [0.0, 0.0, 0.0]

landmarkA = [ 7,  7]
varA = 0.1
landmarkB = [-7, -7]
varB = 0.1
landmarkC = [ 7, -7] 
varC = 0.1

def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]

def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)


def trilateration_pub():
    global landmarkA, landmarkB, landmarkC
    rospy.init_node('Trilateration_node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)
    
    pub = rospy.Publisher('trilateration_data', Trilateration, queue_size=10)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        lA = Landmark(landmarkA[0], landmarkA[1], dist(pose, landmarkA)+np.random.normal(0,varA), varA)
        lB = Landmark(landmarkB[0], landmarkB[1], dist(pose, landmarkB)+np.random.normal(0,varB), varB)
        lC = Landmark(landmarkC[0], landmarkC[1], dist(pose, landmarkC)+np.random.normal(0,varC), varC)
        t = Trilateration(lA, lB, lC)
        rospy.loginfo("Sent a message!")
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        trilateration_pub()
    except rospy.ROSInterruptException:
        pass
