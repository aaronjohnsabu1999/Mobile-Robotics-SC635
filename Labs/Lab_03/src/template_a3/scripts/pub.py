#!/usr/bin/env python
import rospy
from template_a3.msg import Trilateration, Landmark

import numpy as np


def talker():
    pub = rospy.Publisher('chatter', Trilateration, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        l1 = Landmark(10.0, 0.001, 11.0)
        l2 = Landmark(11.0, 0.001, 12.0)
        l3 = Landmark(12.0, 0.001, 13.0)
        t = Trilateration(l1,l2,l3)
        #l.x = 10.0
        #l.y = 0.001
        #dist = ((l.x**2)+(l.y**2))**(0.5)
        #l.distance = dist
        rospy.loginfo("Sent a message!")
        pub.publish(t)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
