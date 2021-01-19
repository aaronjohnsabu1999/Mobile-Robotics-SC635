#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from quat2euler import quat2euler


import numpy
import time
from math import sin, cos, atan2, pi
import matplotlib.pyplot as plt


pose = [0.0, 0.0, 0.0]
### This parameter controls how close we want to generate the waypoints
K_samp = 0.07

### This function will give access to robots current position in `pose` variable
def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, quat2euler(x,y,z,w)[2]]
    

### this function will help in sampling the waypoints
def get_waypoint(t):
    global K_samp       ## defined at line 17
    A =3 
    B =3
    a =1
    b =2
    x  = A*cos(a*t*K_samp)
    y  = B*sin(b*t*K_samp)
    return [x,y] 

### Plot the waypoints :   uses K_samp defined in line 17
def plot_waypoints():
    global K_samp
    
    fig, ax = plt.subplots(1,1)
    Num_Pts = int(2*pi/K_samp)+1    # Number of points in 1 complete iteration

    W = [get_waypoint(i) for i in range(Num_Pts)] # Sampling points
    for w in W:
        ax.plot(w[0],w[1],'bs')
        
    ax.plot(w[0], w[1], 'bs', label="Waypoints")
    ax.set_xlabel(r'x (in meter)')
    ax.set_ylabel(r'y (in meter)')
    ax.set_title('Waypoints with K_samp={}'.format(K_samp))
    plt.xlim([-3.5, 3.5])
    plt.ylim([-3.5, 3.5])
    plt.savefig('waypoints_{}_Ksamp.png'.format(K_samp))

### Map angle to within [-pi, pi]
def map_angle(theta):
    if(theta > pi):
        theta = -(2*pi-theta)
    elif(theta < -pi):
        theta = (2*pi+theta)
    return theta
    

### Saturate control to reasonable values
def sat(x, Th):
    if x<-Th:
        return -Th
    if x > Th:
        return Th
    return x


def control_loop():
    ## Routine tasks
    rospy.init_node('turtlebot_trajectory_tracker')
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    pubw = rospy.Publisher('/bot_0/waypoint', String, queue_size=10)

    rospy.Subscriber('/odom', Odometry, callback)
    rate = rospy.Rate(10) 
    
    #### Uncomment the line to save the Waypont plot
    ## plot_waypoints()
    ################################################

    ## SAMPLE 1 WAYPONT
    i = 0
    wp= get_waypoint(i)
    dist_error = 2  # some random value
    
    while not rospy.is_shutdown():
    
        ## When close to waypoint, sample a new waypoint
        if dist_error < 0.3:
            i = i+1
            wp = get_waypoint(i)
        


        ### Compute errors
        x_err = wp[0]-pose[0]
        y_err = wp[1]-pose[1]
        theta_ref = atan2(y_err, x_err)
        
        dist_error = (x_err**2 + y_err**2)**0.5
        
        theta_err = map_angle(theta_ref - pose[2])

        ### Debug string 
        print("\n heading:{:0.5f},\tref:{:0.5f},\terror:{:0.5f}".format(pose[2], theta_ref, theta_err))
        
        ### Apply the proportional control
        K1=0.4  ## not aggressive
        K2=2.0  ## aggressive
        
        
        velocity_msg = Twist()
        velocity_msg.linear.x = sat(K1*dist_error*cos(theta_err), 0.25)
        velocity_msg.angular.z = sat(K2*theta_err, 0.5)
        
        
        pub.publish(velocity_msg)

        ### for dynamic plot
        pubw.publish("[{},{}]".format(wp[0], wp[1]))
        
        #print("Controller message pushed at {}".format(rospy.get_time()))
        rate.sleep()

if __name__ == '__main__':
    try:
        print("Waiting for gazebo to start")
        time.sleep(5)
        print("Starting the control loop")
        control_loop()
    except rospy.ROSInterruptException:
        pass
