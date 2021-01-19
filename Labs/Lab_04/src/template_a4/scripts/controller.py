#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
import math
from math import atan, atan2, pi
from nav_msgs.msg import Odometry
from template_a4.msg import Trilateration
from geometry_msgs.msg import Twist
from quat2euler import quat2euler, euler2quat
from trilateration import varA, varB, varC, landmarkA, landmarkB, landmarkC

## Define global variables
pose      = [0.0, 0.0, 0.0]
prev_pose = [0.0, 0.0, 0.0]
input1    = 0.0
input2    = 0.0
Kp        = 0.40
Kp_w_z    = 0.15

## System noise related
noisy_pose = [0.0, 0.0, 0.0]
varTHETA   = 0.01
varX       = 0.05
varY       = 0.05

Q = np.diag([varX, varY, varTHETA])
R = np.diag([varA, varB, varC])   ## Imported from trilateration
P = np.diag([0.5, 0.5, 0.5])      ## Some reasonable initial values
F = np.eye(3)                     ## System matrix for discretized unicycle is Identity 
H = np.zeros((3,2)) #get_current_H(pose, landmarkA, landmarkB, landmarkC)  ## H has to be calculated on the fly

distanceLandmarkA = 10.0
distanceLandmarkB = 10.0
distanceLandmarkC = 10.0

FILTER_ORDER = 5     ## Change filter settings
filter_a     = [0 for i in range(FILTER_ORDER)]
filter_b     = [0 for i in range(FILTER_ORDER)]
filter_c     = [0 for i in range(FILTER_ORDER)]

idxA  = 0
idxB  = 0
idxC  = 0

theta = 0

odoo  = Odometry()
depub = ''

def dist(p1, p2):
    ### Given a pair of points the function returns euclidean distance
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)

def get_current_H(pose, lA, lB, lC):
    ### Calculate the linearized measurement matrix H(k+1|k) at the current robot pose

    ## x and y co-ordinates of landmarks
    xA, yA = lA
    xB, yB = lB
    xC, yC = lC
    ## current robot pose
    x = pose[0]
    y = pose[1]
    print("")
    print("Printing x:")
    print(x)
    print("")
    print("Printing y:")
    print(y)
    ## Linearized H
    H = [[x-xA, y-yA, 0],[x-xB, y-yB, 0],[x-xC, y-yC, 0]]
    print("")
    print("Printing H matrix:")
    print(H)
    H = np.array(H).reshape(3,3)
    print("")
    print("Printing H:")
    print(H)
    return 2*H

def predict_state():
    ### System evolution
    return noisy_pose

def predict_measurement(predicted_pose, landmark_A, landmark_B, landmark_C):
    ### Predicts the measurement (d1, d2, d3) given the current position of the robot
    d1 = dist(predicted_pose, landmark_A)
    d2 = dist(predicted_pose, landmark_B)
    d3 = dist(predicted_pose, landmark_C)
    measurement = [d1, d2, d3]
    measurement = np.array(measurement).reshape(3,1)
    return measurement

def callback2(data):
    global noisy_pose, varX, varY, varTHETA
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z  = data.pose.pose.orientation.z;
    w  = data.pose.pose.orientation.w;
    noise = [np.random.normal(0,varX), np.random.normal(0,varY), np.random.normal(0,varTHETA)]
    noisy_pose = np.array([data.pose.pose.position.x + noise[0], data.pose.pose.position.y + noise[1], quat2euler(x,y,z,w)[2] + noise[2]]).reshape(3,1)

def callback(data):
    global distanceLandmarkA, distanceLandmarkB, distanceLandmarkC
    global idxA, idxB, idxC
    global filter_a, filter_b, filter_c
    global prev_pose, theta, pose
    global P, Q, R, F

    lA = data.landmarkA
    lB = data.landmarkB
    lC = data.landmarkC

    ##############################################################
    ################### FILTER MODIFICATION ######################

    filter_a[idxA] = lA.distance
    filter_b[idxB] = lB.distance
    filter_c[idxC] = lC.distance

    idxA += 1
    idxB += 1
    idxC += 1

    if idxA>=FILTER_ORDER:
        idxA = 0
    if idxB>=FILTER_ORDER:
        idxB = 0
    if idxC>=FILTER_ORDER:
        idxC = 0

    ##############################################################
    ##############################################################

    ## Calculate filtered measurements (d1, d2, d3)  
    x1, y1 =  7, 7
    x2, y2 = -7,-7
    x3, y3 =  7,-7
    d1, d2, d3 = sum(filter_a)/FILTER_ORDER, sum(filter_b)/FILTER_ORDER, sum(filter_c)/FILTER_ORDER 

    Y_measured = np.matrix([[d1],[d2],[d3]])

    ##############################################################
    ################### EXTENDED KALMAN FILTER ###################

    ## State Prediction
    x_temp = predict_state()
    ## Covariance Prediction
    P_temp = np.dot(np.dot(F, P), np.transpose(F)) + Q

    H = get_current_H(pose, [lA.x, lA.y], [lB.x, lB.y], [lC.x, lC.y])
    ## Measurement Residual
    v = Y_measured - predict_measurement(x_temp, [lA.x, lA.y], [lB.x, lB.y], [lC.x, lC.y])
    ## Measurement Residual Covariance
    S = np.dot(np.dot(H, P_temp), np.transpose(H)) + R
    ## Kalman Gain
    W = np.dot(np.dot(P_temp, np.transpose(H)), np.linalg.inv(S))

    ## State Update
    pose = x_temp + np.transpose(np.dot(W, v))
    pose = np.transpose(pose)

    ## Covariance Update
    P    = P_temp - np.dot(np.dot(W, S), np.transpose(W))

    ##############################################################
    ##############################################################

    print("x:{}, \ty:{}, \ttheta:{}\n".format(pose[0], pose[0], pose[0]*(180/pi)))

    odoo.pose.pose.position.x = pose[0]
    odoo.pose.pose.position.y = pose[1]
    quaternion_val = euler2quat(0,0,pose[2])
    odoo.pose.pose.orientation.x = quaternion_val[0]
    odoo.pose.pose.orientation.y = quaternion_val[1]
    odoo.pose.pose.orientation.z = quaternion_val[2]
    odoo.pose.pose.orientation.w = quaternion_val[3]
    depub.publish(odoo)

def waypoint(t):
    x = math.cos(t*1*np.pi/180)
    y = math.sin(t*1*np.pi/180)
    return [x,y]

def map_angle(theta):
    if(theta > np.pi):
        theta = -(2*np.pi-theta)
    elif(theta < -np.pi):
        theta = (2*np.pi+theta)
    return theta

def control_loop():
    global pose, range_data, depub, input1, input2
    
    rospy.init_node('controller_node')
    pub   = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rospy.Subscriber('/trilateration_data', Trilateration, callback)
    rospy.Subscriber('/odom', Odometry, callback2)
    depub = rospy.Publisher('/odom2', Odometry, queue_size=10)
    rate  = rospy.Rate(5) 
    
    ## Twist values to move the robot
    velocity_msg   = Twist()

    t     = 0
    d_err = 2.0
    rot   = False
    while not rospy.is_shutdown():
        [x_fin, y_fin] = waypoint(t)
        x_err          = x_fin-float(pose[0])
        y_err          = y_fin-pose[1]
        d_err          = (x_err**2 + y_err**2)**0.5
        theta_ref      = math.atan2(y_err, x_err)
        theta_err      = map_angle(theta_ref - pose[2])

        if(abs(theta_err) > (2.0*np.pi/180)):
            if rot:
                input2 = -Kp_w_z*theta_err
            else:
                input1 = 0.10
            rot = not rot
        else:
            input1 = Kp*d_err

        velocity_msg.linear.x  = input1
        velocity_msg.angular.z = input2

        if d_err < 0.3:
            velocity_msg = Twist()
            t = t + 1

        print(str(t))
        pub.publish(velocity_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        control_loop()
    except rospy.ROSInterruptException:
        pass
