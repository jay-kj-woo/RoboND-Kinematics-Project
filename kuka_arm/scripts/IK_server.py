#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')   #theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        
        # Create Modified DH parameters
        dh = {alpha0:     0,    a0:      0,   d1:  0.75,       
              alpha1: -pi/2,    a1:   0.35,   d2:     0,   q2:q2-pi/2,
              alpha2:     0,    a2:   1.25,   d3:     0,
              alpha3: -pi/2,    a3: -0.054,   d4:   1.5,
              alpha4:  pi/2,    a4:      0,   d5:     0,
              alpha5: -pi/2,    a5:      0,   d6:     0,
              alpha6:     0,    a6:      0,   d7: 0.303,   q7:0}
        #
        # Define Modified DH Transformation matrix
        def TF_matrix(alpha, a, d, q):
            TF = Matrix([[           cos(q),           -sin(q),           0,             a],
                         [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                         [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                         [                0,                 0,           0,             1]])
            return TF

        
        #
        # Create individual transformation matrices
        T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(dh)
        T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(dh)
        T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(dh)
        T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(dh)
        T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(dh)
        T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(dh)
        T6_G = TF_matrix(alpha6, a6, d7, q7).subs(dh)
        
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

   
	    #
	    # Extract rotation matrices from the transformation matrices
	    #
	    #
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            # first get the orientation from roll, pitch, and yaw angles     	        
            R, P, Y = symbols('R P Y')
            R_X = Matrix([[1,            0,        0],
                          [0,       cos(R),  -sin(R)],
                          [0,       sin(R),   cos(R)]])      #roll
            R_Y = Matrix([[ cos(P),      0,   sin(P)],
                          [      0,      1,        0],
                          [-sin(P),      0,   cos(P)]])      #pitch
            R_Z = Matrix([[cos(Y), -sin(Y),        0],
                          [sin(Y),  cos(Y),        0],
                          [     0,       0,        1]])      #yaw
            R_G = R_Z * R_Y * R_X
            
            # second, make correction for DH vs URDF
            R_corr = R_Z.subs(Y,radians(180)) * R_Y.subs(P,radians(-90))
            R_G = R_G * R_corr
            
            # evalueate at the given rpy
            R_G = R_G.subs({'R': roll, 'P':pitch, 'Y':yaw})
            G = Matrix([[px],
                        [py],
                        [pz]])
            WC = G - (0.303)*R_G[:,2]
            WCx = WC[0]
            WCy = WC[1]
            WCz = WC[2]
            
            

            # Calculate joint angles using Geometric IK method
            # theta 1
            theta1 = atan2(WCy, WCx)

            # theta 2 and 3
            sideA = 1.501
            sideB = sqrt((sqrt(WCx*WCx + WCy*WCy) -0.35)**2 + (WCz-0.75)**2)
            sideC = 1.25
            angleA = acos((sideB*sideB + sideC**2 - sideA**2)/(2*sideB*sideC))
            angleB = acos((sideA**2 + sideC**2 - sideB**2)/(2*sideA*sideC))
            angleWC2base = atan2((WCz-0.75),sqrt(WCx**2 + WCy**2) - 0.35)
            angleWC2link3 = atan2(0.054,1.5)   
                              
            
            theta2 = pi/2 - angleA - angleWC2base
            theta3 = pi/2 - angleB - angleWC2link3            

            # theta 4, 5, and 6
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.transpose() * R_G

            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
