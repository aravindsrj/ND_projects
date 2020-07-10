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
import kuka_aux
import numpy as np
import math

class IK_server:
    def __init__(self):
        # Symbols

        # Joint angles
        self.q1, self.q2, self.q3, self.q4, self.q5, self.q6,\
         self.q7 = symbols('q1:8')

        # link offsets
        self.d1, self.d2, self.d3, self.d4, self.d5, self.d6,\
         self.d7 = symbols('d1:8')

        # link lengths
        self.a0, self.a1, self.a2, self.a3, self.a4, self.a5,\
         self.a6 = symbols('a0:7')

        # twist angles
        self.alpha0, self.alpha1, self.alpha2, self.alpha3, \
        self.alpha4, self.alpha5, self.alpha6 = symbols('alpha0:7')

        # roll, pitch, yaw
        self.r = symbols('r')
        self.p = symbols('p')
        self.y = symbols('y')

        # DH parameters
        self.DH = {self.alpha0: 0,    self.a0: 0,      self.d1: 0.75, self.q1: self.q1,\
            self.alpha1: -pi/2, self.a1: 0.35,   self.d2: 0,    self.q2: self.q2-pi/2,\
            self.alpha2: 0,     self.a2: 1.25,   self.d3: 0,    self.q3: self.q3,\
            self.alpha3: -pi/2, self.a3: -0.054, self.d4: 1.50, self.q4: self.q4,\
            self.alpha4: pi/2,  self.a4: 0,      self.d5: 0,    self.q5: self.q5,\
            self.alpha5: -pi/2, self.a5: 0,      self.d6: 0,    self.q6: self.q6,\
            self.alpha6: 0,     self.a6:0,       self.d7: 0.303,self.q7: 0}


    def handle_calculate_IK(self, req):
        rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
        if len(req.poses) < 1:
            print "No valid poses received"
            return -1
        else:
            # Create individual transformation matrices
            T0_to_1 = kuka_aux.Ti_to_j(self.q1, self.alpha0, self.a0, self.d1)
            T0_to_1 = T0_to_1.subs(self.DH)

            T1_to_2 = kuka_aux.Ti_to_j(self.q2, self.alpha1, self.a1, self.d2)
            T1_to_2 = T1_to_2.subs(self.DH)

            T2_to_3 = kuka_aux.Ti_to_j(self.q3, self.alpha2, self.a2, self.d3)
            T2_to_3 = T2_to_3.subs(self.DH)

            T0_to_2 = T0_to_1 * T1_to_2
            T0_to_3 = T0_to_2 * T2_to_3

            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_corr = kuka_aux.rotations('z', np.pi) * kuka_aux.rotations('y',-np.pi/2)
            Rrpy1 = kuka_aux.rotations('z',self.y) * kuka_aux.rotations('y',self.p)\
                        * kuka_aux.rotations('x',self.r) * R_corr

            T3_to_6_sym = T0_to_3.T * Rrpy1

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
            
                Rrpy = Rrpy1.subs({self.r: roll, self.p: pitch, self.y: yaw})
                #
                # Calculate joint angles using Geometric IK method

                # The homogenous transformation can be represented as:
                # [[lx, mx, nx, 0],
                #  [ly, my, ny, 0],
                #  [lz, mz, nz, 0],
                #  [0,  0,  0,  1]]
                nx, ny, nz = Rrpy.extract([0,1,2],[2])

                wx = px - (self.DH[self.d6] + self.DH[self.d7])*nx
                wy = py - (self.DH[self.d6] + self.DH[self.d7])*ny
                wz = pz - (self.DH[self.d6] + self.DH[self.d7])*nz
                
                # distance b/w joint 2 and WC
                d_2_WC_z = wz - self.DH[self.d1]
                d_2_WC_xy = math.sqrt(wx**2 + wy**2) - self.DH[self.a1]
                B = math.sqrt(d_2_WC_z**2 + d_2_WC_xy**2)

                # distance b/w joint 3 and WC
                A = math.sqrt((self.DH[self.a3]**2 + self.DH[self.d4]**2))

                # distance b/w joint 2 and joint 3
                C = self.DH[self.a2]

                # cosine law
                b = math.acos((C**2 + A**2 - B**2)/(2*C*A))
                a = math.acos((C**2 + B**2 - A**2)/(2*B*C))

                theta1 = math.atan2(wy, wx)
                theta2 = np.pi/2 - b - math.atan2(d_2_WC_z,d_2_WC_xy)
                theta3 = np.pi/2 - a + math.atan2(self.DH[self.a3],self.DH[self.d4])

                # T3_to_6 = T0_to_3.T * Rrpy
                T3_to_6 = T3_to_6_sym.subs({self.q1: theta1, self.q2: theta2, \
                            self.q3: theta3, self.r: roll, self.p: pitch, self.y: yaw})

                theta4 = math.atan2(T3_to_6[2,2], -T3_to_6[0,2])
                theta5 = math.atan2(math.sqrt(T3_to_6[0,2]**2 + T3_to_6[2,2]**2), T3_to_6[1,2])
                theta6 = math.atan2(-T3_to_6[1,1],T3_to_6[1,0])

                joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
                joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            return CalculateIKResponse(joint_trajectory_list)


if __name__ == "__main__":
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    i = IK_server()
    s = rospy.Service('calculate_ik', CalculateIK, i.handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()