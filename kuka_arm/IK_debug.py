#! /usr/bin/env python
from sympy import *
from time import time
from mpmath import radians
import tf
import math
import kuka_aux
import numpy as np
from tf import transformations

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)

    ########################################################################################
    # Setting up symbols

    # Joint angles
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

    # link offsets
    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

    # link lengths
    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

    # twist angles
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

    # DH parameters
    s = {alpha0: 0,     a0: 0,      d1: 0.75, q1: q1,\
        alpha1: -pi/2, a1: 0.35,   d2: 0,    q2: q2-pi/2,\
        alpha2: 0,     a2: 1.25,   d3: 0,    q3: q3,\
        alpha3: -pi/2, a3: -0.054, d4: 1.50, q4: q4,\
        alpha4: pi/2,  a4: 0,      d5: 0,    q5: q5,\
        alpha5: -pi/2, a5: 0,      d6: 0,    q6: q6,\
        alpha6: 0,     a6:0,       d7: 0.303, q7: 0}

    r = symbols('r') # roll
    p = symbols('p') # pitch
    y = symbols('y') # yaw

    # Corrections for difference between definitions of link in URDF and DH convention
    R_corr = kuka_aux.rotations('z', np.pi) * kuka_aux.rotations('y',-np.pi/2)
    Rrpy1 = kuka_aux.rotations('z',y) * kuka_aux.rotations('y',p)\
         * kuka_aux.rotations('x',r) * R_corr

    T0_to_1 = kuka_aux.Ti_to_j(q1, alpha0, a0, d1)
    T0_to_1 = T0_to_1.subs(s)

    T1_to_2 = kuka_aux.Ti_to_j(q2, alpha1, a1, d2)
    T1_to_2 = T1_to_2.subs(s)

    T2_to_3 = kuka_aux.Ti_to_j(q3, alpha2, a2, d3)
    T2_to_3 = T2_to_3.subs(s)

    T3_to_4 = kuka_aux.Ti_to_j(q4, alpha3, a3, d4)
    T3_to_4 = T3_to_4.subs(s)

    T4_to_5 = kuka_aux.Ti_to_j(q5, alpha4, a4, d5)
    T4_to_5 = T4_to_5.subs(s)

    T5_to_6 = kuka_aux.Ti_to_j(q6, alpha5, a5, d6)
    T5_to_6 = T5_to_6.subs(s)

    T6_to_G = kuka_aux.Ti_to_j(q7, alpha6, a6, d7)
    T6_to_G = T6_to_G.subs(s)

    T0_to_2 = T0_to_1 * T1_to_2
    T0_to_3 = T0_to_2 * T2_to_3
    T0_to_4 = T0_to_3 * T3_to_4
    T0_to_5 = T0_to_4 * T4_to_5
    T0_to_6 = T0_to_5 * T5_to_6
    T0_to_G = T0_to_6 * T6_to_G

    T3_to_6_sym = T0_to_3.T * Rrpy1

    WC_transform = T0_to_5 * R_corr
    FK_transform = T0_to_G * R_corr

    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    px, py, pz = req.poses[0].position.x, req.poses[0].position.y, req.poses[0].position.z    

    # Get rpy from quaternions
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([req.poses[0].orientation.x,\
                        req.poses[0].orientation.y, req.poses[0].orientation.z, req.poses[0].orientation.w])

    Rrpy = Rrpy1.subs({r: roll, p: pitch, y: yaw})

    # n is the vector along z axis of the gripper
    #
    # The homogenous transformation can be represented as:
    # [[lx, mx, nx, 0],
    #  [ly, my, ny, 0],
    #  [lz, mz, nz, 0],
    #  [0,  0,  0,  1]]
    nx, ny, nz = Rrpy.extract([0,1,2],[2])

    wx = px - (s[d6] + s[d7])*nx
    wy = py - (s[d6] + s[d7])*ny
    wz = pz - (s[d6] + s[d7])*nz

    theta1 = math.atan2(wy, wx)

    #     # Find the distances between joint 2 and the wrist center
    # L2_WC = sqrt(X2_WC**2 + Y2_WC**2 + Z2_WC**2)

    # # Find the distance between joint 3 and the wrist center
    # L3_4 = 0.96     # Distance from joint 3 to joint 4
    # L4_5 = 0.54     # Distance from joint 4 to joint 5 (WC)
    # L3_4_x = sqrt(L3_4**2 + abs(s[a3])**2)  # X distance from joint 3 to joint 4
    # phi1 = pi - atan2(abs(s[a3]), L3_4_x)
    # L3_WC = sqrt(L3_4**2 + L4_5**2 - 2 * L3_4 * L4_5 * cos(phi1))

    # # Determine the angle for joint 3
    # cos_phi2 = (L2_WC**2 - L3_WC**2 - s[a2]**2) / (-2 * L3_WC * s[a2])
    # if abs(cos_phi2) > 1:
    #     cos_phi2 = 1
    #     #print('cos_phi2 is greater than 1.')
    # phi2 = atan2(sqrt(1 - cos_phi2**2), cos_phi2)
    # theta3 = (pi/2 - phi2).evalf()
    # theta3 = np.clip(theta3, -210*dtr, (155-90)*dtr)

    # # Determine the angle for joint 2
    # L2_WC_xy = sqrt(X2_WC**2 + Y2_WC**2)
    # phi3 = atan2(Z2_WC, L2_WC_xy)
    # cos_phi4 = (L3_WC**2 - L2_WC**2 - s[a2]**2) / (-2 * L2_WC * s[a2])
    # if abs(cos_phi4) > 1:
    #     cos_phi4 = 1
    # phi4 = atan2(sqrt(1 - cos_phi4**2), cos_phi4)
    # theta2 = (pi/2 - (phi3 + phi4)).evalf()
    # theta2 = np.clip(theta2, -45*dtr, 85*dtr)

    # distance b/w joint 2 and WC
    d_2_WC_z = wz - s[d1]
    d_2_WC_xy = math.sqrt(wx**2 + wy**2) - s[a1]
    B = math.sqrt(d_2_WC_z**2 + d_2_WC_xy**2)

    # distance b/w joint 3 and WC
    A = math.sqrt((s[a3]**2 + s[d4]**2))

    # distance b/w joint 2 and joint 3
    C = s[a2]
    
    # cosine law
    b = math.acos((C**2 + A**2 - B**2)/(2*C*A))
    a = math.acos((C**2 + B**2 - A**2)/(2*B*C))

    theta2 = np.pi/2 - b - math.atan2(d_2_WC_z,d_2_WC_xy)
    theta3 = np.pi/2 - a + math.atan2(s[a3],s[d4])
    # theta3 = np.clip(theta3, -210*np.pi/180, (155-90)*np.pi/180)

    # T3_to_6 = T3_to_6_sym.subs(s)
    T3_to_6 = T3_to_6_sym.subs({r: roll, y: yaw, p: pitch})
    T3_to_6 = T3_to_6.subs({q1: theta1, q2: theta2, q3: theta3})

    theta4 = math.atan2(T3_to_6[2,2], -T3_to_6[0,2])
    theta5 = math.atan2(math.sqrt(T3_to_6[0,2]**2 + T3_to_6[2,2]**2), T3_to_6[1,2])
    theta6 = math.atan2(-T3_to_6[1,1],T3_to_6[1,0])

    
    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    angles = {q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6}
    FK_transform = FK_transform.subs(angles)

    ee0,ee1,ee2 = FK_transform.extract([0,1,2],[3])

    wx = ee0 - (s[d6] + s[d7])*nx
    wy = ee1 - (s[d6] + s[d7])*ny
    wz = ee2 - (s[d6] + s[d7])*nz

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [wx, wy, wz] # <--- Load your calculated WC values in this array
    
    
    your_ee = [ee0,ee1,ee2] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################    

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 1

    test_code(test_cases[test_case_number])
