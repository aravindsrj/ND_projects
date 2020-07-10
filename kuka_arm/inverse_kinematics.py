#! /usr/bin/env python
from sympy import symbols, sin, cos, pi, simplify
from sympy.matrices import Matrix
import numpy as np
from tf import transformations
# import tf
import math
import kuka_aux

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

r = symbols('r')
p = symbols('p')
y = symbols('y')

# Test cases format:
# [[[EE position], [EE orientation as quaternions]], [WC location], [Joint angles]]
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
              4:[[[2.1529,0,1.9465],
                  [0, -0.00014835, 0, 1]],
                  [1.8499,0,1.9465],
                  [0,0,0,0,0,0]],
              5:[]}

# End effector positions
case = 1
px, py, pz = test_cases[case][0][0]

# Corrections for difference between definitions of link in URDF and DH convention
R_corr = kuka_aux.rotations('z', np.pi) * kuka_aux.rotations('y',-np.pi/2)

# Get rpy from quaternions
roll, pitch, yaw = transformations.euler_from_quaternion(test_cases[case][0][1])

# Calculating end effector pose with respect to base link
# Rrpy = kuka_aux.rotations('z',yaw) * kuka_aux.rotations('y',pitch)\
#          * kuka_aux.rotations('x',roll) * R_corr

Rrpy1 = kuka_aux.rotations('z',y) * kuka_aux.rotations('y',p)\
         * kuka_aux.rotations('x',r) * R_corr

Rrpy = Rrpy1.subs({r: roll, p: pitch, y: yaw})

# n is the vector along z axis of the gripper
#
# The homogenous transformation can be represented as:
# [[lx, mx, nx, 0],
#  [ly, my, ny, 0],
#  [lz, mz, nz, 0],
#  [0,  0,  0,  1]]
nx, ny, nz = Rrpy.extract([0,1,2],[2])

# w is the wrist center position
# p is the end effector position
# s[d6] is link offset (d6)
# s[d7] is the end-effector length
wx = px - (s[d6] + s[d7])*nx
wy = py - (s[d6] + s[d7])*ny
wz = pz - (s[d6] + s[d7])*nz

print(wx, wy, wz)

theta1 = math.atan2(wy, wx)

# distance b/w joint 2 and WC
d_2_WC_z = wz - s[d1]
d_2_WC_xy = math.sqrt(wx**2 + wy**2) - s[a1]
B = math.sqrt(d_2_WC_z**2 + d_2_WC_xy**2)

# distance b/w joint 3 and WC
A = math.sqrt((s[a3]**2 + s[d4]**2))

# distance b/w joint 2 and joint 3
C = s[a2]

print(A,B,C)

# cosine law
b = math.acos((C**2 + A**2 - B**2)/(2*C*A))
a = math.acos((C**2 + B**2 - A**2)/(2*B*C))

theta2 = np.pi/2 - b - math.atan2(d_2_WC_z,d_2_WC_xy)
theta3 = np.pi/2 - a + math.atan2(s[a3],s[d4])

T0_to_1 = kuka_aux.Ti_to_j(q1, alpha0, a0, d1)
T0_to_1 = T0_to_1.subs(s)

T1_to_2 = kuka_aux.Ti_to_j(q2, alpha1, a1, d2)
T1_to_2 = T1_to_2.subs(s)

T2_to_3 = kuka_aux.Ti_to_j(q3, alpha2, a2, d3)
T2_to_3 = T2_to_3.subs(s)

T0_to_2 = T0_to_1 * T1_to_2
T0_to_3 = T0_to_2 * T2_to_3

T3_to_6 = T0_to_3.T * Rrpy
T3_to_6 = T3_to_6.subs({q1: theta1, q2: theta2, q3: theta3})

theta4 = math.atan2(T3_to_6[2,2], -T3_to_6[0,2])
theta5 = math.atan2(math.sqrt(T3_to_6[0,2]**2 + T3_to_6[2,2]**2), T3_to_6[1,2])
theta6 = math.atan2(-T3_to_6[1,1],T3_to_6[1,0])

print(wx, wy, wz)
print(theta1, theta2, theta3, theta4, theta5, theta6)