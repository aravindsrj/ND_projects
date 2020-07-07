#! /usr/bin/env python
'''Forward kinematics of KUKA arm using DH parameters
'''
from sympy import symbols, sin, cos, pi, simplify
from sympy.matrices import Matrix
import numpy as np

def rotations(q, alpha):
    R_z = Matrix([[cos(q),-sin(q), 0],\
                    [sin(q),cos(q), 0],\
                    [0,0,1]])

    R_x = Matrix([[1,0,0],\
                [0, cos(alpha), -sin(alpha)],\
                [0, sin(alpha), cos(alpha)]])

    return R_x * R_z

def Ti_to_j(q, alpha, a, d):

    Ri_to_j = rotations(q, alpha)
    Ti2j = Ri_to_j.row_join(Matrix([a, -sin(alpha)*d, cos(alpha)*d]))
    Ti2j = Ti2j.col_join(Matrix([[0, 0, 0, 1]]))

    return Ti2j


# Joint angles
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')

# link offsets
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')

# link lengths
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')

# twist angles
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

# DH parameters
s = {alpha0: 0,     a0: 0,      d1: 0.75,\
     alpha1: -pi/2, a1: 0.35,   d2: 0,    q2: q2-pi/2,\
     alpha2: 0,     a2: 1.25,   d3: 0,\
     alpha3: -pi/2, a3: -0.054, d4: 1.50,\
     alpha4: pi/2,  a4: 0,      d5: 0,\
     alpha5: -pi/2, a5: 0,      d6: 0,\
     alpha6: 0,     a6:0,       d7: 0.303, q7: 0}

T0_to_1 = Ti_to_j(q1, alpha0, a0, d1)
T0_to_1 = T0_to_1.subs(s)

T1_to_2 = Ti_to_j(q2, alpha1, a1, d2)
T1_to_2 = T1_to_2.subs(s)

T2_to_3 = Ti_to_j(q3, alpha2, a2, d3)
T2_to_3 = T2_to_3.subs(s)

T3_to_4 = Ti_to_j(q4, alpha3, a3, d4)
T3_to_4 = T3_to_4.subs(s)

T4_to_5 = Ti_to_j(q5, alpha4, a4, d5)
T4_to_5 = T4_to_5.subs(s)

T5_to_6 = Ti_to_j(q6, alpha5, a5, d6)
T5_to_6 = T5_to_6.subs(s)

T6_to_G = Ti_to_j(q7, alpha6, a6, d7)
T6_to_G = T6_to_G.subs(s)

T0_to_2 = T0_to_1 * T1_to_2
T0_to_3 = T0_to_2 * T2_to_3
T0_to_4 = T0_to_3 * T3_to_4
T0_to_5 = T0_to_4 * T4_to_5
T0_to_6 = T0_to_5 * T5_to_6
T0_to_G = T0_to_6 * T6_to_G

# Joint angles
angles = {q1: 1.0, q2: 0, q3: 0, q4: 1.0, q5: 0, q6: 0}

# print('T0_1 = ', T0_to_1.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
# print('T0_2 = ', T0_to_2.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
# print('T0_3 = ', T0_to_3.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
# print('T0_4 = ', T0_to_4.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))
# print('T0_5 = ', T0_to_5.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}))


# print(T2_to_3.evalf(subs={q1:0,q2:0,q3:0,q4:0,q5:0,q6:0}),'')
# Corrections for difference between definitions of link in URDF and DH convention
R_z = Matrix([[cos(np.pi), -sin(np.pi), 0, 0],\
              [sin(np.pi), cos(np.pi),  0, 0],\
              [0,          0,           1, 0],\
              [0,          0,           0, 1]])

R_y = Matrix([[cos(-np.pi/2),  0, sin(-np.pi/2), 0],\
              [0,              1, 0,             0],\
              [-sin(-np.pi/2), 0, cos(-np.pi/2), 0],\
              [0,              0, 0,             1]])

R_corr = R_z * R_y

T_total = T0_to_G * R_corr

print('T0_G = ', T_total.evalf(subs=angles))