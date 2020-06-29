#! /usr/bin/env python
'''Forward kinematics of SCARA manipulator using DH parameters
'''
from sympy import symbols, sin, cos, pi, simplify
from sympy.matrices import Matrix

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
q1, q2, q3, q4 = symbols('q1:5')

# link offsets
d1, d2, d3, d4 = symbols('d1:5')

# link lengths
a0, a1, a2, a3 = symbols('a0:4')

# twist angles
alpha0, alpha1, alpha2, alpha3 = symbols('alpha0:4')

a12 = 0.45 # meters
a23 = 0.30 # meters

# DH Parameters of SCARA manipulator
s = {alpha0: 0, a0: 0, d1: 0,\
     alpha1: 0, a1: a12, d2: 0,\
     alpha2: 0, a2: a23, q3: 0,\
     alpha3: 0, a3: 0, d4: 0}

T0_to_1 = Ti_to_j(q1, alpha0, a0, d1)
T0_to_1 = T0_to_1.subs(s)

T1_to_2 = Ti_to_j(q2, alpha1, a1, d2)
T1_to_2 = T1_to_2.subs(s)

T2_to_3 = Ti_to_j(q3, alpha2, a2, d3)
T2_to_3 = T2_to_3.subs(s)

T3_to_4 = Ti_to_j(q4, alpha3, a3, d4)
T3_to_4 = T3_to_4.subs(s)


# Transform from base link to end effector
T0_to_4 = simplify(T0_to_1 * T1_to_2 * T2_to_3 * T3_to_4)

print(T0_to_4.evalf(subs={q1:0, q2: 0, d3: 0, q4: 0}),'')