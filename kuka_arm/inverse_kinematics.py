#! /usr/bin/env python
from sympy import symbols, sin, cos, pi, simplify
from sympy.matrices import Matrix
import numpy as np
from tf import transformations
import tf

def rotations(axis, angle):
    if axis == 'x':
        R_x = Matrix([[1, 0,          0,           0],\
                      [0, cos(angle), -sin(angle), 0],\
                      [0, sin(angle), cos(angle),  0],\
                      [0, 0,          0,           1]])
        return R_x
    
    if axis == 'y':
        R_y = Matrix([[cos(angle),  0, sin(angle), 0],\
                      [0,           1, 0,          0],\
                      [-sin(angle), 0, cos(angle), 0],\
                      [0,           0, 0,          1]])
        return R_y
    
    if axis == 'z':
        R_z = Matrix([[cos(angle), -sin(angle), 0, 0],\
                      [sin(angle), cos(angle),  0, 0],\
                      [0,          0,           1, 0],\
                      [0,          0,           0, 1]])
        return R_z
    
    return []

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

# Corrections for difference between definitions of link in URDF and DH convention
R_corr = rotations('z', np.pi) * rotations('y',-np.pi/2)

Rrpy = rotations('z',yaw) * rotations('y',pitch) * rotations('x',roll) * R_corr