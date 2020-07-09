from sympy import symbols, sin, cos, pi, simplify
from sympy.matrices import Matrix
# def DH_Params(alphas, linkLens, offsets, joints):
#     alpha0,alpha1,alpha2
#     s = {alpha0: 0,     a0: 0,      d1: 0.75, q1: q1,\
#      alpha1: -pi/2, a1: 0.35,   d2: 0,    q2: q2-pi/2,\
#      alpha2: 0,     a2: 1.25,   d3: 0,    q3: q3,\
#      alpha3: -pi/2, a3: -0.054, d4: 1.50, q4: q4,\
#      alpha4: pi/2,  a4: 0,      d5: 0,    q5: q5,\
#      alpha5: -pi/2, a5: 0,      d6: 0,    q6: q6,\
#      alpha6: 0,     a6:0,       d7: 0.303, q7: 0}

#     return s

def Ti_to_j(q, alpha, a, d):
    '''Transformation from i frame to j frame using DH parameters

    Returns: 4x4 transformation matrix
    '''
    Ri_to_j = _rot(q, alpha)
    Ti2j = Ri_to_j.row_join(Matrix([a, -sin(alpha)*d, cos(alpha)*d]))
    Ti2j = Ti2j.col_join(Matrix([[0, 0, 0, 1]]))

    return Ti2j

def _rot(q, alpha):
    R_z = Matrix([[cos(q),-sin(q), 0],\
                    [sin(q),cos(q), 0],\
                    [0,0,1]])

    R_x = Matrix([[1,0,0],\
                [0, cos(alpha), -sin(alpha)],\
                [0, sin(alpha), cos(alpha)]])

    return R_x * R_z

def rotations(axis, angle):
    '''4x4 rotation matrix along x, y or z axes

    parameters:
        'axis' should be either 'x', 'y' or 'z'
        'angle' is the angle (in radians) to be rotated

    returns: 4x4 rotation matrix
    '''
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