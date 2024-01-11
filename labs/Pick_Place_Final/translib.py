import numpy as np
from math import sin,cos,pi
def trans(d):
    """
    Compute pure translation homogenous transformation
    """
    return np.array([
        [ 1, 0, 0, d[0] ],
        [ 0, 1, 0, d[1] ],
        [ 0, 0, 1, d[2] ],
        [ 0, 0, 0, 1    ],
    ])

def roll(a):
    """
    Compute homogenous transformation for rotation around x axis by angle a
    """
    return np.array([
        [ 1,     0,       0,  0 ],
        [ 0, cos(a), -sin(a), 0 ],
        [ 0, sin(a),  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def pitch(a):
    """
    Compute homogenous transformation for rotation around y axis by angle a
    """
    return np.array([
        [ cos(a), 0, -sin(a), 0 ],
        [      0, 1,       0, 0 ],
        [ sin(a), 0,  cos(a), 0 ],
        [ 0,      0,       0, 1 ],
    ])

def yaw(a):
    """
    Compute homogenous transformation for rotation around z axis by angle a
    """
    return np.array([
        [ cos(a), -sin(a), 0, 0 ],
        [ sin(a),  cos(a), 0, 0 ],
        [      0,       0, 1, 0 ],
        [      0,       0, 0, 1 ],
    ])

def transform(d,rpy):
    """
    Helper function to compute a homogenous transform of a translation by d and
    rotation corresponding to roll-pitch-yaw euler angles
    """
    return trans(d) @ roll(rpy[0]) @ pitch(rpy[1]) @ yaw(rpy[2])

# def swap(a,b):
#     #swap any two variables
#     inter = 0
#     inter = a
#     a=b
#     b=inter
#     return a,b
# def z_swap(a,eps):
#     #input: a is rotation matrix, eps is acceptable error
#     #output: transformed rotation matrix
#     column_no=2
#     for i in range(3):
#         #check which column has [0,0,1]
#         if np.abs(a[0,i])<eps and np.abs(a[1,i])<eps and np.abs(np.abs(a[2,i])-1)<eps:
#             column_no=i
#         #swap the column with last column
#     for i in range(3):
#         a[i,2],a[i,column_no]=swap(a[i,2],a[i,column_no]) 
#     return a

def swap(a,b):
    #swap any two variables
    inter = 0
    inter = a
    a=b
    b=inter
    #print(a,b)
    return a,b
    
def z_swap(a,eps):
    #input: a is rotation matrix, eps is acceptable error
    #output: transformed rotation matrix
    column_no=10
    for i in range(3):
        #check which column has [0,0,1]
        if np.abs(abs(a[2,i])-1)<eps:
            column_no=i
        #swap the column with last column
    for i in range(3):
        a[i,2],a[i,column_no]=swap(a[i,2],a[i,column_no])
    if a[0,0]*a[1,1]<0:
        for i in range(3):
            a[i,0],a[i,1]=swap(a[i,0],a[i,1])
    return a
