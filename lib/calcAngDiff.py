import numpy as np


def calcAngDiff(R_des, R_curr):
    """
    Helper function for the End Effector Orientation Task. Computes the axis of rotation 
    from the current orientation to the target orientation

    This data can also be interpreted as an end effector velocity which will
    bring the end effector closer to the target orientation.

    INPUTS:
    R_des - 3x3 numpy array representing the desired orientation from
    end effector to world
    R_curr - 3x3 numpy array representing the "current" end effector orientation

    OUTPUTS:
    omega - 0x3 a 3-element numpy array containing the axis of the rotation from
    the current frame to the end effector frame. The magnitude of this vector
    must be sin(angle), where angle is the angle of rotation around this axis
    """
    omega = np.zeros(3)
    ## STUDENT CODE STARTS HERE

    R_des_cur= np.dot(np.transpose(R_curr),R_des)      # converting R from des to current

    S= 0.5*(R_des_cur - np.transpose(R_des_cur))        # Skey Symmetric Matrix 

    a = np.array([S[2, 1], S[0, 2], S[1, 0]])           # Skey Symmetric Matrix 

    sin_theta = np.linalg.norm(a)                       # Calculate the magnitude of the rotation angle

    if sin_theta != 0:
        omega = a
    else:
        omega = np.zeros(3)

    omega=np.dot(R_curr,omega)                       # wc,d_0 = Rc_0 * wc,d_c

    return omega
