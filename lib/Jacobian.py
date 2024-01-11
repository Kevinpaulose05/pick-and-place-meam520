import numpy as np
from lib.calculateFK import FK
# from calculateFK import FK

def calcJacobian(q_in):
    """
    Calculate the full Jacobian of the end effector in a given configuration
    :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    rows correspond to the linear velocity and the last three rows correspond to
    the angular velocity, expressed in world frame coordinates
    """
    fk = FK()
    joint_positions, T0e = fk.forward(q_in)
    z_cap = fk.get_axis_of_rotation(q_in)

    J = np.zeros((6, 7))
    J_v= np.zeros((7,3))
    J_omega= np.zeros((7,3))

    O_end_0=joint_positions[7]     # Taking end-effector position

    for i in range(0,7):
        J_v[i]=np.cross(z_cap[i],(O_end_0-joint_positions[i]))          # calculatin Jacobian for all revolute joints

    J_omega=z_cap       

    J_=np.hstack((J_v,J_omega))


    ## STUDENT CODE GOES HERE
    J=np.transpose(J_)
    # print(z_cap)
    return J

if __name__ == '__main__':
    q= np.array([0, 0, 0, 0, 0, np.pi, np.pi/4])
    print(np.round(calcJacobian(q),3))