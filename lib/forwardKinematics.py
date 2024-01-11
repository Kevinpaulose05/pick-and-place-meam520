import numpy as np
from math import pi
cos=np.cos
sin = np.sin



class FK():  

    def __init__(self):

        # TODO: you may want to define geometric parameters here that will be
        # useful in computing the forward kinematics. The data you will need
        # is provided in the lab handout

        pass

    def forward(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        jointPositions -8 x 3 matrix, where each row corresponds to a rotational joint of the robot or end effector
                  Each row contains the [x,y,z] coordinates in the world frame of the respective joint's center in meters.
                  The base of the robot is located at [0,0,0].
        T0e       - a 4 x 4 homogeneous transformation matrix,
                  representing the end effector frame expressed in the
                  world frame
        """

        # Initalizing arryas
        global R                            # Defining global to use later
        global T
        global jointPositions                            
        jointPositions = np.zeros((8,3))
        T0e = np.identity(4)
        A= np.zeros((7,4,4))   
        T=np.zeros((7,4,4))
        R= np.zeros((7,3,3))


        # Joint 1 position is fixed.
        jointPositions[0] = [0.0,0.0,0.141]

        # DH Parameters in the orientation shown - ai, alphai, di, qi.
        DH_Parameters= np.array([[0.0 ,-pi/2, 0.333 , q[0]],
                                [0.0 , pi/2, 0.0   , q[1]],
                                [0.0825 ,pi/2, 0.316, q[2]],
                                [0.0825, pi/2, 0.0    , pi+q[3]],    #Subtracted -pi/2 as in initial position there is some offset.
                                [0.0 , -pi/2, 0.384, q[4]],
                                [0.088 ,pi/2, 0.0   , q[5]-pi],      #Subtracted pi/2 as in initial position there is some offset.
                                [0.0 ,0.0, 0.21  , q[6]-(pi/4)]])    #Subtracted pi/4 as in initial position there is some offset.


        # Offset of sitance in joint frame as the origin of frames are not on the joint.
        # So have to add this to find joint position.
        d_offset= np.array([ [0.0, 0.0 ,0.0],
                             [0.0, 0.0, 0.195],
                             [0.0, 0.0, 0.0 ],
                             [0.0, 0.0, 0.125],
                             [0.0, 0.0, -0.015],
                             [0.0, 0.0, 0.051],
                             [0.0, 0.0, 0.0]])
        
        for i in range (0,7):
            # A matrix for DH Convention.
            A_= [[cos(DH_Parameters[i][3]), -sin(DH_Parameters[i][3])*cos(DH_Parameters[i][1]), sin(DH_Parameters[i][3])*sin(DH_Parameters[i][1]), DH_Parameters[i][0]*cos(DH_Parameters[i][3])],
                [sin(DH_Parameters[i][3]), cos(DH_Parameters[i][3])*cos(DH_Parameters[i][1]), -cos(DH_Parameters[i][3])*sin(DH_Parameters[i][1]), DH_Parameters[i][0]*sin(DH_Parameters[i][3])],
                [0 , sin(DH_Parameters[i][1]), cos(DH_Parameters[i][1]), DH_Parameters[i][2]],
                [0, 0, 0, 1]]        
            A[i]=A_                         # Saving all A's in a (7x4x4) matrix

            if i==0:      
                T[i] = A_                    # For 1st Frame, the T10 = A10.
                R[i]=T[i,:3,:3]             # Extraction R matrix from  T.
                jointPositions[i+1]=T[i,:3,3] + np.dot(R[i],d_offset[i])
            else:         
                T[i]= np.dot(T[i-1],A_)      # Finding T matrix for all joints and saving to T(7x4x4) matrix
                R[i]=T[i,:3,:3]             # Extraction R matrix from each T.

                # doing Joint Position = d0n +  R0n.d_offest for the nth joint.
                jointPositions[i+1]=T[i,:3,3] + np.dot(R[i],d_offset[i])

        T0e=T[6]                            # End Effector Transformation Matrix
        
        return jointPositions, T0e

    
    # This code is for Lab 2, you can ignore it for Lab 1
    def get_axis_of_rotation(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        axis_of_rotation_list z_cap: - 3x7 np array of unit vectors describing the axis of rotation for each joint in the
                                 world frame

        """
        # STUDENT CODE HERE: This is a function needed by lab 2
        global z_cap                    # Making global to be used in other code    
        z_cap=np.zeros((7,3))
        zth_axis= np.array([0,0,1])     
        z_cap[0]=[0,0,1]
        for i in range(1,7):
            z_cap[i]= np.dot(R[i-1],zth_axis)           # Taking 3rd column of every rotation matrix
        

        return z_cap
    
    def compute_Ai(self, q):
        """
        INPUT:
        q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]

        OUTPUTS:
        Ai: - 4x4 list of np array of homogenous transformations describing the FK of the robot. Transformations are not
              necessarily located at the joint locations
        """
        # STUDENT CODE HERE: This is a function needed by lab 2

        return()
    
if __name__ == "__main__":

    fk = FK()

    # matches figure in the handout
    # q = np.array([ 0.55211 , 1.00363,  0.1385 , -2.5897,  -0.21756 , 1.83606,  0.28301])
    q = np.array([0,0 ,0,-pi/2,0,pi/2,pi/4])
    

    joint_positions, T0e = fk.forward(q)
    fk.get_axis_of_rotation(q)

    
    print("Joint Positions:\n",joint_positions)
    print("End Effector Pose:\n",T0e)