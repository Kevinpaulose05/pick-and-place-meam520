import sys
import numpy as np
from copy import deepcopy
from math import pi, sin, cos
import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector
from lib.IK_position_null import IK
from lib.calculateFK import FK

from time import perf_counter
from scipy.spatial.transform import Rotation


# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds
from translib import trans, roll, pitch, yaw, transform,z_swap
from Ik_new import franka_IK_EE


ik = IK()
fk=FK()


def complementary_filter(data, reference_data, alpha):
    return alpha * data + (1 - alpha) * reference_data

def block_Detection():

    alpha=0.85

    for i in range(5):
        block_pose1=[]
        for (name, pose) in detector.get_detections():
            block_pose1.append(pose)
            print(name,'\n',pose)
        block_pose1=np.array(block_pose1)
        # print(block_pose1.shape)

        block_pose2=[]
        for (name, pose) in detector.get_detections():
            block_pose2.append(pose)
            # print(name,'\n',pose)
        block_pose2=np.array(block_pose2)
        # print(block_pose2.shape)

        block_pose_comp=complementary_filter(block_pose1,block_pose2,alpha)
        block_pose1=block_pose2
        block_pose2=block_pose_comp


    block_pose_world=[]
    for i in range(block_pose_comp.shape[0]):
        block_pose_world.append(T_CW@block_pose_comp[i])

    return block_pose_world

def static_pick_place(target_placing, block_pose_world):

    for target_block in block_pose_world:
        Rot=T_EW.T@target_block
        Rotationmat = z_swap(Rot[:3,:3],0.35)
        print(Rotationmat)
        rz=np.arccos(Rotationmat[0][0])
        print("rz_inital=", rz)

        if rz > pi/4:
            rz=rz-pi/2
        elif rz< -pi/4 :
            rz=rz + pi/2
    
        else:
            rz=rz
        print("rz_final=", rz)

        arm.open_gripper()

        targets_b = transform(np.array([target_block[0, 3], target_block[1, 3], 0.230]), np.array([0, pi, pi]))
        tar = franka_IK_EE(targets_b, q7, q_actual_array)
        q=tar[2]
        if ((Rotationmat[0,0]>0 and Rotationmat[0,1]>0) or (Rotationmat[0,0]<0 and Rotationmat[0,1]>0)):
            q[6] -= rz
            print("Hello")
        else:
            q[6] += rz
        # q[6] += rz
        arm.safe_move_to_position(q)
        # arm.close_gripper()
        arm.exec_gripper_cmd(0.049,50)

        pos=np.linalg.norm(arm.get_gripper_state()["position"])
        print(pos)

        if pos < 0.01:
            target= transform( np.array([0.502, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
            tar = franka_IK_EE(target, q7, q_actual_array)
            q=tar[2]
            print(q)
            arm.safe_move_to_position(q)
            print("Broken")
            continue


        targets_p = transform(np.array([target_placing[0], target_placing[1], target_placing[2]+0.03]), np.array([0, pi, pi]))
        tar = franka_IK_EE(targets_p, q7, q_actual_array)
        q=tar[-1]
        arm.safe_move_to_position(q)
        

        targets_p = transform(np.array([target_placing[0], target_placing[1], target_placing[2]-0.03]), np.array([0, pi, pi]))
        tar = franka_IK_EE(targets_p, q7, q_actual_array)
        q=tar[-1] 
        arm.safe_move_to_position(q)
        arm.open_gripper()

        targets_pp = transform(np.array([target_placing[0], target_placing[1], target_placing[2]+0.03]), np.array([0, pi, pi]))
        tar = franka_IK_EE(targets_pp, q7, q_actual_array)
        q=tar[-1]
        arm.safe_move_to_position(q)


        target_placing[2] += 0.05

def dynamic_pick():

    arm.open_gripper()

    target= transform( np.array([0.4, -0.7, 0.285]), np.array([ 0,-pi/2,pi]) )
    seed= np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    q, rollout, success, message = ik.inverse(target, seed, method='J_pseudo', alpha=.5)
    #print(q)
    q[3]=q[3]+pi/3
    q[5]-=pi/3 -pi/10
    q[4]-=pi/7
    q[0]=q[0]-pi/12
    q[6]+=pi/18
    print(q)
    arm.safe_move_to_position(q)

    q[0]=q[0]+pi/9
    q[5]-=pi/12
    q[6]-=pi/20
    arm.safe_move_to_position(q)

    rospy.sleep(7)


    arm.exec_gripper_cmd(0.0475,60)
    pos=np.linalg.norm(arm.get_gripper_state()["position"])
    print(pos)

    q[5]+=pi/3
    q[0]+=pi/5
    q[6]+=pi/20
    q[1]-=pi/55

    arm.safe_move_to_position(q)
    
    if pos > 0.015:
        target_placing[2]+=0.05
        dynamic_place(target_placing)
        print("continue")
    else:
        target= transform( np.array([0.522, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
        print(target)
        tar = franka_IK_EE(target, q7, q_actual_array)
        q=tar[2]
        print(q)
        arm.safe_move_to_position(q)
        

   

def dynamic_place(target_placing):
    targets_p = transform(np.array([target_placing[0]+0.105, target_placing[1], target_placing[2]+0.02]), np.array([0, pi, pi]))
    tar = franka_IK_EE(targets_p, q7, q_actual_array)
    q=tar[-1]
    q[5]-=2*pi/15
    arm.safe_move_to_position(q)
    

    targets_p = transform(np.array([target_placing[0]+0.105, target_placing[1], target_placing[2]-0.062]), np.array([0, pi, pi]))
    tar = franka_IK_EE(targets_p, q7, q_actual_array)
    q=tar[-1]
    q[5]-=2*pi/15
    arm.safe_move_to_position(q)
    arm.open_gripper()

    targets_pp = transform(np.array([target_placing[0]+0.05, target_placing[1], target_placing[2]+0.1]), np.array([0, pi, pi]))
    tar = franka_IK_EE(targets_pp, q7, q_actual_array)
    q=tar[-1]
    q[5]-=2*pi/15
    arm.safe_move_to_position(q)

    # target= transform( np.array([0.522, -0.169, 0.58364056]), np.array([ 0,pi,pi]) )

    # print(target)
    # tar = franka_IK_EE(target, q7, q_actual_array)
    # q=tar[2]
    # arm.safe_move_to_position(q)
    # arm.open_gripper()


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    detector = ObjectDetector()

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!

    arm.set_arm_speed(0.3)

    q7 = np.pi/4
    q_actual_array = [ 0 ]

    # arm.open_gripper()

    target= transform( np.array([0.522, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
    print(target)
    tar = franka_IK_EE(target, q7, q_actual_array)
    q=tar[2]
    print(q)
    arm.safe_move_to_position(q)
   
    H_ee_camera = detector.get_H_ee_camera()
    T_CE = H_ee_camera # T of camera in end_effector
    _,T_EW = fk.forward(q)
    T_CW= T_EW@T_CE

    rospy.sleep(2)

    block_pose_world=block_Detection()

    # arm.set_arm_speed(0.3)


    target_placing=np.array([0.57, -0.169, 0.27])

    dynamic_pick()


    static_pick_place(target_placing, block_pose_world[:1])

    dynamic_pick()

    # target= transform( np.array([0.522, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
    # print(target)
    # tar = franka_IK_EE(target, q7, q_actual_array)
    # q=tar[2]
    # print(q)
    # arm.safe_move_to_position(q)

    target= transform( np.array([0.4, -0.7, 0.285]), np.array([ 0,-pi/2,pi]) )
    seed= np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    q, rollout, success, message = ik.inverse(target, seed, method='J_pseudo', alpha=.5)
    #print(q)
    q[3]=q[3]+pi/3
    q[5]-=pi/3 -pi/10
    q[4]-=pi/7
    q[0]=q[0]-pi/12
    q[6]+=pi/18
    print(q)
    arm.safe_move_to_position(q)

    q[0]=q[0]+pi/9
    q[5]-=pi/12
    q[6]-=pi/20
    arm.safe_move_to_position(q)

    rospy.sleep(7)


    # arm.exec_gripper_cmd(0.0475,60)
    pos=np.linalg.norm(arm.get_gripper_state()["position"])
    print(pos)

    q[5]+=pi/3
    q[0]+=pi/5
    q[6]+=pi/20
    q[1]-=pi/55

    arm.safe_move_to_position(q)

    static_pick_place(target_placing, block_pose_world[1:])

    target= transform( np.array([0.522, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
    print(target)
    tar = franka_IK_EE(target, q7, q_actual_array)
    q=tar[2]
    print(q)
    arm.safe_move_to_position(q)

    rospy.sleep(2)

    block_pose_world=block_Detection()
    target_placing[2]=target_placing[2]+0.05
    static_pick_place(target_placing, block_pose_world)


    # q7 = np.pi/4
    # q_actual_array = [ 0 ]

    # target= transform( np.array([0.522, 0.169, 0.58364056]), np.array([ 0,pi,pi]) )
    # print(target)
    # tar = franka_IK_EE(target, q7, q_actual_array)
    # q=tar[2]
    # print(q)
    # arm.safe_move_to_position(q)
    # arm.open_gripper()

    target= transform( np.array([0.4, -0.7, 0.29]), np.array([ 0,-pi/2,pi]) )
    seed= np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    q, rollout, success, message = ik.inverse(target, seed, method='J_pseudo', alpha=.5)
    #print(q)
    q[3]=q[3]+pi/3
    q[5]-=pi/3 -pi/10
    q[4]-=pi/7
    q[0]=q[0]-pi/12
    q[6]+=pi/18
    print(q)
    arm.safe_move_to_position(q)

    q[0]=q[0]+pi/9
    q[5]-=pi/12
    q[6]-=pi/20
    arm.safe_move_to_position(q)
    


    

    # for side view
    
    # q7=0.84168682

    # targets_p = transform(np.array([target_placing[0], target_placing[1], target_placing[2]+0.3]), np.array([0, -pi/2, pi]))
    # tar = franka_IK_EE(targets_p, q7, q_actual_array)
    # q=tar[-1]
    # print(q)
    # arm.safe_move_to_position(q)
    # arm.open_gripper()

    # Move around...

    # END STUDENT CODE