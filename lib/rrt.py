import numpy as np
import random

from lib.detectCollision import detectCollision
from lib.loadmap import loadmap
from copy import deepcopy
from lib.calculateFK import FK


# from detectCollision import detectCollision
# from loadmap import loadmap
# from copy import deepcopy
# from calculateFK import FK

class Node:
    def __init__(self, x, y):
        self.current_value = x
        self.parent_index = y


def is_within_bounds(matrix, lower_bound, upper_bound):
    return np.all(np.greater(matrix, lower_bound)) and np.all(np.less(matrix, upper_bound))

def isRobotCollided (map, q_from , q_to):
    fk=FK()
    linept_from,_= fk.forward(q_from)
    linept_to,_= fk.forward(q_to)
    for obstacle in map[0]:                     # Checking for all obstacles
        tolerances = [-0.05, -0.05, -0.05, 0.05, 0.05, 0.05]                # inflating obstacles
        box = [value + tolerance for value, tolerance in zip(obstacle, tolerances)]

        # Checking for small distances from start to goal position along the way.
        num_div=10      
        for i in range(num_div):                    
            a = q_from + i * (q_to - q_from) / num_div
            b = q_from + (i+1)*(q_to - q_from)/num_div
            pt1,_=fk.forward(a)
            pt2,_=fk.forward(b)
            iscollide = np.array(detectCollision(pt1, pt2, box))
            if np.any(iscollide):
                return True
            
        #Link collision check
        iscollide = np.array(detectCollision(linept_to[0:6], linept_to[1:7], box)) 
        if np.any(iscollide):
            return True
    return False

def isRobotSelfCollided(q):                         # Checking distance bettween all the joints for self collision check
    fk = FK() 
    link_positions,_=fk.forward(q)
    collision_tolerance = 0.04 
    for i in range(7):
        for j in range(i + 1, 7):
            distance = np.linalg.norm(link_positions[i] - link_positions[j])
            if distance < collision_tolerance:
                return True  

    return False  


def distance(q_from,q_to):
    fk=FK()
    linept_from,_= fk.forward(q_from)
    linept_to,_= fk.forward(q_to)
    point_from=linept_from[-1]
    point_to=linept_to[-1]
    distance = np.linalg.norm(point_to - point_from)
    return distance


def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (0x7).
    :param goal:        goal pose of the robot (0x7).
    :return:            returns an mx7 matrix, where each row consists of the configuration of the Panda at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is empty
    """
    # initialize path
    path = [] 
    q_start=Node(start,None)
    q_goal=Node(goal,None)
    Tst=[q_start]
    Tg=[q_goal]
    # get joint limits
    lowerLim = np.array([-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973])
    upperLim = np.array([2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973])

    i=0
    min_dis_st = float('inf')
    min_dis_g = float('inf')
    iter=100

    while i<iter:
        q_random = np.random.uniform(lowerLim, upperLim, size=(1, 7))
        q_random=q_random.flatten()

        for j in range(len(Tst)):                                   #Checking nearest point in the Tree
            dst=distance(q_random,Tst[j].current_value)
            if dst < min_dis_st:
                min_dis_st=dst
                q_st = Tst[j]
        for l in range(len(Tg)):
            dg=distance(q_random,Tg[l].current_value)
            if dg < min_dis_g:
                min_dis_g=dg
                q_g = Tg[l]

        if  not isRobotCollided(map,q_st.current_value,q_random) and not isRobotSelfCollided(q_random) :
            q_TST=Node(q_random,Tst.index(q_st))
            Tst.append(q_TST)                                   # appending the nearest point with no collision to the Tree.
        if  not isRobotCollided(map,q_g.current_value,q_random) and not isRobotSelfCollided(q_random):
            q_TG=Node(q_random,Tg.index(q_g))
            Tg.append(q_TG) 

        if np.array_equiv(Tst[-1].current_value,Tg[-1].current_value):
            break
        i=i+1

    # Retrace path from start to goal
    current_index_g = len(Tg) - 1
    while current_index_g is not None:
        path.append(Tg[current_index_g].current_value)
        current_index_g = Tg[current_index_g].parent_index

    current_index_st = Tst[-1].parent_index
    while current_index_st is not None:
        path.insert(0,Tst[current_index_st].current_value)
        current_index_st = Tst[current_index_st].parent_index
    
    start_check=is_within_bounds(start, lowerLim, upperLim)                        # Start and Goal check if in limits.
    goal_check=is_within_bounds(goal, lowerLim, upperLim)                          

    if i == iter or not start_check or not goal_check:
        print("Failed to find a path within the maximum number of iterations.")
        path=[]
    else:
        print("Path found!")

    return path

if __name__ == '__main__':
    map_struct = loadmap("../maps/map1.txt")
    start = np.array([0,-1,0,-2,0,1.57,0])
    goal =  np.array([-1.2, 1.57 , 1.57, -2.07, -1.57, 1.57, 0.7])
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))
    print(path)

