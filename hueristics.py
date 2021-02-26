import numpy as np
from math import sqrt,inf
import matplotlib.pylab as plt
import sys
import API
 
block = np.dtype([('heuristic',np.int32),('count',np.int32),('orientation',np.int32)])

def log(string):
    sys.stderr.write("{}\n".format(string))
    sys.stderr.flush()

#values

wall_flag = [False,False,False]
#initial pose
robot_pose = [0,15]
orientation = ["north","east","south","west"]
orientation_index = 0

def orient_update(turn):
    global orientation_index
    if(turn == 'L'):
        orientation_index = (orientation_index - 1) % 4 
    else:
        orientation_index = (orientation_index + 1) % 4
    log(orientation[orientation_index])

#helper function
def pose_update():
    #left = 0, straight = 1, right = 2
    if(orientation_index == 0):
        robot_pose[1] = robot_pose[1] - 1
    elif(orientation_index == 1):
        robot_pose[0] = robot_pose[0] + 1
    elif(orientation_index == 2):
        robot_pose[1] = robot_pose[1] + 1
    else:
        robot_pose[0] = robot_pose[0] - 1
    heuristic_map[robot_pose[1],robot_pose[0]]['count'] = heuristic_map[robot_pose[1],robot_pose[0]]['count'] + 1
    log("pose updates!!")


def wall_checker():
    if(API.wallLeft()):
        wall_flag[0] = True
    else:
        wall_flag[0] = False
    if(API.wallFront()):
        wall_flag[1] = True
    else:
        wall_flag[1] = False
    if(API.wallRight()):
        wall_flag[2] = True
    else:
        wall_flag[2] = False
    log(str(wall_flag))

def oneway(count = 1):
    while(sum(wall_flag) == 2):
        if(not wall_flag[1]):
            API.moveForward()
            pose_update()
            if(count == 4):
                heuristic_map[robot_pose[1],robot_pose[0]]['count'] = 4
        elif(not wall_flag[0]):
            API.turnLeft()
            orient_update('L')
        else:
            API.turnRight()
            orient_update('R')
        wall_checker()
        log(str(robot_pose))
    API.moveForward()
    pose_update()
    log(str(robot_pose))

def retrieve_huerestic(turn):
    #TURN , ORIENTATION
    adj_cell = robot_pose.copy()
    log(str("robo_pose:" + str(robot_pose)))
    if(turn == 'L'):
        temp = (orientation_index - 1) % 4
    elif(turn == 'R'):
        temp = (orientation_index + 1) % 4
    else:
        temp = orientation_index

    if(temp == 0):
        adj_cell[1] = robot_pose[1] - 1
    elif(temp == 1):
        adj_cell[0] = robot_pose[0] + 1
    elif(temp == 2):
        adj_cell[1] = robot_pose[1] + 1
    else:
        adj_cell[0] = robot_pose[0] - 1
    log(str("adj_cell" + str(adj_cell)))

    return heuristic_map[adj_cell[1],adj_cell[0]]['heuristic']


def dir_choose():
    candidates = [inf,inf,inf]
    if(sum(wall_flag) < 2):
        if(not wall_flag[0]):
            candidates[0] =  retrieve_huerestic('L')
        if(not wall_flag[2]):
            candidates[2] = retrieve_huerestic('R')
        if(not wall_flag[1]):
            candidates[1] = retrieve_huerestic('C')
        log(str(candidates))


def dead_end_mark():
    while(wall_flag[1]):
        API.turnRight()
        orient_update('R')
        wall_checker()
    oneway(4)


flood=[[14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14],
        [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
        [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
        [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
        [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
        [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
        [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
        [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
        [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
        [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
        [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
        [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
        [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
        [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
        [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
        [14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14]]

# class Cell:
#     def __init__(self, heuristic, count, action_space):
#         self.heuristic = 0
#         self.count = 0
#         self.action_space = 0
#     # def __str__(self):
#     #     return str(self.heuristic)

    
heuristic_map = np.empty((16,16),dtype=block)
# print(heuristic_map.shape)

def euclidean_distance(x,y):
    return (sqrt((x-7.5)**2 + (y-7.5)**2))
    # return abs(x-8.5) +abs(y-8.5)

def heurestic_assign():
    for x in range(0,16):
        for y in range(0,16):
            # heuristic_map = (flood[x][y],0,0)

            heuristic_map[x,y] = (euclidean_distance(x,y),0,0)
            print(heuristic_map[x,y])
            print((euclidean_distance(x,y),0,0))
            
heurestic_assign()

# print(heuristic_map['heuristic'])

# fig = plt.figure()
# plt.imshow(heuristic_map['heuristic'], interpolation='nearest')
# plt.show()
log(str(robot_pose))
wall_checker()
oneway()
wall_checker()
dir_choose()
# oneway()
log(str(heuristic_map['count']))
log(str(heuristic_map['heuristic']))

