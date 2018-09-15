#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist, ForbEdges
from std_msgs.msg import Int16
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import pylab
import rospkg
import sys
import os
import copy
from threading import Thread
import time as tm

rp = rospkg.RosPack()
path = rp.get_path('distributed')
path_0 = path
sys.path.insert(0, path + '/CPP_MIT')
import postman as CPPlib
import library2018 as myLib
import Algorithm_2 as Alg2

global vel, pub_stage, pub_broadcast, change_plan, Hole_path_list, new_Hists, original_graph,virtual_graph,list_of_H, list_of_robs







global SP # search points
global pose
pose = [0.1, 0.2, 0.001] # [x, y, theta]
global d  # for feedback linearization
d = 0.07
global time
time = 0
global Kp # Controller's proportional gain
Kp = 0.8
global laserVec # Distances measured by the laser
laserVec = [1 for i in range(270)]
global detected_pose # Initial flag to wait stage start running
detected_pose = 0

#Flag to indicate that the routes should be replanned
global replan_tasks
replan_tasks = False

#Lists that carries the history of the robot
global e_v, e_uv, e_g
global T_a, T_f
global H, H_new




#EH PRECISO CRIAR A HISTORIA DE CADA ROBO !!!!!!!!!!!!


poses_1 = [[[21.5600, 8.4000]],
[[6.9600, 1.2800]],
[[13.7000, 10.4200]],
[[14.6000, 9.5400]],
[[18.6200, 5.0200]],
[[8.4600, 3.3400]],
[[11.9600, 3.3400]],
[[2.1200, 2.5600]],
[[9.9400, 8.4000]],
[[18.5600, 1.2600]],
[[16.0000, 6.0800]],
[[18.3200, 7.3600]],
[[8.5800, 5.6200]],
[[9.2600, 2.5400]],
[[2.0800, 8.3600]],
[[11.3800, 8.4000]],
[[8.4800, 9.2800]],
[[8.4600, 3.3400]],
[[2.1200, 2.5600]],
[[2.4200, 5.5800]]]

poses_2 = [[[16.1800, 1.3800],[2.1200, 2.5600]],
[[11.9600, 3.3400],[18.6200, 4.3200]],
[[6.0200, 7.3800],[12.2000, 9.6400]],
[[5.2400, 5.0200],[16.1800, 1.3800]],
[[10.2000, 5.6200],[2.4200, 5.5800]],
[[18.3200, 7.3600],[12.9800, 10.4200]],
[[11.3800, 0.4600],[6.0200, 7.3800]],
[[7.0000, 8.3600],[20.1000, 9.2800]],
[[13.7400, 2.5600],[15.7400, 7.5000]],
[[20.8800, 2.5400],[10.6600, 7.6800]],
[[20.5800, 5.6200],[17.7000, 5.9400]],
[[17.7000, 5.9400],[11.3800, 8.4000]],
[[13.7000, 10.4200],[20.5800, 5.6200]],
[[16.0400, 3.2600],[6.4600, 3.7600]],
[[1.2600, 9.6200],[12.1000, 0.4600]],
[[2.0800, 8.3600],[0.4800, 0.4600]],
[[3.0200, 3.4600],[4.5200, 5.0200]],
[[12.1000, 0.4600],[14.6200, 3.4600]],
[[19.6200, 7.5800],[4.5600, 1.3800]],
[[6.4600, 3.7600],[23.0200, 9.1000]]]

poses_3 = [[[3.3000, 7.1400],[6.3200, 6.1000],[16.6000, 8.3600]],
[[11.9600, 3.3400],[8.4600, 3.3400],[20.5800, 5.6200]],
[[23.0200, 9.1000],[1.2600, 9.6200],[16.6000, 8.3600]],
[[6.9600, 1.2800],[8.4800, 9.2800],[10.4000, 1.4400]],
[[18.5600, 1.2600],[13.0000, 5.5600],[3.0200, 3.4600]],
[[1.2600, 9.6200],[21.5600, 8.4000],[14.0600, 5.3800]],
[[4.5600, 1.3800],[16.1800, 1.3800],[2.4200, 5.5800]],
[[10.2000, 5.6200],[14.0600, 5.3800],[13.7000, 7.0800]],
[[22.0400, 1.4400],[21.5600, 8.4000],[5.2400, 5.0200]],
[[10.6600, 7.6800],[6.4600, 3.7600],[6.3200, 6.1000]],
[[16.6000, 8.3600],[14.6000, 9.5400],[3.2200, 6.3200]],
[[23.0200, 9.1000],[22.2800, 7.6800],[9.9400, 8.4000]],
[[0.4800, 0.4600],[13.7000, 10.4200],[6.8800, 2.8600]],
[[20.1000, 3.3000],[22.3800, 9.7600],[17.7000, 5.9400]],
[[16.0400, 3.2600],[3.0200, 3.4600],[10.2000, 5.6200]],
[[17.3200, 8.3600],[3.2200, 6.3200],[0.4800, 0.4600]],
[[6.3200, 6.1000],[1.4000, 5.5800],[16.6000, 8.3600]],
[[1.2800, 1.2800],[12.9200, 1.2800],[1.2600, 9.6200]],
[[22.0400, 1.4400],[4.4200, 3.2600],[3.3000, 7.1400]],
[[17.7000, 5.9400],[8.4800, 9.2800],[18.3200, 7.3600]]]

poses_4 = [[[1.4000, 5.5800],[6.9600, 1.2800],[1.2600, 9.6200],[5.2400, 5.0200]],
[[6.9600, 1.2800],[3.3000, 7.1400],[3.2200, 6.3200],[14.0600, 5.3800]],
[[11.3800, 0.4600],[12.9800, 10.4200],[6.4600, 3.7600],[6.3200, 6.1000]],
[[6.4600, 3.7600],[1.4000, 5.5800],[16.0000, 6.0800],[5.2400, 5.0200]],
[[23.0200, 0.4600],[1.4000, 5.5800],[14.5800, 7.9600],[21.8200, 5.6200]],
[[6.8800, 2.8600],[3.0200, 3.4600],[0.4800, 10.4200],[10.2000, 5.6200]],
[[11.2400, 3.3600],[18.3200, 7.3600],[23.0200, 0.4600],[2.4200, 5.5800]],
[[10.2000, 5.6200],[18.0600, 3.7800],[15.7400, 7.5000],[17.3200, 8.3600]],
[[17.3200, 8.3600],[4.5600, 1.3800],[21.8200, 5.6200],[3.0200, 3.4600]],
[[15.7400, 7.5000],[12.9800, 10.4200],[6.8800, 2.8600],[13.0000, 5.5600]],
[[20.5800, 5.6200],[12.9800, 10.4200],[11.3800, 0.4600],[5.2400, 5.0200]],
[[12.2000, 9.6400],[14.6200, 3.4600],[11.3800, 0.4600],[2.4200, 5.5800]],
[[10.6600, 7.6800],[22.2800, 7.6800],[12.9200, 1.2800],[3.2200, 6.3200]],
[[22.2800, 7.6800],[20.1000, 9.2800],[23.0200, 10.4200],[18.6200, 4.3200]],
[[16.0000, 6.0800],[12.1000, 0.4600],[10.2000, 5.6200],[4.4200, 3.2600]],
[[14.6200, 3.4600],[10.2000, 5.6200],[18.3200, 7.3600],[6.3200, 6.1000]],
[[9.9400, 8.4000],[16.0400, 3.2600],[1.4000, 5.5800],[10.2000, 5.6200]],
[[10.6600, 7.6800],[12.9200, 1.2800],[22.0400, 1.4400],[8.4800, 9.2800]],
[[16.1800, 1.3800],[7.0200, 4.3200],[19.3400, 2.5400],[16.0400, 3.2600]],
[[2.0800, 8.3600],[2.1200, 2.5600],[10.6600, 7.6800],[14.5800, 7.9600]]]


global poses
poses = [poses_1,poses_2,poses_3,poses_4]
#print '\n--------------\nposes: ', poses, '\n--------------\n'





# Primary routine
def run_expriment():

    global poses

    Whole_path_0 = [11, 9, 10, 9, 8, 7, 72, 7, 8, 12, 11, 12, 13, 45, 13, 14, 15, 29, 30, 31, 30, 32, 30, 29, 28, 42, 43, 42, 18, 17, 38, 20, 38, 37, 39, 37, 36, 22, 36, 2, 1, 2, 3, 4, 3, 28, 27, 33, 27, 26, 34, 26, 24, 25, 24, 23, 35, 23, 22, 21, 40, 21, 20, 19, 41, 19, 18, 17, 16, 44, 16, 15, 14, 5, 4, 73, 4, 5, 6, 7, 6, 61, 60, 66, 60, 59, 67, 59, 57, 58, 57, 56, 68, 69, 68, 70, 68, 56, 55, 63, 62, 61, 62, 65, 62, 63, 64, 63, 55, 51, 52, 53, 52, 54, 52, 51, 47, 48, 49, 48, 50, 48, 47, 46, 71, 46]


    R = 4

    path = path_0 + "/experiments/timming/" + "exp_A2_" + str(R) + "_robs.txt"
    #path = path_0 + "/experiments/timming/" + "exp_A2_x" + str(R) + "_robs.txt"
    FILE_2 = open(path, 'w')


    for bat in range(1,21,1):


        list_of_H = []
        print 'list_of_H:\n', list_of_H

        for r in range(R):


            H0 = History()
            IL = Intlist()

            H0.id = r #int16
            H0.specs = [Vd[r], Vs[r]] #float32[]
            H0.e_v = [] #int16[]
            H0.e_uv = range(1,len(PolC[0])+1) #int16[]
            H0.e_g = [] #int16[]
            H0.T_a = [IL for i in range(len(PolC[0]))] #Intlist[]
            H0.T_f = [] #int16[]
            [current_node, dist] = myLib.get_current_node(original_graph, poses[R-1][bat-1][r])
            current_node = current_node + 1
            print 'current_node = ', current_node
            H0.currEdge = current_node #int16
            Whole_path = Whole_path_0
            for k in range(len(Whole_path_0)):
                if Whole_path[0] == current_node:
                    break
                else:
                    Whole_path = Whole_path[1:] + Whole_path[:1]
            Whole_path.append(Whole_path[0])
            print 'Whole_path: ', Whole_path
            H0.nextNode = Whole_path[1] #int16
            H0.pose = poses[R-1][bat-1][r] #?? #float32[]
            H0.lastMeeting = [] #int16[]
            H0.Whole_path = Whole_path #int16[]
            H0.abort_curr_edge = False #bool
            H0.available = True #bool
            H0.popped_edges = False #bool

            #list_of_H.listOfH.append(H0)
            list_of_H.append(H0)







        #print 'list_of_H:\n', list_of_H
        # New (better) replanning method
        tot_time = tm.time()
        change_plan, Hole_path_list, new_Hists = Alg2.replanning_heuristics(original_graph, virtual_graph, list_of_H, FILE_2)
        tot_time = tm.time()-tot_time
        print '\ntot_time: ', tot_time
        # ----------  ----------  ----------  ----------  ----------  ----------  ----------

        FILE_2.write("\t" + str(tot_time) + "\t\n")

    FILE_2.close()
    """
    loop_time
    atri_time
    heu
    cost0
    cost1
    tot_tot_time
    """






    #Main loop
    while not rospy.is_shutdown():


        # Create list of History


        # New (better) replanning method
        # change_plan, Hole_path_list, new_Hists = Alg2.replanning_heuristics(original_graph, virtual_graph, list_of_H)
        # ----------  ----------  ----------  ----------  ----------  ----------  ----------


        break

        # Wait
        rate.sleep()

# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':

    global id
    global number_robots #Used only foor creating the topics
    global H
    global Vd, Vs #moving speed, search speeds
    global SP
    number_robots = 2

    """
    #Read the robots index
    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe robot id was not provided'
    else:
        id = int(sys.argv[1])
    """

    Vs = [1.5, 1.3, 1.0, 1.5]  # search speeds (rad/s) maximum is pi/2
    Vd = [0.4, 0.55, 0.5, 0.4]  # moving speeds (m/s)
    #Vs = Vs[id] / 1.0
    #Vd = Vd[id] / 1.0

    EXP_NAME = rospy.get_param('EXP_NAME')

    #Open txt files to write results of positions and velocities
    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/text/path_robot' + str(id) + '.txt'
    file_path = open(path, 'w')
    path = rp.get_path('distributed')
    path = path + '/text/vel_robot' + str(id) + '.txt'
    file_vel = open(path, 'w')

    #Read the original graph
    original_graph = myLib.read_graph("Original_graph_"+EXP_NAME+".mat")
    n = original_graph['n']
    nodes = original_graph['nodes']
    C = original_graph['C']
    PathM = original_graph['PathM']
    w_s = original_graph['w_s']
    PolC = original_graph['PolC']
    EdgeMap = original_graph['EdgeMap']

    #Initialization of the history of each robot
    e_v = [] # visited
    e_uv = range(1,len(PolC[0])+1) # unvisited (must be visited)
    e_g = [] # assigned to other robots
    IL = Intlist()
    T_a = [IL for i in range(len(PolC[0]))] # list of list of robots assigned to an edge
    T_f = []  # list of list of robots forbidden to visit an edge
    lastMeeting = [] #list of who was in the communicatio graph in the last meeting
    H = {'id': id, 'specs': [Vd, Vs], 'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f, 'lastMeeting': lastMeeting, 'Whole_path': [], 'available': True, 'popped_edges': False}
    H_new = {'id': id, 'specs': [Vd, Vs], 'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f, 'lastMeeting': lastMeeting, 'Whole_path': [], 'available': True, 'popped_edges': False}


    
    # Read the virtual graph
    virtual_graph = myLib.read_graph("Virtual_graph_"+EXP_NAME+".mat")

    # Read search point sets
    SP = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")  # points will be removed from this list
    SP_fix = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")  # this list wont be changed

    SP_visited = [] # As search points are visited they will be placed in this list

    global stop_the_robot
    stop_the_robot = False

    """
    #Clear File
    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/text/visited_' + str(id) + '.txt'
    FILE = open(path, 'w')
    FILE.close()
    """

    # Start running Algorithm 1
    try:
        run_expriment()
    except rospy.ROSInterruptException:
        pass

    """
    # Close the txt files
    file_path.close()
    file_vel.close()
    """









