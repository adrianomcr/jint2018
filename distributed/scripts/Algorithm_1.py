#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist
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


# Callback routine to obtain the robot's pose
def callback_pose(data):
    global file_path, time
    global detected_pose

    x_n = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y_n = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta_n = euler[2]  # orientaco 'theta' do robo no mundo

    if not file_path.closed:
        mystr = str(x_n) + "\t" + str(y_n) + "\t" + str(theta_n) + "\t" + str(time) + "\n"
        file_path.write(mystr)

    pose[0] = x_n
    pose[1] = y_n
    pose[2] = theta_n

    detected_pose = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the laser information
def callback_laser(data):
    global laserVec

    laserVec = data.ranges

    return
# ----------  ----------  ----------  ----------  ----------



# Callback routine to obtain data from the sensor simulator
# Here is were the robot wth minimum id calls the replanning function
def callback_comm_graph(data):
    # This function recieves messages from the centralized sensor simulator

    global list_of_H, list_of_robs, list_of_vels, H
    global replan_tasks, finish_edge, id
    global vel, pub_stage, pub_broadcast, change_plan, Hole_path_list, new_Hists, original_graph, virtual_graph, list_of_H, list_of_robs
    global count
    global stop_the_robot
    global waitting_new_plan
    global new_plan_flag

    R = len(data.listOfH)
    ids = []
    for r in range(R):
        ids.append(data.listOfH[r].id)

    #If this robot is involved in the communication graph
    if id in ids:
        # If some positive change (which adds information) happend in the communication graph
        if data.comGraphEvent == True:
            replan_tasks = True
            finish_edge = False
            list_of_H = data.listOfH
            list_of_robs = data.robList

            print '\n          ---------- Communication graph event received ----------\n'
            print 'Finishing Edge ...'

            #Uptate lastMeeting from now
            H['lastMeeting'] = list(list_of_robs)

        stop_the_robot = False


        # Compute the new routes if the current robot has munimum index
        if (id == min(list_of_robs)):
            # Stop the robot
            VX, WZ = 0, 0
            vel.linear.x, vel.angular.z = VX, WZ
            pub_stage.publish(vel)

            #Flag to stop the robot while it is computting a replan
            stop_the_robot = True

            # Call the replanning function (Algorithm 2) ----------  ----------  ----------
            #Old replanning method
            #change_plan, Hole_path_list, new_Hists = Alg2.replanning(original_graph, virtual_graph, list_of_H)

            # New (better) replanning method
            change_plan, Hole_path_list, new_Hists = Alg2.replanning_heuristics(original_graph, virtual_graph, list_of_H)
            # ----------  ----------  ----------  ----------  ----------  ----------  ----------
            stop_the_robot = False

            #Broadcast new plan to other robots
            print '\nCreating Broadcast message ...\n'
            B = Broadcast()
            B.sender = id
            R = len(new_Hists)
            B.destinations = []
            for r in range(R):
                B.destinations.append(new_Hists[r].id)
            for r in range(R):
                IL = Intlist()
                IL.data = list(Hole_path_list[r])
                B.new_Hole_paths.append(IL)
            B.listOfH = new_Hists

            pub_broadcast.publish(B)
            print 'New plan broadcasted'

            # -----------------------------------------------

    return
# ----------  ----------  ----------  ----------  ----------



# Callback function to receive a new plan
def callback_new_plan(data):

    global Hole_path
    global id, H, H_new, Hole_path_new
    global replan_tasks
    global waitting_new_plan
    global new_task
    global pose, original_graph
    global new_plan_flag

    #If the current robot is one of the ones the replanning rout is destinated to
    if id in data.destinations:

        position = data.destinations.index(id)

        Hole_path_new = list(data.new_Hole_paths[position].data)

        Hmsg = data.listOfH[position]

        H_new['e_v'] = list(Hmsg.e_v)
        H_new['e_uv'] = list(Hmsg.e_uv)
        H_new['e_g'] = list(Hmsg.e_g)
        H_new['T_a'] = list(Hmsg.T_a)
        H_new['T_f'] = list(Hmsg.T_f)
        H_new['lastMeeting'] = list(Hmsg.lastMeeting)

        global abort_curr_edge
        abort_curr_edge = Hmsg.abort_curr_edge
        print 'Here is abort_curr_edge: ', abort_curr_edge


        print '\n     ----- New plan received -----'

        # The flag is set because the robot's History is not updated when it receives it
        # The robot waits to finish an edge before replacing its History by a new one
        new_plan_flag = True
        print '\nFlag setted\n'



    return
# ----------  ----------  ----------  ----------  ----------






# Primary routine
def Algorithm_1():

    global pose
    global pub_rviz, pub_targ, pub_pose
    global freq
    global time, i
    global file_vel
    global N
    global n, nodes, C, PathM, w_s, PolC
    global Vd
    global laserVec
    global id
    global replan_tasks, finish_edge
    global H, H_new, Hole_path_new
    global list_of_H, list_of_robs, list_of_vels
    global Hole_path
    global waitting_new_plan
    global new_task
    global original_graph
    global SP, SP_fix
    global new_plan_flag
    global vel, pub_stage, pub_broadcast, change_plan, Hole_path_list, new_Hists, original_graph, virtual_graph, list_of_H, list_of_robs

    new_plan_flag = False

    global abort_curr_edge
    abort_curr_edge = False

    vel = Twist()

    global count
    count = 0

    #Define the node, publisher and subscribers
    my_str = '/robot_' + str(id) + '/cmd_vel'
    pub_stage = rospy.Publisher(my_str, Twist, queue_size=1) #Publish command velocities to stage
    my_str = '/robot_' + str(id) + '/history'
    pub_hist = rospy.Publisher(my_str, History, queue_size=1) #Publish History to the centralied communication sensor simulator
    my_str = 'follow_graph_' + str(id)
    rospy.init_node(my_str) #Initialize the node
    my_str = '/robot_' + str(id) + '/base_pose_ground_truth'
    rospy.Subscriber(my_str, Odometry, callback_pose) #Subscribe to obtain robot's position
    my_str = '/robot_' + str(id) + '/base_scan'
    rospy.Subscriber(my_str, LaserScan, callback_laser) #Subscribe to obtain robot's lasr scanner data
    rospy.Subscriber("/comm_graph", HistList, callback_comm_graph) #Subscribe to eventually receive communication graph event (other robots close)

    #Read and Write in the new plan topic
    pub_broadcast = rospy.Publisher('/new_path_topic', Broadcast, queue_size=4) #Eventually publish new routes to other robots
    rospy.Subscriber('/new_path_topic',Broadcast,callback_new_plan) #Eventually receive a new route

    #Publish the searched point in order to ???
    pub_SP = rospy.Publisher('/searched_point', Intlist, queue_size=4)

    freq = 15.0  # Frequency of operation in Hz
    rate = rospy.Rate(freq)

    # Wait initialization
    while (detected_pose == 0 and not rospy.is_shutdown()):
        rate.sleep()


    # Run CPP in orer to obtain an initial route ----------  ----------  ----------
    [current_node, dist] = myLib.get_current_node(original_graph, [pose[0], pose[1]])
    current_node = current_node + 1
    active_edges = range(1, len(PolC[0]) + 1)
    edges_listOfTuples = myLib.write_listOfTuples(original_graph, active_edges)
    print "Computting first CPP ..."
    elapsed = tm.time()
    Hole_path = CPPlib.main_CPP(edges_listOfTuples, current_node)
    elapsed = tm.time()-elapsed
    print 'Elapsed time for CPP: ', elapsed
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------

    #Uncomment this section in order to have a deterministic starting route
    """
    if(id == 0):
        Hole_path = [1, 2, 3, 4, 5, 6, 1, 2, 14, 13, 12, 10, 11, 9, 8, 7, 24, 25, 26, 27, 3]
    elif (id == 1):
        Hole_path = [30, 28, 24, 23, 22, 31, 20, 7, 1, 30, 28, 24, 23, 22, 21, 32, 17, 3]
    elif (id == 2):
        Hole_path = [33, 15, 16, 29, 21, 22, 31, 11]
    elif (id == 3):
        Hole_path = [5, 4, 3, 2, 1,14, 15]
    """
    """
    if(id == 0):
        #Hole_path = [2, 3] + Hole_path
        Hole_path = [] + Hole_path
    if (id == 1):
        Hole_path = [18, 17, 16] + Hole_path
    if (id == 2):
        Hole_path = [15, 16] + Hole_path
    """
    """
    if(id == 0):
        Hole_path = [15, 14, 2, 1, 2, 3, 4, 5, 4, 6, 4, 3, 7, 8, 72, 8, 9, 71, 9, 10, 11, 10, 12, 70, 12, 13, 14, 13,
                     18, 19, 18, 17, 16, 17, 20, 68, 20, 21, 22, 67, 22, 7, 23, 24, 25, 24, 26, 24, 23, 27, 28, 29, 30, 63, 30, 31, 32,
                     33, 32, 34, 32, 31, 35, 36, 62, 36, 37, 61, 37, 38, 39, 38, 40, 60, 40, 41, 29, 41, 42, 59, 42, 43, 44, 58, 44, 45,
                     46, 57, 46, 35, 47, 48, 49, 48, 50, 48, 47, 51, 56, 51, 52, 45, 52, 53, 43, 53, 54, 55, 54, 28, 27, 64, 66, 64, 65,
                     21, 65, 16, 15, 69]
    if (id == 1):
        Hole_path = [15, 14, 2, 1, 2, 3, 4, 5, 4, 6, 4, 3, 7, 8, 72, 8, 9, 71, 9, 10, 11, 10, 12, 70, 12, 13, 14, 13,
                     18, 19, 18, 17, 16, 17, 20, 68, 20, 21, 22, 67, 22, 7, 23, 24, 25, 24, 26, 24, 23, 27, 28, 29, 30, 63, 30, 31, 32,
                     33, 32, 34, 32, 31, 35, 36, 62, 36, 37, 61, 37, 38, 39, 38, 40, 60, 40, 41, 29, 41, 42, 59, 42, 43, 44, 58, 44, 45,
                     46, 57, 46, 35, 47, 48, 49, 48, 50, 48, 47, 51, 56, 51, 52, 45, 52, 53, 43, 53, 54, 55, 54, 28, 27, 64, 66, 64, 65,
                     21, 65, 16, 15, 69]
    if (id == 2):
        Hole_path = [18, 19, 18, 17, 16, 17, 20, 68, 20, 21, 22, 67, 22, 7, 23, 24, 25, 24, 26, 24, 23, 27, 28, 29, 30,
                     63, 30, 31, 32, 33, 32, 34, 32, 31, 35, 36, 62, 36, 37, 61, 37, 38, 39, 38, 40, 60, 40, 41, 29, 41, 42, 59, 42, 43, 44, 58, 44, 45,
                     46, 57, 46, 35, 47, 48, 49, 48, 50, 48, 47, 51, 56, 51, 52, 45, 52, 53, 43, 53, 54, 55, 54, 28, 27, 64, 66, 64, 65,
                     21, 65, 16, 15, 69, 15, 14, 2, 1, 2, 3, 4, 5, 4, 6, 4, 3, 7, 8, 72, 8, 9, 71, 9, 10, 11, 10, 12, 70, 12, 13, 14,
                     13]
     """

    # Publish the History to the centralized communication simulator in order to keep it actualized
    Hmsg = History()
    Hmsg.id, Hmsg.specs, Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge, Hmsg.nextNode, Hmsg.pose, Hmsg.lastMeeting, Hmsg.abort_curr_edge, Hmsg.available = id, [Vd, Vs], H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], EdgeMap[Hole_path[0]-1][Hole_path[1]-1], Hole_path[0], pose, [], False, True
    Hmsg.Whole_path = Hole_path
    pub_hist.publish(Hmsg)


    #Atributting initial values to some variables
    time_start = 0
    T = -2 #"initial flag value"
    pathNode, cx, cy, p, signal, new_task, new_path = [],[],[],[],[],[],[]
    edge = -1
    #waitting_new_plan = False
    finish_edge = False

    pop_all_edges_flag = False

    global stop_the_robot
    stop_the_robot = False


    #Main loop
    while not rospy.is_shutdown():

        count = count + 1 #counter

        time = count / float(freq) #variable that counts time

        # If there is no need to do replanning, just keep moving through the current edge
        if not replan_tasks:
            [H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge, pop_all_edges_flag] = myLib.keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge, pop_all_edges_flag)
        # If it is necessary to do replanning ...
        else:
            # First, just keep moving through the current edge only to finish it
            if(not finish_edge):
                [H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge, pop_all_edges_flag] = myLib.keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge, pop_all_edges_flag)
                if change_edge:
                    finish_edge = True
                    print '\nEdge finished\n\nWaitting new plan broadcast\n'
                    VX, WZ = 0, 0
                    vel.linear.x, vel.angular.z = VX, WZ
                    pub_stage.publish(vel)
            else:

                    VX, WZ = 0, 0
                    vel.linear.x, vel.angular.z = 0, 0
                    pub_stage.publish(vel)

        if end_flag:
            # Terminate the execution
            vel.linear.x, vel.angular.z = 0, 0
            pub_stage.publish(vel)
            break
        # --------------------------------------------------------------------------------

        Threshold = 0.20  # if the distance of robot is less than threshold
        [SP, closeFlag, SP_id, SP_edge] = myLib.CheckOnSP(pose[0:2], SP, SP_fix, Threshold)


        if(len(H['e_v']) > 0 and not stop_the_robot):
            if (closeFlag and SP_edge == (H['e_v'])[-1] and not abort_curr_edge):
                print 'Robot searching in a search point', str(SP_id), '...'
                print 'SP_edge = ', SP_edge
                cnt = 0
                while (cnt < 2 * pi):  # robot rotates and search based on its Searching Speed
                    # Publish the computed speed to stage
                    rate.sleep()
                    vel.linear.x, vel.angular.z = 0, Vs
                    pub_stage.publish(vel)
                    cnt = cnt + (1.0 / freq) * Vs
                IL = Intlist()
                IL.data = [SP_id, id]
                pub_SP.publish(IL)


        #Publish the computed speed to stage
        if not stop_the_robot:
            vel.linear.x, vel.angular.z = VX, WZ
            pub_stage.publish(vel)


        #Write the computed velocities to a txt file
        if not file_vel.closed:
            mystr = str(VX) + "\t" + str(WZ) + "\t" + "\t" + str(time) + "\n"
            file_vel.write(mystr)


        #Publish the History to the centralized communication simulator in order to keep it actualized
        Hmsg = History()
        Hmsg.id, Hmsg.specs, Hmsg.e_v, Hmsg.e_uv, Hmsg.e_g, Hmsg.T_a, Hmsg.T_f, Hmsg.currEdge, Hmsg.nextNode, Hmsg.pose, Hmsg.lastMeeting, Hmsg.abort_curr_edge = id, [Vd, Vs], H['e_v'], H['e_uv'], H['e_g'], H['T_a'], H['T_f'], edge, pathNode[0], pose, H['lastMeeting'], False
        Hmsg.Whole_path = Hole_path
        Hmsg.available = H['available']
        pub_hist.publish(Hmsg)


        #Waitting a replan AND new plan received AND current edge finished
        if (replan_tasks and new_plan_flag and finish_edge):
            finish_edge = False
            new_plan_flag = False
            H = copy.deepcopy(H_new)
            Hole_path = copy.deepcopy(Hole_path_new)
            replan_tasks = False
            new_task = 1
            abort_curr_edge = False

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

    #Read the robots index
    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe robot id was not provided'
    else:
        id = int(sys.argv[1])

    print '__________ __________ __________ __________ __________ __________ __________'
    print 'Robot ', id
    print '\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n'

    #Define the characteristics of the robot given its ID
    Vs = [pi / 2.0, pi / 2.0, pi / 3.0, pi / 2.0] #search speeds (rad/s) maximum is pi/2
    Vd = [0.4, 0.55, 0.5, 0.4] #moving speeds (m/s)
    Vs = Vs[id]/1.1
    Vd = Vd[id]/1.1

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
    #original_graph = myLib.read_graph('Original_graph_35.mat')
    #original_graph = myLib.read_graph('Original_graph_A1.mat')
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
    H = {'id': id, 'specs': [Vd, Vs], 'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f, 'lastMeeting': lastMeeting, 'Whole_path': [], 'available': True}
    H_new = {'id': id, 'specs': [Vd, Vs], 'e_v': e_v, 'e_uv': e_uv, 'e_g': e_g, 'T_a': T_a, 'T_f': T_f, 'lastMeeting': lastMeeting, 'Whole_path': [], 'available': True}


    
    # Read the virtual graph
    #virtual_graph = myLib.read_graph('Virtual_graph_35.mat')
    #virtual_graph = myLib.read_graph('Virtual_graph_A1.mat')
    virtual_graph = myLib.read_graph("Virtual_graph_"+EXP_NAME+".mat")

    # Read search point sets
    #SP = myLib.ReadSearchPoints("Map_35_SP.txt") # points will be removed from this list
    #SP_fix = myLib.ReadSearchPoints("Map_35_SP.txt") # this list wont be changed
    #SP = myLib.ReadSearchPoints("Map_A1_SP.txt")  # points will be removed from this list
    #SP_fix = myLib.ReadSearchPoints("Map_A1_SP.txt")  # this list wont be changed
    SP = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")  # points will be removed from this list
    SP_fix = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")  # this list wont be changed

    SP_visited = [] # As search points are visited they will be placed in this list

    global stop_the_robot
    stop_the_robot = False


    #Clear File
    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/text/visited_' + str(id) + '.txt'
    FILE = open(path, 'w')
    FILE.close()

    # Start running Algorithm 1
    try:
        Algorithm_1()
    except rospy.ROSInterruptException:
        pass


    # Close the txt files
    file_path.close()
    file_vel.close()
