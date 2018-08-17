#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist, ForbEdges
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import scipy.io
import rospkg
import sys

import copy

import library2018 as myLib



global DIST_INTO # Radius to enter in the communication graph
global DIST_LEAVE # Radius to leave the communication graph
global N
DIST_INTO = rospy.get_param('DIST_INTO')
DIST_LEAVE = rospy.get_param('DIST_LEAVE')
N = rospy.get_param('NUM_OF_ROBOTS')


global poses
poses = [[10*r, 10*r, 0] for r in range(N)]

global flag, flag_h
flag = [0 for r in range(N)]
flag_h = [0 for r in range(N)]

if N == 1:
    flag.append([])
    flag_h.append([])
    poses.append([10, 10, 0]) #pose of the auxiliary robot


global list_T_f
list_T_f = [[] for i in range(N)]
global list_T_f_last
list_T_f_last = [[] for i in range(N)]





# Callback routine to obtain the pose of robot 0
def callback_pose_0(data):

    global poses
    global flag

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    poses[0] = [x,y,theta]

    flag[0] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 1
def callback_pose_1(data):

    global poses
    global flag

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    poses[1] = [x, y, theta]

    flag[1] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 2
def callback_pose_2(data):

    global poses
    global flag

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    poses[2] = [x, y, theta]

    flag[2] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 2
def callback_pose_3(data):

    global poses
    global flag

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    poses[3] = [x, y, theta]

    flag[3] = 1

    return
# ----------  ----------  ----------  ----------  ----------



# Callback routine to obtain the history of robot 0
def callback_hist_0(data):

    global H_all
    global flag_h

    H_all[0]['id'] = data.id
    H_all[0]['specs'] = data.specs
    H_all[0]['e_v'] = data.e_v
    H_all[0]['e_uv'] = data.e_uv
    H_all[0]['e_g'] = data.e_g
    H_all[0]['T_a'] = data.T_a
    H_all[0]['T_f'] = data.T_f
    H_all[0]['currEdge'] = data.currEdge
    H_all[0]['nextNode'] = data.nextNode
    H_all[0]['pose'] = data.pose
    H_all[0]['lastMeeting'] = list(data.lastMeeting)
    H_all[0]['Whole_path'] = list(data.Whole_path)

    H_all[0]['available'] = data.available

    flag_h[0] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 1
def callback_hist_1(data):

    global H_all
    global flag_h

    H_all[1]['id'] = data.id
    H_all[1]['specs'] = data.specs
    H_all[1]['e_v'] = data.e_v
    H_all[1]['e_uv'] = data.e_uv
    H_all[1]['e_g'] = data.e_g
    H_all[1]['T_a'] = data.T_a
    H_all[1]['T_f'] = data.T_f
    H_all[1]['currEdge'] = data.currEdge
    H_all[1]['nextNode'] = data.nextNode
    H_all[1]['pose'] = data.pose
    H_all[1]['lastMeeting'] = list(data.lastMeeting)
    H_all[1]['Whole_path'] = list(data.Whole_path)

    H_all[1]['available'] = data.available

    flag_h[1] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 2
def callback_hist_2(data):

    global H_all
    global flag_h

    H_all[2]['id'] = data.id
    H_all[2]['specs'] = data.specs
    H_all[2]['e_v'] = data.e_v
    H_all[2]['e_uv'] = data.e_uv
    H_all[2]['e_g'] = data.e_g
    H_all[2]['T_a'] = data.T_a
    H_all[2]['T_f'] = data.T_f
    H_all[2]['currEdge'] = data.currEdge
    H_all[2]['nextNode'] = data.nextNode
    H_all[2]['pose'] = data.pose
    H_all[2]['lastMeeting'] = list(data.lastMeeting)
    H_all[2]['Whole_path'] = list(data.Whole_path)

    H_all[2]['available'] = data.available


    flag_h[2] = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 3
def callback_hist_3(data):

    global H_all
    global flag_h

    H_all[3]['id'] = data.id
    H_all[3]['specs'] = data.specs
    H_all[3]['e_v'] = data.e_v
    H_all[3]['e_uv'] = data.e_uv
    H_all[3]['e_g'] = data.e_g
    H_all[3]['T_a'] = data.T_a
    H_all[3]['T_f'] = data.T_f
    H_all[3]['currEdge'] = data.currEdge
    H_all[3]['nextNode'] = data.nextNode
    H_all[3]['pose'] = data.pose
    H_all[3]['lastMeeting'] = list(data.lastMeeting)
    H_all[3]['Whole_path'] = list(data.Whole_path)

    H_all[3]['available'] = data.available

    flag_h[3] = 1

    return
# ----------  ----------  ----------  ----------  ----------





# Function to send the Histories to a set of robots in a connected component of a communication graph
def send_message_recompute_close(pub_comm_graph, ids):

    global HL
    global H_all

    R = len(ids)

    HL = HistList()

    HL.comGraphEvent = True

    # Adding information from the robots
    for r in range(R):
        H = History()
        H.id = H_all[ids[r]]['id']
        H.specs = H_all[ids[r]]['specs']
        H.e_v = H_all[ids[r]]['e_v']
        H.e_uv = H_all[ids[r]]['e_uv']
        H.e_g = H_all[ids[r]]['e_g']
        H.T_a = H_all[ids[r]]['T_a']
        H.T_f = H_all[ids[r]]['T_f']
        H.currEdge = H_all[ids[r]]['currEdge']
        H.nextNode = H_all[ids[r]]['nextNode']
        H.pose = H_all[ids[r]]['pose']
        H.lastMeeting = copy.deepcopy(ids)
        H.Whole_path = H_all[ids[r]]['Whole_path']
        HL.robList.append(H_all[ids[r]]['id'])
        HL.listOfH.append(H)

    pub_comm_graph.publish(HL)

    return
# ----------  ----------  ----------  ----------  ----------






"""
def update_forbidden_edges(CG):

    for c in range(len(CG)):
        ids = CG[c]
        print 'ids = ', ids
        if len(ids):
            updated_list = []
            for id in ids:
                updated_list = updated_list + list(H_all[id]['T_f'])
            updated_list = list(set(updated_list))
            print 'updated_list = ', updated_list
            #print 'updated_list = ', list(set(updated_list))

    return
# ----------  ----------  ----------  ----------  ----------
"""

# Callback function to keep changing
def callback_ForbEdges(data):

    global list_T_f, list_T_f_last

    #print 'data:\n', data

    if (list(data.destinations) == [-1]):
        id = data.sender

        list_T_f_last[id] = copy.deepcopy(list_T_f[id])
        list_T_f[id] = list(set(list_T_f[id] + list(data.forbiden_Edges)))

    return
# ----------  ----------  ----------  ----------  ----------


def update_forbidden_edges(pub_forb,CG):

    global list_T_f

    for c in range(len(CG)):
        ids = CG[c]
        #print 'ids = ', ids
        if len(ids) > 1:
            updated_list = []
            for id in ids:
                updated_list = updated_list + list(list_T_f[id])
            updated_list = list(set(updated_list))
            #print 'Send to robots ', ids,' updated_list = ', updated_list
            #print 'updated_list = ', list(set(updated_list))
            forb_edges_msg = ForbEdges()
            forb_edges_msg.sender = -1
            forb_edges_msg.destinations = list(ids)
            forb_edges_msg.forbiden_Edges = updated_list
            pub_forb.publish(forb_edges_msg)




    return
# ----------  ----------  ----------  ----------  ----------



# Primary routine
def sensor_simulator():

    global HL
    global H_all
    global list_T_f, list_T_f_last

    H_all = []
    for r in range(N):
        H_all.append({'id': [],
         'specs': [],
         'e_v': [],
         'e_uv': [],
         'e_g': [],
         'T_a': [],
         'T_f': [],
         'currEdge': None,
         'lastMeeting': [],
         'Whole_path': [],
         'available': True,
    })



    rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose_0)
    rospy.Subscriber("/robot_1/base_pose_ground_truth", Odometry, callback_pose_1)
    rospy.Subscriber("/robot_2/base_pose_ground_truth", Odometry, callback_pose_2)
    rospy.Subscriber("/robot_3/base_pose_ground_truth", Odometry, callback_pose_3)
    rospy.Subscriber("/robot_0/history", History, callback_hist_0)
    rospy.Subscriber("/robot_1/history", History, callback_hist_1)
    rospy.Subscriber("/robot_2/history", History, callback_hist_2)
    rospy.Subscriber("/robot_3/history", History, callback_hist_3)
    rospy.init_node("sensor_similator")
    pub_comm_graph = rospy.Publisher("/comm_graph", HistList, queue_size=1)

    pub_forb = rospy.Publisher('/forb_edges', ForbEdges, queue_size=10)  #
    rospy.Subscriber("/forb_edges", ForbEdges, callback_ForbEdges)


    close_var = Bool()
    close_var.data = False
    HL = HistList()
    HL.comGraphEvent = False



    count = 0
    freq = 4.0  # Hz --> smaller than the frequency of the robots
    rate = rospy.Rate(freq)



    # Wait until the first measurements are made
    for r in range(N):
        while flag[r] == 0 or flag_h[r] == 0:
            rate.sleep()


    state_matrix = [[False for i in range(N)] for j in range(N)]

    #sleep(2)



    previous_CG = []
    for r in range(N):
        previous_CG.append([])


    while (not rospy.is_shutdown()):

        count = count + 1
        time = count / float(freq)


        M = [[0 for i in range(N)] for j in range(N)]
        BM = [[0 for i in range(N)] for j in range(N)]
        for i in range(N):
            for j in range(N):
                if (i<j):
                    M[i][j] = ((poses[i][0]-poses[j][0])**2+(poses[i][1]-poses[j][1])**2)**0.5

                    #Strategy to disconsider those robots who popped all edges
                    if((not H_all[i]['available']) or (not H_all[j]['available'])):
                        M[i][j] = M[i][j] + 10*DIST_LEAVE


                    if M[i][j] < DIST_INTO:
                        BM[i][j] = 1.0
                        state_matrix[i][j] = True
                    elif M[i][j] < DIST_LEAVE:
                        if state_matrix[i][j] == True:
                            BM[i][j] = 1.0
                    else:
                        state_matrix[i][j] = False
                elif(j<i):
                    M[i][j] = M[j][i]
                    BM[i][j] = BM[j][i]

        CG = myLib.TarjansAlg(BM)

        #update_forbidden_edges(CG)


        if list_T_f != list_T_f_last or not CG==previous_CG:
            list_T_f_last = copy.deepcopy(list_T_f)
            #print 'list_T_f:\n', list_T_f
            #Always that this thing happens we should broadcast to the robots in a comm graph
            update_forbidden_edges(pub_forb,CG)




        # Code to indicate replanning computation
        # ----------  ----------  ----------  ----------  ----------
        #If the communication graph has changed
        if (not CG==previous_CG):
            previous_CG = CG # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            #Loop for all connected coponnents in CG
            for c in range(len(CG)):
                ids = CG[c]

                compute_flag = False
                #If there is more than one robot in the current connected componnent
                if len(ids)>1:
                    # If there is an id in 'ids' that is not in one of the last meetings
                    for id1 in ids:
                        for id2 in ids:
                            if (not (id1 in H_all[id2]['lastMeeting'])):
                                compute_flag = True
                    if compute_flag:
                        send_message_recompute_close(pub_comm_graph, ids)

            for c in range(4*5):
                rate.sleep()
                #print "waitting ..."
        # ----------  ----------  ----------  ----------  ----------


        """
        if list_T_f != list_T_f_last:
            list_T_f_last = copy.deepcopy(list_T_f)
            print 'list_T_f:\n', list_T_f
            #Always that this thing happens we should broadcast to the robots in a comm graph
            update_forbidden_edges(pub_forb,CG)
        """

        #print 'list_T_f:        ', list_T_f
        #print 'list_T_f_last:   ', list_T_f_last

        """
        # Code to constantly share the forbidden edges
        # ----------  ----------  ----------  ----------  ----------
        # If the communication graph has changed
        if (not CG == previous_CG):
            previous_CG = CG
            # Loop for all connected coponnents in CG
            for c in range(len(CG)):
                ids = CG[c]

                compute_flag = False
                # If there is more than one robot in the current connected componnent
                if len(ids) > 1:
                    # If there is an id in 'ids' that is not in one of the last meetings
                    for id1 in ids:
                        for id2 in ids:
                            if (not (id1 in H_all[id2]['lastMeeting'])):
                                compute_flag = True
                    if compute_flag:
                        send_message_recompute_close(pub_comm_graph, ids)
        # ----------  ----------  ----------  ----------  ----------
        """




        rate.sleep()
# ---------- !! ---------- !! ---------- !! ---------- !! ----------







# Initial function
if __name__ == '__main__':



    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass

