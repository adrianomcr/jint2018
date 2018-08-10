#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist
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




global pose_0
pose_0 = [-10, 20, 0] #[x ,y, theta]
global pose_1
pose_1 = [20, -10, 0] #[x ,y, theta]
global pose_2
pose_2 = [-10, -10, 0] #[x ,y, theta]
global pose_3
pose_3 = [20, 20, 0] #[x ,y, theta]


global rho # Radius of communication
rho = 4
global DIST_INTO # Radius to enter in the communication graph
global DIST_LEAVE # Radius to leave the communication graph





#DIST_INTO = -0.5
#DIST_LEAVE = 30.0
#DIST_INTO = 1.0
#DIST_LEAVE = 2.5
#DIST_INTO = 1.5
#DIST_LEAVE = 3.0
#DIST_INTO = 2.5/1.0
#DIST_LEAVE = 3.5/1.0
#DIST_INTO = 3.5
#DIST_LEAVE = 4.5
#DIST_INTO = 4.5
#DIST_LEAVE = 5.5


DIST_INTO = rospy.get_param('DIST_INTO')
DIST_LEAVE = rospy.get_param('DIST_LEAVE')


global flag_0, flag_1, flag_2, flag_3
global flag_h0, flag_h1, flag_h2, flag_h3
flag_0 = 0
flag_1 = 0
flag_2 = 0
flag_3 = 0
flag_h0 = 0
flag_h1 = 0
flag_h2 = 0
flag_h3 = 0


# Callback routine to obtain the pose of robot 0
def callback_pose_0(data):

    global pose_0
    global flag_0

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_0[0] = x
    pose_0[1] = y
    pose_0[2] = theta

    flag_0 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 1
def callback_pose_1(data):

    global pose_1
    global flag_1

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_1[0] = x
    pose_1[1] = y
    pose_1[2] = theta

    flag_1 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 2
def callback_pose_2(data):

    global pose_2
    global flag_2

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_2[0] = x
    pose_2[1] = y
    pose_2[2] = theta

    flag_2 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the pose of robot 2
def callback_pose_3(data):

    global pose_3
    global flag_3

    x = data.pose.pose.position.x  # posicao 'x' do robo no mundo
    y = data.pose.pose.position.y  # posicao 'y' do robo no mundo
    x_q = data.pose.pose.orientation.x
    y_q = data.pose.pose.orientation.y
    z_q = data.pose.pose.orientation.z
    w_q = data.pose.pose.orientation.w
    euler = euler_from_quaternion([x_q, y_q, z_q, w_q])
    theta = euler[2]  # orientaco 'theta' do robo no mundo

    pose_3[0] = x
    pose_3[1] = y
    pose_3[2] = theta

    flag_3 = 1

    return
# ----------  ----------  ----------  ----------  ----------



# Callback routine to obtain the history of robot 0
def callback_hist_0(data):

    global H_all
    global flag_h0

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

    flag_h0 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 1
def callback_hist_1(data):
    global H_all
    global flag_h1

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

    flag_h1 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 2
def callback_hist_2(data):

    global H_all
    global flag_h2

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

    flag_h2 = 1

    return
# ----------  ----------  ----------  ----------  ----------

# Callback routine to obtain the history of robot 3
def callback_hist_3(data):

    global H_all
    global flag_h3

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

    flag_h3 = 1

    return
# ----------  ----------  ----------  ----------  ----------






def send_message_recompute_close(pub_comm_graph, ids):

    global H_0, H_1, H_2, H_3, HL
    global H_all
    global meeting_matrix

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




# Primary routine
def sensor_simulator():

    global pose_0, pose_1, pose_2, pose_3
    global rho
    global H_0, H_1, H_2, H_3, HL
    global H_all
    global meeting_matrix

    H_0 = {'id': [],
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
    }
    H_1 = copy.deepcopy(H_0)
    H_2 = copy.deepcopy(H_0)
    H_3 = copy.deepcopy(H_0)

    H_all = [H_0,H_1,H_2,H_3]


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


    close_var = Bool()
    close_var.data = False
    HL = HistList()
    HL.comGraphEvent = False

    #Used to implement the "meeting trigger"
    meeting_matrix = [[False,False,False,False],[False,False,False,False],[False,False,False,False],[False,False,False,False]]


    count = 0
    freq = 4.0  # Hz --> smaller than the frequency of the robots
    rate = rospy.Rate(freq)



    # Wait until a first measurement is made
    while flag_0 == 0 or flag_1 == 0 or flag_2 == 0 or flag_h0 == 0 or flag_h1 == 0 or flag_h2 == 0:
        rate.sleep()


    for k in range(10):
        rate.sleep()



    state_matrix = [[False, False, False, False], [False, False, False, False], [False, False, False, False],
                      [False, False, False, False]]

    sleep(2)


    N = rospy.get_param('NUM_OF_ROBOTS')
    #N = 3  # Number of robots
    if N == 1:
        poses = [pose_0]
    elif N == 2:
        poses = [pose_0, pose_1]
    elif N == 3:
        poses = [pose_0, pose_1, pose_2]
    elif N == 4:
        poses = [pose_0, pose_1, pose_2, pose_3]
    else:
        print "!!! ERROR - invalid value for N !!!"


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


        #If the communication graph has changed
        if (not CG==previous_CG):
            """
            print "Different CG=",CG," dif ",previous_CG,"=previous_CG\n"
            """
            previous_CG = CG
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



        rate.sleep()
# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':




    try:
        sensor_simulator()
    except rospy.ROSInterruptException:
        pass

