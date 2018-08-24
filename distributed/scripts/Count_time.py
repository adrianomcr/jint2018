#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from distributed.msg import History, HistList, Broadcast, Intlist
from std_msgs.msg import Int16
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, floor
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
import time

import library2018 as myLib



global SP # search points
global tempo
tempo = 0



global list_SPS
list_SPS = [] #[[SP_ID, robot_id, time], [], [], ...]


# Callback routine to obtain the laser information
def callback_SP(data):

    global list_SPS, tempo
    global id_SP_target


    new_data = list(data.data) + [tempo]

    list_SPS.append(new_data)

    """
    print 'Robot', str(new_data[1]), 'searching SP', str(new_data[0]), 'at time', str(new_data[2]),'\n'
    """

    global path_SP
    FILE = open(path_SP, 'a')
    FILE.write(str(new_data[0]) + ' ' + str(new_data[1]) + ' ' + str(new_data[2]) + '\n')
    FILE.close()

    """
    #Choose this to terminate when a given search point is achieved
    if(new_data[0] == id_SP_target):
        print '\n\n----------  ----------  ----------  ----------  ----------  ----------'
        print 'Robot', str(new_data[1]), 'foud the target in search point', str(new_data[0]), 'at time', str(new_data[2])
        print '----------  ----------  ----------  ----------  ----------  ----------\n\n'
        sleep(0.5)
        string_cmd = "gnome-terminal -x bash -c 'killall python'"
        os.system(string_cmd)
    """

    """
    #Choose this to indicate when all search points are achieved
    global SP_LIST
    if new_data[0] in  SP_LIST:
        rm_id = SP_LIST.index(new_data[0])
        SP_LIST.pop(rm_id)
        if len(SP_LIST) <= 5:
            print '\n\n----------  ----------  ----------  ----------  ----------  ----------'
            print 'Task finished in', str(tempo), 'seconds'
            print '----------  ----------  ----------  ----------  ----------  ----------\n\n'
    """


    return
# ----------  ----------  ----------  ----------  ----------





# Primary routine
def Time_counter():

    global pose
    global pub_rviz, pub_targ, pub_pose
    global freq
    global tempo, i
    global file_vel
    global N
    global n, nodes, C, PathM, w_s, PolC
    global Vd
    global laserVec
    global id
    global replan_tasks, finish_edge
    global H
    global list_of_H, list_of_robs, list_of_vels
    global Hole_path
    global waitting_new_plan
    global new_task
    global original_graph
    global SP


    count = 0

    my_str = 'time_counter'
    rospy.init_node(my_str) #Initialize the node
    rospy.Subscriber('/searched_point',Intlist,callback_SP) #Eventually receive a new route


    freq = 10.0  # Frequency of operation in Hz
    rate = rospy.Rate(freq)

    global SP_LIST
    #SP_LIST = myLib.ReadSearchPoints("Map_36_SP.txt")
    #SP_LIST = myLib.ReadSearchPoints("Map_A1_SP.txt")
    SP_LIST = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")
    SP_LIST = [k + 1 for k in range(len(SP_LIST))]
    #SP_LIST = [k + 1 for k in range(76)]

    #print '\nHere is initial SP_LIST', SP_LIST, '\n'

    #Main loop
    while not rospy.is_shutdown():

        count = count + 1 #counter

        tempo = count / float(freq) #variable that counts time

        if count % 10 == 0:
            #print 'time: ', tempo
            mm = int(tempo / 60)
            ss = int((tempo-60*mm))

            if mm<10:
                myStr = "0" + str(mm)
            else:
                myStr = str(mm)
            if ss<10:
                myStr = myStr + ":0" + str(ss)
            else:
                myStr = myStr + ":" + str(ss)

            print 'time (mm:ss): ', myStr, '\t\t\ttime (ss): ', tempo

        #Wait
        rate.sleep()





# ---------- !! ---------- !! ---------- !! ---------- !! ----------









# Initial function
if __name__ == '__main__':



    print '__________ __________ __________ __________ __________ __________ __________'
    print '\n\n\n\n\n\n\n\n\n\n\n'
    print '----------  Time counter ----------'

    global id
    global number_robots #Used only foor creating the topics

    global id_SP_target

    global EXP_NAME

    #"""
    #Read the robots index
    if len(sys.argv) < 2:
        print 'ERROR!!!\nThe id of the SP where the object is was not provided'
    else:
        id_SP_target = int(sys.argv[1])
    #"""

    EXP_NAME = rospy.get_param('EXP_NAME')

    # Read search point set
    #SP = myLib.ReadSearchPoints("Map_36_SP.txt")
    #SP = myLib.ReadSearchPoints("Map_35_SP.txt")
    #SP = myLib.ReadSearchPoints("Map_A1_SP.txt")
    SP = myLib.ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")



    #"""
    #Clear File
    global path_SP
    hour = time.strftime("%Hh%Mm%Ss")
    date = time.strftime("d%dm%my%Y")
    rp = rospkg.RosPack()
    path_SP = rp.get_path('distributed')
    path_SP = path_SP + '/resultsSP/Results_'+date+'_'+hour+'.txt'
    FILE = open(path_SP, 'w')
    FILE.close()
    #"""

    # Start running Algorithm 1
    try:
        Time_counter()
    except rospy.ROSInterruptException:
        pass
