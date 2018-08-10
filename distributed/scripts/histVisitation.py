#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
from random import randrange
from time import sleep
import tf
import pylab
import sys
import rospkg

import matplotlib.pyplot as plt




def read_files():
    global R, E

    global List, List_all




    for r in range(R):
        rp = rospkg.RosPack()
        path = rp.get_path('distributed')
        path = path + '/text/visited_' + str(r) + '.txt'
        myfile = open(path, "r")

        line_list = myfile.readlines()
        line_list = [int(val) for val in line_list[0:]]
        List[r] = line_list

    List_all = []
    for r in range(R):
        List_all = List_all + List[r]

    #print 'Here is List_all:\n', List_all

    flag = True
    for r in range(E):
        if((r+1) not in List_all):
            flag = False
            break
    #print 'flag = ', flag
    if flag:
        print '\n----------  ----------  ----------\nAll edges seached\n----------  ----------  ----------\n'

# ---------- !! ---------- !! ---------- !! ---------- !! ----------





# Rotina primaria
def hist():

    global freq
    global R, E
    global List, List_all


    #pub_pose = rospy.Publisher("/marker_pose", Marker, queue_size=1)
    #pub_circ0 = rospy.Publisher("/marker_circ0", MarkerArray, queue_size=1)
    #pub_targ = rospy.Publisher("/marker_targ", Marker, queue_size=1)
    rospy.init_node("histogram")
    #rospy.Subscriber("/robot_0/base_pose_ground_truth", Odometry, callback_pose)
    #rospy.Subscriber("/robot_0/base_scan", LaserScan, callback_laser)


    freq = 1.0  # Hz
    rate = rospy.Rate(freq)

    sleep(1)


    List = []
    for r in range(R):
        List.append([])


    pylab.figure(50)
    pylab.xlim(0, E)
    pylab.ylim(0, 4)
    pylab.xlabel('Edges')
    pylab.ylabel('Visitations')
    pylab.title('Histogram of visitations')

    #x = [1, 2, 3, 4, 5, 6, 7, 8, 9, 1, 2, 3, 6, 7, 8, 23, 43, 23, 54, 43, 3, 23, 32, 34, 34]

    pylab.ion()
    pylab.show()

    sleep(38)

    while not rospy.is_shutdown():

        #Read file

        #x.append(E * randrange(0, 1000, 1) / 1000)
        #print 'Here\n',x

        pylab.clf()
        pylab.figure(50)
        pylab.xlim(0, E)
        pylab.ylim(0, 4)
        pylab.xlabel('Edges')
        pylab.ylabel('Visitations')
        pylab.title('Histogram of visitations')

        read_files()
        pylab.hist(List_all, E, color='b')

        #pylab.ion()
        #pylab.show()
        pylab.draw()

        rate.sleep()


# ---------- !! ---------- !! ---------- !! ---------- !! ----------






# Funcao inicial
if __name__ == '__main__':

    global R, E

    #Read the target index
    if len(sys.argv) < 3:
        #print 'ERROR!!!\nThe number of robots was not informed'
        print 'ERROR!!!\nInform the number of robots and the number of edges'
    else:
        R = int(sys.argv[1])
        E = int(sys.argv[2])
        #E = int(sys.argv[1])


    try:
        hist()
    except rospy.ROSInterruptException:
        pass



