#!/usr/bin/env python
import rospy
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
#from time import sleep, time
import time
#from pylab import *
import pylab
from lpsolve55 import *
from lp_maker import *
import rospkg
import scipy.io
import copy
from geometry_msgs.msg import Twist

import sys
rp = rospkg.RosPack()
path = rp.get_path('distributed')
sys.path.insert(0, path + '/CPP_MIT')
import postman as CPPlib
import MST


EXP_NAME = rospy.get_param('EXP_NAME')







# Function to define the closest node from a given position
def get_current_node(graph,pose):
    nodes = graph['nodes']

    close_node = 0
    current_dist = (pose[0]-nodes[0][0])**2 + (pose[1]-nodes[0][1])**2
    for k in range(1,len(nodes)):
        dist = (pose[0] - nodes[k][0]) ** 2 + (pose[1] - nodes[k][1]) ** 2
        if(dist < current_dist):
            close_node = k
            current_dist = dist

    return close_node, current_dist ** 0.5
# ----------  ----------  ----------  ----------  ----------




# Function to write a list of tuples to describe a sub-graph
def write_listOfTuples(graph,list_active):

    PolC = graph['PolC']

    list_tuple = []
    for k in range(len(PolC[0])):
        if (k + 1 in list_active):
            [fr, to, cx, cy, cost] = getCoefs(k, PolC)
            list_tuple.append((fr, to, cost))

    return list_tuple
# ----------  ----------  ----------  ----------  ----------



# Function to read the data of the graph
def read_graph(name):

    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/graph/' + name
    mat = scipy.io.loadmat(path)
    g = mat['graph']
    n = g['number_nodes']
    n = n.tolist()
    n = n[0][0]
    n = n.tolist()
    n = n[0][0]
    nodes = g['node_list']
    nodes = nodes.tolist()
    nodes = nodes[0][0]
    nodes = nodes.tolist()
    C = g['edge_matrix']
    C = C.tolist()
    C = C[0][0]
    C = C.tolist()
    Ccom = g['complete_edge_matrix']
    Ccom = Ccom.tolist()
    Ccom = Ccom[0][0]
    Ccom = Ccom.tolist()
    C_sp = g['complete_SP_matrix']
    C_sp = C_sp.tolist()
    C_sp = C_sp[0][0]
    C_sp = C_sp.tolist()
    EdgeMap = g['map_edge_matrix']
    EdgeMap = EdgeMap.tolist()
    EdgeMap = EdgeMap[0][0]
    EdgeMap = EdgeMap.tolist()
    PathM = g['path_matrix']
    PathM = PathM.tolist()
    PathM = PathM[0][0]
    PathM = PathM.tolist()
    w_s = g['w_s']
    w_s = w_s.tolist()
    w_s = w_s[0][0]
    w_s = w_s.tolist()
    w_s = w_s[0]
    PolC = g['Pol_coefs']
    PolC = PolC.tolist()
    PolC = PolC[0][0]
    PolC = PolC.tolist()


    output = {'n': n,
              'nodes': nodes,
              'C': C,
              'Ccom': Ccom,
              'C_sp': C_sp,
              'EdgeMap': EdgeMap,
              'PathM': PathM,
              'w_s': w_s,
              'PolC': PolC
              }

    return output
# ----------  ----------  ----------  ----------  ----------







# Function to get the path (sequency of nodes) between to nodes (meybe not drirectly connected)
def getNodePath(i,j,PathM):

    pathNode = PathM[i][j]
    pathNode = pathNode[0]
    pathNode = pathNode.tolist()
    pathNode = pathNode[0]

    return pathNode
# ----------  ----------  ----------  ----------  ----------







# Function to get the polynomial coefficients
def getCoefs(edge,PolC):
    P = PolC[0][edge]

    #get "from" node
    fr = P[0]
    fr = fr.tolist()
    fr = fr[0]
    fr = fr[0]

    #get "to" node
    to = P[1]  # [0, 1, 2, 3, 4] equiv [from, to, cx, cy, cost ]
    to = to.tolist()
    to = to[0]
    to = to[0]

    #get coefficients in x direction
    cx = P[2]
    cx.tolist()
    cx = np.matrix(cx)
    cx = cx.T
    cx = cx.tolist()
    cx = cx[0]

    #get coefficients in y direction
    cy = P[3]
    cy.tolist()
    cy = np.matrix(cy)
    cy = cy.T
    cy = cy.tolist()
    cy = cy[0]

    #get cost of the edge
    cost = P[4]
    cost = cost.tolist()
    cost = cost[0]
    cost = cost[0]

    return fr, to, cx, cy, cost
# ----------  ----------  ----------  ----------  ----------





# Function to get the number of an edge given two adjacent nodes
def getEdge(i,j,PolC):


    for k in range(len(PolC[0])):
        [fr, to, cx, cy, cost] = getCoefs(k,PolC)
        if (fr == i and to == j):
            return k, 1
        elif (fr == j and to == i):
            return k, -1

    print '\n!! --- Ther is no direct path between i and j --- !!\n'
    return 0
# ----------  ----------  ----------  ----------  ----------




# Feedback linearization
def feedback_linearization(Ux, Uy, pose, d):

    theta_n = pose[2]

    VX = cos(theta_n) * Ux + sin(theta_n) * Uy
    WZ = (-sin(theta_n) / d) * Ux + (cos(theta_n) / d) * Uy

    return (VX, WZ)
# ----------  ----------  ----------  ----------  ----------




# Compute the necessary velocity of a robot in order to make it
def compute_velocity(cx, cy, p, dt, signal, pose, Vd, Kp):

    xref = 0
    yref = 0
    for k in range(6):  # Compute refference position
        xref = xref + cx[6 - k - 1] * p ** k
        yref = yref + cy[6 - k - 1] * p ** k
    vx = 0
    vy = 0
    for k in range(1, 6):  # Compute refference velocity (diect evaluatioin of the polynimial)
        vx = vx + (k) * cx[6 - k - 1] * p ** (k - 1)
        vy = vy + (k) * cy[6 - k - 1] * p ** (k - 1)

    # Atualization of p such as the speed is constant
    v = sqrt((vx) ** 2 + (vy) ** 2)
    p = p + ((dt) * (Vd / v)) * signal

    # Compuete the speeds 'vx' and 'vy' for the feed forward such the resultant speed is constant
    vx_ff = (Vd * (vx / v)) * signal
    vy_ff = (Vd * (vy / v)) * signal

    # Compute the controller signals
    ux = Kp * (xref - pose[0]) + vx_ff
    uy = Kp * (yref - pose[1]) + vy_ff

    return (ux, uy, p)
# ----------  ----------  ----------  ----------  ----------





#Function to apply a repulsive velocity and avoid collisions
def repulsive_potential(laserVec, pose, ux, uy):

    index = laserVec.index(min(laserVec))
    do = 0.35*2/1.5*1
    if laserVec[index] < do:
        phi = (-135 + index) * pi / 180.0  # angle of the object in the local frame
        theta = pose[2]
        beta = phi + theta  # angle of the object in the world frame
        grad_ob = [cos(beta), sin(beta)]
        gain = 0.15*(1/(laserVec[index]+0.000001) - 1/do)*(1/(laserVec[index]**1+0.000001))
        vx_repulsive0 = -gain * grad_ob[0]
        vy_repulsive0 = -gain * grad_ob[1]

        #Rotate the repulsive vector
        #Heuristcs for making each robot deviate from on side
        angle = 45
        angle = angle * (1-(135/90)*abs(index-135)/135.0)
        angle = angle * pi / 180
        vx_repulsive = cos(angle) * vx_repulsive0 - sin(angle) * vy_repulsive0
        vy_repulsive = sin(angle) * vx_repulsive0 + cos(angle) * vy_repulsive0


        ux = ux + vx_repulsive
        uy = uy + vy_repulsive
        #print '                                       !!!!! Repulsive potential active !!!!!'


    return ux, uy
# ----------  ----------  ----------  ----------  ----------


# When a robot ends its taks it pops all edges to attempt to find the object
def pop_all_edges(H):


    #Select edges

    #Transform to nodes

    #Run MST

    #Run CPP


    return
# ----------  ----------  ----------  ----------  ----------



#def keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge,pop_all_edges_flag, pub_stage):
def keep_moving(H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, original_graph, freq, pose, laserVec, d, Vd, Kp, id, edge, pub_stage):

    C = original_graph['C']
    PathM = original_graph['PathM']
    PolC = original_graph['PolC']
    EdgeMap = original_graph['EdgeMap']

    VX = 0
    WZ = 0

    end_flag = False
    change_edge = False #Indicated a change on edge - useful to force robots to finish a edge before accept a replan

    if time - time_start > T + 1 / freq:
        change_edge = True
        time_start = time
        if len(pathNode) > 1:
            new_path = 1 #robot ended a direct path only
        else:
            new_task = 1 #robot ended a complete path as well


    if new_task == 1:
        if len(Hole_path) > 1:
            i = Hole_path.pop(0)
            j = Hole_path[0]
            pathNode = getNodePath(i - 1, j - 1, PathM)

            new_task = 0
            new_path = 1
        else:
            print'\nNodes search completed\n'
            new_path = 1 #In theory,len(Hole_path) <= 1 implies len(e_uv) = 0 !!!!!!!!!!!!!!!!!!!!!!


    if new_path == 1:
        #if ((len(H['e_uv']) == 0 and not pop_all_edges_flag)):
        if ((len(H['e_uv']) == 0 and not H['popped_edges'])):
            #pop_all_edges_flag = True
            H['popped_edges'] = True
            #H['available'] = False
            # H['popped_edges'] = True

            print '\n\n----------  ----------  ----------\nPOPPING ALL EDGES\n----------  ----------  ----------\n'
            for kk in range(len(PolC[0])):
                print kk+1,'/',len(PolC[0])
                if not (kk + 1 in H['e_v']) and not (kk + 1 in H['T_f']):
                    H['e_uv'].append(kk + 1)
                    print kk + 1, '/', len(PolC[0]), ' included'
                else:
                    print kk + 1, '/', len(PolC[0])
            if(len(H['e_uv']) == 0):
                print'\n----------  All the edges were already visited  ----------\n'
                end_flag = True
                VX, WZ = 0, 0
                #return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge, pop_all_edges_flag
                return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge
            H['e_g'] = []
            print "Here is H['e_uv']"
            print H['e_uv']
            print 'Here is pose', pose
            curr_node, lixo = get_current_node(original_graph,pose)
            curr_node = curr_node+1
            print 'Here is curr_node', curr_node
            vel = Twist()
            pub_stage.publish(vel)
            connected_subgraph = MST.MSTconnect(original_graph,H['e_uv'], curr_node, 'k', False)
            edges_listOfTuples = write_listOfTuples(original_graph, connected_subgraph)
            Hole_path = CPPlib.main_CPP(sorted(edges_listOfTuples), curr_node)
            print 'Here is Hole path'
            print Hole_path
            pathNode = [Hole_path[0],Hole_path[1]]
            i = Hole_path.pop(0)
            j = Hole_path[0]
            pathNode = getNodePath(i - 1, j - 1, PathM)
            new_task = 0
            new_path = 1
        #elif (len(H['e_uv']) == 0 and pop_all_edges_flag):
        #elif (len(H['e_uv']) == 0 and H['popped_edges']):
        elif (len(H['T_f']) == len(PolC[0])):
            print'\n----------  All popped edges were visited  ----------\n'
            end_flag = True
            #return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge, pop_all_edges_flag
            return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge

        i = pathNode.pop(0)
        j = pathNode[0]
        T = C[i - 1][j - 1] / Vd #This is valid because i and j are always direct neighbors
        [edge, signal] = getEdge(i, j, PolC)
        [fr, to, cx, cy, cost] = getCoefs(edge, PolC)
        if signal == 1:
            p = 0 #start the edge (polynomial) at the beginning and move to the end
        elif signal == -1:
            p = 1 #start the edge (polynomial) at the end and move to the beginning
        new_path = 0

        # Update the History
        edge = EdgeMap[i - 1][j - 1]
        # Remove the edge from the list of unvisited nodes
        if edge in H['e_uv']:
            H['e_uv'].pop(H['e_uv'].index(edge))
            # Add the edge to the list of visited edges
            if not edge in H['e_v']:
                H['e_v'].append(edge)
                #Write in file that the edge was searched for posterior analysis
                rp = rospkg.RosPack()
                path = rp.get_path('distributed')
                path = path + '/text/visited_' + str(id) + '.txt'
                FILE = open(path, 'a')
                FILE.write(str(edge)+'\n')
                FILE.close()
            if not edge in H['T_f']:
                H['T_f'].append(edge)


        #Print information on screen
        print '\nRobot ' + str(id)
        print 'e_v:\n', H['e_v']
        print 'e_uv:\n', H['e_uv']
        print 'e_g:\n', H['e_g']
        print 'T_f:\n', H['T_f']
        print 'Whole_path:\n', Hole_path


    else:
        # Compute the velocity according to the polynomial edge
        [ux, uy, p] = compute_velocity(cx, cy, p, 1.0 / freq, signal, pose, Vd, Kp)

        # Apply th repulsive potential to avoid collision
        [ux, uy] = repulsive_potential(laserVec, pose, ux, uy)

        # Apply feedback linearization
        [VX, WZ] = feedback_linearization(ux, uy, pose, d)


    #return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge, pop_all_edges_flag
    return H, time, time_start, T, pathNode, Hole_path, cx, cy, p, signal, new_task, new_path, VX, WZ, end_flag, edge, change_edge
# ----------  ----------  ----------  ----------  ----------




def CheckOnSP(pos,SP,SP_fix,Threshold):

    posn = np.array([pos[0], pos[1]])
    d1 = (posn - SP[:,0:2]) ** 2
    d=np.sqrt(d1[:,0]+d1[:,1])
    for i in range(0,len(SP[:,0])):
        if (d[i]<float(Threshold)):
            SP_id = SP_fix[:,0:2].tolist().index(SP[i,0:2].tolist()) + 1
            SP_edge = int(SP[i,2:3])
            SPnew = np.delete(SP[:, 0:3], i, 0)
            return SPnew,1,SP_id,SP_edge

    return SP,0,-1,-1 #No search point close
# ----------  ----------  ----------  ----------  ----------




def ReadSearchPoints(name):

    rp = rospkg.RosPack()
    path = rp.get_path('distributed')
    path = path + '/maps/' + name
    SPfile=open(path,"r")
    line_list=SPfile.readlines()
    SP=[[float(val) for val in line.split()] for line in line_list[0:] ]
    SP1=np.array(SP)

    return SP1
# ----------  ----------  ----------  ----------  ----------



def get_cost(graph, H, v, vs):

    # H - History list
    # v - velocity list
    # vs - search velocity speed

    R = len(v)
    HP = [] #path
    for r in range(R):
        HP.append(H[r].Whole_path)

    Ccom = graph['Ccom']

    c = [0 for r in range(R)]
    for r in range(R):
        for k in range(len(HP[r]) - 1):
            c[r] = c[r] + Ccom[HP[r][k] - 1][HP[r][k + 1] - 1] / v[r]  # Length cost


    SP_0 = ReadSearchPoints("Map_"+EXP_NAME+"_SP.txt")

    for k in range(len(SP_0)):
        for r in range(R):
            if SP_0[k, 2] in H[r].e_uv:
                c[r] = c[r] + 2 * pi / vs[r]  # Search cost

    C = max(c)

    return C
# ----------  ----------  ----------  ----------  ----------





# BFS Algorithm ----------  ----------  ----------  ----------
def BFS(v, BM):
    if (len(BM) != len(BM[1])):
        print 'ERROR - Matrix is not square !!!'

    N = len(BM)
    BM[v][v] = 1
    O = [[v, v]]  # v is father of v
    C = []

    while (O):  # while o is not empty
        u = O[0][0]
        O.pop(0)

        if (not u in C):
            C.append(u)

            for i in range(N):
                if (BM[u][i] == 1):
                    if (not i in C):
                        O.append([i, u])

    return C


# ----------  ----------  ----------  ----------  ----------



# Tarjan's Algorithm ----------  ----------  ----------  ----------
def TarjansAlg(BM):
    N = len(BM)

    placed = [0 for i in range(N)]
    CG = []

    for k in range(N):
        if placed[k] == 0:
            C = BFS(k, BM)
            CG.append(C)
            for l in C:
                placed[l] = 1

    return CG

# ----------  ----------  ----------  ----------  ----------
