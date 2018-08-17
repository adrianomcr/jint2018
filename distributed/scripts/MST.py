#!/usr/bin/env python
import rospy
import rospkg
import scipy.io
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import minimum_spanning_tree
import numpy as np
import pylab
import os


import rospkg
import sys
import library2018 as myLib




def MSTconnect(graph,edgeset_all, robot_start_node, cor, PLOT_IT):

    # edges_all -> list of edges that where assigned to the robot
    # robot_start -> node in which the robot starts


    #Read original graph
    #graph = myLib.read_graph('Original_graph_36.mat')
    #graph = myLib.read_graph('Original_graph_35.mat')
    #graph = myLib.read_graph('Original_graph_A1.mat')
    #graph = myLib.read_graph("Original_graph_"+EXP_NAME+".mat")
    n = graph['n']
    nodes = graph['nodes']
    C = graph['C']
    Ccom = graph['Ccom']
    PathM = graph['PathM']
    EdgeMap = graph['EdgeMap']
    w_s = graph['w_s']
    PolC = graph['PolC']



    #Given the selected edges, create a set of nodes that must be visited by MST
    nodeset_all = []
    for edge in edgeset_all:
        [fr, to, cx, cy, cost] = myLib.getCoefs(edge - 1, PolC)
        nodeset_all.append(fr)
        nodeset_all.append(to)
    #Add the node where the robot is
    nodeset_all.append(robot_start_node)
    #Exclude repeated nodes
    nodeset_all = list(set(nodeset_all))
    #Sort the nodes acording to their indexes
    nodeset_all = sorted(nodeset_all)

    #print 'Here is nodeset_all - INSIDE MST'
    #print nodeset_all



    # Obtain a subgraph that contains only the selected nodes
    CA = np.matrix(C)
    CcomA = np.matrix(Ccom)
    exclude_list = []
    for k in range(n):
        if not k+1 in nodeset_all:
            exclude_list.append(k)
    CA = np.delete(CA,exclude_list,0)
    CA = np.delete(CA,exclude_list,1)
    CcomA = np.delete(CcomA,exclude_list,0)
    CcomA = np.delete(CcomA,exclude_list,1)
    CA = CA.tolist()
    CcomA = CcomA.tolist()

    # Apply MST on the complete subgraph
    XA = csr_matrix(CcomA)
    MSTA = minimum_spanning_tree(XA)

    # Make the resultant matrix be symmetric
    MSTarrayA = (np.matrix(MSTA.toarray())+(np.matrix(MSTA.toarray())).T).tolist()


    #Create a list of final edges (INTERSECTION OF DISCNECTED GRAPH and MST)
    final_list = []
    for iA in range(len(MSTarrayA)):
        for jA in range(len(MSTarrayA)):
            if (MSTarrayA[iA][jA] != 0 and iA>jA):
                i = nodeset_all[iA] - 1
                j = nodeset_all[jA] - 1
                path_i_j = myLib.getNodePath(i,j,PathM)

                # Loop for conections composed by more than one direct edge
                for count in range(len(path_i_j) - 1):
                    # Get the position of the edge in the list
                    [edge, signal] = myLib.getEdge(path_i_j[count], path_i_j[count + 1], PolC)

                    # Get the polynomial coefficients of the edge
                    #[fr, to, cx, cy, cost] = myLib.getCoefs(edge, PolC)

                    final_list.append(edge+1)
    #Make the union of the outputs of TA and MST
    final_list = final_list+edgeset_all
    final_list = list(set(final_list))




    #Plot the results of the MST based algorithm
    # ----------  ----------  ---------- ----------
    if PLOT_IT:
        pylab.axis('equal')
        pylab.axis(w_s)
        #pylab.title('Connected graph after MST')
        pylab.title('After MST')
        for e in range(len(PolC[0])):

            [fr, to, cx, cy, cost] = myLib.getCoefs(e, PolC)

            p0 = [0.01 * k for k in range(101)]
            x = []
            y = []
            for p in p0:
                x.append(0)
                y.append(0)
                for k in range(6):  # Compute refference position
                    x[-1] = x[-1] + cx[6 - k - 1] * p ** k
                    y[-1] = y[-1] + cy[6 - k - 1] * p ** k

            if  e+1 in  final_list:
                pylab.plot(x, y, cor, linewidth=3.0)
            else:
                #pylab.plot(x, y, 'k--', linewidth=1.0)
                pylab.plot(x, y, 'k-', linewidth=1.0)
        # pylab.show()
        # ----------  ----------  ---------- ----------



    return final_list
# ----------  ----------  ----------  ----------  ----------  ----------  ----------