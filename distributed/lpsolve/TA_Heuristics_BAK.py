#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
#from time import sleep, time
import time
#from pylab import *
import pylab
#import sys
#sys.path.insert(0, '/usr/lib/lp_solve/')
from lpsolve55 import *
from lp_maker import *
import rospkg
import library2018 as myLib
import LP_function as LP












# Recursive function to find the cluster each node belongs  ----------  ----------
def find_cluster(list_tree,id,level):



    matriz = list_tree[level]
    if (matriz[id][1] == -1):
        result = matriz[id][0]
        return result
    else:
        x = find_cluster(list_tree,matriz[id][1],level+1)
        return x
# ----------  ----------  ----------  ----------  ----------





def compute_cluster_costs(graph,pts_id):


    PolC = graph['PolC']
    Ccom = graph['Ccom']
    C_sp = graph['C_sp']


    C_edge_len = []
    C_edge_sp = []
    for edge in pts_id:
        [fr, to, cx, cy, cost] = myLib.getCoefs(edge-1, PolC)
        C_edge_len.append(Ccom[fr-1][to-1])
        C_edge_sp.append(C_sp[fr-1][to-1])

    return C_edge_len, C_edge_sp
# ----------  ----------  ----------  ----------  ----------





def heuristic_loop(original_graph, speeds, search_speeds, C, Csp, pts, pts_id,depots):



    n = len(C) #number of nodes
    m = n * (n - 1) #number of edges
    R = len(speeds) #number of robots


    pts_0 = pts

    list_tree = []

    iterations = 0

    example_colors = ['r','g','b','y','m','c','k','w']


    MAX_NUM_CLUSTERS = 2 * R + 5
    #MAX_NUM_CLUSTERS = 6

    PLOT = False


    count_time = time.time()
    G = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    # for run in range(iterations):
    while len(G) > MAX_NUM_CLUSTERS:

        run = iterations
        iterations = iterations + 1

        n = len(pts)

        # list_of_clusters_points.append(pts) # adding the first cluster (original graph)


        matriz = []
        for k in range(n):
            matriz.append([k, -1])
        # print 'matrz:\n', matriz

        TA_time = time.time()


        #"""
        # First heuristics to compute the costs
        if iterations == 1:
            mean_speeds = float(sum(speeds))/len(speeds)
            mean_search_speeds = float(sum(search_speeds)) / len(search_speeds)
            CostM = [[0 for i in range(n)] for j in range(n)]
            for i in range(n):
                for j in range(n):
                    #CostM[i][j] = C[i][j]/mean_speeds + Csp[i][j]/mean_search_speeds
                    CostM[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
        else:
            CostM = [[0 for i in range(n)] for j in range(n)]
            #Heuristics: Euclidian distance
            for i in range(n):
                for j in range(n):
                    CostM[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
        #"""
        """
        # First heuristics to compute the costs
        if iterations == 1:
            mean_speeds = float(sum(speeds))/len(speeds)
            mean_search_speeds = float(sum(search_speeds)) / len(search_speeds)
            CostM = [[0 for i in range(n)] for j in range(n)]
            for i in range(n):
                for j in range(n):
                    CostM[i][j] = C[i][j]/mean_speeds + Csp[i][j]/mean_search_speeds
                    #CostM[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
        else:
            CostM = [[0 for i in range(n)] for j in range(n)]
            #Heuristics: Euclidian distance
            for i in range(n):
                ii = inverse_cluster[i][0]
                for j in range(n):
                    jj = inverse_cluster[j][0]
                    #CostM[i][j] = sqrt((pts[i][0] - pts[j][0]) ** 2 + (pts[i][1] - pts[j][1]) ** 2)
                    CostM[i][j] = C[ii][jj] / mean_speeds + Csp[ii][jj] / mean_search_speeds
        """


        tour = LP.execute_lp(pts, CostM, example_colors[run], PLOT)
        print 'Time of iteration ', iterations, ': ', time.time() - TA_time


        # Compute the new set of points
        tour = tour
        tour = tour.tolist()

        BM = [[0 for i in range(n)] for j in range(n)]
        for k in range(n):
            BM[tour[0][k]][tour[1][k]] = 1
            BM[tour[1][k]][tour[0][k]] = 1

        G = myLib.TarjansAlg(BM)

        new_pts = []
        for clu in range(len(G)):
            cx = 0
            cy = 0
            for node in G[clu]:
                matriz[node][1] = clu
                cx = cx + pts[node][0]
                cy = cy + pts[node][1]
            cx = cx / len(G[clu])
            cy = cy / len(G[clu])
            new_pts.append([cx, cy])


        pts = new_pts

        list_tree.append(matriz)

        """
        import copy
        temp_list_tree = copy.deepcopy(list_tree)
        n = len(pts)
        matriz = []
        for k in range(n):
            matriz.append([k, -1])
        temp_list_tree.append(matriz)
        temp_cluster = []
        print 'temp_list_tree: ', temp_list_tree
        for k in range(len(pts_0)):
            cluster_of_k = find_cluster(temp_list_tree, k, 0)
            temp_cluster.append([k, cluster_of_k])
        inverse_cluster = [[] for k in range(len(G))]
        for k in range(len(pts_0)):
            inverse_cluster[temp_cluster[k][1]].append(k)

        print '\nHere is inverse_cluster: ', inverse_cluster,
        print '\nHere is temp_cluster: ', temp_cluster, '\n--------------------------------------'
        """

    count_time = time.time() - count_time


    pylab.show()

    # Define a sort of end flag to identify the top cluster
    n = len(pts)
    matriz = []
    for k in range(n):
        matriz.append([k, -1])
    list_tree.append(matriz)


    print '\nNumber of iterations: ', iterations



    final_cluster = []
    for k in range(len(pts_0)):
        cluster_of_k = find_cluster(list_tree, k, 0)
        final_cluster.append([k, cluster_of_k])

    #print '\nHere is final_cluster: ', final_cluster, '\n--------------------------------------'

    #Here we will apply the simple task assignment to associate agents to clusters

    n_clusters = 0
    for k in range(len(final_cluster)):
        if (final_cluster[k][1]>n_clusters):
            n_clusters = final_cluster[k][1]
    n_clusters = n_clusters + 1

    #print 'Here is final cluster:\n', final_cluster

    C_edge_len, C_edge_sp = compute_cluster_costs(original_graph, pts_id)
    Power_robot_len = speeds
    Power_robot_sp = search_speeds


    Cost_cluster_len = [0 for k in range(n_clusters)] # cost of the length of the cluster
    Cost_cluster_sp = [0 for k in range(n_clusters)] # cost of the search points of the cluster
    for k in range(len(final_cluster)):
        Cost_cluster_len[final_cluster[k][1]] = Cost_cluster_len[final_cluster[k][1]] + C_edge_len[k]
        Cost_cluster_sp[final_cluster[k][1]] = Cost_cluster_sp[final_cluster[k][1]] + C_edge_sp[k]

    R = len(speeds)
    T = len(Cost_cluster_len)

    #depots = [1,46,1,1]#AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
    Cost_cluster_to_go = [] # cost of the length for robot r go to cluster t
    for r in range(R):
        Cost_cluster_to_go.append([])
        for t in range(T):
            Cost_cluster_to_go[r].append(100000)
            for k in range(len(final_cluster)):
                if (final_cluster[k][1] == t):
                    dist = C[k][depots[r]]
                    #dist = (()**2+()**2)**0.5
                    if (dist < Cost_cluster_to_go[r][t]):
                        Cost_cluster_to_go[r][t] = dist

    print 'Cost_cluster_to_go:\n', Cost_cluster_to_go



    print '\nCalling MILP to assign the clusters ...'

    #sol = LP.atribute_clusters(Power_robot, Cost_cluster)
    #sol = LP.atribute_clusters(Power_robot_len,Power_robot_sp, Cost_cluster_len, Cost_cluster_sp)
    sol = LP.atribute_clusters(Power_robot_len,Power_robot_sp, Cost_cluster_len, Cost_cluster_sp, Cost_cluster_to_go)
    # ---------- ---------- ----------




    if PLOT:
        for k in range(len(pts_0)):
            pylab.plot(pts_0[k][0], pts_0[k][1], 'ok', markersize=3.0)
        if (len(list_tree[iterations]) <= 8):
            for k in range(len(pts_0)):
                cor = example_colors[final_cluster[k][1]]
                pylab.plot(pts_0[k][0], pts_0[k][1], 'o', markersize=10.0, color=cor)
        pylab.show()



    print 'Elapsed time: ', count_time, '\n'



    division = []
    for r in range(R):
        division.append([])
        for t in range(T):
            index = r*T+t
            if(abs(1-sol[index]) < 0.00001):
                #cluster t goes to robot r
                for k in range(len(final_cluster)):
                    if final_cluster[k][1] == t:
                        division[r].append(final_cluster[k][0])


    print 'Here is the internal division:\n', division



    #division = []
    #for r in range(R):
    #    division.append([])


    return sol, division

# ----------  ----------  ----------  ----------  ----------  ----------
# end of def heuristics_loop





