#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
#from time import sleep, time
import time
#from pylab import *
import pylab
from lpsolve55 import *
from lp_maker import *
import rospkg
#import libraries2018
import LP_function
from random import randrange






def BFS(v,BM):
    #"""
    if (len(BM) != len(BM[1])):
        print 'ERROR - Matrix is not square !!!'
    #"""
    N = len(BM)
    BM[v][v] = 1
    O = [[v,v]] #v is father of v
    C = []

    while (O): #while o is not empty
        u = O[0][0]
        O.pop(0)

        if(not u in C):
            C.append(u)

            for i in range(N):
                if(BM[u][i] == 1):
                    if(not i in C):
                        O.append([i,u])


    return C

# ----------  ----------  ----------  ----------  ----------


def TarjansAlg(BM):
    N = len(BM)

    placed = [0 for i in range(N)]
    CG = []

    for k in range(N):
        if placed[k] == 0:
            C = BFS(k,BM)
            CG.append(C)
            for l in C:
                placed[l] = 1


    return CG
# ----------  ----------  ----------  ----------  ----------



# ----------  ----------  ----------  ----------
def find_cluster(list_tree,id,level):
    matriz = list_tree[level]
    if (matriz[id][1] == -1):
        result = matriz[id][0]
        return result
    else:
        x = find_cluster(list_tree,matriz[id][1],level+1)
        return x
# ----------  ----------  ----------  ----------


# Here the script begins ----------  ----------
# ----------  ----------  ----------  ----------
n = 150 #number of nodes
m = n * (n - 1) #number of edges
iterations = 3 #number of times the LP is going to run


"""
depots = {0: [0],
          1: [0, 1],
          2: [4, 1, 6],
          #2: [38, 38, 38],
          3: [0, 1, 2, 3],
          }
speeds = {0: [1],
          1: [1, 1],
          2: [1, 1.1, 1.2],
          3: [1, 1, 1, 1],
          }

depots = depots[R-1]
speeds = speeds[R-1]
"""

colors = {0 : 'r',
          1 : 'g',
          2: 'y',
          3: 'm',
          4: 'c',
          5: 'b',
          6: 'k',
          7: 'w',
          }


#Definition of n random nodes
pts = []
for k in range(n):
    #pts.append([(6/1000.0)*randrange(0,1000,1)-3, (4/1000.0)*randrange(0,1000,1)-2])
    pts.append([(25/1000.0)*randrange(0,1000,1), (12.5/1000.0)*randrange(0,1000,1)])
pts = np.array(pts)





pts_0 = pts

list_tree = []

iterations = 0

count_time = time.time()
G = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
#for run in range(iterations):

while len(G) > 5:


    run = iterations
    iterations = iterations+1

    print 'Iteration', iterations, ' ...'
    n = len(pts)

    C = [[0 for i in range(n)] for j in range(n)]
    for i in range(n):
        for j in range(n):
            C[i][j] = ((pts[i][0]-pts[j][0])**2+(pts[i][1]-pts[j][1])**2)**0.5

    #list_of_clusters_points.append(pts) # adding the first cluster (original graph)

    matriz = []
    for k in range(n):
        matriz.append([k,-1])
    #print 'matrz:\n', matriz

    TA_time = time.time()
    tour = LP_function.execute_lp(pts,C,colors[run],True)
    print 'Time of iteration ',iterations,': ', time.time()-TA_time

    #print 'Here is tour:'
    #print tour

    #list_of_solutioins.append(tour)


    #Compute the new set of points
    tour = tour-1*0
    tour = tour.tolist()

    BM = [[0 for i in range(n)] for j in range(n)]
    for k in range(n):
        BM[tour[0][k]][tour[1][k]] = 1
        BM[tour[1][k]][tour[0][k]] = 1




    n_clusters = 0


    t = time.time()
    new_pts = []


    G = TarjansAlg(BM)


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
        new_pts.append([cx,cy])

    pts = new_pts

    list_tree.append(matriz)


count_time = time.time()-count_time


#Define a sort of end flag to identify the top cluster
n = len(pts)
matriz = []
for k in range(n):
    matriz.append([k,-1])
list_tree.append(matriz)


#print 'list_tree:\n', list_tree
"""
print '\n\n\n\n\n\n'
print 'list_tree[0]:\n', list_tree[0]
print 'list_tree[1]:\n', list_tree[1]
print 'list_tree[2]:\n', list_tree[2]
print 'list_tree[3]:\n', list_tree[3]
"""

print 'Number of iterations: ', iterations

Recursive_time = time.time()
final_cluster = []
for k in range(len(pts_0)):
    cluster_of_k = find_cluster(list_tree, k, 0)
    final_cluster.append([k,cluster_of_k])
print 'Time of recursive function: ', time.time() - Recursive_time

#"""
for k in range(len(pts_0)):
    pylab.plot(pts_0[k][0], pts_0[k][1], 'ok', markersize=3.0)
if(len(list_tree[iterations]) <= 8):
    for k in range(len(pts_0)):
        cor = colors[final_cluster[k][1]]
        pylab.plot(pts_0[k][0], pts_0[k][1], 'o', markersize=10.0, color=cor)
#"""


#x = find_cluster(list_tree,15,0)
#print 'Cluster of 15 is: ', x

print 'Elapsed time: ', count_time, 'seconds'

pylab.show()

