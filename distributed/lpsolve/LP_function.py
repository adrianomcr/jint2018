#!/usr/bin/env python
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
import time
import pylab
from lpsolve55 import *
from lp_maker import *


"""
LP_MAKER  Makes mixed integer linear programming problems.

SYNOPSIS: lp_handle = lp_maker(f,a,b,e,vlb,vub,xint,scalemode,setminim)
  make the MILP problem
    max v = f'*x
      a*x <> b
        vlb <= x <= vub
        x(int) are integer

ARGUMENTS: The first four arguments are required:
        f: n vector of coefficients for a linear objective function.
        a: m by n matrix representing linear constraints.
        b: m vector of right sides for the inequality constraints.
        e: m vector that determines the sense of the inequalities:
                  e(i) < 0  ==> Less Than
                  e(i) = 0  ==> Equals
                  e(i) > 0  ==> Greater Than
      vlb: n vector of non-negative lower bounds. If empty or omitted,
           then the lower bounds are set to zero.
      vub: n vector of upper bounds. May be omitted or empty.
     xint: vector of integer variables. May be omitted or empty.
scalemode: Autoscale flag. Off when 0 or omitted.
 setminim: Set maximum lp when this flag equals 0 or omitted.

OUTPUT: lp_handle is an integer handle to the lp created.
"""




def execute_lp(pts,C,cor,PLOT):


    n = len(pts)
    m = n * (n - 1) #number of edges

    w_s = [0, 25, 0, 12.5]
    #Definition of the edges
    E = [[0 for i in range(2)] for j in range(m)]
    k = 0
    for i in range(n):
        for j in range(n):
            if(i != j):
                E[k][0] = i
                E[k][1] = j
                k = k+1
    # ----------  ----------  ----------


    #Definition of the cost vector
    c = []
    for i in range(n):
        for j in range(n):
            if (i != j):
                c.append(C[i][j])
    # ----------  ----------  ----------

    # "Arival constraints" ----------  ----------  ----------
    Aeq1 = [[0 for i in range(m)] for j in range(n)]
    count_line = -1
    for j in range(n): # 0 until n-1
        count_line = count_line + 1
        for k in range(m):  # 0 until m-1
            if(E[k][1] == j):
                Aeq1[count_line][k] = 1
    beq1 = [1 for i in range(n)]
    #----------  ----------  ----------  ----------  ----------  ----------

    #"Flux conservation constraints" ----------  ----------  ----------
    Aeq2 = [[0 for i in range(m)] for j in range(n)]
    for p in range(n): # 0 until n-1
        for k in range(0, m):  # 0 until m-1
            if (E[k][1] == p):
                Aeq2[p][k] = 1
            if (E[k][0] == p):
                Aeq2[p][k] = -1
    beq2 = [0 for i in range(n)]
    #----------  ----------  ----------  ----------  ----------  ----------



    #Joining all equality constraints ----------  ----------  ----------
    Aeq = []
    Aeq = Aeq + Aeq1 + Aeq2
    beq = []
    beq = beq + beq1 + beq2
    #----------  ----------  ----------  ----------  ----------  ----------


    #Definition of the x variables bounds
    vlb = []
    vub = []
    #Bouds of "X" variables
    vlb = vlb + [0 for k in range(m)]
    vub = vub + [1 for k in range(m)]
    # ----------  ----------  ----------


    # Definition of the type of constranis (> < =) === (1, -1, 0)
    e = []
    #Equality constraints (=)
    for k in range(len(Aeq)):
        e.append(0)
    # ----------  ----------  ----------


    # Definition of the integer variable
    #int_var = range(1, m + 1, 1)
    int_var = []
    # ----------  ----------  ----------

    # Auto scale flag
    scalemode = 0
    # ----------  ----------  ----------

    # Definition of cost function operand (min or max)
    setminim = 1
    # ----------  ----------  ----------


    lp = lp_maker(c, Aeq, beq, e, vlb, vub, int_var, scalemode, setminim)

    #TIMEOUT = 1.0  # seconds
    #lpsolve('set_timeout', lp, TIMEOUT)

    solvestat = lpsolve('solve', lp)

    #obj = lpsolve('get_objective', lp)

    x = lpsolve('get_variables', lp)[0]

    lpsolve('delete_lp', lp)


    #PLOT = True
    if PLOT:
        #Plotting results  ----------  ----------  ----------
        pylab.axis('equal')
        pylab.axis(w_s)
        for k in range(n):
            pylab.text(pts[k][0]+0.04,pts[k][1]+0.04,k,fontsize=10.0,color=cor)

        k = -1
        for i in range(n):
            for j in range(n):
                if (i != j):
                    k = k + 1
                    if (abs(x[k] - 1) < 10 ** (-6)):
                        pylab.plot([pts[i][0], pts[j][0]], [pts[i][1], pts[j][1]], cor+'-', linewidth=2.0)

        #pylab.show()
    # ----------  ----------  ----------  ----------  ----------  ----------



    if(solvestat == 0):
        chosen = []
        for k in range(m):
            if(abs(x[k]-1) < 10**(-6)):
                chosen.append(E[k])
        chosen = np.matrix(chosen)
        chosen = chosen.T
    else:
        chosen = -1
        print 'Optimal solution not found'
        print 'Here is solvestat: ', solvestat


    return (chosen)
# ----------  ----------  ----------  ----------  ----------  ----------






def atribute_clusters(speeds, search_speeds, Cost_cluster_len, Cost_cluster_sp, Cost_cluster_to_go):


    R = len(speeds)
    T = len(Cost_cluster_len)


    print 'Number on robots: ', R, '\nNumber of clusters: ', T

    #C = [[r/t for r in Cost_cluster] for t in Power_robot]
    #C[r][t] is the cost of r search t

    C = [[0 for r in Cost_cluster_len] for t in speeds]
    for r in range(R):
        for t in range(T):
            C[r][t] = Cost_cluster_len[t]/speeds[r] + Cost_cluster_sp[t]/search_speeds[r] + Cost_cluster_to_go[r][t]/speeds[r]


    m = R*T


    #Definition of the cost vector
    c = []
    #Cost of "x" variables
    c = c + [0 for k in range(R*T)]
    # Cost of "F" variable
    c.append(1)
    # ----------  ----------  ----------


    #Assignment constraints ----------  ----------  ----------
    Aeq1 = [[0 for i in range(R*T)] for j in range(T)]
    for t in range(T):
        for r in range(R):
            Aeq1[t][r*T+t] = 1
    for k in range(len(Aeq1)):
        Aeq1[k].append(0)
    beq1 = [1 for i in range(T)]
    #----------  ----------  ----------  ----------  ----------  ----------


    #MinMax constraints ----------  ----------  ----------
    AF = [[0 for i in range(R*T)] for j in range(R)]
    for r in range(R):
        for t in range(T):
                AF[r][r*T+t] = C[r][t]
    bF = [0 for r in range(R)]
    #Adding 'F' variable
    for k in range(len(AF)):
        AF[k].append(-1)
    #----------  ----------  ----------  ----------  ----------  ----------


    Aineq = AF
    bineq = bF
    Aeq = Aeq1
    beq = beq1


    #Joining all constraints ----------  ----------  ----------
    A = []
    b = []
    #Inequality constraints:
    for line in Aineq:
        A.append(line)
    for number in bineq:
        b.append(number)
    #Equality constraints:
    for line in Aeq:
        A.append(line)
    for number in beq:
        b.append(number)
    #----------  ----------  ----------  ----------  ----------  ----------


    #Definition of the x variables bounds
    vlb = []
    vub = []
    #Bouds of "X" variables
    vlb = vlb + [0 for k in range(R * T)]
    vub = vub + [1 for k in range(R * T)]
    # Bouds of "F" variable
    vlb.append(0)
    vub.append(10000) #big positive number
    # ----------  ----------  ----------


    # Definition of the type of constranis (> < =)
    e = []
    # Inequality constraints (<)
    for k in range(len(Aineq)):
        e.append(-1)
    #Equality constraints (=)
    for k in range(len(Aeq)):
        e.append(0)
    # ----------  ----------  ----------


    # Definition of the integer variable
    int_var = range(1, R*T + 1, 1)
    # ----------  ----------  ----------

    # Auto scale flag
    scalemode = 0
    # ----------  ----------  ----------

    # Definition of cost function operand (min or max)
    setminim = 1
    # ----------  ----------  ----------


    lp = lp_maker(c, A, b, e, vlb, vub, int_var, scalemode, setminim)

    solvestat = lpsolve('solve', lp)

    #obj = lpsolve('get_objective', lp)

    x = lpsolve('get_variables', lp)[0]

    lpsolve('delete_lp', lp)


    return x
# ----------  ----------  ----------  ----------  ----------  ----------





