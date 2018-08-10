#!/usr/bin/env python
import rospy
from distributed.msg import History, HistList, Broadcast, Intlist
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2
import numpy as np
from time import sleep
import tf
import scipy.io
import rospkg
import sys
import pylab
import copy





rp = rospkg.RosPack()
path = rp.get_path('distributed')
sys.path.insert(0, path + '/CPP_MIT')
sys.path.insert(0, path+'/lpsolve')
import postman as CPPlib
import TA_Heuristics as TAHEU
import library2018 as myLib
import Task_Assignment as TA
import MST










def define_subsets_new(list_of_H,virtual_graph):

    pts_0 = virtual_graph['nodes']

    R = len(list_of_H)
    for r in range(R):
        exec ('H%d = list_of_H[%d]' % (r, r))

    set_uv = []
    set_v = []
    set_g = []
    pts = []

    currEdge_all = []
    e_vT_f_all = []
    e_g_all = []
    for r in range(R):
        exec ('currEdge_all = currEdge_all +  [H%d.currEdge]' % (r))
        exec ('e_vT_f_all = e_vT_f_all +  list(H%d.e_v) + list(H%d.T_f)' % (r,r))
        exec ('e_g_all = e_g_all +  list(H%d.e_g)' % (r))
    T_a_all = []
    for e in range(len(list_of_H[0].T_a)):
        T_a_all.append([])
        for r in range(R):
            T_a_all[e] = T_a_all[e] + list(list_of_H[r].T_a[e].data)

    set_uv = []
    set_v = []
    set_g = []
    pts = []
    pts_id = []

    for k in range(len(pts_0)):  # loop over all of the orignal edges
        if((k + 1) in e_vT_f_all):  # Forgot about the visited edges (we are done with them)
            set_v.append(k + 1)
        elif ((k + 1) in e_g_all):  # If the edge is assigned to other robot -> analyze if ths other robot is in the network
            flag = False
            for r in range(R): # If the edge was assigned to a robot in the network -> set the flag
                if (list_of_H[r].id in T_a_all[k]):
                    flag = True
            if (flag): # Include the edge in the "unvisited" set
                set_uv.append(k + 1)
                pts.append(pts_0[k])
                pts_id.append(k)
            else: # Include the edge in the set of "assigned to other robots" set
                set_g.append(k + 1)
        else:  # In other case, we should search on this edge
            set_uv.append(k + 1)
            pts.append(pts_0[k])
            pts_id.append(k)


    return set_uv, set_v, set_g, pts, pts_id
# ----------  ----------  ----------  ----------  ----------  ----------  ----------













def replanning_heuristics(original_graph, virtual_graph, list_of_H):


    print 'Here is the beguinning of my function'

    PolC = original_graph['PolC']

    colors_0 = ['b', 'r', 'g', 'y']

    R = len(list_of_H)

    for r in range(R):
        print 'LastMeeting robot ', list_of_H[r].id, ': ', list_of_H[r].lastMeeting

    list_of_robs = []
    for r in range(R):
        list_of_robs.append(list_of_H[r].id)
    id = min(list_of_robs) #id of the robot that is computing the replan

    speeds = []
    search_speeds = []
    colors = []
    for r in range(R):
        speeds.append(list_of_H[r].specs[0])
        search_speeds.append(list_of_H[r].specs[1])
        colors.append(colors_0[list_of_H[r].id])

    # Compute
    old_cost = myLib.get_cost(original_graph, list_of_H, speeds, search_speeds)


    # Call function to define the sests E_v, E_uv and E_g
    set_uv, set_v, set_g, pts, pts_id = define_subsets_new(list_of_H, virtual_graph)
    # ----------  ----------  ----------  ----------  ----------

    print '\nHere is set_v (visited edges) in original inedexes:'
    print set_v
    print 'Here is set_uv (unvisited edges) in original inedexes:'
    print set_uv
    print 'Here is set_g (assigned edges) in original inedexes:'
    print set_g, '\n\n'

    # Map the unvisited nodes with new labels
    C = virtual_graph['Ccom'] #matrix with the length costs
    Cuv = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    C = virtual_graph['C_sp'] #matrix with the number of search points
    Cuv_sp = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    exclude_list = (np.array(set_v) - 1).tolist() + (np.array(set_g) - 1).tolist()
    Cuv = np.delete(Cuv, exclude_list, 0) # exclude lines
    Cuv = np.delete(Cuv, exclude_list, 1) # exclude columns
    Cuv = Cuv.tolist()
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 0) # exclude lines
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 1) # exclude columns
    Cuv_sp = Cuv_sp.tolist()

    C_edge = []
    for e in range(len(PolC[0])):
        cost = PolC[0][e][4]
        cost = cost.tolist()
        cost = cost[0]
        cost = cost[0]
        C_edge.append(cost)

    print '\n ----- Task assignment function called -----'
    sol, division = TAHEU.heuristic_loop(original_graph,speeds, search_speeds, Cuv, Cuv_sp, pts, pts_id)
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------

    # Sort the lists
    for r in range(R):
        division[r] = list(set(division[r]))

    # Map the nodes back to the original indexation
    for r in range(R):
        for k in range(len(division[r])):
            division[r][k] = set_uv[division[r][k]]


    print '\nAssigned edges for the robots:'
    for r in range(R):
        print 'Subset for robot ' + str(list_of_H[r].id) + ': ', division[r]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------




    # Call MST for every robot in the communication graph in order to make the graph connected
    print '\n ----- Applying MST to the disconnected subgraphs ------'
    subgraphs = []
    for r in range(R):
        subgraphs.append([])
        pylab.figure(id)
        pylab.subplot(2, R, R + (r + 1))
        subgraphs[r] = MST.MSTconnect(original_graph,division[r], (list_of_H[r]).nextNode, colors[r], True)
    for r in range(R):
        print 'Subset of edges for robot ' + str(r) + ': (original indexes)\n', subgraphs[r]
        print 'Start node for robot ' + str(list_of_H[r].id) + ':', (list_of_H[r]).nextNode, '\n'
    # ----------  ----------  ----------  ----------  ----------




    # Cal CPP for every graph generated by the MST based algorithm
    print '\n ----- Applying CPP to the connected subgraphs -----'
    Hole_path_list = []
    for k in range(R):
        Hole_path_list.append([])
        current_node = (list_of_H[k]).nextNode
        edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[k])
        Hole_path_list[k] = CPPlib.main_CPP(sorted(edges_listOfTuples), current_node)
    for k in range(R):
        print 'Route for robot ' + str(k), ': ', Hole_path_list[k], '\n'
    print '\n'
    # ----------  ----------  ----------  ----------  ----------



    # Create list T_a
    pts_0 = virtual_graph['nodes']
    T_a0 = [Intlist()]
    for k in range(len(PolC[0]) - 1):
        IL = Intlist()
        T_a0.append(IL)
    for k in range(len(pts_0)):
        T_a0[k].data = list(list_of_H[0].T_a[0].data) + list(list_of_H[1].T_a[1].data)
        T_a0[k].data = list(T_a0[k].data)
        if k + 1 in set_uv:  # if in unvisite list
            if k + 1 in division[0]:  # if in the assigned to the other one
                T_a0[k].data.append(list_of_H[0].id)
            elif k + 1 in division[1]:
                T_a0[k].data.append(list_of_H[1].id)
        T_a0[k].data = list(set(T_a0[k].data))


    # Create list T_f
    T_f0 = []
    for r in range(R):
        T_f0 = T_f0 + list(list_of_H[r].T_f)
    T_f0 = T_f0 + set_v
    T_f0 = list(set(T_f0))


    # Create the new list of histories
    new_Hists = []
    for r in range(R):
        H = History()
        H.id = list_of_H[r].id
        H.e_v = set_v
        H.e_uv = division[r]
        for r2 in range(R):
            if r2 != r:
                H.e_g = H.e_g+division[r2]# + set_g !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        H.T_a = T_a0
        H.T_f = T_f0
        H.lastMeeting = []
        for r2 in range(R):
            H.lastMeeting.append(list_of_H[r2].id)
        H.Whole_path = Hole_path_list[r]
        new_Hists.append(H)

    new_cost = myLib.get_cost(original_graph, new_Hists, speeds, search_speeds)


    print 'Old cost: ', old_cost
    print 'New cost: ', new_cost

    # Check if the new plan is better than the previous one
    change_plan = False
    if new_cost < old_cost:
        change_plan = True

        # Initialize the plot of the result of the TA algorithm
        for r in range(R):
            pylab.subplot(2, R, (r + 1))
            pylab.axis('equal')
            pylab.axis(virtual_graph['w_s'])
            pylab.title('Task assig. robot %d' % list_of_H[r].id)

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

            # Plot the the edge in the background
            for r in range(R):
                pylab.subplot(2, R, (r + 1))
                pylab.plot(x, y, 'k--', linewidth=1.0)


            for r in range(R):
                if e + 1 in division[r]:
                    pylab.subplot(2, R, (r + 1))
                    pylab.plot(x, y, colors[r], linewidth=3.0)



    # If the new plan is not better compute a simple replan
    if not change_plan:

        # If the new plan from TA->MST->CPP is not better ok. However, we need to run CPP again because we have


        print 'New cost is not better ...'

        print '----------  ----------  ----------  ----------  ---------- ----------  ----------  ----------'
        print '----------  ---------- ---------- APPLYING THE SIMPLE REPLAN ---------- ----------  ----------'
        print '----------  ----------  ----------  ----------  ---------- ----------  ----------  ----------'


        division = []
        for r in range(R):
            division.append([])

        for r in range(R):
            for k in list_of_H[r].e_uv:
                if k in set_uv:
                    division[r].append(k)

        print 'Here is new division (for the simple replan)'
        print division

        # Call MST for every robot in the communication graph in order to make the graph connected
        pylab.close()
        pylab.axis(virtual_graph['w_s'])
        print '\n ----- Applying MST to the disconnected subgraphs ------'
        subgraphs = []
        for r in range(R):
            subgraphs.append([])
            pylab.subplot(2, R, R + (r + 1))
            subgraphs[r] = MST.MSTconnect(original_graph,division[r], (list_of_H[r]).nextNode, colors[r], True)
        for r in range(R):
            print 'Subset of edges for robot ' + str(list_of_H[r].id) + ': (original indexes)\n', subgraphs[r]
            print 'Start node for robot ' + str(list_of_H[r].id) + ':', (list_of_H[r]).nextNode, '\n'
        # ----------  ----------  ----------  ----------  ----------

        # Cal CPP for every graph generated by the MST based algorithm
        print '\n ----- Applying CPP to the connected subgraphs -----'
        Hole_path_list = []
        for r in range(R):
            Hole_path_list.append([])
            current_node = (list_of_H[r]).nextNode
            edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[r])
            #Hole_path_list[r] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
            Hole_path_list[r] = CPPlib.main_CPP(sorted(edges_listOfTuples), current_node)
        #for r in range(R):
            print 'Route for robot ' + str(list_of_H[r].id), ': ', Hole_path_list[r], '\n'
        print '\n'
        # ----------  ----------  ----------  ----------  ----------



        # Create the new list of histories (STILL MUST ADAPT TO A GENERAL NUMBER OF ROBOS)
        new_Hists = []
        for r in range(R):
            H = History()
            H.id = list_of_H[r].id
            H.e_v = set_v
            H.e_uv = division[r]
            for r2 in range(R):
                if r2 != r:
                    H.e_g = division[r2] + set_g
            H.T_a = T_a0
            H.T_f = T_f0
            H.lastMeeting = []
            for r2 in range(R):
                H.lastMeeting.append(list_of_H[r2].id)
            H.Whole_path = Hole_path_list[r]
            new_Hists.append(H)
    # ----------  ----------  ----------  ----------  ----------




    """
    Nothing is being done with this yet
    """
    # Set the abort_curr_edge flag in te case to robots are in the same edge
    for r1 in range(R):
        for r2 in range(r1 + 1, R, 1):  # This way r1<r2
            if (list_of_H[r1].currEdge == list_of_H[r2].currEdge):
                new_Hists[r2].abort_curr_edge = True
    # """
    for r1 in range(R):
        for r2 in range(R):
            if (new_Hists[r1].currEdge in new_Hists[r2].e_v[0:-1]):
                new_Hists[r1].abort_curr_edge = True
    # """
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------
    """
    THE PROBLEM WITH THIS IS THAT THE MINIMUM ID ROBOT WILL KEEP SERACHING THE SPs THAT WERE ALREADY SEARCHED BY THE MAX ID ROBOT
    """


    # Set the available flag as False
    for r in range(R):
        new_Hists[r].available = False

    # Save the result of TA and MST in a image
    import time
    hour = time.strftime("%Hh%Mm%Ss")
    date = time.strftime("d%dm%my%Y")
    rp = rospkg.RosPack()
    fig_name = rp.get_path('distributed')
    fig_name = fig_name + '/imagesResults/Results_' + date + '_' + hour + '.png'
    pylab.savefig(fig_name)
    # ----------  ----------  ----------  ----------  ----------

    # Choose if the result of the planning will be platted or not
    SHOW_NEW_PLAN = False
    if SHOW_NEW_PLAN:
        pylab.show()
    pylab.close()

    return change_plan, Hole_path_list, new_Hists
# ----------  ----------  ----------  ----------  ----------  ----------  ----------



























"""
# ----------  ----------  ----------  ----------  ----------  ----------
# OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES

# OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES

# OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES

# OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES

# OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES --- OLD CODES
# ----------  ----------  ----------  ----------  ----------  ----------
"""















def define_subsets(list_of_H,virtual_graph):

    pts_0 = virtual_graph['nodes']

    R = len(list_of_H)
    for r in range(R):
        exec ('H%d = list_of_H[%d]' % (r, r))

    set_uv = []
    set_v = []
    set_g = []
    pts = []

    #print "Here is H0.T_f: ", H0.T_f
    #print "Here is H1.T_f: ", H1.T_f

    #"""
    currEdge_all = []
    e_vT_f_all = []
    e_g_all = []
    for r in range(R):
        exec ('currEdge_all = currEdge_all +  [H%d.currEdge]' % (r))
        exec ('e_vT_f_all = e_vT_f_all +  list(H%d.e_v) + list(H%d.T_f)' % (r,r))
        exec ('e_g_all = e_g_all +  list(H%d.e_g)' % (r))
        #exec ('e_g_all = e_g_all +  list(H%d.e_g)' % (r))
    T_a_all = []
    for e in range(len(list_of_H[0].T_a)):
        T_a_all.append([])
        for r in range(R):
            #T_a_all[e].append(list_of_H[r].T_a[e].data)
            T_a_all[e] = T_a_all[e] + list(list_of_H[r].T_a[e].data)
    #"""

    """
    print "\n------------------------\n"
    print "Here is currEdge_all: ", currEdge_all
    print "Here is e_vT_f_all: ", e_vT_f_all
    print "Here is e_g_all: ", e_g_all
    print "\n------------------------\n"

    print "\n------------------------\n"
    print "Here is [H0.currEdge, H1.currEdge]: ", [H0.currEdge, H1.currEdge]
    print "Here is H0.e_uv: ", H0.e_uv
    print "Here is H1.e_uv: ", H1.e_uv
    print "Here is H0.e_v: ", H0.e_v
    print "Here is H1.e_v: ", H1.e_v
    print "Here is H0.e_g: ", H0.e_g
    print "Here is H1.e_g: ", H1.e_g
    print "Here is H0.T_f: ", H0.T_f
    print "Here is H1.T_F: ", H1.T_f
    print "\n------------------------\n"
    """
    print "Here is T_a_all: ", T_a_all

    set_uv = []
    set_v = []
    set_g = []
    pts = []

    # k+1 - in - currEdge
    # k+1 - in - e_v + T_f
    # k+1 - in - e_g
    # H.id - in - T_a (combination)

    #"""
    for k in range(len(pts_0)):  # loop over all of the orignal edges
        if ((k + 1) in currEdge_all):  # Add the current edges in the unvisited set (THEY SHOULD BE REMOVED NEXT)
            set_uv.append(k + 1)
            pts.append(pts_0[k])
        elif((k + 1) in e_vT_f_all):  # Forgot about the visited edges (we are done with them)
            set_v.append(k + 1)
        elif ((k + 1) in e_g_all):  # If the edge is assigned to other robot -> analyze if ths other robot is in the network
            flag = False
            for r in range(R): # If the edge was assigned to a robot in the network -> set the flag
                if (list_of_H[r].id in T_a_all[k]):
                    flag = True
            if (flag): # Include the edge in the "unvisited" set
                set_uv.append(k + 1)
                pts.append(pts_0[k])
            else: # Include the edge in the set of "assigned to other robots" set
                set_g.append(k + 1)
        else:  # In other case, we should search on this edge
            set_uv.append(k + 1)
            pts.append(pts_0[k])

    #"""


    return set_uv, set_v, set_g, pts
# ----------  ----------  ----------  ----------  ----------

#Test function to be called as a thread
# ----------  ----------  ----------  ----------  ----------
def test_function(arg1,arg2):
    for k in range(arg1):
        print '--->>> Thread k = ', k, '<<<---'
        sleep(arg2)
# ----------  ----------  ----------  ----------  ----------




#Thread to call the replanning algorithm
# ----------  ----------  ----------  ----------  ----------
def replan_thread(original_graph, virtual_graph, list_of_H,pub_broadcast,id):

    # Call the replanning function (Algorithm 2)
    change_plan, Hole_path_list, new_Hists = replanning(original_graph, virtual_graph, list_of_H)
    pylab.close("all")

    # Broadcast new plan to other robots
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

    print 'New plan broadcasted'
    waitting_new_plan = True
    pub_broadcast.publish(B)
# ----------  ----------  ----------  ----------  ----------





def replanning(original_graph, virtual_graph, list_of_H):





    PolC = original_graph['PolC']

    colors_0 = ['b', 'r', 'g', 'y']

    # 'change_plan' is True if the new plan is better than the old one
    change_plan = False

    R = len(list_of_H)

    list_of_robs = []
    for r in range(R):
        list_of_robs.append(list_of_H[r].id)
    id = min(list_of_robs) #id of the robot that is computing the replan

    speeds = []
    search_speeds = []
    colors = []
    for r in range(R):
        #exec ('H%d = list_of_H[%d]' % (r, r))
        #exec ('speeds.append(H%d.specs[0])' % r)
        #exec ('search_speeds.append(H%d.specs[1])' % r)
        #exec ('colors.append(colors_0[H%d.id])' % r)
        speeds.append(list_of_H[r].specs[0])
        search_speeds.append(list_of_H[r].specs[1])
        colors.append(colors_0[list_of_H[r].id])


    old_cost = myLib.get_cost(original_graph, list_of_H, speeds, search_speeds)


    set_uv, set_v, set_g, pts = define_subsets(list_of_H, virtual_graph)


    print '\nHere is set_v (visited edges) in original inedexes:'
    print set_v
    print 'Here is set_uv (unvisited edges) in original inedexes:'
    print set_uv
    print 'Here is set_g (assigned edges) in original inedexes:'
    print set_g, '\n\n'


    # Map the unvisited nodes with new labels
    C = virtual_graph['Ccom'] #matrix with the length costs
    Cuv = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    C = virtual_graph['C_sp'] #matrix with the number of search points
    Cuv_sp = np.matrix(C) # Reduced cost matrix with only the unvisited edges (unvisited virtual nodes)
    #exclude_list = (np.array(set_v) - 1).tolist()
    exclude_list = (np.array(set_v) - 1).tolist() + (np.array(set_g) - 1).tolist()
    Cuv = np.delete(Cuv, exclude_list, 0) # exclude lines
    Cuv = np.delete(Cuv, exclude_list, 1) # exclude columns
    Cuv = Cuv.tolist()
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 0) # exclude lines
    Cuv_sp = np.delete(Cuv_sp, exclude_list, 1) # exclude columns
    Cuv_sp = Cuv_sp.tolist()



    # Define the depot virtual nodes for the task assignment
    depots = []
    for r in range(R):
        #exec ('depots.append(set_uv.index(H%d.currEdge))' % r)
        depots.append(set_uv.index(list_of_H[r].currEdge))



    print '\nHere is depot virtual nodes (new indexes):'
    print depots
    print 'Here is depot virtual nodes (original indexes):'
    depots_ori = [] #original depot points??
    for r in range(R):
        depots_ori.append(set_uv[depots[r]])
    print depots_ori, '\n'





    print '\n ----- Task assignment function called -----'
    #[sol, C_check0, C_check1] = TA.execute_lp(speeds, search_speeds, depots, colors, Cuv, Cuv_sp, pts)
    sol = TA.execute_lp(speeds, search_speeds, depots, colors, Cuv, Cuv_sp, pts)

    #print "Here is the solution of the TA problem: \n", sol



    # ----------  ----------  ----------  ----------  ----------  ----------  ----------

    # Map the solution back to the original indexes
    n_uv = len(Cuv)
    m_uv = n_uv*(n_uv-1)
    x_var = []
    for r in range(R):
        #exec ('x%d_var = []' % r)
        x_var.append([])


    F_var = sol.pop()
    for r in range(R):
        for k in range(m_uv):
            #exec ('x%d_var.insert(0,sol.pop())' % (R - r - 1))
            x_var[R-r-1].insert(0,sol.pop())


    division = []
    for r in range(R):
        division.append([])
    k = -1
    for i in range(n_uv):
        for j in range(n_uv):
            if i!=j:
                k = k+1
                for r in range(R):
                    #exec ('xAux_var = x%d_var' % r)
                    xAux_var = x_var[r]
                    if (xAux_var[k] == 1):
                        division[r].append(i)
                        division[r].append(j)
    for r in range(R):
        division[r] = list(set(division[r]))


    #Map the nodes back to the original indexation
    for r in range(R):
        for k in range(len(division[r])):
            division[r][k] = set_uv[division[r][k]]



    print '\nAssigned edges for the robots: (before excluding the current edges)'
    for r in range(R):
        print 'Subset for robot ' + str(list_of_H[r].id) + ': ', division[r]
    print '\n'


    # Exclude/Remove the used depot virtual nodes (they are edges that were already visited)
    for r in range(R):
        division[r].pop(division[r].index(list_of_H[r].currEdge))

    # Include the used depot virtual nodes in the visited set
    for r in range(R):
        set_v.append(list_of_H[r].currEdge)


    print '\nAssigned edges for the robots:'
    for r in range(R):
        print 'Subset for robot ' + str(list_of_H[r].id) + ': ', division[r]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------








    # Call MST for every robot in the communication graph in order to make the graph connected
    print '\n ----- Applying MST to the disconnected subgraphs ------'
    subgraphs = []
    for r in range(R):
        subgraphs.append([])
        #pylab.figure(100)
        pylab.figure(id)
        pylab.subplot(2,R,R+(r+1))
        subgraphs[r] = MST.MSTconnect(original_graph,division[r], (list_of_H[r]).nextNode, colors[r], True)
    for r in range(R):
        print 'Subset of edges for robot ' + str(r) + ': (original indexes)\n', subgraphs[r]
        print 'Start node for robot ' + str(r) + ':', (list_of_H[r]).nextNode
    # ----------  ----------  ----------  ----------  ----------




    # Cal CPP for every graph generated by the MST based algorithm
    print '\nApplying CPP to the connected subgraphs'
    Hole_path_list = []
    for r in range(R):
        Hole_path_list.append([])
        current_node = (list_of_H[r]).nextNode
        edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[r])
        #Hole_path_list[r] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
        Hole_path_list[r] = CPPlib.main_CPP(sorted(edges_listOfTuples), current_node)
    #for k in range(R):
        print 'Route for robot ' + str(k), ': ', Hole_path_list[k]
    print '\n'
    # ----------  ----------  ----------  ----------  ----------




    #Create list T_a
    pts_0 = virtual_graph['nodes']
    T_a0 = [Intlist()]
    for k in range(len(PolC[0])-1):
        IL = Intlist()
        T_a0.append(IL)
    for k in range(len(pts_0)):
        T_a0[k].data = list(list_of_H[0].T_a[0].data) + list(list_of_H[1].T_a[1].data)
        T_a0[k].data = list(T_a0[k].data)
        if k + 1 in set_uv:  # if in unvisite list
            if k + 1 in division[0]:  # if in the assigned to the other one
                T_a0[k].data.append(list_of_H[0].id)
            elif k + 1 in division[1]:
                T_a0[k].data.append(list_of_H[1].id)
        T_a0[k].data = list(set(T_a0[k].data))



    #Create list T_f
    T_f0 = []
    for r in range(R):
        T_f0 = T_f0 + list(list_of_H[r].T_f)
    T_f0 = T_f0 + set_v
    T_f0 = list(set(T_f0))




    # Create the new list of histories (STILL MUST ADAPT TO A GENERAL NUMBER OF ROBOS)
    new_Hists = []
    for r in range(R):
        H = History()
        H.id = list_of_H[r].id
        H.e_v = set_v
        H.e_uv = division[r]
        for r2 in range(R):
            if r2!=r:
                H.e_g = division[r2] + set_g
        H.T_a = T_a0
        H.T_f = T_f0
        #print "\n\n----------------\n Here is T_f0:",T_f0, "\n----------------\n\n"
        H.lastMeeting = []
        for r2 in range(R):
            H.lastMeeting.append(list_of_H[r2].id)
        H.Whole_path = Hole_path_list[r]
        new_Hists.append(H)


    new_cost = myLib.get_cost(original_graph, new_Hists, speeds, search_speeds)

    #print "\n\n-------------------\nHere is new_Hists[0].T_f", new_Hists[0].T_f, "\n-------------------\n\n"
    #print "\n\n-------------------\nHere is new_Hists[1].T_f", new_Hists[1].T_f, "\n-------------------\n\n"




    print 'Here is old cost: ', old_cost
    print 'Here is new cost: ', new_cost









    # Check if the new plan is better than the previous one
    change_plan = False
    if new_cost < old_cost:
        change_plan = True
        # Plot the disconnected graph
        # ----------  ----------  ----------  ----------  ----------  ----------  ----------
        #pylab.figure(100)
        #pylab.figure(id)
        """
        pylab.subplot(2, 2, 1)
        pylab.axis('equal')
        pylab.axis(virtual_graph['w_s'])
        pylab.title('MILP result for robot %d' % H0.id)
        pylab.subplot(2, 2, 2)
        pylab.axis('equal')
        pylab.axis(virtual_graph['w_s'])
        pylab.title('MILP result for robot %d' % H1.id)
        """
        #Initialize the plot of the result of the TA algorithm
        for r in range(R):
            pylab.subplot(2, R, (r+1))
            pylab.axis('equal')
            pylab.axis(virtual_graph['w_s'])
            pylab.title('Task assig. robot %d' % list_of_H[r].id)


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

            """
            pylab.subplot(2, 2, 1)
            pylab.plot(x, y, 'k--', linewidth=1.0)
            pylab.subplot(2, 2, 2)
            pylab.plot(x, y, 'k--', linewidth=1.0)
            """
            #Plot the the edge in the background
            for r in range(R):
                pylab.subplot(2, R, (r+1))
                pylab.plot(x, y, 'k--', linewidth=1.0)

            """
            if e + 1 in division[0]:
                pylab.subplot(2, 2, 1)
                pylab.plot(x, y, colors[0], linewidth=3.0)
            if e + 1 in division[1]:
                pylab.subplot(2, 2, 2)
                pylab.plot(x, y, colors[1], linewidth=3.0)
            """
            for r in range(R):
                if e + 1 in division[r]:
                    pylab.subplot(2, R, (r+1))
                    pylab.plot(x, y, colors[r], linewidth=3.0)




            # ----------  ----------  ----------  ----------  ----------  ----------  ----------
        """
        # Plotting the heuristic costs
        # ----------  ----------  ----------  ----------  ----------  ----------  ----------
        for i in range(len(pts)):
            x = pts[i][0]
            y = pts[i][1]

            pylab.subplot(2, 2, 1)
            pylab.text(x, y, ("%.2f" % C_check0[i]), fontsize=8.0)
            pylab.subplot(2, 2, 2)
            pylab.text(x, y, ("%.2f" % C_check1[i]), fontsize=8.0)
        # ----------  ----------  ----------  ----------  ---------  ----------  ----------
        """
    if not change_plan:

        print 'New cost is not better ...'

        print '----------  ----------  ----------  ----------  ---------- ----------  ----------  ----------'
        print '----------  ---------- ---------- APPLYING THE SIMPLE REPLAN ---------- ----------  ----------'
        print '----------  ----------  ----------  ----------  ---------- ----------  ----------  ----------'
        #OBS: This simple new plan is necessary.
        #If the new plan from MILP->MST->CPP is not better ok. However, we need to run CPP again because we have

        #division = [[],[]]
        division = []
        for r in range(R):
            division.append([])

        for r in range(R):
            for k in list_of_H[r].e_uv:
                if k in set_uv:
                    division[r].append(k)


        print 'Here is new division (for the simple replan)'
        print division


        #Simple replan ----------  ----------  ----------

        # Call MST for every robot in the communication graph in order to make the graph connected
        pylab.close()
        pylab.axis(virtual_graph['w_s'])
        print '\n ----- Applying MST to the disconnected subgraphs ------'
        subgraphs = []
        for r in range(R):
            subgraphs.append([])
            pylab.subplot(2, R, R+(r+1))
            print 'Here is new division (for the simple replan)'
            print division
            subgraphs[r] = MST.MSTconnect(original_graph,division[r], (list_of_H[r]).nextNode, colors[r], True)
        for r in range(R):
            print 'Subset of edges for robot ' + str(list_of_H[r].id) + ': (original indexes)\n', subgraphs[r]
            print 'Start node for robot ' + str(list_of_H[r].id) + ':', (list_of_H[r]).nextNode
        # ----------  ----------  ----------  ----------  ----------

        # Cal CPP for every graph generated by the MST based algorithm
        print '\nApplying CPP to the connected subgraphs'
        Hole_path_list = []
        for r in range(R):
            Hole_path_list.append([])
            current_node = (list_of_H[r]).nextNode
            edges_listOfTuples = myLib.write_listOfTuples(original_graph, subgraphs[r])
            #Hole_path_list[r] = cppsolver.CPP(sorted(edges_listOfTuples), current_node)
            Hole_path_list[r] = CPPlib.main_CPP(sorted(edges_listOfTuples), current_node)
        #for r in range(R):
            print 'Route for robot ' + str(list_of_H[r].id), ': ', Hole_path_list[r]
        print '\n'
        # ----------  ----------  ----------  ----------  ----------



        # Create the new list of histories (STILL MUST ADAPT TO A GENERAL NUMBER OF ROBOS)
        new_Hists = []
        for r in range(R):
            H = History()
            H.id = list_of_H[r].id
            H.e_v = set_v
            H.e_uv = division[r]
            for r2 in range(R):
                if r2 != r:
                    H.e_g = division[r2] + set_g
            H.T_a = T_a0
            H.T_f = T_f0
            H.lastMeeting = []
            for r2 in range(R):
                H.lastMeeting.append(list_of_H[r2].id)
            H.Whole_path = Hole_path_list[r]
            new_Hists.append(H)
    # ----------  ----------  ----------  ----------  ----------



    # Set the abort_curr_edge flag in te case to robots are in the same edge
    for r1 in range(R):
        for r2 in range(r1+1,R,1): #This way r1<r2
            if(list_of_H[r1].currEdge == list_of_H[r2].currEdge):
                new_Hists[r2].abort_curr_edge = True
                print '\nabort_curr_edge strategy:\nrob ', new_Hists[r1].id, ' / rob ', new_Hists[r2].id, 'set flag'
            else:
                print '\nabort_curr_edge strategy:\nrob ', new_Hists[r1].id, ' / rob ', new_Hists[r2].id
    # Set the abort_curr_edge flag in te case the edge was previously (thus, completely) visited by another robot
    #"""
    for r1 in range(R):
        for r2 in range(R):
            if (new_Hists[r1].currEdge in new_Hists[r2].e_v[0:-1]):
                new_Hists[r1].abort_curr_edge = True
    #"""
    # ----------  ----------  ----------  ----------  ----------  ----------  ----------
    """
    THE PROBLEM WITH THIS IS THAT THE MINIMUM ID ROBOT WILL KEEP SERACHING THE SPs THAT WERE ALREADY SEARCHED BY THE MAX ID ROBOT
    """

    # Set the available flag as False
    for r in range(R):
        new_Hists[r].available = False



    # Save the result of TA and MST in a image
    import time
    hour = time.strftime("%Hh%Mm%Ss")
    date = time.strftime("d%dm%my%Y")
    rp = rospkg.RosPack()
    fig_name = rp.get_path('distributed')
    fig_name = fig_name + '/imagesResults/Results_' + date + '_' + hour + '.png'
    pylab.savefig(fig_name)
    # ----------  ----------  ----------  ----------  ----------

    # Choose if the result of the planning will be platted or not
    #SHOW_NEW_PLAN = True
    SHOW_NEW_PLAN = False
    if SHOW_NEW_PLAN:
        pylab.show()
    pylab.close()



    return change_plan, Hole_path_list, new_Hists
# ----------  ----------  ----------  ----------  ----------











