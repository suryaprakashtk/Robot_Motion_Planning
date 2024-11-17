# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from RRT_source.rrt.rrt import RRT
from RRT_source.rrt.rrt_connect import RRTConnect
from RRT_source.rrt.rrt_star_bid import RRTStarBidirectional
from RRT_source.search_space.search_space import SearchSpace
from RRT_source.utilities.plotting import Plot

def myRRT_Func(bound, blocks, start, goal, filename):

    X_dimensions = np.array([(bound[0,0], bound[0,3]), (bound[0,1], bound[0,4]), (bound[0,2], bound[0,5])])  # dimensions of Search Space
    Obstacles = np.zeros(shape=(blocks.shape[0],6), dtype = float)
    for i in range(blocks.shape[0]):
        Obstacles[i,0:6] = tuple(blocks[i,0:6])
    # obstacles
    x_init = tuple(start)  # starting location
    x_goal = tuple(goal)  # goal location

   
    r = 0.01  # length of smallest edge to check for intersection with obstacles
    max_samples = 10240000  # max number of samples to take before timing out
    prc = 0.4  # probability of checking for a connection to goal

    simpleRRT(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename+"_RRT")
    simpleRRT_bi(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename+"_BiRRT")
    simpleRRT_connect(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename+"_ConnectRRT")
    return

def simpleRRT(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename):
    Q = np.array([(1, 1)])  # length of tree edges
    # create Search Space
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt_search
    rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
    path = rrt.rrt_search()
    print(len(path))
    # plot
    plot = Plot(filename)
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)
    return

def simpleRRT_bi(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename):
    # create search space
    Q = np.array([2]) 
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt_search
    rrt_connect = RRTConnect(X, Q, x_init, x_goal, max_samples, r, prc)
    path = rrt_connect.rrt_connect()
    # plot
    plot = Plot("rrt_connect_3d")
    plot.plot_tree(X, rrt_connect.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)
    return

def simpleRRT_connect(X_dimensions,Obstacles,x_init,x_goal,max_samples,prc,r,filename):
    # create Search Space
    Q = np.array([(1,1)])
    rewire_count = 32 
    X = SearchSpace(X_dimensions, Obstacles)

    # create rrt_search
    rrt = RRTStarBidirectional(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
    path = rrt.rrt_star_bidirectional()

    # plot
    plot = Plot("rrt_star_bid_3d")
    plot.plot_tree(X, rrt.trees)
    if path is not None:
        plot.plot_path(X, path)
    plot.plot_obstacles(X, Obstacles)
    plot.plot_start(X, x_init)
    plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)
    return