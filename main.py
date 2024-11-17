import numpy as np
import time
import matplotlib.pyplot as plt; plt.ion()
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from my_astar import *
from my_rrt import *

global_path = "./"

def tic():
  return time.time()
def toc(tstart, nm=""):
  print('%s took: %s sec.\n' % (nm,(time.time() - tstart)))
  

def load_map(fname):
  '''
  Loads the bounady and blocks from map file fname.
  
  boundary = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  
  blocks = [['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'],
            ...,
            ['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']]
  '''
  mapdata = np.loadtxt(fname,dtype={'names': ('type', 'xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b'),\
                                    'formats': ('S8','f', 'f', 'f', 'f', 'f', 'f', 'f','f','f')})
  blockIdx = mapdata['type'] == b'block'
  boundary = mapdata[~blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  blocks = mapdata[blockIdx][['xmin', 'ymin', 'zmin', 'xmax', 'ymax', 'zmax','r','g','b']].view('<f4').reshape(-1,11)[:,2:]
  return boundary, blocks


def draw_map(boundary, blocks, start, goal):
  '''
  Visualization of a planning problem with environment boundary, obstacle blocks, and start and goal points
  '''
  # fig = plt.figure()
  # ax = fig.add_subplot(221, projection='3d')
  # hb = draw_block_list(ax,blocks)
  # hs = ax.plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  # hg = ax.plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')  
  # ax.set_xlabel('X')
  # ax.set_ylabel('Y')
  # ax.set_zlabel('Z')
  # ax.set_xlim(boundary[0,0],boundary[0,3])
  # ax.set_ylim(boundary[0,1],boundary[0,4])
  # ax.set_zlim(boundary[0,2],boundary[0,5])
  # ax.view_init(elev=30, azim=45)

  # Create a figure and subplots
  fig, axs = plt.subplots(2, 2, figsize=(12, 8), subplot_kw={'projection': '3d'})
  # Subplot 1: X-axis projection
  axs[0, 0].set_title('Projection 1')
  hb = draw_block_list(axs[0, 0], blocks)
  hs = axs[0, 0].plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = axs[0, 0].plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')
  axs[0, 0].set_xlabel('X')
  axs[0, 0].set_ylabel('Y')
  axs[0, 0].set_zlabel('Z')
  axs[0, 0].set_xlim(boundary[0,0],boundary[0,3])
  axs[0, 0].set_ylim(boundary[0,1],boundary[0,4])
  axs[0, 0].set_zlim(boundary[0,2],boundary[0,5])
  axs[0, 0].view_init(elev=-90, azim=-90)

  # Subplot 2: Y-axis projection
  axs[0, 1].set_title('Projection 2')
  hb = draw_block_list(axs[0, 1], blocks)
  hs = axs[0, 1].plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = axs[0, 1].plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')
  axs[0, 1].set_xlabel('X')
  axs[0, 1].set_ylabel('Y')
  axs[0, 1].set_zlabel('Z')
  axs[0, 1].set_xlim(boundary[0,0],boundary[0,3])
  axs[0, 1].set_ylim(boundary[0,1],boundary[0,4])
  axs[0, 1].set_zlim(boundary[0,2],boundary[0,5])
  axs[0, 1].view_init(elev=0, azim=90)

  # Subplot 3: Z-axis projection
  axs[1, 0].set_title('Projection 3')
  hb = draw_block_list(axs[1, 0], blocks)
  hs = axs[1, 0].plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = axs[1, 0].plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')
  axs[1, 0].set_xlabel('X')
  axs[1, 0].set_ylabel('Y')
  axs[1, 0].set_zlabel('Z')
  axs[1, 0].set_xlim(boundary[0,0],boundary[0,3])
  axs[1, 0].set_ylim(boundary[0,1],boundary[0,4])
  axs[1, 0].set_zlim(boundary[0,2],boundary[0,5])
  axs[1, 0].view_init(elev=0, azim=180)

  # Subplot 4: 3D projection
  axs[1, 1].set_title('Projection 4')
  hb = draw_block_list(axs[1, 1], blocks)
  hs = axs[1, 1].plot(start[0:1],start[1:2],start[2:],'ro',markersize=7,markeredgecolor='k')
  hg = axs[1, 1].plot(goal[0:1],goal[1:2],goal[2:],'go',markersize=7,markeredgecolor='k')
  axs[1, 1].set_xlabel('X')
  axs[1, 1].set_ylabel('Y')
  axs[1, 1].set_zlabel('Z')
  axs[1, 1].set_xlim(boundary[0,0],boundary[0,3])
  axs[1, 1].set_ylim(boundary[0,1],boundary[0,4])
  axs[1, 1].set_zlim(boundary[0,2],boundary[0,5])
  return fig, axs, hb, hs, hg

def draw_block_list(ax,blocks):
  '''
  Subroutine used by draw_map() to display the environment blocks
  '''
  v = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]],dtype='float')
  f = np.array([[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[0,1,2,3],[4,5,6,7]])
  clr = blocks[:,6:]/255
  n = blocks.shape[0]
  d = blocks[:,3:6] - blocks[:,:3] 
  vl = np.zeros((8*n,3))
  fl = np.zeros((6*n,4),dtype='int64')
  fcl = np.zeros((6*n,3))
  for k in range(n):
    vl[k*8:(k+1)*8,:] = v * d[k] + blocks[k,:3]
    fl[k*6:(k+1)*6,:] = f + k*8
    fcl[k*6:(k+1)*6,:] = clr[k,:]
  
  if type(ax) is Poly3DCollection:
    ax.set_verts(vl[fl])
  else:
    pc = Poly3DCollection(vl[fl], alpha=0.25, linewidths=1, edgecolors='k')
    pc.set_facecolor(fcl)
    h = ax.add_collection3d(pc)
    return h


def runtest(mapfile, start, goal, verbose = True, filename = "Default"):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  # MP = Planner.MyPlanner(boundary, blocks) # TODO: replace this with your own planner implementation
  MP = MyAStar(boundary, blocks, start, goal, map_resolution=0.5, epsilon=1, minDistToGoal = 0.5)
  # Display the environment
  if verbose:
    fig, ax, hb, hs, hg = draw_map(boundary, blocks, start, goal)

  # Call the motion planner
  t0 = tic()
  MP.Plan()
  path = MP.getPath()
  toc(t0,"Planning")
  
  # Plot the path
  if verbose:
    ax[0,0].plot(path[:,0],path[:,1],path[:,2],'r-')
    ax[0,1].plot(path[:,0],path[:,1],path[:,2],'r-')
    ax[1,0].plot(path[:,0],path[:,1],path[:,2],'r-')
    ax[1,1].plot(path[:,0],path[:,1],path[:,2],'r-')

  path_to_save = global_path + "Astar_output/" + filename + '.png'

  # TODO: You should verify whether the path actually intersects any of the obstacles in continuous space
  # TODO: You can implement your own algorithm or use an existing library for segment and 
  #       axis-aligned bounding box (AABB) intersection

  # Collision check already handled in the Astar class
  collision = False
  # path has elements from goal to start in reverse order
  goal_reached = sum((path[0]-goal)**2) <= 0.5
  success = (not collision) and goal_reached
  pathlength = np.sum(np.sqrt(np.sum(np.diff(path,axis=0)**2,axis=1)))
  return success, pathlength

def runtestRRT(mapfile, start, goal, filename_rrt, verbose = True):
  '''
  This function:
   * loads the provided mapfile
   * creates a motion planner
   * plans a path from start to goal
   * checks whether the path is collision free and reaches the goal
   * computes the path length as a sum of the Euclidean norm of the path segments
  '''
  
  # Load a map and instantiate a motion planner
  boundary, blocks = load_map(mapfile)
  t0 = tic()
  myRRT_Func(boundary, blocks,start,goal,filename_rrt)
  toc(t0,"Planning")
  return


def test_single_cube(verbose = True):
  print('Running single cube test...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  success, pathlength = runtest(global_path + 'maps/single_cube.txt', start, goal, verbose, filename = "Cube")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_maze(verbose = True):
  print('Running maze test...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  success, pathlength = runtest(global_path + 'maps/maze.txt', start, goal, verbose, filename = "Maze")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

    
def test_window(verbose = True):
  print('Running window test...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  success, pathlength = runtest(global_path + 'maps/window.txt', start, goal, verbose, filename = "Window")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

  
def test_tower(verbose = True):
  print('Running tower test...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  success, pathlength = runtest(global_path + 'maps/tower.txt', start, goal, verbose, filename = "Tower")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

     
def test_flappy_bird(verbose = True):
  print('Running flappy bird test...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  success, pathlength = runtest(global_path + 'maps/flappy_bird.txt', start, goal, verbose, filename = "Flappy_bird")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength) 
  print('\n')

  
def test_room(verbose = True):
  print('Running room test...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  success, pathlength = runtest(global_path + 'maps/room.txt', start, goal, verbose, filename = "Room")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')


def test_monza(verbose = True):
  print('Running monza test...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  success, pathlength = runtest(global_path + 'maps/monza.txt', start, goal, verbose, filename = "Monza")
  print('Success: %r'%success)
  print('Path length: %d'%pathlength)
  print('\n')

def test_single_cubeRRT(verbose = True):
  print('Running single cube test RRT...\n') 
  start = np.array([2.3, 2.3, 1.3])
  goal = np.array([7.0, 7.0, 5.5])
  runtestRRT(global_path + 'maps/single_cube.txt', start, goal, "single_cube", verbose)

def test_mazeRRT(verbose = True):
  print('Running maze test RRT...\n') 
  start = np.array([0.0, 0.0, 1.0])
  goal = np.array([12.0, 12.0, 5.0])
  runtestRRT(global_path + 'maps/maze.txt', start, goal,  "maze",verbose)

def test_flappy_birdRRT(verbose = True):
  print('Running flappy bird test RRT...\n') 
  start = np.array([0.5, 2.5, 5.5])
  goal = np.array([19.0, 2.5, 5.5])
  runtestRRT(global_path + 'maps/flappy_bird.txt', start, goal, "flappy_bird", verbose)

def test_monzaRRT(verbose = True):
  print('Running monza test RRT...\n')
  start = np.array([0.5, 1.0, 4.9])
  goal = np.array([3.8, 1.0, 0.1])
  runtestRRT(global_path + 'maps/monza.txt', start, goal, "monza", verbose)

def test_windowRRT(verbose = True):
  print('Running window test RRT...\n') 
  start = np.array([0.2, -4.9, 0.2])
  goal = np.array([6.0, 18.0, 3.0])
  runtestRRT(global_path + 'maps/window.txt', start, goal, "window", verbose)

def test_towerRRT(verbose = True):
  print('Running tower test RRT...\n') 
  start = np.array([2.5, 4.0, 0.5])
  goal = np.array([4.0, 2.5, 19.5])
  runtestRRT(global_path + 'maps/tower.txt', start, goal, "tower", verbose)


def test_roomRRT(verbose = True):
  print('Running room test RRT...\n') 
  start = np.array([1.0, 5.0, 1.5])
  goal = np.array([9.0, 7.0, 1.5])
  runtestRRT(global_path + 'maps/room.txt', start, goal, "room", verbose)

if __name__=="__main__":  
  print("Uncomment any function to run")
#   test_single_cube()
  # test_maze()
  test_flappy_bird()
  # test_monza()
  # test_window()
  # test_tower()
  # test_room()
  # plt.show(block=True)
  # test_single_cubeRRT()
  # test_mazeRRT()
  # test_flappy_birdRRT()
  # test_monzaRRT()
  # test_windowRRT()
  # test_towerRRT()
  # test_roomRRT()









