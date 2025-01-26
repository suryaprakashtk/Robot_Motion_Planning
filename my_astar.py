# priority queue for OPEN list
from pqdict import pqdict
import math
import numpy as np

class MyAStarNode(object):
    '''
    This class represent a node which has the state information and the heuristic values
    '''
    def __init__(self, pqkey, coord, hval):
        self.pqkey = pqkey
        self.coord = coord
        self.g = math.inf
        self.h = hval
        self.parent_node = None
        self.parent_action = None
        self.closed = False
    def __lt__(self, other):
        '''Implements the less than operator'''
        return self.g < other.g     


class MyAStar(object):
    '''
    This class implements the A* algorithm
    '''
    def __init__(self, boundary, blocks, start, end, map_resolution = 0.5, epsilon = 1, minDistToGoal = 0.1):
        '''
        Initialises the bounding box dimensions, start, end, resolution, OPEN and CLOSED lists
        '''
        self.boundary = boundary
        self.blocks = blocks
        self.start = start
        self.goal = end
        self.finish = None
        self.resolution = map_resolution
        self.epsilon = epsilon
        self.minDistToGoal = minDistToGoal
        self.start_node = MyAStarNode(tuple(self.start), self.start, self.getHVal(self.start))
        self.start_node.g = 0
        self.open_list = pqdict({tuple(self.start):self.start_node}, key=self.priority_function)
        self.closed_list = pqdict({}, key=lambda x: x.g)
    
    
    def priority_function(self,key):
        ''' Assigns priority to each node'''
        priority = key.g + self.epsilon*key.h
        return priority
    
    def Plan(self):
        '''
        Core A* iteration, which adds and removes elements from the Priority Queue.
        '''
        dR = self.getDirections()
        for key,node in self.open_list.popitems():
            self.closed_list[key] = node
            self.closed_list[key].closed = True
            # construct graph based on collision check and boundary
            for k in range(dR.shape[0]):
                next_coord = node.coord + dR[k,:]
                if( next_coord[0] < self.boundary[0,0] or next_coord[0] > self.boundary[0,3] or \
                    next_coord[1] < self.boundary[0,1] or next_coord[1] > self.boundary[0,4] or \
                    next_coord[2] < self.boundary[0,2] or next_coord[2] > self.boundary[0,5] ):
                    continue
                is_collision = self.collision(node.coord, next_coord)
                if(is_collision):
                    continue
                if(tuple(next_coord) in self.open_list):
                    if(self.open_list[tuple(next_coord)].g > node.g + cij):
                        del self.open_list[tuple(next_coord)]
                        new_node = MyAStarNode(tuple(next_coord), next_coord, self.getHVal(next_coord))
                        cij = self.stage_cost(node.coord, next_coord)
                        new_node.g = node.g + cij
                        new_node.parent_node = key
                        new_node.parent_action = k
                        self.open_list[tuple(next_coord)] = new_node
                elif(tuple(next_coord) in self.closed_list):
                    continue
                else:
                    new_node = MyAStarNode(tuple(next_coord), next_coord, self.getHVal(next_coord))
                    cij = self.stage_cost(node.coord, next_coord)
                    new_node.g = node.g + cij
                    new_node.parent_node = key
                    new_node.parent_action = k
                    self.open_list[tuple(next_coord)] = new_node
            if sum((node.coord-self.goal)**2) <= self.minDistToGoal:
                self.finish = node.coord
                break
        return 

    def getPath(self):
        '''
        Extracts the path from start to end based on the order in which it was added to the CLOSED List.
        Each node has a reference to the previous node and traverses up the order to extract the parents until NULL.
        '''
        goal_node = self.closed_list[tuple(self.finish)]
        path = [goal_node.coord]
        while True:
            next_node = self.closed_list[goal_node.parent_node]
            path.append(next_node.coord)
            if(np.array_equal(next_node.coord, self.start)):
                break
            goal_node = next_node
        return np.array(path)


    def collision(self, point1, point2):
        '''
        Input: point1 (x,y,z), point2 (x,y,z) and block (N,9)
        Output: True if it is colliding with any block, else false
        only checks the block and siregards boundaries
        Retruns true if colliding
        '''
        
        block = self.blocks.copy()
        # Delta error in bounding box checks. Inflates the boxes by this amount
        delta_error = 0.0001
        for i in range(block.shape[0]):
            # Checking if point 1 is within the block
            if( point1[0] >= block[i,0] - delta_error and point1[0] <= block[i,3] + delta_error and \
                    point1[1] >= block[i,1] - delta_error and point1[1] <= block[i,4] + delta_error and \
                    point1[2] >= block[i,2] - delta_error and point1[2] <= block[i,5] + delta_error):
                return True
            # Checking if point 2 is within the block
            if( point2[0] >= block[i,0] - delta_error and point2[0] <= block[i,3] + delta_error and \
                    point2[1] >= block[i,1] - delta_error and point2[1] <= block[i,4] + delta_error and \
                    point2[2] >= block[i,2] - delta_error and point2[2] <= block[i,5] + delta_error):
                return True
            
            dir_vec = point2 - point1
            t = np.zeros(shape=(6),dtype=float)
            t[:] = np.inf
            if(dir_vec[0]!=0):
                # tleft
                t[0] = (block[i,0] - point1[0])/dir_vec[0]
                if(t[0]>=0 and t[0]<=1):
                    y_int = point1[1] + t[0]*dir_vec[1]
                    z_int = point1[2] + t[0]*dir_vec[2]
                    if(y_int>=block[i,1] and y_int<=block[i,4] and z_int>=block[i,2] and z_int<=block[i,5]):
                        return True
                # tright
                t[1] = (block[i,3] - point1[0])/dir_vec[0]
                if(t[1]>=0 and t[1]<=1):
                    y_int = point1[1] + t[1]*dir_vec[1]
                    z_int = point1[2] + t[1]*dir_vec[2]
                    if(y_int>=block[i,1] and y_int<=block[i,4] and z_int>=block[i,2] and z_int<=block[i,5]):
                        return True
            if(dir_vec[1]!=0):
                # tfront
                t[2] = (block[i,1] - point1[1])/dir_vec[1]
                if(t[2]>=0 and t[2]<=1):
                    x_int = point1[0] + t[2]*dir_vec[0]
                    z_int = point1[2] + t[2]*dir_vec[2]
                    if(x_int>=block[i,0] and x_int<=block[i,3] and z_int>=block[i,2] and z_int<=block[i,5]):
                        return True
                # tback
                t[3] = (block[i,4] - point1[1])/dir_vec[1]
                if(t[3]>=0 and t[3]<=1):
                    x_int = point1[0] + t[3]*dir_vec[0]
                    z_int = point1[2] + t[3]*dir_vec[2]
                    if(x_int>=block[i,0] and x_int<=block[i,3] and z_int>=block[i,2] and z_int<=block[i,5]):
                        return True
            if(dir_vec[2]!=0):
                # tbottom
                t[4] = (block[i,2] - point1[2])/dir_vec[2]
                if(t[4]>=0 and t[4]<=1):
                    x_int = point1[0] + t[4]*dir_vec[0]
                    y_int = point1[1] + t[4]*dir_vec[1]
                    if(x_int>=block[i,0] and x_int<=block[i,3] and y_int>=block[i,1] and y_int<=block[i,4]):
                        return True
                # ttop
                t[5] = (block[i,5] - point1[2])/dir_vec[2]
                if(t[5]>=0 and t[5]<=1):
                    x_int = point1[0] + t[5]*dir_vec[0]
                    y_int = point1[1] + t[5]*dir_vec[1]
                    if(x_int>=block[i,0] and x_int<=block[i,3] and y_int>=block[i,1] and y_int<=block[i,4]):
                        return True
        return False
    
    def getHVal(self,coord):
        '''Heuristic function which returns the H-value to each node'''
        distance = coord - self.goal
        return np.linalg.norm(distance)
    
    def stage_cost(self,coord1, coord2):
        '''Assigns stage cost between 2 nodes. This is used for constructing graph incrementally. Stage cost here ie Euclidean distance'''
        distance = coord1 - coord2
        return np.linalg.norm(distance)

    def getDirections(self):
        '''Template to define the neighbours. Each node has 27 neighbors in 3D space.'''
        dR = np.zeros(shape=(26,3),dtype=float)

        dR[0,:] = np.array([1,0,0],dtype = float)
        dR[1,:] = np.array([0,1,0],dtype = float)
        dR[2,:] = np.array([0,0,1],dtype = float)
        dR[3,:] = np.array([-1,0,0],dtype = float)
        dR[4,:] = np.array([0,-1,0],dtype = float)
        dR[5,:] = np.array([0,0,-1],dtype = float)

        dR[6,:] = np.array([1,1,0],dtype = float)
        dR[7,:] = np.array([1,-1,0],dtype = float)
        dR[8,:] = np.array([-1,1,0],dtype = float)
        dR[9,:] = np.array([-1,-1,0],dtype = float)

        dR[10,:] = np.array([1,0,1],dtype = float)
        dR[11,:] = np.array([1,0,-1],dtype = float)
        dR[12,:] = np.array([-1,0,1],dtype = float)
        dR[13,:] = np.array([-1,0,-1],dtype = float)

        dR[14,:] = np.array([0,1,1],dtype = float)
        dR[15,:] = np.array([0,1,-1],dtype = float)
        dR[16,:] = np.array([0,-1,1],dtype = float)
        dR[17,:] = np.array([0,-1,-1],dtype = float)

        dR[18,:] = np.array([1,1,1],dtype = float)
        dR[19,:] = np.array([-1,1,1],dtype = float)
        dR[20,:] = np.array([1,-1,1],dtype = float)
        dR[21,:] = np.array([-1,-1,1],dtype = float)

        dR[22,:] = np.array([1,1,-1],dtype = float)
        dR[23,:] = np.array([-1,1,-1],dtype = float)
        dR[24,:] = np.array([1,-1,-1],dtype = float)
        dR[25,:] = np.array([-1,-1,-1],dtype = float)

        dR = dR*self.resolution
        return dR




