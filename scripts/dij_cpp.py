#!/usr/bin/env python

import rospy
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState,GetModelState,GetWorldProperties
from geometry_msgs.msg import Pose,Point,Quaternion,Wrench,Vector3
from gazebo_msgs.msg import ModelState
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

class world:
    
    def __init__(self,canvas,agent_pose = [0,0,0],agent_dimension = [1,1]):
        
        self.get_m()
        
        self.x_lim = canvas[0]
        
        self.y_lim = canvas[1]
        
        self.init_ros_services()

    def get_m(self):

        get_map = rospy.ServiceProxy('/static_map',GetMap)

        mapa = get_map()

        self.field = mapa.map.data

        self.field = np.array(self.field,dtype=float)
        
        for i in range(len(self.field)):
            
            #print('value',self.field[i])
            if self.field[i]== -1 or self.field[i]==100:
                
                self.field[i] = 1

        self.obstacles = np.split(self.field,mapa.map.info.height,axis=0)

        #print('obstaculo',self.obstacles)

        #print('largura',len(self.obstacles[0]),'altura',len(self.obstacles))

        self.obstacles = np.array(self.obstacles,dtype=float)

        pass
        
    
    
    def init_ros_services(self):
        
        ##self.get_components = rospy.ServiceProxy('/gazebo/get_world_properties',GetWorldProperties)
        self.set_component = rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        #self.get_map = rospy.ServiceProxy('/static_map',GetMapRequest)
        #rospy.Subscriber('/map',OccupancyGrid,self.get_map_cb)
        pass

class agent:
    
    def __init__(self,dimension,pose):
        get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        self.pose = get_state('')
        self.dimensions = dimension
        pass

class dijkstra:
    def __init__(self,init_node,target_node,obstacles):
        self.path = self.dijkstra_algorithm(init_node,target_node,obstacles)
        pass
    
    def valid_node(self,node, size_of_grid):
        """Checks if node is within the grid boundaries."""
        if node[0] < 0 or node[0] >= size_of_grid:
            return False
        if node[1] < 0 or node[1] >= size_of_grid:
            return False
        return True

    def up(self,node):
        return (node[0]-1,node[1])

    def down(self,node):
        return (node[0]+1,node[1])

    def left(self,node):
        return (node[0],node[1]-1)

    def right(self,node):
        return (node[0],node[1]+1)

    def backtrack(self,initial_node, desired_node, distances):
        # idea start at the last node then choose the least number of steps to go back
        # last node
        path = [desired_node]

        size_of_grid = distances.shape[0]

        while True:
            # check up down left right - choose the direction that has the least distance
            potential_distances = []
            potential_nodes = []

            directions = [self.up,self.down,self.left,self.right]

            for direction in directions:
                node = direction(path[-1])
                if self.valid_node(node, size_of_grid):
                    potential_nodes.append(node)
                    potential_distances.append(distances[node[0],node[1]])

            least_distance_index = np.argmin(potential_distances)
            path.append(potential_nodes[least_distance_index])

            if path[-1][0] == initial_node[0] and path[-1][1] == initial_node[1]:
                break

        return list(reversed(path))

    def dijkstra_algorithm(self,initial_node, desired_node, obstacles):
        """Dijkstras algorithm for finding the shortest path between two nodes in a graph.

        Args:
            initial_node (list): [row,col] coordinates of the initial node
            desired_node (list): [row,col] coordinates of the desired node
            obstacles (array 2d): 2d numpy array that contains any obstacles as 1s and free space as 0s

        Returns:
            list[list]: list of list of nodes that form the shortest path
        """
        # initialize cost heuristic map
        obstacles = obstacles.copy()
        # obstacles should have very high cost, so we avoid them.
        obstacles *= 1000
        # normal tiles should have 1 cost (1 so we can backtrack)
        obstacles += np.ones(obstacles.shape)
        # source and destination are free
        obstacles[initial_node[0],initial_node[1]] = 0
        obstacles[desired_node[0],desired_node[1]] = 0


        # initialize maps for distances and visited nodes
        size_of_floor = obstacles.shape[0]

        # we only want to visit nodes once
        visited = np.zeros([size_of_floor,size_of_floor],bool)

        # initiate matrix to keep track of distance to source node
        # initial distance to nodes is infinity so we always get a lower actual distance
        distances = np.ones([size_of_floor,size_of_floor]) * np.inf
        # initial node has a distance of 0 to itself
        distances[initial_node[0],initial_node[1]] = 0

        # start algorithm
        current_node = [initial_node[0], initial_node[1]]
        while True:
            directions = [self.up, self.down, self.left, self.right]
            for direction in directions:
                potential_node = direction(current_node)
                if self.valid_node(potential_node, size_of_floor): # boundary checking
                    if not visited[potential_node[0],potential_node[1]]: # check if we have visited this node before
                        # update distance to node
                        distance = distances[current_node[0], current_node[1]] + obstacles[potential_node[0],potential_node[1]]

                        # update distance if it is the shortest discovered
                        if distance < distances[potential_node[0],potential_node[1]]:
                            distances[potential_node[0],potential_node[1]] = distance


            # mark current node as visited
            visited[current_node[0]  ,current_node[1]] = True

            # select next node
            # by choosing the the shortest path so far
            t=distances.copy()
            # we don't want to visit nodes that have already been visited
            t[np.where(visited)]=np.inf
            # choose the shortest path
            node_index = np.argmin(t)

            # convert index to row,col.
            node_row = node_index//size_of_floor
            node_col = node_index%size_of_floor
            # update current node.
            current_node = (node_row, node_col)

            # stop if we have reached the desired node
            if current_node[0] == desired_node[0] and current_node[1]==desired_node[1]:
                break

        # backtrack to construct path
        return self.backtrack(initial_node,desired_node,distances)

def plota(obstacles,path):
    p = np.zeros(shape=obstacles.shape)
    
    for i in range(len(path)):
        p[path[i][0],path[i][1]] = np.nan

    plt.imshow(p+obstacles, cmap='jet')
    plt.show()

def planner():
    
    canvas_x_lim = 10   ## limite em x e y do mundo 
    canvas_y_lim = 10 
    ubot_pose = [0,0] # A gente pode pegar isso do gazebo
    ubot_dim = [1,1] # quantos grids o ubot ocupa coloquei 1 quadrado por default depois a gente troca
    target = [5,5] # alvo define o alvo 

    #mundo = world(canvas=[canvas_x_lim,canvas_y_lim],
    #              agent_pose= ubot_pose,
    #              agent_dimension= ubot_dim)

#    mundo = world(canvas=[canvas_x_lim,canvas_y_lim],
#                  agent_pose= ubot_pose,
#                  agent_dimension= ubot_dim)
#
#    mundo.define_objs() ## define os objetos no espaco, pode jogar pro init_do world de boas
#
#    mundo.construct_field(mundo.models) ## passa os modelos criados para o mundo 
#

    obstacles = np.array([  [0,0,0,0,0,0,0,0,0,0],
                            [0,0,0,0,0,0,0,0,0,0],
                            [1,0,0,0,1,0,0,0,0,1],
                            [1,0,0,1,0,1,0,0,0,1],
                            [1,1,1,1,0,1,1,0,0,1],
                            [1,0,0,0,0,0,0,0,0,1],
                            [1,0,0,0,0,1,1,0,0,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1],
                            [1,1,1,1,1,1,1,1,1,1]], dtype=float)

    # t = dijkstra(init_node=[20,20],target_node=[220,240],obstacles=mundo.obstacles)
    
    t = dijkstra(init_node=[0,0],target_node=[6,8],obstacles=obstacles)
    
    plota(obstacles = obstacles,path = t.path)
    
    #print(t.path)
    
    
    
    
    while not (rospy.is_shutdown()):
    
        continue 
    
    pass


if __name__ == "__main__":
    
    rospy.init_node("cpp_planner",anonymous=False)
    
    planner()
