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

class RRT(object):

    def __init__(self,z_start,z_target,dimensions=[],max_int = 900):
        self.space  = Space() # defino o space
        self.z_start = z_start # start point
        self.z_target = z_target # goal point
        self.r_tree = Tree(Node(z_start)) # inicializo a arvore com start point
        for t in range(max_int): 
            #print('n',t)
            self.get_z_random() # Pego aleatoriamente um no que pertenca ao espaco 
            self.get_near_node() # defino o no mais perto do gerado aleatorio
            if self.col_free(): # verifico se esse caminho entre eles e livre de colisao
                self.node = Node(self.z_random) ## cria random como um node
                self.r_tree.pertence_arvore(self.node)
                self.node.parent.append(self.z_near) ## adiciona z_near como pai de z_random
                self.z_near.child.append(self.node) ## adiciona z_random como filho de z_near
                self.node.h = self.min_space ## atualiza a distancia entre z_near e z_random
                self.node.g = norm(np.array(self.node.pose)-np.array(self.z_start))
                self.r_tree.add_node(self.node)
        
        self.r_tree.find_minimal_path()
                
        
        pass       

    def col_free(self):
        return True

    def get_near_node(self):
        self.min_space = float('inf')
        distance = []
        for i in self.r_tree.nodes:
            distance.append(norm(np.array(self.z_random) - np.array(i.pose)))
        distance = np.array(distance)
        dmin = min(distance)
        ind_min = distance.tolist().index(dmin)
        self.z_near = self.r_tree.nodes[ind_min] 
        pass

    def get_z_random(self):
        if random.random() <=0.50:
            self.z_random = self.z_target
            #print('prob')
        else:
            self.z_random  = [random.randint(self.space.x_min,self.space.x_max),
                              random.randint(self.space.y_min,self.space.y_max),
                              random.randint(self.space.z_min,self.space.z_max)]

class Node(object):
    def __init__(self,node):
        self.pose = node
        self.x = node[0]
        self.y = node[1]
        self.z = node[2]
        self.parent = []
        self.child = []
        self.h = 0 ## distance between parent and actual node 
        self.g = 0 
        pass

class Tree(object):
    
    def __init__(self,node):
        self.nodes = []
        self.nodes.append(node)
        pass
    
    def add_node(self,node):
        self.nodes.append(node)
        pass
    
    def find_minimal_path(self):
        self.last_node = self.nodes[0]
        actual_node = self.nodes[len(self.nodes)-1]
        self.path = []
        new_node = []
        
        while norm(np.array(self.last_node.pose)-np.array(actual_node.pose)) != 0: ## self.nodes[0] = Z_start
            min_g = float('inf')
            
            for i in actual_node.parent: ##pega todos os itens parents
                if i.g < min_g:  ## verifica qual tem a menor distancia 
                    new_node = i
                    min_g = i.g
            actual_node = new_node
            self.path.append(actual_node.pose)
        
        pass
    
    def find_paths(self):
        self.first_node = self.nodes[0]
        self.current_node = self.first_node
        path = []
        while norm(np.array(self.current_node) - np.array(self.z_target)) != 0: # enquanto current node != target node
            random.choice()     
               
    def pertence_arvore(self,node):
        if node in self.nodes:
            print('estou na arvore')

class Space(object):
    def __init__(self,dimensions = [[0,10],[0,10],[0,10]]):
        self.x_min = dimensions[0][0]
        self.x_max = dimensions[0][1]
        self.y_min = dimensions[1][0]
        self.y_max = dimensions[1][1]
        self.z_min = dimensions[2][0]
        self.z_max = dimensions[2][1]
        pass

def rrt_algorithm():
    
    path = RRT(z_start=[8,9,8],z_target=[10,10,10])
    print(path.r_tree.path)
    
    pass

def planner():
    
    canvas_x_lim = 10   ## limite em x e y do mundo 
    canvas_y_lim = 10 
    ubot_pose = [0,0] # A gente pode pegar isso do gazebo
    ubot_dim = [1,1] # quantos grids o ubot ocupa coloquei 1 quadrado por default depois a gente troca
    target = [5,5] # alvo define o alvo 

    mundo = world(canvas=[canvas_x_lim,canvas_y_lim],
                  agent_pose= ubot_pose,
                  agent_dimension= ubot_dim)


    path = RRT(z_start=[8,9,8],z_target=[10,10,10])
    
    print(path.r_tree.path)

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

    

    
    p = np.zeros(shape=mundo.obstacles.shape)
    for i in range(len(t.path)):
        p[t.path[i][0],t.path[i][1]] = np.nan

    plt.imshow(p+mundo.obstacles, cmap='jet')
    plt.show()
    
    
    while not (rospy.is_shutdown()):
    
        continue 
    
    pass


if __name__ == "__main__":
    
    rospy.init_node("path_planer",anonymous=False)
    
    planner()
