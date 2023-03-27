#!/usr/bin/env python

import rospy
import numpy as np
import time
import sys
import random


from gazebo_msgs.srv import SpawnModel,DeleteModel,SetModelState,GetModelState
from geometry_msgs.msg import Pose,Point,Quaternion,Wrench,Vector3
from gazebo_msgs.msg import ModelState

class world():
    
    def __init__(self):
        
        self.init_clients()

        
        pass
    def init_clients(self):
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        
        rospy.wait_for_service('/gazebo/get_model_state')
        
        rospy.wait_for_service('/gazebo/set_model_state')
        
        self.spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

        self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

    
        pass
    
    def spwan(self,pose,model_path,model_name):
        
        p = Point(pose[0],pose[1],pose[2])
        o = Quaternion(0,0,0,0)
        pose = Pose(position= p,orientation=o)
        self.spawn_model(
	        	model_name=model_name,
	        	model_xml=open(model_path, 'r').read(),
	        	robot_namespace='/foo',
	        	initial_pose= pose,
	        	reference_frame='world'
	    	)
        pass
    
    def move_model(self,model_name,vel):
        

        t = self.get_state(model_name,"world")

        new_pose = Pose()
        new_pose.position.x = t.pose.position.x + vel[0]
        new_pose.position.y = t.pose.position.y + vel[1]
        new_pose.position.z = t.pose.position.z + vel[2]
        new_pose.orientation = t.pose.orientation
        self.set_state(ModelState(
            model_name = model_name,
            pose = new_pose
        ))


        
        


        pass

def constroi_mundo():
    mundo = world()
    
    numero_de_mesas = 1
    
    
    for i in range(numero_de_mesas):
        mundo.spwan(pose =[0,0,2*i],model_path ='/home/gelo/.gazebo/models/table/model.sdf' ,model_name = 'mesa_'+str(i))

    
    fim = 0

    while(not rospy.is_shutdown()):
        
        inicio = time.time()
        
        if (inicio - fim) > 1:
            mundo.move_model(model_name='mesa_0',vel=[0.3,0,0])
            fim = time.time()
        else: 
            pass

if __name__ == '__main__':
    
    rospy.init_node('world',anonymous=False)
    
    constroi_mundo()