import pybullet as pybullet
import pybullet_data
import os
import glob
import xml.etree.ElementTree as ET
import time
import math
import pybulletX as px
import cv2
import hydra
import pdb
import sys
import numpy as np
sys.path.append('/app/Taxim')
import taxim_robot
sys.path.append('/app/Taxim/experiments')
import utils
from robot import Robot
from collections import defaultdict
from setup2 import getObjInfo
import robot_functions
from math import pi
import csv
from stl import mesh
#to enumarate states and events in FSM
from enum import Enum 

physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files
'''simulation'''
pybullet.setGravity(0, 0, -9.81)  # Set gravity
pybullet.setTimeStep(0.001)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)


planeId = pybullet.loadURDF("plane.urdf")
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)
# Construct the full path to the URDF file
script_dir = os.path.dirname(os.path.realpath(__file__))
robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85_friction.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)
rob = Robot(robot)

urdf_path = os.path.join("/app/Taxim/experiments/setup2/objects2/test_obj1/model.urdf")
tree = ET.parse(urdf_path)

#change the version urdf files to 1.0 for compatibility, otherwise it gives error
root = tree.getroot()
for elem in root.iter():
    if 'version' in elem.attrib:
        elem.attrib['version'] = '1.0'  # Change the version here to the desired version
# Save the modified URDF file
tree.write(os.path.join("/app/Taxim/experiments/setup2/objects2/test_obj1/model.urdf"))
objStartPos = [-0.09, 0.65, 0.1]
objStartOrientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
objID = pybullet.loadURDF(urdf_path, objStartPos, objStartOrientation)
obj_weight = pybullet.getDynamicsInfo(objID, -1)[0]
new_mass = 0.1
new_friction = 1.0
pybullet.changeDynamics(objID, -1, mass=new_mass)

while True:
        pybullet.stepSimulation()
        cdist=2
        cyaw=180 
        cpitch=-20
        focuse_position,_ = pybullet.getBasePositionAndOrientation(robot)
        pybullet.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=focuse_position)
        
        keys = pybullet.getKeyboardEvents()
        
        #Keys to change camera
        if keys.get(104):  #H
            cyaw+=0.5
        if keys.get(102):   #F
            cyaw-=0.5
        if keys.get(103):   #G
            cpitch+=0.5
        if keys.get(116):  #T
            cpitch-=0.5
        if keys.get(122):  #Z
            cdist+=.01
        if keys.get(120):  #X
            cdist-=.01
        time.sleep(1)