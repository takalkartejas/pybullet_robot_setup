import pybullet as pybullet
import pybullet_data
import os
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


class simulation():

    def __init__(self):
        self.x =0
    def set_camera_to_robot(self,cdist=3, cyaw=90, cpitch=-40):
        global pybullet
        focuse_position,_ = pybullet.getBasePositionAndOrientation(robot)
        pybullet.resetDebugVisualizerCamera(cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=focuse_position)

    def starting_robot_joints(self,initial_joint_positions):
        global robot
        num_joints = 6
        for joint_index in range(num_joints):
            pybullet.resetJointState(robot, joint_index, initial_joint_positions[joint_index])

class control():
    def __init__(self):
        self.x =0
sim = simulation()
        
# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))

# Start the simulation
physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files

# Load the ground plane
planeId = pybullet.loadURDF("plane.urdf")

# Load your robot URDF file
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file

robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)
pybullet_data_location = "/app/bullet3/examples/pybullet/gym/pybullet_data"

#save robot in rob variable
rob= Robot(robot)

urdfObj, obj_mass, obj_height, force_range, deformation, _ = getObjInfo("RubiksCube")
objStartPos = [0.09, 0.35, obj_height / 2 ]
objStartOrientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
objID = pybullet.loadURDF(urdfObj, objStartPos, objStartOrientation)
obj_weight = pybullet.getDynamicsInfo(objID, -1)[0]
new_mass = 0.1
new_friction = 1.0
pybullet.changeDynamics(objID, -1, mass=new_mass)
'''simulation'''
pybullet.setGravity(0, 0, -9.81)  # Set gravity
pybullet.setTimeStep(0.001)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)

rob.gripper_open()

gripperNames = [
    "base_joint_gripper_left",
    "base_joint_gripper_right",

]
gripperJoints = rob.get_id_by_name(gripperNames)







# Call the main function if this script is executed directly
if __name__ == "__main__":

    #start the robot in this position
    initial_joint_positions = [-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2] 
    sim.starting_robot_joints(initial_joint_positions)

    #move to grip object
    gripping_joint_positions = [-math.pi/2,-2,-1.8,2,-math.pi/2,-math.pi/2] 

    pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=gripping_joint_positions, targetVelocities=[0.00001,0.00001,0.00001,0.00001,0.00001,0.00001])

    #camera properties
    focuse_position,_ = pybullet.getBasePositionAndOrientation(robot)
    cdist=1
    cyaw=180
    cpitch=-20
    cubePos=focuse_position

    #try and execept are used for keyboard interrupt cntrl+c
    try:
        # Get the starting time
        start_time = time.time()
        last_time = start_time
        t=0
        gripper_position_log=[]
        obj_position_z_array = []
        sum =0
        while True:
            pybullet.stepSimulation()
            # Calculate the elapsed real time
            current_time = time.time()
            elapsed_real_time = current_time - start_time        
            # Calculate the time taken for the simulation step
            step_time = current_time - last_time
    
            # Introduce a delay to match real-time if the simulation is running too fast
            time_step = pybullet.getPhysicsEngineParameters()["fixedTimeStep"]
            if step_time < 50*time_step:
                time.sleep(50*time_step - step_time) 
            last_time = current_time

            #control camera using keyboard
            pybullet.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)
            
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
            if t == 200:
                rob.gripper_control_force(0.05,200)
                print('gripper close')
            #after closing gripper
            if t== 300:
                    print('pick up')
                    maxForces = np.ones(8) * 200
                    gripperControlID= [5, 6]
                    
                    joint_positions =[-pi/2,-pi/2,pi/2,-pi/2,-pi/2,0.1, -0.1] 
                    #5= gripper right,6 gripper left, 0= axis 2, 1= axis 3, 2= axis 4, 3=axis 5, 4=axis 
                    maxForces[gripperControlID] = 20
                    # rob.go([0.09,0.35,0.2],width=0.5)

                    pybullet.setJointMotorControlArray(
                    robot, tuple([1, 2, 3, 4, 5, 8, 10]), pybullet.POSITION_CONTROL,
                    targetPositions=joint_positions, targetVelocities=[1,1,1,1,1,1,1], forces=[500,200,100,100,100,300,300])
            #after robot picked up object and went back
            

            if t > 600: 
                pybullet.changeDynamics(objID, -1, mass=new_mass)
                if t%3 == 0:   
                    obj_position,obj_orientation = pybullet.getBasePositionAndOrientation(objID)
                    obj_position_z = obj_position[2] 
                    gripper_positions = [pybullet.getJointState(robot, joint_index)[0] for joint_index in gripperJoints]
                    gripper_position_log.append(gripper_positions)
                    
            if 600 < t < 640:
                if t%3 ==0:
                    obj_position_z_array.append(obj_position_z)

            if t ==641:
                for pos in obj_position_z_array:
                    sum = sum + pos
                avg_obj_position_z = sum/len(obj_position_z_array)
            if t >640:
                if t%3==0:
                    pos_diff_z = avg_obj_position_z - obj_position_z
                    print(pos_diff_z)
                    if pos_diff_z < 0.01:
                        new_mass = new_mass + 0.1
                

                    
            t=t+1
            
    except KeyboardInterrupt:
        print('keyboard interrupt')
pybullet.disconnect()
# Save the gripper positions log to a CSV file
with open("gripper_positions_log.csv", "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    for positions in gripper_position_log:
        writer.writerow(positions)