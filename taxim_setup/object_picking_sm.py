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

#to enumarate states and events in FSM
from enum import Enum 

class setup():

    def __init__(self):
        self.pybullet_data_location = "/app/bullet3/examples/pybullet/gym/pybullet_data"
        global Entity
    def set_camera_to_robot(self):
        global pybullet
        self.cdist=1 
        self.cyaw=180 
        self.cpitch=-20
        self.focuse_position,_ = pybullet.getBasePositionAndOrientation(Entity.robot)
        pybullet.resetDebugVisualizerCamera(cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=self.focuse_position)

    def start_simulation(self):
        self.physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
        pybullet.resetSimulation()
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files
        '''simulation'''
        pybullet.setGravity(0, 0, -9.81)  # Set gravity
        pybullet.setTimeStep(0.001)  # Set the simulation time step
        pybullet.setRealTimeSimulation(0)
    
    def adjust_camera_with_keyboard(self):
            #control camera using keyboard
            pybullet.resetDebugVisualizerCamera( cameraDistance=self.cdist, cameraYaw=self.cyaw, cameraPitch=self.cpitch, cameraTargetPosition=self.focuse_position)
            
            keys = pybullet.getKeyboardEvents()
            
            #Keys to change camera
            if keys.get(104):  #H
                self.cyaw+=0.5
            if keys.get(102):   #F
                self.cyaw-=0.5
            if keys.get(103):   #G
                self.cpitch+=0.5
            if keys.get(116):  #T
                self.cpitch-=0.5
            if keys.get(122):  #Z
                self.cdist+=.01
            if keys.get(120):  #X
                self.cdist-=.01


class control():
    def __init__(self):
        self.x =0
        global Robot

    def start_timer(self):
        # Get the starting time
        self.start_time = time.time()
        self.last_time = self.start_time
    
    def dynamic_delay(self):
        # Calculate the elapsed real time
        self.current_time = time.time()
        self.elapsed_real_time = self.current_time - self.start_time        
        # Calculate the time taken for the simulation step
        step_time = self.current_time - self.last_time

        # Introduce a delay to match real-time if the simulation is running too fast
        time_step = pybullet.getPhysicsEngineParameters()["fixedTimeStep"]
        if step_time < 50*time_step:
            time.sleep(50*time_step - step_time) 
        self.last_time = self.current_time


class entities():
    def __init__(self):
        self.x =0
    def loadRobot(self, script_dir):
        self.robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
        self.robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)
        # Construct the full path to the URDF file
        self.robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85.urdf")
        self.robot = pybullet.loadURDF(self.robot_urdf_file, self.robotStartPos, self.robotStartOrientation, useFixedBase=1)
        self.rob = Robot(self.robot)
    def initialize_robot_at(self,initial_joint_positions):
        num_joints = 6
        for joint_index in range(num_joints):
            pybullet.resetJointState(self.robot, joint_index, initial_joint_positions[joint_index])

    def loadPlane(self):
            # Load the ground plane
        self.planeId = pybullet.loadURDF("plane.urdf")

    def load_object(self):
        self.urdfObj, self.obj_mass, self.obj_height, self.force_range, self.deformation, _ = getObjInfo("RubiksCube")
        self.objStartPos = [0.09, 0.35, self.obj_height / 2 ]
        self.objStartOrientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
        self.objID = pybullet.loadURDF(self.urdfObj, self.objStartPos, self.objStartOrientation)
        self.obj_weight = pybullet.getDynamicsInfo(self.objID, -1)[0]
        self.new_mass = 0.1
        self.new_friction = 1.0
        pybullet.changeDynamics(self.objID, -1, mass=self.new_mass)
    def init_gripper(self):
        self.gripperNames = [
            "base_joint_gripper_left",
            "base_joint_gripper_right",

        ]
        self.gripperJoints = Entity.rob.get_id_by_name(self.gripperNames)

class slip_simulation():
    def __init__(self):
        self.x =0
        global Entity
        self.create_slip_entry_point = True
        self.obj_position_z_array = []
        self.sum =0
        self.new_mass = 0.1

    def init_ss(self):
        #start the robot in this position
        self.initial_joint_positions = [-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2] 
        Entity.initialize_robot_at(self.initial_joint_positions)



    def go_to_object(self):
        #move to grip object
        gripping_joint_positions = [-math.pi/2,-2,-1.8,2,-math.pi/2,-math.pi/2] 

        pybullet.setJointMotorControlArray(
        Entity.robot, range(6), pybullet.POSITION_CONTROL,
        targetPositions=gripping_joint_positions, targetVelocities=[0.00001,0.00001,0.00001,0.00001,0.00001,0.00001])

    def grasp_object(self):
        Entity.rob.gripper_control_force(0.05,200)
        
    def pick_up(self):
            
            maxForces = np.ones(8) * 200
            gripperControlID= [5, 6]
            
            # joint_positions =[-pi/2,-pi/2,pi/2,-pi/2,-pi/2,0.1, -0.1] 
            #5= gripper right,6 gripper left, 0= axis 2, 1= axis 3, 2= axis 4, 3=axis 5, 4=axis 
            joint_positions =[-1,-pi/2,pi/2,-pi/2,-pi/2,0.1, -0.1] 
            maxForces[gripperControlID] = 20
            # rob.go([0.09,0.35,0.2],width=0.5)

            pybullet.setJointMotorControlArray(
            Entity.robot, tuple([1, 2, 3, 4, 5, 8, 10]), pybullet.POSITION_CONTROL,
            targetPositions=joint_positions, targetVelocities=[0.01,0.1,0.1,0.1,0.1,0.1,0.1], forces=[500,200,100,100,100,300,300])
    
    def check_pick_up(self):
        # Get current positions of all joints
        num_joints = 7
        current_positions = pybullet.getJointState(Entity.robot, 2)[0]
        #1 =  axis2, 
        # current_positions = [pybullet.getJointState(Entity.robot, i)[0] for i in range(num_joints)]
        print('current positions=', current_positions)
        # # Check if all joints have reached their desired positions
        # all_in_desired_position = all(abs(current_positions[i] - target_positions[i]) < position_threshold
        #                             for i in range(num_joints))

    
    def create_slip(self, t):
        pybullet.changeDynamics(Entity.objID, -1, mass=self.new_mass)
        if self.create_slip_entry_point==True:
            self.entry_time = t
            self.create_slip_entry_point=False


        #save the current z pos of object
        if t%3 == 0:   
            obj_position,obj_orientation = pybullet.getBasePositionAndOrientation(Entity.objID)
            obj_position_z = obj_position[2] 

        # save the position in an array
        if self.entry_time < t < self.entry_time +40:
            if t%3 ==0:
                self.obj_position_z_array.append(obj_position_z)

        #calculate the average of the array, used to calculate pos difference and thus slip
        if t == self.entry_time+41:
            for pos in self.obj_position_z_array:
                self.sum = self.sum + pos
            self.avg_obj_position_z = self.sum/len(self.obj_position_z_array)

        # calculate if the object is slipping, increase the mass if not
        if t >self.entry_time+ 45:
            if t%3==0:
                pos_diff_z = self.avg_obj_position_z - obj_position_z
                print(pos_diff_z)
                if pos_diff_z < 0.01:
                    self.new_mass = self.new_mass + 0.1
    
    def reset_grasp(self,gripper_positions):
        diff = abs(gripper_positions[0] -  gripper_positions[1])
        print('diff =', diff)
        target = diff - 0.001
        Entity.rob.gripper_control_force(target,200)


class StateMachine():
    def __init__(self):
        
        global pap

        self.State = self.state.sInit
        self.Event = self.event.eNone
        self.previous_state = self.state.sInit

        #flags to ensure that the assignment or trasition occurs only ones per loop
        '''self.assign_function_to_state self.state_transition '''
        self.assign = True
        self.transit = True     
        
        # TCS used in FSM
        '''state_machine'''
        self.TimeSliceCounter = 0


    # This function calls the respective function of a state
    # for eg. if sInit state is active it calles init() function
    # every function returns an event 
    # self.assign flag is set to false which will prevent it from calling multiple functions in a single loop
    def assign_function_to_state(self,function, state):
        if self.State == state and self.assign == True:        
            self.Event = function()
            self.assign =False


    # next state will be selected by using previous state and event
    def state_transition(self,PrevState, event, NextState):
        if self.State == PrevState and self.Event == event and self.transit == True:
            self.State = NextState
            self.transit = False
            # it is important to know previous state in a situation where a ...
            # single state can be called by different states
            if PrevState != NextState:
                self.previous_state = PrevState   

    class event(Enum):
        eNone = 0


    class state(Enum):
        sInit = 0


    def state_machine(self): 
        global look
        
        try:
            # Get the starting time
            start_time = time.time()
            last_time = start_time
            while True:
                self.TimeSliceCounter += 1

                # the assign flag will be set to false once any one assign statement has been executed ...
                # to avoid repition of assignment in one loop. 
                # Same for transit flag - to avoid multiple transitions
                self.assign = True
                self.transit = True
            
                '''******** assign_function_to_state(function, state) ********'''
                self.assign_function_to_state(pap.init, self.state.sInit)
                self.assign_function_to_state(pap.moveToObject, self.state.sGrasp)
                self.assign_function_to_state(pap.grasp, self.state.sGrasp)
                self.assign_function_to_state(pap.drop, self.state.sDrop)

                # print current state and event
                # the print statement should mention the node name in short form at the beginning 
                print('\n MASTER: state= ' + str(self.State) + ' event= ' + str(self.Event))
                
            
                '''******* state_transition(self,PrevState, event, NextState) *******'''
                #sinit
                self.state_transition(self.state.sInit, self.event.eNone, self.state.sInit)
                self.state_transition(self.state.sInit, self.event.eTargetReached, self.state.sRemap)
        except KeyboardInterrupt:

            time.sleep(1)



#initialize classes
Setup = setup()
Entity = entities()
Control = control()
SS = slip_simulation()
# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))


# Call the main function if this script is executed directly
if __name__ == "__main__":

    Setup.start_simulation()
    Entity.loadPlane()
    Entity.loadRobot(script_dir)
    Entity.load_object()
    Entity.rob.gripper_open()
    Entity.init_gripper()
    Setup.set_camera_to_robot()

    #start slip simulation by moving robot to object
    SS.init_ss()
    SS.go_to_object()
    Control.start_timer()

    #try and execept are used for keyboard interrupt cntrl+c
    try:
        
        t=0
        gripper_position_log=[]

        while True:
            pybullet.stepSimulation()
            Setup.adjust_camera_with_keyboard()            
            Control.dynamic_delay()
            

            if t == 200:
                print('gripper close')
                SS.grasp_object()    

            #after closing gripper
            if t== 300:
                print('pick up')
                SS.pick_up()
            
            if 300<t<600:
                SS.check_pick_up() 
            if t> 600:
                if t%3 ==0:
                    gripper_positions = [pybullet.getJointState(Entity.robot, joint_index)[0] for joint_index in Entity.gripperJoints]
                    gripper_position_log.append(gripper_positions)
            if t==610:
                SS.reset_grasp(gripper_positions)
            #after robot picked up object and went back

            if t > 650: 
                SS.create_slip(t)



                    
            t=t+1
            
    except KeyboardInterrupt:
        print('keyboard interrupt')
pybullet.disconnect()


# Save the gripper positions log to a CSV file
with open("gripper_positions_log.csv", "w", newline='') as csvfile:
    writer = csv.writer(csvfile)
    for positions in gripper_position_log:
        writer.writerow(positions)