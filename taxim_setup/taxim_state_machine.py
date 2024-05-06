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

class Setup():

    def __init__(self):
        self.pybullet_data_location = "/app/bullet3/examples/pybullet/gym/pybullet_data"
        self.objects_location = '/app/Taxim/experiments/setup2/objects2'
        global entity
        self.object_names = [name for name in os.listdir(self.objects_location) if os.path.isdir(os.path.join(self.objects_location, name))]
        self.gui = True

    def turn_off_gui(self):
        self.gui = False

    def set_camera_to_robot(self) :
        global pybullet
        self.cdist=1 
        self.cyaw=180 
        self.cpitch=-20
        self.focuse_position,_ = pybullet.getBasePositionAndOrientation(entity.robot)
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

    def get_object_list(self):
        
        print(self.object_names)
        # return object_names'

    def get_height(self,obj_path):
        lcut = hcut = None
        with open(obj_path) as file:
            while 1:
                line = file.readline()
                if not line:
                    break
                strs = line.split(" ")
                if strs[0] == "v":
                    z_cor = float(strs[3])
                    if lcut is None or z_cor < lcut:
                        lcut = z_cor
                    if hcut is None or z_cor > hcut:
                        hcut = z_cor
                if strs[0] == "vt":
                    break
        return hcut - lcut

    def getObjInfo(self,objName):
        assert objName in self.object_names
        urdf_path = os.path.join("/app/Taxim/experiments/setup2/objects2", objName, "model.urdf")
        tree = ET.parse(urdf_path)

        #change the version urdf files to 1.0 for compatibility, otherwise it gives error
        root = tree.getroot()
        for elem in root.iter():
            if 'version' in elem.attrib:
                elem.attrib['version'] = '1.0'  # Change the version here to the desired version
        # Save the modified URDF file
        tree.write(os.path.join("/app/Taxim/experiments/setup2/objects2", objName, "model.urdf"))



        mass_node = next(tree.iter('mass'))
        if mass_node is None:
            raise KeyError("No mass in the urdf file.")
        mass = float(mass_node.attrib["value"])

        friction_node = next(tree.iter('lateral_friction'))
        if friction_node is None:
            raise KeyError("No friction in the urdf file.")
        friction = float(friction_node.attrib["value"])

        obj_mesh_list = glob.glob(os.path.join(self.objects_location, objName, "*.obj"))
        if len(obj_mesh_list) > 0:
            obj_path = obj_mesh_list[0]
            height = self.get_height(obj_path)
        else:
            mesh_file_name = os.path.join(self.objects_location, objName, "*.stl")
            mesh_file_list = glob.glob(mesh_file_name)
            stl_path = mesh_file_list[0]
            stl_file = mesh.Mesh.from_file(stl_path)
            height = np.max(stl_file.z) - np.min(stl_file.z)

        force_range = np.array(
            [0.013903, 0.08583400000000001, 0.18635599999999997, 0.301228, 0.44313, 0.6062639999999999,
            0.7980979999999996,
            1.006655, 1.255222, 1.498395, 1.791708, 2.10153, 2.4639089999999997, 2.873739, 3.3301070000000004,
            3.8420690000000004, 4.392766999999999, 4.958345, 5.535276, 6.171562, 6.803239000000002,
            7.445841000000001,
            8.154558, 8.841395, 9.530221, 10.181313, 10.924032999999998, 11.770725, 12.293285000000001,
            13.091981999999998])
        deformation = np.arange(0.0001, 0.0031, 0.0001)

        return urdf_path, mass, height, force_range, deformation, friction


class Control():
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


class Entities():
    def __init__(self):
        # Get the directory of the Python file
        self.script_dir = os.path.dirname(os.path.realpath(__file__))
        
    def loadRobot(self):
        self.robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
        self.robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)
        # Construct the full path to the URDF file
        self.robot_urdf_file = os.path.join(self.script_dir, "urdf/ur5_robotiq_85_friction.urdf")
        self.robot = pybullet.loadURDF(self.robot_urdf_file, self.robotStartPos, self.robotStartOrientation, useFixedBase=1)
        self.rob = Robot(self.robot)

    def set_joints_friction(self,joint_friction):
                    # Set friction for each joint
        num_joints = 6
        # Set friction for each joint
        for joint_index in range(num_joints):
            # Get joint information
            joint_info = pybullet.getJointInfo(self.robot, joint_index)
            joint_name = joint_info[1].decode("utf-8")

            # Check if joint is revolute or prismatic
            if joint_info[2] == pybullet.JOINT_REVOLUTE or joint_info[2] == pybullet.JOINT_PRISMATIC:
                # Set friction
                pybullet.changeDynamics(self.robot, joint_index, lateralFriction=joint_friction)  # Adjust friction value as needed

    def robot_stopped(self,threshold):
        num_joints = 11
        current_velocities = [pybullet.getJointState(entity.robot, i)[1] for i in range(num_joints)]
        robot_stopped = all(abs(current_velocities[i] - 0) < threshold
                                    for i in range(num_joints))
        return robot_stopped

    def initialize_robot_at(self,initial_joint_positions):
        num_joints = 11
        for joint_index in range(num_joints):
            pybullet.resetJointState(self.robot, joint_index, initial_joint_positions[joint_index])
   
    def initialize_robot(self):
        #start the robot in this position
        self.initial_joint_positions = [-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2,0,0,-0.05,0,0.05] 
        entity.initialize_robot_at(self.initial_joint_positions)

    def loadPlane(self):
            # Load the ground plane
        self.planeId = pybullet.loadURDF("plane.urdf")

    def load_object(self,obj_select_id):
        # self.urdfObj, self.obj_mass, self.obj_height, self.force_range, self.deformation, _ = getObjInfo("RubiksCube")
        #['YcbPottedMeatCan', 'YcbStrawberry','YcbTomatoSoupCan', 
        # 'YcbMustardBottle', 'YcbChipsCan', 'YcbFoamBrick', 'YcbPear', 'MustardBottle', 'TomatoSoupCan', 
        # 'RubiksCube', 'YcbTennisBall', 'YcbMediumClamp']
        self.urdfObj, self.obj_mass, self.obj_height, self.force_range, self.deformation, _ = setup.getObjInfo(setup.object_names[obj_select_id])
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
        self.gripperJoints = entity.rob.get_id_by_name(self.gripperNames)


class Sensor():
    def __init__(self):
        self.x =0  
        self.rot=0  
        self.gripForce = 20
        self.visualize_data = []
 
    def align_image(self, img1, img2):
        img_size = [480, 640]
        new_img = np.zeros([img_size[0], img_size[1] * 2, 3], dtype=np.uint8)
        new_img[:img1.shape[0], :img1.shape[1]] = img2[..., :3]
        new_img[:img2.shape[0], img_size[1]:img_size[1] + img2.shape[1], :] = (img1[..., :3])[..., ::-1]
        return new_img
   
    def setup_sensor(self):
        self.gelsight = taxim_robot.Sensor(width=640, height=480, visualize_gui=True)
        self.cam = utils.Camera(pybullet,[640,480])
        sensorLinks = entity.rob.get_id_by_name(["guide_joint_finger_left"])
        self.gelsight.add_camera(entity.robot, sensorLinks)
        nbJoint = pybullet.getNumJoints(entity.robot)
        self.sensorID1= entity.rob.get_id_by_name(["guide_joint_finger_left"])
        self.gelsight.add_object(entity.urdfObj, entity.objID, force_range=entity.force_range, deformation=entity.deformation)

    def sensor_start(self):
            tactileColor_tmp, _ = self.gelsight.render()
            visionColor_tmp, _ = self.cam.get_image()
            self.visualize_data.append(self.align_image(tactileColor_tmp[0], visionColor_tmp))
            vision_size, tactile_size = visionColor_tmp.shape, tactileColor_tmp[0].shape
            video_path = os.path.join("video", "demo.mp4")
            self.rec = utils.video_recorder(vision_size, tactile_size, path=video_path, fps=30)

    def sensor_read(self):
        #what is the grip force?
        normalForce0, lateralForce0 = utils.get_forces(pybullet, entity.robot, entity.objID, self.sensorID1[0], -1)
        tactileColor, tactileDepth = self.gelsight.render()
        data_dict={}
        data_dict["tactileColorL"], data_dict["tactileDepthL"] = tactileColor[0], tactileDepth[0]
        data_dict["visionColor"], data_dict["visionDepth"] = self.cam.get_image()
        normalForce = [normalForce0]
        data_dict["normalForce"] = normalForce
        data_dict["height"], data_dict["gripForce"], data_dict["rot"] = utils.heightSim2Real(
            [0,0,0]), self.gripForce, self.rot
        # objPos0, objOri0, _ = utils.get_object_pose(pb, objID)
        self.visualize_data.append(self.align_image(data_dict["tactileColorL"], data_dict["visionColor"]))

    def save_data(self):
        tactileColor_tmp, depth = self.gelsight.render()
        visionColor_tmp, _ = self.cam.get_image()
        self.visualize_data.append(self.align_image(tactileColor_tmp[0], visionColor_tmp))
        
        # gelsight.updateGUI(tactileColor_tmp, depth)
        # objPos, objOri, _ = utils.get_object_pose(pb, objID)
        # label = 1*(normalForce0 > 0.1 and np.linalg.norm(objOri - objStartOrientation) < 0.1)
        # data_dict["label"] = label
        self.data_dict["visual"] = self.visualize_data        
  
    def save_data2(self):
        tactileColor_tmp, depth = self.gelsight.render()
        visionColor, visionDepth = self.cam.get_image()
        self.rec.capture(visionColor.copy(), tactileColor_tmp[0].copy())   

    def generate_config_list(self):
            ## maybe a look up table
            force_range_list = {
                "cube": [10],
            }
            gripForce_list = force_range_list["cube"]
            dx_range_list = defaultdict(lambda: np.linspace(-0.015, 0.02, 10).tolist())
            dx_range_list['cube'] = np.array([0.05]) + 0.04
            dx_list = dx_range_list['cube']
            config_list = []
            total_data = 0
            for j, force in enumerate(gripForce_list):
                for k, dx in enumerate(dx_list):
                    config_list.append((force, dx))
                    total_data += 1
            print('config=',config_list)
            t=num_pos=num_data = 0
            rot=0


class SlipSimulation():
    def __init__(self):
        self.x =0
        global entity
        
        self.obj_position_z_array = []
        self.sum =0
        self.new_mass = 0.1

        #flags
        self.create_slip_entry_point = True
        self.object_fell = False
        self.reset_grasp_flag = True
        self.after_picking_up = False

        #counters
        self.slip_counter = 0
        self.TimeSliceCounter=0

    def reset_variables(self):
        self.create_slip_entry_point = True
        self.obj_position_z_array = []
        self.sum =0
        self.new_mass = 0.1
        self.object_fell = False      
        self.reset_grasp_flag = True
        self.slip_counter = 0
        self.after_picking_up = False

    def init_ss(self):
        entity.rob.gripper_control_force(0.25,200)
        entity.load_object(ss.obj_select_id)
        if ss.sensor_on ==True:
            sensor.setup_sensor()
            sensor.sensor_start()
        time.sleep(3)
        return stateMachine.event.eTargetReached
    
    def one_time_setup(self):
        setup.start_simulation()
        entity.loadPlane()
        entity.loadRobot()
        setup.set_camera_to_robot()
        self.obj_select_id = 1
        entity.initialize_robot()
        control.start_timer()
        setup.get_object_list()
        self.gripper_position_log=[]
        entity.init_gripper()

    def repitative_commands(self):
        pybullet.stepSimulation()
        setup.adjust_camera_with_keyboard()            
        control.dynamic_delay()
        if ss.sensor_on == True:
            if ss.TimeSliceCounter>0:
                if ss.TimeSliceCounter%3 == 0:
                    sensor.save_data2()
                sensor.gelsight.update()
        ss.TimeSliceCounter=ss.TimeSliceCounter+1
   
    def go_to_object(self):
        if stateMachine.stateChange == True:
            gripping_joint_positions = [-math.pi/2,-2,-1.8,2,-math.pi/2,-math.pi/2] 
            pybullet.setJointMotorControlArray(
            entity.robot, range(6), pybullet.POSITION_CONTROL,
            targetPositions=gripping_joint_positions, targetVelocities=[0.00001,0.00001,0.00001,0.00001,0.00001,0.00001])
            return stateMachine.event.eNone
        
        elif entity.robot_stopped(0.0006):
            return stateMachine.event.eTargetReached
        else:
            return stateMachine.event.eNone
        
    def grasp_object(self):
        if stateMachine.stateChange==True:
            entity.rob.gripper_control_force(0.05,200)
            return stateMachine.event.eNone
        
        elif entity.robot_stopped(0.0005):
            return stateMachine.event.eTargetReached
        else:
            return stateMachine.event.eNone

    def pick_up(self):
        if stateMachine.stateChange==True:

            maxForces = np.ones(8) * 200
            gripperControlID= [5, 6]
            
            # joint_positions =[-pi/2,-pi/2,pi/2,-pi/2,-pi/2,0.1, -0.1] 
            #5= gripper right,6 gripper left, 0= axis 2, 1= axis 3, 2= axis 4, 3=axis 5, 4=axis 
            joint_positions =[-pi/2,-pi/2,pi/2,-pi/2,-pi/2,0.1, -0.1] 
            self.pick_up_target =[-pi/2]
            for i in range(5):
                self.pick_up_target.append(joint_positions[i])
            # print(self.pick_up_target)
            maxForces[gripperControlID] = 20
            # rob.go([0.09,0.35,0.2],width=0.5)

            pybullet.setJointMotorControlArray(
            entity.robot, tuple([1, 2, 3, 4, 5, 8, 10]), pybullet.POSITION_CONTROL,
            targetPositions=joint_positions, targetVelocities=[0.01,0.1,0.1,0.1,0.1,0.1,0.1], forces=[500,200,100,100,100,300,300])
            return stateMachine.event.eNone
        elif entity.robot_stopped(0.004):
            return stateMachine.event.eTargetReached
        else:
            return stateMachine.event.eNone
        
    def reset_grasp(self):
        if stateMachine.stateChange==True:        
            gripper_positions = [pybullet.getJointState(entity.robot, joint_index)[0] for joint_index in entity.gripperJoints]
            diff = abs(gripper_positions[0] -  gripper_positions[1])
            target = diff - 0.001
            entity.rob.gripper_control_force(target,200)
            return stateMachine.event.eNone
        elif entity.robot_stopped(0.0005):
            return stateMachine.event.eTargetReached
        else:
            return stateMachine.event.eNone        
   
    def create_slip(self):
        pybullet.changeDynamics(entity.objID, -1, mass=self.new_mass)
        if self.create_slip_entry_point==True:
            self.entry_time = self.TimeSliceCounter
            self.create_slip_entry_point=False


        #save the current z pos of object
        if self.TimeSliceCounter%3 == 0:   
            obj_position,obj_orientation = pybullet.getBasePositionAndOrientation(entity.objID)
            obj_position_z = obj_position[2] 

        # save the position in an array
        if self.entry_time < self.TimeSliceCounter < self.entry_time +40:
            if self.TimeSliceCounter%3 ==0:
                self.obj_position_z_array.append(obj_position_z)

        #calculate the average of the array, used to calculate pos difference and thus slip
        diff = self.TimeSliceCounter- self.entry_time
        # print(diff)
        if self.TimeSliceCounter == self.entry_time +41:
            for pos in self.obj_position_z_array:
                self.sum = self.sum + pos
            self.avg_obj_position_z = self.sum/len(self.obj_position_z_array)

        # calculate if the object is slipping, increase the mass if not
        if self.TimeSliceCounter >self.entry_time+ 50:
            if self.TimeSliceCounter%3==0:
                pos_diff_z = self.avg_obj_position_z - obj_position_z
                # print(pos_diff_z)
                if pos_diff_z < 0.02:
                    self.new_mass = self.new_mass + 0.1
                    return stateMachine.event.eNone
                else:
                    return stateMachine.event.eTargetReached
   
    def check_pick_up(self, threshold):
        # Get current positions of all joints
        num_joints = 6

        # current_positions = pybullet.getJointState(Entity.robot, 10)[0]
        #1 = axis2, 2 =axis3, 3 =axis4, 4 = axis5, 5= axis6, 8 = gripper1
        current_positions = [pybullet.getJointState(entity.robot, i)[0] for i in range(num_joints)]
        
        # # Check if all joints have reached their desired positions
        all_in_desired_position = all(abs(current_positions[i] - self.pick_up_target[i]) < threshold
                                    for i in range(num_joints))
        print('current positions=', current_positions)
        print('goal=', self.pick_up_target)
        print(all_in_desired_position)

    def check_object_fall(self):
        if ss.TimeSliceCounter%3 == 0:
            obj_position,obj_orientation = pybullet.getBasePositionAndOrientation(entity.objID)
            obj_position_z = obj_position[2] 
            if abs(obj_position_z - self.avg_obj_position_z) > 0.2:
                self.object_fell = True
            else:
                self.object_fell = False
        if self.object_fell == True:
            return stateMachine.event.eTargetReached
        else:
            return stateMachine.event.eNone
    
    def reset(self):
        pybullet.removeBody(entity.objID)
        ss.TimeSliceCounter=0
        ss.obj_select_id = ss.obj_select_id + 1
        ss.reset_variables()
        return stateMachine.event.eTargetReached
    
    def end(self):
        print('simulation ended')
        raise KeyboardInterrupt


class StateMachine():
    def __init__(self):
        
        global ss

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

        self.stateChange = True

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
                self.stateChange = True
            else:
                self.stateChange = False

    class event(Enum):
        eNone = 0
        eTargetReached = 1


    class state(Enum):
        sInit = 0
        sGoToObject = 1
        sGraspObject = 2
        sPickUp = 3 
        sResetGrasp = 4
        sCreateSlip = 5
        sCheckFall = 6
        sReset = 7
        sEnd = 8

    def state_machine(self): 
        ss.one_time_setup()
        try:
            while True:

                #These commands repeat irregardless of the state
                ss.repitative_commands()


                # the assign flag will be set to false once any one assign statement has been executed ...
                # to avoid repition of assignment in one loop. 
                # Same for transit flag - to avoid multiple transitions
                self.assign = True
                self.transit = True
            
                '''******** assign_function_to_state(function, state) ********'''
                self.assign_function_to_state(ss.init_ss, self.state.sInit)
                self.assign_function_to_state(ss.go_to_object, self.state.sGoToObject)
                self.assign_function_to_state(ss.grasp_object, self.state.sGraspObject)
                self.assign_function_to_state(ss.pick_up, self.state.sPickUp)
                self.assign_function_to_state(ss.reset_grasp, self.state.sResetGrasp)
                self.assign_function_to_state(ss.create_slip, self.state.sCreateSlip)
                self.assign_function_to_state(ss.check_object_fall, self.state.sCheckFall)
                self.assign_function_to_state(ss.reset, self.state.sReset)
                self.assign_function_to_state(ss.end, self.state.sEnd)
                # print current state and event
                # the print statement should mention the node name in short form at the beginning 
                if ss.TimeSliceCounter%20 == 0 or self.stateChange==True:
                    print('\n MASTER: state= ' + str(self.State) + ' event= ' + str(self.Event))
                
            
                '''******* state_transition(self,PrevState, event, NextState) *******'''
                #sinit
                self.state_transition(self.state.sInit, self.event.eNone, self.state.sInit)
                self.state_transition(self.state.sInit, self.event.eTargetReached, self.state.sGoToObject)

                self.state_transition(self.state.sGoToObject, self.event.eNone, self.state.sGoToObject)
                self.state_transition(self.state.sGoToObject, self.event.eTargetReached, self.state.sGraspObject)

                self.state_transition(self.state.sGraspObject, self.event.eNone, self.state.sGraspObject)
                self.state_transition(self.state.sGraspObject, self.event.eTargetReached, self.state.sPickUp)
        
                self.state_transition(self.state.sPickUp, self.event.eNone, self.state.sPickUp)
                self.state_transition(self.state.sPickUp, self.event.eTargetReached, self.state.sResetGrasp)

                self.state_transition(self.state.sResetGrasp, self.event.eNone, self.state.sResetGrasp)
                self.state_transition(self.state.sResetGrasp, self.event.eTargetReached, self.state.sCreateSlip)

                self.state_transition(self.state.sCreateSlip, self.event.eNone, self.state.sCreateSlip)
                self.state_transition(self.state.sCreateSlip, self.event.eTargetReached, self.state.sCheckFall)

                self.state_transition(self.state.sCheckFall, self.event.eNone, self.state.sCheckFall)
                self.state_transition(self.state.sCheckFall, self.event.eTargetReached, self.state.sReset)

                self.state_transition(self.state.sReset, self.event.eNone, self.state.sReset)
                self.state_transition(self.state.sReset, self.event.eTargetReached, self.state.sInit)
        except KeyboardInterrupt:
            print('keyboard interrupt')
            



#initialize classes
setup = Setup()
entity = Entities()
control = Control()
sensor = Sensor()
ss = SlipSimulation()
stateMachine = StateMachine()
ss.sensor_on = False

if __name__ == "__main__":

    stateMachine.state_machine()

pybullet.disconnect()


# # Save the gripper positions log to a CSV file
# with open("gripper_positions_log.csv", "w", newline='') as csvfile:
#     writer = csv.writer(csvfile)
#     for positions in gripper_position_log:
#         writer.writerow(positions)