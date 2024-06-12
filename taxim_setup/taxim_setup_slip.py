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




def _align_image(img1, img2):
    img_size = [480, 640]
    new_img = np.zeros([img_size[0], img_size[1] * 2, 3], dtype=np.uint8)
    new_img[:img1.shape[0], :img1.shape[1]] = img2[..., :3]
    new_img[:img2.shape[0], img_size[1]:img_size[1] + img2.shape[1], :] = (img1[..., :3])[..., ::-1]
    return new_img

def starting_robot_joints(initial_joint_positions):
    global robot
    num_joints = 6
    for joint_index in range(num_joints):
        pybullet.resetJointState(robot, joint_index, initial_joint_positions[joint_index])

'''taxim'''
# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))


# Start the simulation
physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files

'''taxim'''
gelsight = taxim_robot.Sensor(width=640, height=480, visualize_gui=True)

# Load the ground plane
planeId = pybullet.loadURDF("plane.urdf")

# Load your robot URDF file
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)
pandaStartPos = [0, 1, 0.2]  # Coordinates of the robot in the world
pandaStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file

robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)
pybullet_data_location = "/app/bullet3/examples/pybullet/gym/pybullet_data"
# object_loc = os.path.join(pybullet_data_location, "cube_small.urdf")
# object = pybullet.loadURDF(object_loc, pandaStartPos, pandaStartOrientation)

'''taxim'''
rob= Robot(robot)
cam = utils.Camera(pybullet,[640,480])

sensorLinks = rob.get_id_by_name(["guide_joint_finger_left"])
gelsight.add_camera(robot, sensorLinks)
nbJoint = pybullet.getNumJoints(robot)
sensorID1= rob.get_id_by_name(["guide_joint_finger_left"])
# color, depth = gelsight.render()
# gelsight.updateGUI(color, depth)


urdfObj, obj_mass, obj_height, force_range, deformation, _ = getObjInfo("RubiksCube")
objStartPos = [0.09, 0.35, obj_height / 2 ]
objStartOrientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
objID = pybullet.loadURDF(urdfObj, objStartPos, objStartOrientation)
obj_weight = pybullet.getDynamicsInfo(objID, -1)[0]
new_mass = 2.0
new_friction = 1.0
pybullet.changeDynamics(objID, -1, mass=new_mass, lateralFriction=new_friction)
try:
    visual_file = urdfObj.replace("model.urdf", "visual.urdf")
    gelsight.add_object(visual_file, objID, force_range=force_range, deformation=deformation)
except:
    gelsight.add_object(urdfObj, objID, force_range=force_range, deformation=deformation)
print("\nobjectinfo=", urdfObj)
## generate config list
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
t=num_pos=num_data = 0
rot=0


# # Run the simulation for 30 seconds
# simulation_time = 1  # in seconds
# start_time = time.time()

# while (time.time() - start_time) < simulation_time:
#     pybullet.stepSimulation()
    
# # Run the simulation for 1000 steps (adjust as needed)
# for i in range(1000):
#     pybullet.stepSimulation()
'''focus camera'''
focuse_position,_ = pybullet.getBasePositionAndOrientation(robot)
cdist=3
cyaw=90
cpitch=-40
cubePos=focuse_position
pybullet.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=90, cameraPitch=-40, cameraTargetPosition=focuse_position)




'''simulation'''
pybullet.setGravity(0, 0, -9.81)  # Set gravity
pybullet.setTimeStep(0.0005)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)

rob.gripper_open()


initial_joint_positions = [-math.pi/2,-math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2] 
starting_robot_joints(initial_joint_positions)


# Set initial joint positions manually
gripping_joint_positions = [-math.pi/2,-2,-1.8,2,-math.pi/2,-math.pi/2]  # Example joint positions
end_effector_link_index = 7
# Calculate inverse kinematics
# joint_positions = pybullet.calculateInverseKinematics(robot, end_effector_link_index, target_position, target_orientation)

pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=gripping_joint_positions)

robot_func = robot_functions.robot_functions(robot)


# rob.gripper_control(width=20)
try:
    for _ in range(1000000):
        pybullet.stepSimulation()
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

        '''taxim'''
        if t==0:        
            gripForce = 20
            visualize_data = []
            tactileColor_tmp, _ = gelsight.render()
            visionColor_tmp, _ = cam.get_image()
            visualize_data.append(_align_image(tactileColor_tmp[0], visionColor_tmp))
            vision_size, tactile_size = visionColor_tmp.shape, tactileColor_tmp[0].shape
            video_path = os.path.join("video", "demo.mp4")
            rec = utils.video_recorder(vision_size, tactile_size, path=video_path, fps=30)
            
        elif t> 650 and t <700:
            rob.gripper_control(0.05)
            print("/n gripper close")
            
        if t == 720:
            normalForce0, lateralForce0 = utils.get_forces(pybullet, robot, objID, sensorID1[0], -1)
            tactileColor, tactileDepth = gelsight.render()
            data_dict={}
            data_dict["tactileColorL"], data_dict["tactileDepthL"] = tactileColor[0], tactileDepth[0]
            data_dict["visionColor"], data_dict["visionDepth"] = cam.get_image()
            normalForce = [normalForce0]
            data_dict["normalForce"] = normalForce
            data_dict["height"], data_dict["gripForce"], data_dict["rot"] = utils.heightSim2Real(
                [0,0,0]), gripForce, rot
            # objPos0, objOri0, _ = utils.get_object_pose(pb, objID)
            visualize_data.append(_align_image(data_dict["tactileColorL"], data_dict["visionColor"]))
            
            # gelsight.updateGUI(tactileColor, tactileDepth)
            # pos_copy = pos.copy()
            pybullet.changeDynamics(objID, -1, mass=10)
            num_data += 1

        elif t == 760:
            pybullet.setJointMotorControlArray(
            robot, range(6), pybullet.POSITION_CONTROL,
            targetPositions=initial_joint_positions)           

        elif t > 1000:
                        # Save the data
            tactileColor_tmp, depth = gelsight.render()
            visionColor_tmp, _ = cam.get_image()
            visualize_data.append(_align_image(tactileColor_tmp[0], visionColor_tmp))
            
            # gelsight.updateGUI(tactileColor_tmp, depth)
            # objPos, objOri, _ = utils.get_object_pose(pb, objID)
            # label = 1*(normalForce0 > 0.1 and np.linalg.norm(objOri - objStartOrientation) < 0.1)
            # data_dict["label"] = label
            data_dict["visual"] = visualize_data

            rec.release()
        if t>1200:
            break
                ### seq data
        if t % 3 == 0:
            tactileColor_tmp, depth = gelsight.render()
            visionColor, visionDepth = cam.get_image()
            rec.capture(visionColor.copy(), tactileColor_tmp[0].copy())
        gelsight.update()
        print("\nt=",t)
        t += 1
except KeyboardInterrupt:
    print('keyboard interrupt')
position, orientation = pybullet.getBasePositionAndOrientation(robot)
orientation = pybullet.getEulerFromQuaternion(orientation)
NumberOfjoints = pybullet.getNumJoints(robot)

joint_index = 2
joint_info = pybullet.getJointInfo(robot, joint_index)
name, joint_type, lower_limit, upper_limit = \
    joint_info[1], joint_info[2], joint_info[8], joint_info[9]

joint_positions = [j[0] for j in pybullet.getJointStates(robot, range(6))]
joint_positions

print ('before steps')

# End the simulation
pybullet.disconnect()
print (script_dir)