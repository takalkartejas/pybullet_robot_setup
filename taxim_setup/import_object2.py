import pybullet
import pybullet_data
import os
import numpy as np
import time

# Connect to PyBullet
physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files

# Set gravity and simulation parameters
pybullet.setGravity(0, 0, -9.81)  # Set gravity
pybullet.setTimeStep(0.001)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)

# Load plane URDF
planeId = pybullet.loadURDF("plane.urdf")

# Define robot start position and orientation
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file
script_dir = os.path.dirname(os.path.realpath(__file__))
robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85_friction.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)

# Define object URDF file
obj_select_id = 0
urdf = 'model' + str(obj_select_id) + '.urdf'
urdf_path = os.path.join("/app/egadtrainset/", urdf)

# Define object start position and orientation
objStartPos = [-0.09, 0.65, 1]
objStartOrientation = pybullet.getQuaternionFromEuler([0, 0, np.pi / 2])
# Load the object with a specific scale
# scale_factor = 0.1  # Adjust this value as needed
# objID = pybullet.loadURDF(urdf_path, objStartPos, objStartOrientation, globalScaling=scale_factor)
objID = pybullet.loadURDF(urdf_path, objStartPos, objStartOrientation)

# Change object dynamics (mass, friction)
new_mass = 0.1
pybullet.changeDynamics(objID, -1, mass=new_mass)

# Main simulation loop
cdist = 1
cyaw = 180
cpitch = -20

while True:
    pybullet.stepSimulation()
    focuse_position, _ = pybullet.getBasePositionAndOrientation(objID)
    pybullet.resetDebugVisualizerCamera(cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=focuse_position)
    
    keys = pybullet.getKeyboardEvents()
    
    # Keys to change camera
    if keys.get(104):  # H
        cyaw += 0.5
    if keys.get(102):  # F
        cyaw -= 0.5
    if keys.get(103):  # G
        cpitch += 0.5
    if keys.get(116):  # T
        cpitch -= 0.5
    if keys.get(122):  # Z
        cdist += 0.01
    if keys.get(120):  # X
        cdist -= 0.01
    
    time.sleep(0.001)  # Sleep to allow the simulation to update