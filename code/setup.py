import pybullet as pybullet
import pybullet_data
import os
import time
import math
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
pandaStartPos = [0, 1, 0.2]  # Coordinates of the robot in the world
pandaStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file

robot_urdf_file = os.path.join(script_dir, "urdf/ur5_robotiq_85.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)
pybullet_data_location = "/app/bullet3/examples/pybullet/gym/pybullet_data"
object_loc = os.path.join(pybullet_data_location, "cube_small.urdf")
object = pybullet.loadURDF(object_loc, pandaStartPos, pandaStartOrientation)



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
pybullet.setTimeStep(0.0001)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)

# Set initial joint positions manually
initial_joint_positions = [math.pi/2,math.pi/2,-math.pi/2,math.pi/2,-math.pi/2,-math.pi/2]  # Example joint positions
pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=initial_joint_positions)

for _ in range(100000):
    pybullet.stepSimulation()
    pybullet.resetDebugVisualizerCamera( cameraDistance=cdist, cameraYaw=cyaw, cameraPitch=cpitch, cameraTargetPosition=cubePos)

    keys = pybullet.getKeyboardEvents()
    #Keys to change camera
    if keys.get(100):  #D
        cyaw+=0.1
    if keys.get(97):   #A
        cyaw-=0.1
    if keys.get(99):   #C
        cpitch+=0.1
    if keys.get(102):  #F
        cpitch-=0.1
    if keys.get(122):  #Z
        cdist+=.001
    if keys.get(120):  #X
        cdist-=.001

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