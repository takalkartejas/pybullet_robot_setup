import pybullet as pybullet
import pybullet_data
import os
import time

# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))

# Start the simulation
physicsClient = pybullet.connect(pybullet.GUI)  # or pybullet.DIRECT for non-graphical version
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files

# Load the ground plane
planeId = pybullet.loadURDF("plane.urdf")

# Load your robot URDF file
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = pybullet.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file

robot_urdf_file = os.path.join(script_dir, "urdf/ur5.urdf")
robot = pybullet.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation, useFixedBase=1)



# # Run the simulation for 30 seconds
# simulation_time = 1  # in seconds
# start_time = time.time()

# while (time.time() - start_time) < simulation_time:
#     pybullet.stepSimulation()
    
# # Run the simulation for 1000 steps (adjust as needed)
# for i in range(1000):
#     pybullet.stepSimulation()





'''simulation'''
pybullet.setGravity(0, 0, -9.81)  # Set gravity
pybullet.setTimeStep(0.0001)  # Set the simulation time step
pybullet.setRealTimeSimulation(0)
pybullet.setJointMotorControlArray(
    robot, range(6), pybullet.POSITION_CONTROL,
    targetPositions=[0.9] * 6)
for _ in range(100000):
    pybullet.stepSimulation()

'''get parameters'''
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
print (joint_positions)