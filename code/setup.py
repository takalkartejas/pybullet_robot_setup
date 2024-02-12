import pybullet as p
import pybullet_data
import os
import time

# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))

# Start the simulation
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Set the search path to find URDF files

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Load your robot URDF file
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

# Construct the full path to the URDF file

robot_urdf_file = os.path.join(script_dir, "universal-robot-ur5/ur5robot.urdf")
robotId = p.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation)

# Simulation loop
p.setGravity(0, 0, -9.8)  # Set gravity
p.setTimeStep(1.0 / 240.0)  # Set the simulation time step
print ('before steps')

# Run the simulation for 30 seconds
simulation_time = 30  # in seconds
start_time = time.time()

while (time.time() - start_time) < simulation_time:
    p.stepSimulation()
    
# # Run the simulation for 1000 steps (adjust as needed)
# for i in range(1000):
#     p.stepSimulation()
print ('after steps')
# End the simulation
p.disconnect()