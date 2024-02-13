import pybullet as p
import pybullet_data
import os

# Get the directory of the Python file
script_dir = os.path.dirname(os.path.realpath(__file__))

pyhsicsClient=p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = p.loadURDF("plane.urdf")

# Load your robot URDF file
robotStartPos = [0, 0, 0]  # Coordinates of the robot in the world
robotStartOrientation = p.getQuaternionFromEuler([0, 0, 0])  # Robot's orientation (roll, pitch, yaw)

robot_urdf_file = os.path.join(script_dir, "urdf/ur5.urdf")
robotId = p.loadURDF(robot_urdf_file, robotStartPos, robotStartOrientation)

