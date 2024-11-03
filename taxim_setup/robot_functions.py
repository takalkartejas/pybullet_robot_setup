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

class robot_functions():
    def __init__(self, robot):
        self.robot = robot
        self.eefID = 9

    def print_message(self):
        print('robot_functions')

    def starting_robot_eef(self,pos, ori_q):
        jointPose = pybullet.calculateInverseKinematics(self.robot, self.eefID, pos, ori_q)
        jointPose = np.array(jointPose)
        print('joint Pose=',jointPose)
        # num_joints = pybullet.getNumJoints(self.robot)
        # print(num_joints)
        # num_joints = 6
        # for joint_index in range(num_joints):
        #     pybullet.resetJointState(self.robot, joint_index, initial_joint_positions[joint_index])


    def starting_robot_joints(self,initial_joint_positions):
        num_joints = pybullet.getNumJoints(self.robot)
        print(num_joints)
        num_joints = 6
        for joint_index in range(num_joints):
            pybullet.resetJointState(self.robot, joint_index, initial_joint_positions[joint_index])

    
    def go_to(self,position,orientation,robot):
        print('go_to')

        # Check if the joints have reached their target positions
    def check_joint_positions(self, joint_indices, target_positions, tolerance=0.01):
        all_in_position = True
        for i, joint_index in enumerate(joint_indices):
            current_position = pybullet.getJointState(self.robot, joint_index)[0]
            if abs(current_position - target_positions[i]) > tolerance:
                all_in_position = False
                break
        return all_in_position
    
