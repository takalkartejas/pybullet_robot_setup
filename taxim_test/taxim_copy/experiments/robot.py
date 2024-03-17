# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

import numpy as np
import pybullet as pb
import pdb
from pdb import set_trace

class Robot:
    def __init__(self, robotID):
        self.robotID = robotID

        # Get link/joint ID for arm
        self.armNames = [
            # "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.armJoints = self.get_id_by_name(self.armNames)
        self.armControlID = self.get_control_id_by_name(self.armNames)

        '''change to suit my robot'''
        # # Get link/joint ID for gripper
        # self.gripperNames = [
        #     "base_joint_gripper_left",
        #     "base_joint_gripper_right",
        # ]
        self.gripperNames = [
            "base_joint_gripper_left",
            "base_joint_gripper_right",

        ]
        
        self.gripperJoints = self.get_id_by_name(self.gripperNames)
        self.gripperControlID = self.get_control_id_by_name(self.gripperNames)
        pb.enableJointForceTorqueSensor(self.robotID, self.gripperJoints[0])
        pb.enableJointForceTorqueSensor(self.robotID, self.gripperJoints[1])

        # Get ID for end effector
        '''my robot'''
        self.eeName = ["ee_fixed_joint"]
        self.eefID = self.get_id_by_name(self.eeName)[0]


        self.armHome = [-0.26730250468455913, -1.571434616558095, 1.7978336633640315,
                        -1.797156403649463, -1.570812383908381, -0.26729946203972366]
        self.jointHome = [-0.065, 0.065]

        self.pos = [0.581, 0.002, 0.445]
        self.ori = [0, np.pi / 2, 0]
        self.width = self.jointHome[1] - self.jointHome[0]
        self.rot = 0

        self.tol = 1e-9
        self.delta_pos = 0.05
        self.delta_rot = np.pi / 10
        self.delta_width = 0.01

        # self.init_robot()

    def get_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        nbJoint = pb.getNumJoints(self.robotID)
        jointNames = {}
        for i in range(nbJoint):
            name = pb.getJointInfo(self.robotID, i)[1].decode()
            jointNames[name] = i
        # the function will return multiple ids if multiple names are passed and single id if single name is passed
        return [jointNames[name] for name in names]

    def get_control_id_by_name(self, names):
        """
        get joint/link ID by name
        """
        #tt- the robot id indicates ure_wsg50_simplified
        nbJoint = pb.getNumJoints(self.robotID)
        print("/n nbjoints=", nbJoint)
        jointNames = {}
        ctlID = 0
        for i in range(nbJoint):
    
            jointInfo = pb.getJointInfo(self.robotID, i)
            name = jointInfo[1].decode("utf-8")
            print("\n unfiltered joint =", name)
            # skip fixed joint
            if jointInfo[2] == 4:
                
                continue

            # skip base joint
            if jointInfo[-1] == -1:
                print("\n -1=",name)
                continue
            jointNames[name] = ctlID
            ctlID += 1
            print("\n joint =", name)

        return [jointNames[name] for name in names]
    """
            
        The jointInfo variable returns following value on first iteration-
    (1, b'shoulder_pan_joint', 0, 7, 6, 1, 0.0, 0.0, -6.28318530718, 6.28318530718, 150.0, 1.0, b'shoulder_link', (0.0, 0.0, 1.0), (0.0, 0.0, 0.163), (0.0, 0.0, 0.0, 1.0), 0)

    The jointInfo variable appears to contain information about a joint in the robot model. Breaking down the tuple:

    1: This is likely the joint index or ID.

    b'shoulder_pan_joint': This is the name of the joint. The prefix b indicates that it's a bytes object, and shoulder_pan_joint seems to be the name of this specific joint.

    0: This might represent the parent link index. In this case, it's 0, indicating it's the root link or base link.

    7: This could represent the joint type. Common joint types in robotics include revolute joint, prismatic joint, etc. It is revolute joint here
    6: This might represent the joint axis. In robotics, joints often have an axis around which they rotate or translate.

    1: This could represent whether the joint is considered as part of the robot's movable components. In this case, it's 1, indicating it's movable.

    0.0: This might represent the lower limit of the joint's range of motion.

    0.0: This might represent the upper limit of the joint's range of motion.

    -6.28318530718: This might represent the velocity limit or effort limit of the joint.

    6.28318530718: This might represent the velocity limit or effort limit of the joint.

    150.0: This might represent the maximum force or torque that can be applied to the joint.

    1.0: This might represent the joint damping coefficient.

    b'shoulder_link': This is the name of the link connected to this joint.

    (0.0, 0.0, 1.0): This might represent the joint axis in the local frame of the joint.

    (0.0, 0.0, 0.163): This might represent the position of the joint axis relative to the parent link.

    (0.0, 0.0, 0.0, 1.0): This might represent the orientation of the joint axis relative to the parent link.

    0: This might represent additional flags or information about the joint.
        """

    def reset_robot(self):
        for j in range(len(self.armJoints)):
            pb.resetJointState(self.robotID, self.armJoints[j], self.armHome[j])
        for j in range(len(self.gripperJoints)):
            pb.resetJointState(self.robotID, self.gripperJoints[j], self.jointHome[j])

    def init_robot(self):
        self.reset_robot()
        self.operate(self.pos, self.rot, self.width, wait=True)

    # Get the position and orientation of the UR5 end-effector
    def get_ee_pose(self):
        res = pb.getLinkState(self.robotID, self.eefID)
        world_positions = res[0]
        world_orientations = res[1]
        return world_positions, world_orientations

    def get_all_state(self):
        all_states = [_[0] for _ in pb.getJointStates(self.robotID, self.armJoints)]
        return all_states

    # Get the joint angles (6 ur5 joints)
    def get_arm_angles(self):
        joint_angles = [_[0] for _ in pb.getJointStates(self.robotID, self.armJoints)]
        return joint_angles

    # Get the gripper width gripper width
    def get_gripper_width(self):
        width = 2 * np.abs(pb.getJointState(self.robotID, self.gripperJoints[-1])[0])
        return width

    def get_rotation(self):
        _, ori_q = self.get_ee_pose()
        ori = pb.getEulerFromQuaternion(ori_q)
        rot = ori[-1]
        return rot

    def operate(self, pos, rot=None, width=None, gripForce=20, wait=False):
        ori = self.ori.copy()
        ori[-1] = rot
        self.go(pos, ori, width=width, gripForce=gripForce, wait=wait)

    
    # the function moves robot 
    def go(self, pos, ori=None, width=None, wait=False, gripForce=20):
        if ori is None:
            ori = self.ori

        if width is None:
            width = self.width

        ori_q = pb.getQuaternionFromEuler(ori)

        jointPose = pb.calculateInverseKinematics(self.robotID, self.eefID, pos, ori_q)
        jointPose = np.array(jointPose)

        jointPose[self.gripperControlID[0]] = -width / 2
        jointPose[self.gripperControlID[1]] = width / 2

        maxForces = np.ones(len(jointPose)) * 200
        maxForces[self.gripperControlID] = gripForce

        # Select the relavant joints for arm and gripper
        jointPose = jointPose[self.armControlID + self.gripperControlID]
        maxForces = maxForces[self.armControlID + self.gripperControlID]

        pb.setJointMotorControlArray(
            self.robotID,
            tuple(self.armJoints + self.gripperJoints),
            pb.POSITION_CONTROL,
            targetPositions=jointPose,
            forces=maxForces,
        )

        self.pos = pos
        if ori is not None:
            self.ori = ori
        if width is not None:
            self.width = width

        if wait:
            last_err = 1e6
            while True:
                pb.stepSimulation()
                ee_pose = self.get_ee_pose()
                w = self.get_gripper_width()
                err = (
                        np.sum(np.abs(np.array(ee_pose[0]) - pos))
                        + np.sum(np.abs(np.array(ee_pose[1]) - ori_q))
                        + np.abs(w - width)
                )
                diff_err = last_err - err
                last_err = err

                if np.abs(diff_err) < self.tol:
                    break

        # print("FINISHED")

    # Change the gripper to target width
    def gripper_control(self, width):
        pb.setJointMotorControlArray(
            self.robotID,
            self.gripperJoints,
            pb.POSITION_CONTROL,
            targetPositions=[-width / 2, width / 2],
        )
        self.width = width

    # Gripper close
    def gripper_close(self, width=None, max_torque=None):
        pass

    # Gripper open
    def gripper_open(self):
        self.gripper_control(0.11)
