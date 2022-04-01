import rospy
import rospkg
import numpy as np
import sys
from geometry_msgs.msg import Pose, PoseStamped
from rospy.rostime import Duration
from sensor_msgs.msg import JointState
from multiprocessing import Lock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import message_filters
import matplotlib.pyplot as plt
import copy
import os
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply

PI = 3.141592653

class ViveController(object): #RENAME to something


    def __init__(self):
        super().__init__()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, self.joint_cb)
        self.tracker_sub = rospy.Subscriber("/controller_pose", PoseStamped, self.controller_cb)
        
        # Initializes initial poses as empty until the user decides to set themwrist_pose
        self.set_init = False
        self.init_controller_pose = Pose()
        self.init_controller_ori_inv = []
        self.init_joint_stat = [] # Saves the robot's joint positions. TODO figure out if we want robot position as a Pose class instead
        self.controller_pose = Pose()
        self.joint_stat = []

    # Constantly updates the current controller pose
    def controller_cb(self, msg):
        self.controller_pose = msg.Pose

        # If initial pose is set, calculate difference relative to init_controller_pose
        if self.set_init:
            # Get current position and orientation in list form
            current_pos = [self.controller_pose.position.x,
                            self.controller_pose.position.y,
                            self.controller_pose.position.z]
            current_orientation = [self.controller_pose.orientation.x,
                                   self.controller_pose.orientation.y,
                                   self.controller_pose.orientation.z,
                                   self.controller_pose.orientation.w]

            pos_diff, ori_diff = self.calculate_diff(current_pos, current_orientation)

            # TODO Calculate target velocity in cartesian space and convert it to joint velocity

    def joint_cb(self, msg):
        # The joint angle range of JointState is -pi~pi, we need to convert it into 0~2pi
        pos = []
        for p in msg.position:
            if p < 0:
                pos.append(p + PI)
            else:
                pos.append(p)
        self.joint_stat = pos # RENAME joint_pos or joint_stat

    # Calculate the difference in position and orientation from the starting pose
    def calculate_diff(self, current_pos, current_orientation):
        pos_diff = current_pos - self.init_controller_pos
        ori_diff = quaternion_multiply(current_orientation, self.self.init_controller_ori_inv)
        return pos_diff, ori_diff
        

    def set_init_pose(self):
        input("Press enter to set the intial poses")

        self.set_init = True
        self.init_joint_stat = self.joint_stat

        self.init_controller_pose = self.controller_pose
        
        # Save initial position's x y z pos
        self.init_controller_pos = [self.init_controller_pose.position.x,
                                    self.init_controller_pose.position.y,
                                    self.init_controller_pose.position.z]
        # Calculate initial position's inverse. Code from http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
        self.init_controller_ori_inv = [self.init_controller_pose.orientation.x,
                                        self.init_controller_pose.orientation.y,
                                        self.init_controller_pose.orientation.z,
                                        -self.init_controller_pose.orientation.w]
        
        
        print("Initial controller pose set as: ", self.init_controller_pose)
        print("Initial robot pose set as: ", self.init_robot_pose)
        


if __name__ == "__main__":
    controller = ViveController()
    controller.set_init_pose()

