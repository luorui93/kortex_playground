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

class ViveController(object):


    def __init__(self):
        super().__init__()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, self.joint_cb)
        self.tracker_sub = rospy.Subscriber("/controller_pose", PoseStamped, self.controller_cb)
        
        # Initializes initial poses as empty until the user decides to set themwrist_pose
        self.set_init = False
        self.init_controller_pose = Pose()
        self.init_controller_inv = []
        self.init_robot_pose = [] # Saves the robot's joint positions. TODO figure out if we want robot position as a Pose class instead
        self.controller_pose = Pose()
        self.robot_pose = Pose()

    # Constantly updates the current controller pose
    def controller_cb(self, msg):
        self.controller_pose = msg.Pose

        # If initial pose is set, calculate difference relative to init_controller_pose
        if self.set_init:
            # Calculate the difference in position.
            current_pose = [self.controller_pose.position.x,
                            self.controller_pose.position.y,
                            self.controller_pose.position.z]
            pos_diff = current_pose - self.init_controller_pos
            
            # Calculate difference in orientation.
            current_orientation = [self.controller_pose.orientation.x,
                                   self.controller_pose.orientation.y,
                                   self.controller_pose.orientation.z,
                                   self.controller_pose.orientation.w]
            ori_diff = quaternion_multiply(current_orientation, self.init_controller_pose_inv)

    def joint_cb(self, msg):
        # The joint angle range of JointState is -pi~pi, we need to convert it into 0~2pi
        pos = []
        for p in msg.position:
            if p < 0:
                pos.append(p + PI)
            else:
                pos.append(p)
        self.robot_pose = pos
        

    def set_init_pose(self):
        input("Press enter to set the intial poses")

        self.set_init = True
        self.init_robot_pose = self.robot_pose

        self.init_controller_pose = self.controller_pose
        
        # Save initial position's x y z pos
        self.init_controller_pos = [self.init_controller_pose.position.x,
                                    self.init_controller_pose.position.y,
                                    self.init_controller_pose.position.z]
        # Calculate initial position's inverse. Code from http://wiki.ros.org/tf2/Tutorials/Quaternions#Relative_rotations
        self.init_controller_inv = [self.init_controller_pose.orientation.x,
                                    self.init_controller_pose.orientation.y,
                                    self.init_controller_pose.orientation.z,
                                    -self.init_controller_pose.orientation.w]

        
        
        print("Initial controller pose set as: ", self.init_controller_pose)
        print("Initial robot pose set as: ", self.init_robot_pose)
        


if __name__ == "__main__":
    controller = ViveController()
    controller.set_init_pose()

