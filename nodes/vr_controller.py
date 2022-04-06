#! /usr/bin/env python

import queue
import rospy
import rospkg
import numpy as np
import sys
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform
from std_msgs.msg import Time, Float32
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
import tf2_ros

from kortex_driver.msg import TwistCommand, Twist

PI = 3.141592653

class Filter(object):
    def __init__(self) -> None:
        super().__init__()
        
        # Moving average filter
        self.prev_output = 0
    def moving_average_filter(self):
        pass


class ViveController(object): #RENAME to something


    def __init__(self):
        super().__init__()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        # self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, self.joint_cb)
        self.tracker_sub = rospy.Subscriber("/controller_pose", PoseStamped, self.controller_cb, queue_size=1)
        self.twist_cmd_pub = rospy.Publisher("/my_gen3/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.test_pub = rospy.Publisher("/test_topic", Float32, queue_size=1)
        self.br = tf2_ros.TransformBroadcaster()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Initializes initial poses as empty until the user decides to set themwrist_pose
        self.set_init = False
        self.init_pose = Pose()
        self.init_controller_ori_inv = []
        self.init_joint_stat = [] # Saves the robot's joint positions. TODO figure out if we want robot position as a Pose class instead
        self.cur_pose = Pose()
        self.cur_t = Time()
        self.prev_pose = Pose()
        self.prev_t = Time()
        

        # Initialize robot position
        self.init_robot_position = []
        self.init_robot_ori_inv = []
        self.cur_robot_pose = Transform()
        self.joint_stat = []

    # Constantly updates the current controller pose
    def controller_cb(self, msg):
        start_time = time.time()
        self.prev_pose = self.cur_pose
        self.prev_t = self.cur_t
        self.cur_pose = msg.pose
        self.cur_t = msg.header.stamp

        # If initial pose is set, calculate difference relative to init_pose
        if self.set_init:
            dt = self.cur_t.to_sec() - self.prev_t.to_sec()
            if dt and self.prev_pose.position.x != self.cur_pose.position.x:
                self.calculate_vel(dt=dt)                     
        # Publish in the TF tree
        t = TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "unity"
        t.child_frame_id = "vr_controller"
        t.transform.translation.x = self.cur_pose.position.x
        t.transform.translation.y = self.cur_pose.position.y
        t.transform.translation.z = self.cur_pose.position.z
        t.transform.rotation.x = self.cur_pose.orientation.x
        t.transform.rotation.y = self.cur_pose.orientation.y
        t.transform.rotation.z = self.cur_pose.orientation.z
        t.transform.rotation.w = self.cur_pose.orientation.w

        self.br.sendTransform(t)
        end_time = time.time()
        duration = end_time - start_time
        # print(duration)



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
        
    def calculate_vel(self, dt):
        
        # Calculate linear velocity
        prev_pos = np.array([self.prev_pose.position.x, self.prev_pose.position.y, self.prev_pose.position.z])
        cur_pos  = np.array([self.cur_pose.position.x, self.cur_pose.position.y, self.cur_pose.position.z])
        cartesian_linear_vel = (cur_pos - prev_pos) / dt

        # Calculate angular velocity / Feed forward
        prev_ori = [self.prev_pose.orientation.x,
                    self.prev_pose.orientation.y,
                    self.prev_pose.orientation.z,
                    -self.prev_pose.orientation.w]

        cur_ori = [self.cur_pose.orientation.x,
                    self.cur_pose.orientation.y,
                    self.cur_pose.orientation.z,
                    self.cur_pose.orientation.w]       

        angular_diff = quaternion_multiply(cur_ori, prev_ori)
        d_roll, d_pitch, d_yaw = euler_from_quaternion(angular_diff, axes='sxyz')
        # ang_msg = Float32()
        # ang_msg.data = d_pitch
        # self.angular_diff_pub.publish(ang_msg.data)
        # print(f"Roll: {roll:.5f}, Pitch: {pitch:.5f}, Yaw: {yaw:.5f}\n")

        # Calculate position error / Feed backward
        self.cur_robot_pose = self.tf_buffer.lookup_transform("base_link", "tool_frame", rospy.Time()).transform
        controller_pos_diff = cur_pos - self.init_controller_position

        cur_quat = self.tf_buffer.lookup_transform("tool_frame", "vr_controller", rospy.Time()).transform.rotation
        quat_diff = [cur_quat.x, cur_quat.y, cur_quat.z, cur_quat.w]
        quat_err  = quaternion_multiply(quat_diff, self.init_quat_diff_inv)
        pos_error, ang_error = self.calculate_pose_error(controller_pos_diff, quat_err)


        # TODO: Add a low pass filter here
        # self.cartesian_angular_vel = angular_diff / dt
        twist_msg = TwistCommand()
        twist_msg.reference_frame = 0
        ff_term= 0.0
        fb_term = 0.3

        # print(pos_error, ang_error)

        twist_msg.twist.angular_x = ff_term * -d_pitch / dt + fb_term * ang_error[0]
        twist_msg.twist.angular_y = ff_term * d_yaw / dt    + fb_term * ang_error[1]
        twist_msg.twist.angular_z = ff_term * -d_roll / dt  + fb_term * ang_error[2]
        twist_msg.twist.linear_x  = ff_term * -cartesian_linear_vel[0] + fb_term * pos_error[0]
        twist_msg.twist.linear_y  = ff_term * -cartesian_linear_vel[1] + fb_term * pos_error[1]
        twist_msg.twist.linear_z  = ff_term * cartesian_linear_vel[2] + fb_term * pos_error[2]


        self.twist_cmd_pub.publish(twist_msg)
        # print(f"Angular velocity: {self.cartesian_angular_vel}")

    def calculate_pose_error(self, controller_pos_diff, quat_err):        
        # Get robot's linear difference
        robot_pos = np.array([self.cur_robot_pose.translation.x, self.cur_robot_pose.translation.y, self.cur_robot_pose.translation.z])
        robot_pos_diff = robot_pos - self.init_robot_position

        # Get robot's angular difference
        robot_ori = np.array([self.cur_robot_pose.rotation.x,
                              self.cur_robot_pose.rotation.y,
                              self.cur_robot_pose.rotation.z,
                              self.cur_robot_pose.rotation.w])

        # Get error between controller diff and robot diff
        pos_error = [-controller_pos_diff[0], -controller_pos_diff[1], controller_pos_diff[2]] - robot_pos_diff
        
        ang_error = euler_from_quaternion(quat_err)
        
        test_msg = ang_error[1]
        self.test_pub.publish(test_msg)
        # print(robot_ori)
        return pos_error, ang_error
        
        


    def set_init_pose(self):
        input("Press enter to set the intial position of the vr controller")

        self.set_init = True
        # self.init_joint_stat = self.joint_stat
        init_robot_pose = self.tf_buffer.lookup_transform("base_link", "tool_frame", rospy.Time()).transform
        self.init_robot_position = np.array([init_robot_pose.translation.x, init_robot_pose.translation.y, init_robot_pose.translation.z])
        init_quat = self.tf_buffer.lookup_transform("tool_frame", "vr_controller", rospy.Time()).transform.rotation
        self.init_quat_diff_inv = [init_quat.x, init_quat.y, init_quat.z, -init_quat.w]


        self.init_controller_position = [self.cur_pose.position.x, self.cur_pose.position.y, self.cur_pose.position.z]
        self.init_controller_ori_inv = [self.cur_pose.orientation.x,
                                        self.cur_pose.orientation.y,
                                        self.cur_pose.orientation.z,
                                        -self.cur_pose.orientation.w]  

        
        
        print("Initial controller position set as: ", self.init_controller_position)
        


if __name__ == "__main__":
    rospy.init_node("vive_controller_node", log_level=rospy.INFO)
    controller = ViveController()
    controller.set_init_pose()
    rospy.spin()

