#! /usr/bin/env python
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

PI = 3.141592653

class DataRecorder(object):
    def __init__(self) -> None:
        super().__init__()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        # Init the topics
        # Kortex ip address = 10.75.12.10
        # self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, 
        #     self.joint_state_cb)
        

        # Use: roslaunch rosbridge_server rosbridge_websocket.launch address:=192.168.0.1
        # roslaunch kortex_driver kortex_driver.launch username:=river password:=river ip_address:=10.75.15.10 gripper:=robotiq_2f_85

        # self.tracker_sub = rospy.Subscriber("/wrist_pose", PoseStamped, self.tracker_cb)
        

        self.joint_state_sub = message_filters.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState)
        self.tracker_sub = message_filters.Subscriber("/wrist_pose", PoseStamped)
        self.sync_sub = message_filters.ApproximateTimeSynchronizer([self.joint_state_sub, self.tracker_sub], 1, 1, allow_headerless=True)
        self.sync_sub.registerCallback(self.sync_cb)

        self.sample_rate = 40 #Hz
        self.traj = []

        # Global data
        self.robot_demo_traj = []
        self.enable_record = False
        self.robot_sample_rate = 40 #Hz
        self.total_robot_samples = 0
        self.data_addr = "/home/riverlab/kinova_ws/src/kortex_playground/sync_data/"
    
        self.tracker_demo_traj = []
        self.tracker_sample_rate = 90
        self.total_tracker_samples = 0


    def joint_state_cb(self, msg):
        if self.total_robot_samples != 0 and len(self.robot_demo_traj) < self.total_robot_samples:
            print("Joint", msg.header.stamp)
            # The joint angle range of JointState is -pi~pi, we need to convert it into 0~2pi
            pos = []
            for p in msg.position:
                if p < 0:
                    pos.append(p + PI)
                else:
                    pos.append(p)
            vel = list(msg.velocity)
            eff = list(msg.effort)
            # t   = msg.stamp.to_sec()
            # Ignore the last finger_joint
            entry = pos[:-1] + vel[:-1] + eff[:-1]
            assert(len(entry) == 7*3), "Data entry length is not correct."
            self.robot_demo_traj.append(entry)

    def tracker_cb(self, msg):
        if self.total_tracker_samples != 0 and len(self.tracker_demo_traj) < self.total_tracker_samples:
            print("wrist", msg.header.stamp)
            tracker_pose = msg.pose
            # Only care about the position of the wrist tracker
            self.tracker_demo_traj.append([tracker_pose.position.x, tracker_pose.position.y, tracker_pose.position.z])

    def sync_cb(self, joint_states, tracker):
        self.joint_state_cb(joint_states)
        self.tracker_cb(tracker)


    
    def start_record(self, duration):
        # record data in duration secs
        self.total_robot_samples = duration * self.sample_rate
        self.total_tracker_samples = duration * self.sample_rate

    def save_data(self):
        robot_data_frame = np.array(self.robot_demo_traj)
        tracker_data_frame = np.array(self.tracker_demo_traj)
        data_frame = np.append(robot_data_frame, tracker_data_frame, axis=1)

        np.savetxt(self.data_addr+'sync10.csv', data_frame , delimiter=",")
        rospy.loginfo("Training data saved to "+self.data_addr+'sync1.csv')


if __name__ == "__main__":
    rospy.init_node("data_recorder", log_level=rospy.INFO)
    data_recorder = DataRecorder()
    input("Hit enter to start recording training data")
    duration = 6
    start = time.time()
    data_recorder.start_record(duration=duration)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(data_recorder.robot_demo_traj) >= duration*data_recorder.robot_sample_rate:
            data_recorder.save_data()
            break
        rate.sleep()
    print(time.time() - start)



        
        
