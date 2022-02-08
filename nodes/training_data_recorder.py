#! /usr/bin/env python
import rospy
import rospkg
import numpy as np
import sys
from geometry_msgs.msg import Pose
from rospy.rostime import Duration
from sensor_msgs.msg import JointState
from multiprocessing import Lock
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import message_filters
import matplotlib.pyplot as plt
import copy
import os

PI = 3.141592653

class DataRecorder(object):
    def __init__(self) -> None:
        super().__init__()

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")
        # Init the topics
        self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, 
            self.joint_state_cb)
        

        # Global data
        self.robot_demo_traj = []
        self.enable_record = False
        self.sample_rate = 100 #Hz
        self.total_samples = 0
        self.data_addr = "/home/rui/kinova_ws/src/kortex_playground/data/"
    
    def joint_state_cb(self, msg):
        if self.total_samples != 0 and len(self.robot_demo_traj) < self.total_samples:
            # The joint angle range of JointState is -pi~pi, we need to convert it into 0~2pi
            if msg.position < 0:
                pos = msg.position + PI
            else:
                pos = msg.position
            vel = msg.velocity
            eff = msg.effort
            # t   = msg.stamp.to_sec()
            # Ignore the last finger_joint
            entry = pos[:-1] + vel[:-1] + eff[:-1]
            assert(len(entry) == 7*3), "Data entry length is not correct."
            self.robot_demo_traj.append(entry)
    
    def start_record(self, duration):
        # record data in duration secs
        self.total_samples = duration * self.sample_rate
    
    def save_data(self):
        data_frame = np.array(self.robot_demo_traj)
        np.savetxt(self.data_addr+'test.csv', data_frame , delimiter=",")
        rospy.loginfo("Training data saved to "+self.data_addr+'test.csv')


if __name__ == "__main__":
    rospy.init_node("data_recorder", log_level=rospy.INFO)
    data_recorder = DataRecorder()
    input("Hit enter to start recording training data")
    duration = 4
    data_recorder.start_record(duration=duration)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(data_recorder.robot_demo_traj) == duration*data_recorder.sample_rate:
            data_recorder.save_data()
            break
        rate.sleep()



        
        
