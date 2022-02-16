#! /usr/bin/env python
import rospy
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

msg_rcv_count = 0

def tracker_cb(msg):
    global msg_rcv_count
    msg_rcv_count += 1

if __name__ == '__main__':
    rospy.init_node('find_vive_hz')    

    tracker_sub = rospy.Subscriber("/wrist_pose", PoseStamped, tracker_cb)
    time.sleep(1)

    start_time = time.time()

    time.sleep(100)

    

    print("HZ = " + str(msg_rcv_count/(time.time() - start_time)))

    