#! /usr/bin/env python

import sys
import rospy
import time
import numpy as np

from kortex_driver.srv import *
from kortex_driver.msg import *

if __name__ == "__main__":
  rospy.init_node("joint_velocity_pub", anonymous=True)
  joint_speeds_pub = rospy.Publisher("/my_gen3/in/joint_velocity", Base_JointSpeeds, queue_size=10, latch=True)
  joint_vel = Base_JointSpeeds()
  joint_vel.joint_speeds = [JointSpeed() for i in range(7)]
  for i in range(7):
    joint_vel.joint_speeds[i].joint_identifier = i
    joint_vel.joint_speeds[i].value = 1.0
  
  r = rospy.Rate(10)
  joint_vel.joint_speeds[0].value = 0
  joint_speeds_pub.publish(joint_vel)

  rospy.spin()