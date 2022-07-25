#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform, PointStamped
from std_msgs.msg import Time, Float32, Bool
from rospy.rostime import Duration
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf2_ros
from pynput import keyboard

import copy
import time
import csv

from kortex_driver.srv import *
from kortex_driver.msg import *

class KeyboardController:

    def __init__(self):
        self.robot_name = "my_gen3"
        self.trans_vector = [0, 0, 0]


        self.twist_cmd_pub = rospy.Publisher("/my_gen3/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.trans_vector_sub = rospy.Subscriber("/trans_vector", PointStamped, self.trans_vector_cb)
        self.rot_vector_sub = rospy.Subscriber("/rot_vector", PointStamped, self.rot_vector_cb)


        self.listener = keyboard.Listener(on_press=self.on_press, on_release = self.on_release)
        self.listener.start()
        self.current_key = ""
        self.keyboard_control()

    def keyboard_control(self):
        
        while (self.listener.is_alive()):
            while(self.current_key != ''):
                twist_msg = TwistCommand()
                twist_msg.reference_frame = 1
                if self.current_key == 'w':
                    twist_msg.twist.linear_x = 0.5
                if self.current_key == 's':
                    twist_msg.twist.linear_x = -0.5
                if self.current_key == 'a':
                    twist_msg.twist.linear_y = 0.5
                if self.current_key == 'd':
                    twist_msg.twist.linear_y = -0.5
                if self.current_key == 'z':
                    twist_msg.twist.linear_z = 0.5
                if self.current_key == 'x':
                    twist_msg.twist.linear_z = -0.5
                if self.current_key == 'q':
                    twist_msg.twist.angular_x = -0.1
                if self.current_key == 'e':
                    twist_msg.twist.angular_x = 0.1
                if self.current_key == 'r':
                    twist_msg.twist.angular_y = 0.1
                if self.current_key == 't':
                    twist_msg.twist.angular_y = -0.1
                if self.current_key == 'f':
                    twist_msg.twist.angular_z = 0.1
                if self.current_key == 'g':
                    twist_msg.twist.angular_z = -0.1

                twist_msg.twist.linear_x += self.trans_vector[0]
                twist_msg.twist.linear_y += self.trans_vector[1]
                twist_msg.twist.linear_z += self.trans_vector[2]

                  

                self.twist_cmd_pub.publish(twist_msg)

    def trans_vector_cb(self, msg):
        self.trans_vector = [msg.point.x, msg.point.y, msg.point.z]
        # print(self.trans_vector)

    def rot_vector_cb(self, msg):
        self.rot_vector = [msg.point.x, msg.point.y, msg.point.z]

    def on_press(self, key):
        try:
            self.current_key=key.char
        except AttributeError:
            print('special key {0} pressed'.format(
                key))
            self.current_key=''

    def on_release(self, key):
        self.current_key = ''
        twist_msg = TwistCommand()
        twist_msg.reference_frame = 1
        self.twist_cmd_pub.publish(twist_msg)

        if key == keyboard.Key.esc:
            return False
        if key == keyboard.Key.ctrl:
            return False
        else :
            self.current_key = ''


if __name__ == '__main__':
    rospy.init_node("keyboard_controller")
    kc = KeyboardController()
    rospy.spin()