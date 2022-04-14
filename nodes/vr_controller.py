#! /usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform
from std_msgs.msg import Time, Float32, Bool
from rospy.rostime import Duration
from sensor_msgs.msg import JointState
from multiprocessing import Lock
import copy
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf2_ros
from kortex_driver.srv import *
from kortex_driver.msg import TwistCommand, Twist, Finger, GripperMode

PI = 3.141592653

def quaternion_diff(q1, q2):
    """
    Calculate the relative quaternion to rotate from q2 to q1
    """
    q2_inv = copy.copy(q2)
    q2_inv[3] = -q2_inv[3]

    return quaternion_multiply(q1, q2_inv)

class Filter(object):
    def __init__(self, input = []) -> None:
        super().__init__()
        self.fc = 5
        self.w  = 2 * PI * self.fc
        self.Ts = 1/90
        self.input_coeff = self.w * self.Ts / (2 + self.w * self.Ts)
        self.prev_input_coeff = self.w * self.Ts / (2 + self.w * self.Ts)
        self.prev_output_coeff = (2 - self.w * self.Ts) / (2 + self.w * self.Ts)


        # Moving average filter
        self.prev_output = np.array([0] * 6)
        self.prev_input = np.array([0] * 6)
        self.ouput = np.array([0] * 6)


    def moving_average_filter(self, input = [0]*6):
        input = np.array(input)
        output = ((self.input_coeff * input)
                + (self.prev_input_coeff * self.prev_input)
                + (self.prev_output_coeff * self.prev_output))
        self.prev_input = input
        self.prev_output = output

        return output

class ViveController(object): #RENAME to something
    def __init__(self):
        super().__init__()
    
        # Initializes initial poses as empty until the user decides to set themwrist_pose
        self.set_init = False
        self.trigger_val = 0
        self.init_pose = Pose()
        self.init_joint_stat = [] # Saves the robot's joint positions.
        self.cur_pose = Pose()
        self.cur_t = Time()
        self.prev_pose = Pose()
        self.prev_t = Time()
        self.filter = Filter()

        # Initialize robot position
        self.init_robot_position = []
        self.init_robot_ori_inv = []
        self.cur_robot_pose = Transform()
        self.joint_stat = []

        # Get node params
        self.robot_name = rospy.get_param('~robot_name', "my_gen3")

        # Initialize TF buffer and listener
        self.br = tf2_ros.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize subscribers and publishers
        # self.joint_state_sub = rospy.Subscriber("/" + self.robot_name + "/base_feedback/joint_state", JointState, self.joint_cb)
        self.test_pub = rospy.Publisher("/test_topic", Float32, queue_size=1)
        self.tracker_sub = rospy.Subscriber("/controller_pose", PoseStamped, self.controller_cb, queue_size=1)
        self.twist_cmd_pub = rospy.Publisher("/my_gen3/in/cartesian_velocity", TwistCommand, queue_size=1)
        self.trigger_sub = rospy.Subscriber("/trigger", Float32, self.trigger_cb, queue_size=1)
        self.grip_sub = rospy.Subscriber("/grip", Bool, self.grip_cb, queue_size=1)
        

        # Initialize services
        send_gripper_command_full_name = '/my_gen3/base/send_gripper_command'
        rospy.wait_for_service(send_gripper_command_full_name)
        self.send_gripper_command = rospy.ServiceProxy(send_gripper_command_full_name, SendGripperCommand)

        read_action_full_name = '/' + self.robot_name + '/base/read_action'
        rospy.wait_for_service(read_action_full_name)
        self.read_action = rospy.ServiceProxy(read_action_full_name, ReadAction) # Used for homing the robot

        execute_action_full_name = '/' + self.robot_name + '/base/execute_action'
        rospy.wait_for_service(execute_action_full_name)
        self.execute_action = rospy.ServiceProxy(execute_action_full_name, ExecuteAction)

        

    # Constantly updates the current controller pose
    def controller_cb(self, msg):
        self.prev_pose = copy.deepcopy(self.cur_pose)
        self.prev_t = copy.deepcopy(self.cur_t)
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
        test_msg = self.cur_pose.position.x
        self.test_pub.publish(test_msg)
    
    # Get the value of the trigger. Default is 0. Softmax is 0.75, max is 1
    def trigger_cb(self, msg):
        if self.set_init:
            req = SendGripperCommandRequest()
            finger = Finger()
            finger.finger_identifier = 0
            req.input.gripper.finger.append(finger)
            req.input.mode = GripperMode.GRIPPER_POSITION
            val = round(msg.data, 1)
            if (val * 10) % 2 == 1:
                val += 0.1
            finger.value = val
            self.send_gripper_command(req)


    # Stops all movement and quits if grip is pushed
    def grip_cb(self, msg):
        if msg.data:
            self.tracker_sub.unregister()
            req = SendGripperCommandRequest()
            finger = Finger()
            finger.finger_identifier = 0
            req.input.gripper.finger.append(finger)
            req.input.mode = GripperMode.GRIPPER_POSITION
            finger.value = 0
            self.send_gripper_command(req)

            twist_msg = TwistCommand()
            self.twist_cmd_pub.publish(twist_msg)
            time.sleep(0.5)
            rospy.signal_shutdown(reason="User signalled to stop")
            print("Stopped the node")

    def joint_cb(self, msg):
        # The joint angle range of JointState is -pi~pi, we need to convert it into 0~2pi
        pos = []
        for p in msg.position:
            if p < 0:
                pos.append(p + PI)
            else:
                pos.append(p)
        self.joint_stat = pos # RENAME joint_pos or joint_stat
        
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

        # Calculate position and orientation error / Feed backward
        self.cur_robot_pose = self.tf_buffer.lookup_transform("base_link", "tool_frame", rospy.Time()).transform
        controller_pos_diff = cur_pos - self.init_controller_position

        cur_quat = self.tf_buffer.lookup_transform("tool_frame", "vr_controller", rospy.Time()).transform.rotation
        quat_diff = [cur_quat.x, cur_quat.y, cur_quat.z, cur_quat.w]
        quat_err  = quaternion_multiply(quat_diff, self.init_quat_diff_inv)
        pos_error, ang_error = self.calculate_pose_error(controller_pos_diff, quat_err)

        # TODO: Add a low pass filter here
        twist_msg = TwistCommand()
        twist_msg.reference_frame = 0
        ff_rot= 1.0
        ff_trans = 0.5
        fb_rot = 0.5
        fb_trans = 0.5

        filter_input = np.append(cartesian_linear_vel, [d_pitch, d_yaw, d_roll])
        # Output format is [velx, vely, velz, pitch, yaw, roll]
        f_output = self.filter.moving_average_filter(input=filter_input)



        twist_msg.twist.angular_x = ff_rot * -f_output[3] / dt + fb_rot * ang_error[0]
        twist_msg.twist.angular_y = ff_rot * f_output[4] / dt    + fb_rot * ang_error[1]
        twist_msg.twist.angular_z = ff_rot * -f_output[5] / dt  + fb_rot * ang_error[2]
        twist_msg.twist.linear_x  = ff_trans * -f_output[0] + fb_trans * pos_error[0]
        twist_msg.twist.linear_y  = ff_trans * -f_output[1] + fb_trans * pos_error[1]
        twist_msg.twist.linear_z  = ff_trans * f_output[2]  + fb_trans * pos_error[2]
        # twist_msg.twist.angular_x = ff_rot * -d_pitch / dt + fb_rot * ang_error[0]
        # twist_msg.twist.angular_y = ff_rot * d_yaw / dt    + fb_rot * ang_error[1]
        # twist_msg.twist.angular_z = ff_rot * -d_roll / dt  + fb_rot * ang_error[2]
        # twist_msg.twist.linear_x  = ff_trans * -cartesian_linear_vel[0] + fb_trans * pos_error[0]
        # twist_msg.twist.linear_y  = ff_trans * -cartesian_linear_vel[1] + fb_trans * pos_error[1]
        # twist_msg.twist.linear_z  = ff_trans * cartesian_linear_vel[2]  + fb_trans * pos_error[2]

        self.twist_cmd_pub.publish(twist_msg)

    def calculate_pose_error(self, controller_pos_diff, quat_err):        
        # Get robot's linear difference
        robot_pos = np.array([self.cur_robot_pose.translation.x, self.cur_robot_pose.translation.y, self.cur_robot_pose.translation.z])
        robot_pos_diff = robot_pos - self.init_robot_position

        # Get error between controller diff and robot diff
        pos_error = [-controller_pos_diff[0], -controller_pos_diff[1], controller_pos_diff[2]] - robot_pos_diff
        
        ang_error = euler_from_quaternion(quat_err)

        return pos_error, ang_error

    def set_init_pose(self):
        key = None
        while key != '':
            key = input("Press enter to set the intial position of the vr controller, or \"h\" to home the robot\n")
            if key == 'h':
                req = ReadActionRequest()
                req.input.identifier = 2 # Home action identifier
                res = self.read_action(req)
                req = ExecuteActionRequest()
                req.input = res.output
                self.execute_action(req)
                time.sleep(1)
        # input("Press enter to set the initial position of the vr controller")
        # self.init_joint_stat = self.joint_stat
        init_robot_pose = self.tf_buffer.lookup_transform("base_link", "tool_frame", rospy.Time()).transform
        self.init_robot_position = np.array([init_robot_pose.translation.x, 
                                             init_robot_pose.translation.y, 
                                             init_robot_pose.translation.z])
        init_quat = self.tf_buffer.lookup_transform("tool_frame", "vr_controller", rospy.Time()).transform.rotation
        self.init_quat_diff_inv = [init_quat.x, init_quat.y, init_quat.z, -init_quat.w]

        self.init_controller_position = [self.cur_pose.position.x, self.cur_pose.position.y, self.cur_pose.position.z]
        
        print("Initial controller position set as: ", self.init_controller_position)
        
        self.set_init = True
        


if __name__ == "__main__":
    rospy.init_node("vive_controller_node", log_level=rospy.INFO)
    controller = ViveController()
    controller.set_init_pose()
    rospy.spin()