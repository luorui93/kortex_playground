#! /usr/bin/env python

import rospy
import rospkg
import numpy as np
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Transform, WrenchStamped, Wrench
from std_msgs.msg import Time, Float32, Bool
from rospy.rostime import Duration
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_multiply
import tf2_ros

import copy
import time
from multiprocessing import Lock
import csv

from kortex_driver.srv import *
from kortex_driver.msg import *

PI = 3.141592653
HOME_Q = [270, 251.04, 180, 290, 0, 320.77, 270]
HOME_1 = [180, 15, 180, 300, 0, 200, 90]

# Hack solution to solve the redundant TF warning in simulation
PREV_TF_STAMP = 0
PREV_TF_COUNT = 0
# Translation matrix: express unity frame in base_link frame
# T_BU = [
#     [0, -1, 0],
#     [0, 0, 1],
#     [-1, 0, 0]
#     ]
T_BU = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    ]
# Rotation matrix: Express unity frame in tool_frame frame
# R_TU = [
#     [0, -1, 0],
#     [0, 0, 1],
#     [-1, 0, 0]
# ]

# Real Robot
R_TU = [
    [0, 1, 0],
    [0, 0, 1],
    [1, 0, 0]
]

# Simulation Robot
# R_TU = [
#     [1, 0, 0],
#     [0, 1, 0],
#     [0, 0, 1]
# ]

EE_FRAME = "tool_frame"
# EE_FRAME = "test_frame"

def quaternion_diff(q1, q2):
    """
    Calculate the relative quaternion to rotate from q2 to q1 such that
    diff * q2 = q1
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
    def __init__(self, shared_control=False):
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
        self.gripped = 0
        self.shared_control = shared_control
        self.repulsive_control = False

        # Generate transformation matrix
        padding = np.zeros((3,3))
        self.T = np.asarray(np.bmat([[T_BU, padding], [padding, R_TU]]))

        # Initialize robot position
        self.init_robot_position = []
        self.init_robot_ori_inv = []
        self.cur_tool_frame = Transform()
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
        self.teleport_sub = rospy.Subscriber("/teleport", Bool, self.teleport_cb, queue_size=1)

        self.correction_sub = rospy.Subscriber("/resultant_wrench", WrenchStamped, self.correction_cb, queue_size=1)
        self.correction_vector = np.zeros(6)
        

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

        validate_waypoint_list_full_name = '/' + self.robot_name + '/base/validate_waypoint_list'
        rospy.wait_for_service(validate_waypoint_list_full_name)
        self.validate_waypoint_list = rospy.ServiceProxy(validate_waypoint_list_full_name, ValidateWaypointList)

        # rospy.loginfo("Homing Robot!")
        # # self.send_j   oint_traj(HOME_Q)
        self.HOME_ACTION_IDENTIFIER = 2
        # self.home_the_robot()

        twist_msg = TwistCommand()
        twist_msg.reference_frame = 1
        self.twist_cmd_pub.publish(twist_msg)

        # Turn on/off shared control
        if self.shared_control:
            rospack = rospkg.RosPack()
            exp_traj_path = rospack.get_path('jaco3_camera_proc') + '/data/exp_traj.csv'
            with open(exp_traj_path, 'r', newline='') as f:
                reader = csv.reader(f, delimiter=',')
                self.exp_traj = list(reader)
            rospy.loginfo("Successfully read expert trajectory from file.")
        
        rospy.loginfo("Starting teleoperation with VR controller")

    def set_init_pose(self):

        init_robot_pose = self.tf_buffer.lookup_transform("base_link", EE_FRAME, rospy.Time()).transform
        self.init_robot_position = np.array([init_robot_pose.translation.x, 
                                             init_robot_pose.translation.y, 
                                             init_robot_pose.translation.z])
        init_quat = self.tf_buffer.lookup_transform(EE_FRAME, "vr_controller", rospy.Time()).transform.rotation
        self.init_quat_diff_inv = [init_quat.x, init_quat.y, init_quat.z, -init_quat.w]

        self.init_controller_position = [self.cur_pose.position.x, 
                                         self.cur_pose.position.y, 
                                         self.cur_pose.position.z]
        
        print("Initial controller position set as: ", self.init_controller_position)
        
        self.set_init = True

    def wait_for_action_end_or_abort(self):
        while not rospy.is_shutdown():
            if (self.last_action_notif_type == ActionEvent.ACTION_END):
                rospy.loginfo("Received ACTION_END notification")
                return True
            elif (self.last_action_notif_type == ActionEvent.ACTION_ABORT):
                rospy.loginfo("Received ACTION_ABORT notification")
                return False
            else:
                time.sleep(0.01)

    def home_the_robot(self):
        # The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
        self.last_action_notif_type = None
        req = ReadActionRequest()
        req.input.identifier = self.HOME_ACTION_IDENTIFIER

        try:
            res = self.read_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ReadAction")
            return False
        # Execute the HOME action if we could read it
        else:
            # What we just read is the input of the ExecuteAction service
            req = ExecuteActionRequest()
            req.input = res.output
            rospy.loginfo("Sending the robot home...")
            try:
                self.execute_action(req)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ExecuteAction")
                return False
            else:
                # return self.wait_for_action_end_or_abort()
                return True

    def send_joint_traj(self, traj):
        MAX_ANGULAR_DURATION = 30
        self.last_action_notif_type = None

        req = ExecuteActionRequest()

        trajectory = WaypointList()
        waypoints = Waypoint()
        angularWaypoint = AngularWaypoint()
        
        angular_duration = 1
        angularWaypoint.angles = traj
        angularWaypoint.duration = angular_duration
        waypoints.oneof_type_of_waypoint.angular_waypoint.append(angularWaypoint)
        trajectory.duration = 0
        trajectory.use_optimal_blending = False
        trajectory.waypoints.append(waypoints)
        
        while (True) :
            try:
                res = self.validate_waypoint_list(trajectory)
            except rospy.ServiceException:
                rospy.logerr("Failed to call ValidateWaypointList")
                return False
            error_number = len(res.output.trajectory_error_report.trajectory_error_elements)

            if (error_number >= 1 and angular_duration != MAX_ANGULAR_DURATION):
                angularWaypoint.duration = angular_duration
                angular_duration += 1

            elif (angular_duration == MAX_ANGULAR_DURATION) :
                # It should be possible to reach position within 30s
                # WaypointList is invalid (other error than angularWaypoint duration)
                rospy.loginfo("WaypointList is invalid")
                return False

            else:
                break


        req.input.oneof_action_parameters.execute_waypoint_list.append(trajectory)
        try:
            self.execute_action(req)
        except rospy.ServiceException:
            rospy.logerr("Failed to call ExecuteWaypointjectory")
            return False

    # Constantly updates the current controller pose
    def controller_cb(self, msg):
        global PREV_TF_STAMP
        global PREV_TF_COUNT

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
        
        # When getting time from a simulation clock, eg. published by Gazebo,
        # due to the limited resolution of the clock (1000hz), the miminum time diff is
        # 1ms, so if the callback is somehow called multiple times during that time,
        # Time.now() might return the same time thus creating a tf with duplicate time stamp.
        # This is the cause of the following warning:
        # TF_REPEATED_DATA ignoring data with redundant timestamp for frame vr_controller
        t.header.stamp = rospy.Time.now() 
        if t.header.stamp == PREV_TF_STAMP:
            PREV_TF_COUNT += 1
            t.header.stamp.nsecs += PREV_TF_COUNT * 10
        else:
            PREV_TF_COUNT = 0
        # print(t.header.stamp)
        t.header.frame_id = "unity"
        t.child_frame_id = "vr_controller"
        t.transform.translation.x = self.cur_pose.position.x
        t.transform.translation.y = self.cur_pose.position.y
        t.transform.translation.z = self.cur_pose.position.z
        t.transform.rotation.x = self.cur_pose.orientation.x
        t.transform.rotation.y = self.cur_pose.orientation.y
        t.transform.rotation.z = self.cur_pose.orientation.z
        t.transform.rotation.w = self.cur_pose.orientation.w

        PREV_TF_STAMP = rospy.Time.now()

        self.br.sendTransform(t)
        test_msg = self.cur_pose.position.x
        self.test_pub.publish(test_msg)
    
    # Get the value of the trigger. Default is 0. Softmax is 0.75, max is 1
    def trigger_cb(self, msg):
        # if self.set_init:
        #     req = SendGripperCommandRequest()
        #     finger = Finger()
        #     finger.finger_identifier = 0
        #     req.input.gripper.finger.append(finger)
        #     req.input.mode = GripperMode.GRIPPER_POSITION
        #     val = round(msg.data, 1)
        #     if (val * 10) % 2 == 1:
        #         val += 0.1
        #     finger.value = val
        #     self.send_gripper_command(req)
        if (msg.data > 0.1):
            print("TRIGGER")
            self.home_the_robot()


    # Stops all movement and quits if grip is pushed
    def grip_cb(self, msg):
        if msg.data:
            if self.gripped:
                self.gripped = msg.data
                return
            if self.set_init:
                self.set_init = False
                time.sleep(0.1)
                req = SendGripperCommandRequest()
                finger = Finger()
                finger.finger_identifier = 0
                req.input.gripper.finger.append(finger)
                req.input.mode = GripperMode.GRIPPER_POSITION
                finger.value = 0
                # self.send_gripper_command(req)

                twist_msg = TwistCommand()
                self.twist_cmd_pub.publish(twist_msg)
                time.sleep(0.5)
            else:
                self.set_init_pose()
        self.gripped = msg.data


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
        self.cur_tool_frame = self.tf_buffer.lookup_transform("base_link", EE_FRAME, rospy.Time()).transform
        controller_pos_diff = cur_pos - self.init_controller_position

        tf_quat = self.tf_buffer.lookup_transform(EE_FRAME, \
                                "vr_controller", rospy.Time()).transform.rotation
        cur_quat = [tf_quat.x, tf_quat.y, tf_quat.z, tf_quat.w]
        quat_err  = quaternion_multiply(cur_quat, self.init_quat_diff_inv)
        pos_error, ang_error = self.calculate_pose_error(controller_pos_diff, quat_err)



        twist_msg = TwistCommand()
        twist_msg.reference_frame = 1
        ff_rot= 1.0
        ff_trans = 1.0
        fb_rot = 0.5
        fb_trans = 0.5

        filter_input = np.append(cartesian_linear_vel, [d_roll, d_pitch, d_yaw])
        # Output format is [velx, vely, velz, roll, pitch, yaw]
        f_output = self.filter.moving_average_filter(input=filter_input)

        ee_twist = self.T @ np.array(f_output)[:,np.newaxis]
        ee_twist = ee_twist.flatten()

        if not self.shared_control:
            twist_msg.twist.linear_x  = ff_trans * ee_twist[0] + fb_trans * pos_error[0]
            twist_msg.twist.linear_y  = ff_trans * ee_twist[1] + fb_trans * pos_error[1]
            twist_msg.twist.linear_z  = ff_trans * ee_twist[2] + fb_trans * pos_error[2]
            # twist_msg.twist.angular_x = ff_rot * ee_twist[3] / dt #+ fb_rot * ang_error[0]
            # twist_msg.twist.angular_y = ff_rot * ee_twist[4] / dt #+ fb_rot * ang_error[1]
            twist_msg.twist.angular_z = ff_rot * ee_twist[5] / dt #+ fb_rot * ang_error[2]
            
        else:
            # The tf data is from base_link to tool_frame
            p_bt = [self.cur_tool_frame.translation.x, 
                 self.cur_tool_frame.translation.y,
                 self.cur_tool_frame.translation.z]
            ref_pose = self.get_ref_pose_from_exp_traj(p_bt)
            linear_diff = np.array(ref_pose[0:3]) - np.array(p_bt)
            q_bt0 = ref_pose[3:-1]
            # q_tt0 = q_tb * q_bt0 (equation to calculate the difference where q_tb = cur_quat)
            quat_diff = quaternion_multiply(cur_quat, q_bt0)
            rpy_diff = euler_from_quaternion(quat_diff)
        
        # Apply the correction vector
        if self.repulsive_control:
            twist_msg.twist = self.blend_twist(twist_msg.twist)
        # print(twist_msg)

        self.twist_cmd_pub.publish(twist_msg)

    def blend_twist(self, s_t):
        """
        Blend user input with correction vector
        Currently we only blend angular velocity around z axis
        @param s_t current twist
        """
        delta_t = 0.025 # 40 Hz
        damping = 0
        I = 0.01
        m = 0.4
        w_tz = s_t.angular_z
        v_t  = np.array([
            s_t.linear_x,
            s_t.linear_y,
            s_t.linear_z
        ])
        tau_tz = self.correction_vector[5]
        force_t = self.correction_vector[0:3]

        dw = delta_t * (tau_tz - damping*w_tz) / I
        dv = delta_t * (force_t - damping*v_t) / m
        # print('tau_tz', tau_tz)
        # print('w_tz', w_tz)
        # print('dw ', dw)
        # print('dv ', dv)

        w_tz = w_tz + dw
        s_t.angular_z = w_tz

        v_z  = v_t  + dv
        # s_t.linear_x = v_z[0]
        # s_t.linear_y = v_z[1]
        # s_t.linear_z = v_z[2]

        return s_t

    def get_ref_pose_from_exp_traj(self, p):
        # Get reference/optimal pose at given position from expert trajectory
        min_d = 10000000
        min_x = None
        ind = 0
        start = time.monotonic()
        for x in self.exp_traj:
            # d = np.linalg.norm(np.array(x[0:3]) - np.array(x_0))
            d = np.array(x[0:3]) - np.array(p)
            d = np.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
            if d < min_d:
                min_d = d
                min_x = x[0:3]
                ind += 1
        end = time.monotonic()
        print(f"Trajectory length: {len(self.exp_traj)}")
        print(f"Found nearest point at index: {ind}, {min_x}, minimal distance: {min_d}")
        print(f"Search took {1000*(end-start)}ms")
        return self.exp_traj[ind]

    def calculate_pose_error(self, controller_pos_diff, quat_err):        
        # Get robot's linear change
        robot_pos = np.array([self.cur_tool_frame.translation.x, 
                              self.cur_tool_frame.translation.y, 
                              self.cur_tool_frame.translation.z])
        robot_pos_diff = robot_pos - self.init_robot_position

        # Get error between controller diff and robot diff
        c_pos_diff = np.array(T_BU) @ np.array(controller_pos_diff)[:, np.newaxis]
        c_pos_diff = c_pos_diff.flatten()
        pos_error = c_pos_diff - robot_pos_diff
        
        ang_error = euler_from_quaternion(quat_err)

        return pos_error, ang_error

    def correction_cb(self, msg):
        # print(type(msg.wrench))
        self.correction_vector = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z, \
                                            msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])
        self.correction_vector = np.nan_to_num(self.correction_vector)
    
    def teleport_cb(self, msg):
        if msg.data == True:
            self.repulsive_control = not self.repulsive_control
            rospy.sleep(0.5)
            if self.repulsive_control:
                rospy.loginfo("Repulsive control on")
            else:
                rospy.loginfo("Repulsive control off")
        


if __name__ == "__main__":
    rospy.init_node("vive_controller_node", log_level=rospy.INFO)
    controller = ViveController(shared_control=False)
    # controller.set_init_pose()
    rospy.spin()