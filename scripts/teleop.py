import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries, ListControllers
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse

import copy
import numpy as np
import tf
import time
from tf.transformations import quaternion_matrix, quaternion_from_matrix, rotation_matrix, concatenate_matrices

from util import go_to, attempt_to_go_to_joints

class Teleop():

    def __init__(self):
        ############## Start ROS Node ###################
        rospy.init_node('franka_teleop')

        ############## Global variables #################
        self.vive_last_right_pose = Pose()
        
        self.ee_pos = None
        self.ee_quat = None

        self.gripper_open = None

        self.rate = rospy.Rate(50)
        ############## Subscribers ######################

        # VIVE controller pose
        self.vive_right_subscriber = rospy.Subscriber('/vive/my_right_controller_1_Pose', Pose, self.vive_pose_cb)

        # Franka end effector pose (for some reason can't read topic)
        self.subscribe_ee_pose = rospy.Subscriber('/franka_state_controller/franka_states/O_T_EE', PoseStamped, self.__process_ee_pose, queue_size=1)

        ############## Publishers #######################
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.pub_gripper = rospy.Publisher('/cartesian_impedance_controller/desired_gripper_state', PointStamped, queue_size=0)

        ############## TF ###############################
        self.tf_listener = tf.TransformListener()

    ################################# ROS callback funnctions #####################################################
    def vive_pose_cb(self, msg):
        # Saves vive controller pose to gloabl variable
        self.vive_last_right_pose = msg

    def __process_ee_pose(self, data):
        # Callback to get EE pose
        self.ee_pos = np.asarray([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.ee_quat = np.asarray([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        print("EE pose:",self.ee_pos)


    ########################################## Util functions #######################################################
    def get_NE_pose(self): 
        # Function to get EE pose from tf
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('panda_link0', 'panda_NE', rospy.Time(0))
                if (trans != None) and (rot != None):
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans, rot

    def get_pose_info(self, pose):
        # Gets information from Pose msg into lists
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        rot_x = pose.orientation.x
        rot_y = pose.orientation.y
        rot_z = pose.orientation.z
        rot_w = pose.orientation.w

        return [x,y,z], [rot_x,rot_y,rot_z,rot_w]

    ############################################### Desired pose/state functions ############################################
    def publish_eef_target(self, pos, quat):
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'panda_link0'

        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]

        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        # then apply the action!
        self.pub.publish(msg)

    def publish_gripper_target(self, gripper_action):
        # publish desired gripper state
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        msg_gripper.point.x = gripper_action

        self.pub_gripper.publish(msg_gripper)

    ############################################# ROS controllers functions #############################################

    def list_controllers(self):
        rospy.wait_for_service("/controller_manager/list_controllers")
        list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
        return (list_controllers())

    def load_controller(self, controller_name):
        rospy.wait_for_service("/controller_manager/load_controller")
        load_controller = rospy.ServiceProxy("/controller_manager/load_controller", LoadController)
        load_controller(controller_name)

    def unload_controller(self, controller_name):
        rospy.wait_for_service("/controller_manager/unload_controller")
        unload_controller = rospy.ServiceProxy("/controller_manager/unload_controller", UnloadController)
        unload_controller(controller_name)

    def switch_controller(self, start_controllers, stop_controllers):
        rospy.wait_for_service("/controller_manager/switch_controller")
        switch_controller = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
        switch_controller(start_controllers, stop_controllers, 0, False, 0.0)


    ######################################### Gripper functions ##################################################
    def open_gripper(self):
        self.gripper_open = True

    def open_gripper_w_msg(self):
        # function is needed as otherwise the desired gripper state is only sent upon the node being active!
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        self.gripper_open = True
        msg_gripper.point.x = float(not (self.gripper_open))
        self.pub_gripper.publish(msg_gripper)
        time.sleep(1)


    def close_gripper_w_msg(self):
        # function is needed as otherwise the desired gripper state is only sent upon the node being active!
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        self.gripper_open = False
        msg_gripper.point.x = float(not (self.gripper_open))
        self.pub_gripper.publish(msg_gripper)
        time.sleep(1)


    def close_gripper(self):
        self.gripper_open = False

    ############################################ Robot motion scripts ###############################################
    def startup_procedure(self, desired_joint_config=None, initial_config_pose=None):

        self.cartesian_pose_impedance_controller_loaded = False
        self.cartesian_pose_impedance_controller_running = False

        self.effort_joint_trajectory_controller_loaded = False
        self.effort_joint_trajectory_controller_running = False

        list_controller_res = self.list_controllers()
        for i in range(len(list_controller_res.controller)):
            if (list_controller_res.controller[i].name == "cartesian_pose_impedance_controller"):
                self.cartesian_pose_impedance_controller_loaded = True
                if (list_controller_res.controller[i].state == "running"):
                    self.cartesian_pose_impedance_controller_running = True

            if (list_controller_res.controller[i].name == "effort_joint_trajectory_controller"):
                self.effort_joint_trajectory_controller_loaded = True
                if (list_controller_res.controller[i].state == "running"):
                    self.effort_joint_trajectory_controller_running = True


        action = rospy.resolve_name('effort_joint_trajectory_controller/follow_joint_trajectory')
        client = SimpleActionClient(action, FollowJointTrajectoryAction)
        rospy.loginfo("move_to_start: Waiting for '" + action + "' action to come up")
        client.wait_for_server()

        topic = rospy.resolve_name('franka_state_controller/joint_states')
        rospy.loginfo("move_to_start: Waiting for message on topic '" + topic + "'")
        joint_state = rospy.wait_for_message(topic, JointState)
        initial_pose = dict(zip(joint_state.name, joint_state.position))

        # open gripper first
        self.open_gripper_w_msg()

        if not(self.cartesian_pose_impedance_controller_loaded and self.effort_joint_trajectory_controller_running):
            time.sleep(0.5)
            # if we do not start up for the first time - we first need to switch back!
            self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])

        if (desired_joint_config is None):
            desired_joint_config = np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                             0.0032928755527341326, 1.994446315732633, 0.7839058620188021])

        attempt_to_go_to_joints(client, topic, desired_joint_config, duration=5)

        # before switching, save current eef_pos and eef_quat
        curr_eef_pos = copy.deepcopy(self.ee_pos)
        curr_eef_quat = copy.deepcopy(self.ee_quat)


        # after moving to the desired pose, switch the controller to the effort joint trajectory!
        if not(self.cartesian_pose_impedance_controller_loaded):
            self.load_controller("cartesian_pose_impedance_controller")
            time.sleep(0.5)

        self.switch_controller(["cartesian_pose_impedance_controller"], ["effort_joint_trajectory_controller"])



    def move_robot(self):
        trans_EE, rot_EE = self.get_NE_pose()

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'panda_link0'
        # add some scaling such that the robot can be moved more easily
        msg.pose.position.x = trans_EE[0] +0.2
        msg.pose.position.y = trans_EE[1] +0.2
        msg.pose.position.z = trans_EE[2] +0.1

        msg.pose.orientation.x = rot_EE[0]
        msg.pose.orientation.y = rot_EE[1]
        msg.pose.orientation.z = rot_EE[2]
        msg.pose.orientation.w = rot_EE[3]

        self.pub.publish(msg)

    def run(self):
        while not rospy.is_shutdown():

            # Get current EE pose
            trans_EE, rot_EE = self.get_NE_pose()
            print("EE position:", trans_EE)
            print("---------------------------------------------------")

            # Get VIVE controller pose
            vive_position, vive_orientation = self.get_pose_info(self.vive_last_right_pose)
            print("VIVE position:", vive_position)
            




if __name__ == '__main__':
    teleop = Teleop()
    teleop.startup_procedure()
    teleop.move_robot()
    


