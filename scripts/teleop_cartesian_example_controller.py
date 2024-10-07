# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries, ListControllers
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse

# Frana
from franka_msgs.msg import FrankaState

# General
import copy
import numpy as np
import tf
import time
from tf.transformations import quaternion_matrix, quaternion_from_matrix, rotation_matrix, concatenate_matrices

# Functions from scripts
from util import go_to, attempt_to_go_to_joints

class Teleop():

    def __init__(self):
        ############## Start ROS Node ###################
        rospy.init_node('franka_teleop')

        ############## Global variables #################
        self.vive_last_right_pose = Pose()
        
        self.ee_pose = Pose()

        self.rate = rospy.Rate(50)
        ############## Subscribers ######################

        # VIVE controller pose
        self.vive_right_subscriber = rospy.Subscriber('/vive/my_right_controller_1_Pose', Pose, self.vive_pose_cb)

        # Franka end effector pose (for some reason can't read topic)
        self.subscriber_ee_pose = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__process_ee_pose, queue_size=1)

        ############## Publishers #######################
        self.pub_eq_point = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=0)

        ############## TF ###############################
        self.tf_listener = tf.TransformListener()

    ################################# ROS callback funnctions #####################################################
    def vive_pose_cb(self, msg):
        # Saves vive controller pose to gloabl variable
        self.vive_last_right_pose = msg

    def __process_ee_pose(self, msg):
        # Callback to get EE pose
        initial_quaternion = \
            tf.transformations.quaternion_from_matrix(
                np.transpose(np.reshape(msg.O_T_EE,
                                        (4, 4))))
        initial_quaternion = initial_quaternion / \
            np.linalg.norm(initial_quaternion)

        self.ee_pose.orientation.x = initial_quaternion[0]
        self.ee_pose.orientation.y = initial_quaternion[1]
        self.ee_pose.orientation.z = initial_quaternion[2]
        self.ee_pose.orientation.w = initial_quaternion[3]
        self.ee_pose.position.x = msg.O_T_EE[12]
        self.ee_pose.position.y = msg.O_T_EE[13]
        self.ee_pose.position.z = msg.O_T_EE[14]


    ########################################## Util functions #######################################################
    def get_NE_pose(self): 
        # Function to get EE pose from tf (different way than from franka_state_controller)
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
        self.pub_eq_point.publish(msg)

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


    ############################################ Robot motion scripts ###############################################
    def startup_procedure(self, desired_joint_config=None, initial_config_pose=None):

        self.cartesian_pose_impedance_controller_loaded = False
        self.cartesian_pose_impedance_controller_running = False

        self.effort_joint_trajectory_controller_loaded = False
        self.effort_joint_trajectory_controller_running = False

        list_controller_res = self.list_controllers()
        for i in range(len(list_controller_res.controller)):
            if (list_controller_res.controller[i].name == "cartesian_impedance_example_controller"):
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


        if not(self.cartesian_pose_impedance_controller_loaded and self.effort_joint_trajectory_controller_running):
            time.sleep(0.5)
            # if we do not start up for the first time - we first need to switch back!
            self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_impedance_example_controller"])

        if (desired_joint_config is None):
            desired_joint_config = np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                             0.0032928755527341326, 1.994446315732633, 0.7839058620188021])

        attempt_to_go_to_joints(client, topic, desired_joint_config, duration=5)

        # before switching, save current eef_pos and eef_quat
        curr_eef_pose = copy.deepcopy(self.ee_pose)


        # after moving to the desired pose, switch the controller to the effort joint trajectory!
        if not(self.cartesian_pose_impedance_controller_loaded):
            self.load_controller("cartesian_impedance_example_controller")
            time.sleep(0.5)

        self.switch_controller(["cartesian_impedance_example_controller"], ["effort_joint_trajectory_controller"])



    def update_2_ee_target(self, trans, rot):
        # Function just to test
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'panda_link0'
        # add some scaling such that the robot can be moved more easily
        msg.pose.position.x = trans[0] +0.2
        msg.pose.position.y = trans[1] +0.2
        msg.pose.position.z = trans[2] +0.1

        msg.pose.orientation.x = rot[0]
        msg.pose.orientation.y = rot[1]
        msg.pose.orientation.z = rot[2]
        msg.pose.orientation.w = rot[3]

        return msg

    def run(self):
        counter = 1
        while not rospy.is_shutdown():

            # Get current EE pose
            trans_EE, rot_EE = self.get_NE_pose()
            print("EE position:", trans_EE)
            print("---------------------------------------------------")

            # Get VIVE controller pose
            vive_position, vive_orientation = self.get_pose_info(self.vive_last_right_pose)
            print("VIVE position:", vive_position)

            # Get 0 T EE pose
            print("---------------------------------------------------")
            pos_0_T_EE, rot_0_T_EE = self.get_pose_info(self.ee_pose)
            print("0 T EE:", pos_0_T_EE)
            print("---------------------------------------------------")

            if counter == 1:
                last_vive_position = vive_position

            target_ee_pos = [0,0,0]
            scale_factor = 20

            target_ee_pos[0] = pos_0_T_EE[0] + scale_factor*(vive_position[0] - last_vive_position[0])
            target_ee_pos[1] = pos_0_T_EE[1] + scale_factor*(vive_position[1] - last_vive_position[1])
            target_ee_pos[2] = pos_0_T_EE[2] + scale_factor*(vive_position[2] - last_vive_position[2])


            """
            if counter == 1:
                pos,quat = self.get_pose_info(self.ee_pose)
                self.msg = self.update_2_ee_target(pos,quat)


            self.pub_eq_point.publish(self.msg)
            counter += 1
            """
            self.publish_eef_target(target_ee_pos, rot_0_T_EE)
            last_vive_position = vive_position # storing to take the difference in future loops

            self.rate.sleep()

            counter += 1



if __name__ == '__main__':
    teleop = Teleop()
    teleop.startup_procedure()
    teleop.run()
    


