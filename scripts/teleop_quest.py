########################################################################################
# Author: Sophia Moyen (sophiamoyen@gmail.com)
# Last updated: 07.10.2024
#########################################################################################


# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ReloadControllerLibraries, ListControllers
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryResult
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Trigger, TriggerResponse

# Config
import hydra
from omegaconf import DictConfig

# Franka
from franka_msgs.msg import FrankaState

# General
import copy
import numpy as np
import tf
import time
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix, euler_matrix

# Functions from scripts
from util import go_to, attempt_to_go_to_joints
from grasping import Gripper
from RosbagControlledRecorder import RosbagControlledRecorder
from RobomimicRecorder import RobomimicRecorder


# Macro variables (ideally set them as arguments for python script when done with the coding)
QUEST_CONTROLLER = 1 # If the left controller is being used for teleop, then 0. If right, then 1.
SCALE_FACTOR = 7 # Indicates the multiplier for the delta of translation between vr controller and robot
SCALE_FACTOR_ROTATION = 7 # Indicates the multiplier for the delta of rotation between vr controller and robot

import enum

class Oculus_button(enum.Enum):
    A = 0
    B = 1
    RTr = 2
    RG = 3

class Teleop():

    def __init__(self, cfg: DictConfig):
        ############## Start ROS Node ###################
        rospy.init_node('franka_teleop')

        ############## Global variables #################
        self.gripper = Gripper()       # Define gripper class
        self.ee_pose = Pose()          # Pose of Franka's ed effector
        self.gripper_open = False       # State of Franka's gripper

        self.quest_last_pose = Pose()   # Pose of VR controller
        self.quest_button_state = Joy() # State of ALL buttons from VR controller

        # The state change for each specific button will be defined via the button callback function
        self.quest_side_button = 0
        self.quest_home_button = 0
        self.quest_trigger_button = 0
        self.quest_pad_button = 0

        self.teleop_on = False         # When home button is pressed, teleop stops/starts

        self.rate = rospy.Rate(50)     # Setting rate
        ############## Subscribers ######################

        # QUEST controller pose and button states
        if QUEST_CONTROLLER == 0:
            # Left controller
            self.quest_left_subscriber = rospy.Subscriber('/oculus/my_left_controller_1_Pose', Pose, self.quest_pose_cb)
            self.button_left_subscriber = rospy.Subscriber('/oculus/my_left_controller_1/joy', Joy, self.button_cb)

        elif QUEST_CONTROLLER == 1:
            # Right controller
            self.quest_right_subscriber = rospy.Subscriber('/oculus/my_right_controller_1_Pose', Pose, self.quest_pose_cb)
            self.button_right_subscriber = rospy.Subscriber('/oculus/my_right_controller_1/joy', Joy, self.button_cb)

        # Franka end effector pose (for some reason can't read topic)
        self.subscriber_ee_pose = rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, self.__process_ee_pose, queue_size=1)

        ############## Publishers #######################
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.pub_gripper = rospy.Publisher('/cartesian_impedance_controller/desired_gripper_state', PointStamped, queue_size=0)

        ############## TF ###############################
        self.tf_listener = tf.TransformListener()

        ############# Recorder #########################
        self.recorder_type = cfg.recorder_type
        topics_only = [list(item.keys())[0] for item in cfg.topics]
        self.recorder = RosbagControlledRecorder(
            save_folder=cfg.save_folder,
            topics=topics_only
            )
        
        if cfg.video_topics is not None:
            video_topics_only = [list(item.keys())[0] for item in cfg.video_topics]
            self.video_recorder = RosbagControlledRecorder(
                save_folder=cfg.save_folder,
                topics=video_topics_only,
                is_video=True,
                complementary_recorder=self.recorder
                )
            
        if cfg.recorder_type == "robomimic":
            self.video_recorder = None
            all_topics = [list(item.keys())[0] for item in cfg.video_topics] + [list(item.keys())[0] for item in cfg.topics]
            self.recorder = RobomimicRecorder(
                save_folder=cfg.save_folder,
                topics=all_topics,
            )
            

    ################################# ROS callback funnctions #####################################################
    def quest_pose_cb(self, msg):
        # Saves quest controller pose to global variable
        self.quest_last_pose = msg

    def button_cb(self, button_data):
        """
            button_data.buttons[0] = A
            button_data.buttons[1] = B
            button_data.buttons[2] = RTr
            button_data.buttons[3] = RG
        """

    

        # Collects the state for each of the buttons from the VR controller
        self.quest_button_state = button_data 

        # Add logic for starting, stoping teleop in case button was pressed
        if self.quest_home_button == 0 and button_data.buttons[Oculus_button.A.value] == 1:
            self.teleop_on = not self.teleop_on 

        # Add logic for closing and opening gripper
        if self.quest_trigger_button == 0 and button_data.buttons[Oculus_button.B.value] == 1:
            self.gripper_open = not self.gripper_open

        # Add logic for moving the robot home
        # Switch off teleOp to allow for moving to home position
        if self.quest_pad_button == 0 and button_data.buttons[Oculus_button.RTr.value] == 1 and not self.teleop_on:
            self.move_home()     

        # Storing button states
        self.quest_home_button = button_data.buttons[0]
        self.quest_trigger_button = button_data.buttons[1]
        self.quest_pad_button = button_data.buttons[2]
        self.quest_side_button = button_data.buttons[3]

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
    
    def _skew_symmetric_matrix(self, v : np.array):
        # Returns the skew-symmetric matrix for vector v
        skew_mat = np.array([[ 0, -v[2],  v[1]],
                        [ v[2],  0, -v[0]],
                        [-v[1],  v[0],  0]])
        return skew_mat

    def rotation_matrix_from_axis_angle(self, axis, angle):
        # Normalize the axis vector to ensure it's a unit vector
        axis = axis / np.linalg.norm(axis)
        
        # Compute the rotation matrix using Rodrigues' formula
        cos_theta = np.cos(angle)
        sin_theta = np.sin(angle)
        one_minus_cos = 1 - cos_theta
        
        # Compute the outer product of the axis vector with itself
        axis_outer = np.outer(axis, axis)
        
        # Create the rotation matrix
        R = cos_theta * np.eye(3) + one_minus_cos * axis_outer + sin_theta * self._skew_symmetric_matrix(axis)
        
        return R

    def scale_rotation(self, R):
        """ 
            Input:
                R: Rotation matrix
            return Scale Rotational Matrix
        """
        euler = euler_from_matrix(R, 'rxyz')
        euler = [ SCALE_FACTOR_ROTATION*angle for angle in euler ]
        scaled_rotaion_matrix = euler_matrix(-euler[0], -euler[1], euler[2], 'rxyz')
        return scaled_rotaion_matrix

    ############################################### Desired pose/state functions ############################################
    def publish_eef_target(self, pos, quat):
        # Receives position and quaternion and converts to PoseStamped message (the one used for publishing the target pose)
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


    ############################################ Robot motion scripts ###############################################
    def startup_procedure(self, desired_joint_config=None, initial_config_pose=None):
        """
        Load firstly the effort_joint_trajectory_controller to go to standard pose and then switches to
        cartesian_pose_impedance_controller for teleoperation
        """

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

        # Open gripper first
        action_result = self.gripper.move(0.04, 0.04) 
        if action_result == True:
            print("Gripper opened succesfully")
            self.gripper_open = True
        else:
            print("Gripper failed to open")

        if not(self.cartesian_pose_impedance_controller_loaded and self.effort_joint_trajectory_controller_running):
            time.sleep(0.5)
            # if we do not start up for the first time - we first need to switch back!
            self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])

        if (desired_joint_config is None):
            desired_joint_config = np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                             0.0032928755527341326, 1.994446315732633, 0.7839058620188021])

        attempt_to_go_to_joints(client, topic, desired_joint_config, duration=5)

        # After moving to the desired pose, switch the controller to the pose impedance controller
        if not(self.cartesian_pose_impedance_controller_loaded):
            self.load_controller("cartesian_pose_impedance_controller")
            time.sleep(0.5)

        self.switch_controller(["cartesian_pose_impedance_controller"], ["effort_joint_trajectory_controller"])

    def move_home(self):
        # Disabling teleop
        self.teleop_on = False

        self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])
        time.sleep(0.5)
        # then we can execute the normal go home move
        self.startup_procedure()

    ############################################ Teloperation loop ###############################################
    def run(self):
        """
        Runs the teleoperation. Some notes:
            - The teleoperation can be started/stopped by pressing the home button in the quest controller
            - The gripper can closed/opened by pressing the trigger in the quest controller
            - The robot can be moved to home position if the pad button is pressed
            - For updating the translation of the end-effector, a simple scale factor is used sucha as:
                A' = A + alpha(B'-B)
            - For updating the rotation, a matrix multiplication for quaternions is done:
                A' = (B*Binv)*A
        """
        while not rospy.is_shutdown():
            # Keeping values for taking the difference when the teleop is turned on
            last_gripper_state = self.gripper_open 

            # Storing values from the QUEST controller and creating matrices
            last_quest_position, last_quest_rotation = self.get_pose_info(self.quest_last_pose)
            last_quest_homogeneous = np.eye(4)
            last_quest_homogeneous[:3, 3] = copy.deepcopy(last_quest_position)
            last_quest_homogeneous[:3, :3] = quaternion_matrix(copy.deepcopy(last_quest_rotation))[:-1, :-1]
            last_quest_homogeneous_inv = np.eye(4)
            last_quest_homogeneous_inv[:3,:3] = np.linalg.inv(last_quest_homogeneous[:3,:3])
            last_quest_homogeneous_inv[:3, 3] = -np.matmul(np.linalg.inv(last_quest_homogeneous[:3,:3]),last_quest_homogeneous[:3, 3])

            # Storing values from the end-effector and creating matrix
            pos_0_T_EE, rot_0_T_EE = self.get_pose_info(self.ee_pose)
            last_EE_homogeneous = np.eye(4)
            last_EE_homogeneous[:3, 3] = copy.deepcopy(pos_0_T_EE)
            last_EE_homogeneous[:3,:3] = quaternion_matrix(copy.deepcopy(rot_0_T_EE))[:-1, :-1]


            last_quest_position, last_quest_rotation = self.get_pose_info(self.quest_last_pose)


            while self.teleop_on == True:
                # Start recordor if not started otherwise it will do nothing and keep recording
                if not self.recorder.recording_started:
                    self.recorder.start_recording()
                    if self.video_recorder:
                        self.video_recorder.start_recording() # always after main recorder as it complements the other recorder 
                # To start or stop the teleop, home button in the vr controller must be pressed
                if self.recorder_type == "robomimic":
                    self.recorder.snap()
                # Just printing some stuff
                trans_EE, rot_EE = self.get_NE_pose()                                   # Get current EE pose
                print("EE position:", trans_EE)
                print("---------------------------------------------------")
                quest_position, quest_rotation = self.get_pose_info(self.quest_last_pose)  # Get QUEST controller pose
                print("QUEST position:", quest_position)
                print("---------------------------------------------------")
                pos_0_T_EE, rot_0_T_EE = self.get_pose_info(self.ee_pose)               # Get 0 T EE pose
                print("0 T EE:", pos_0_T_EE)
                print("---------------------------------------------------")

                # Calculate the TRANSLATION of the end-effector based on the delta pose of the VR controller
                target_ee_pos = [0,0,0] # Initialize empty list
                target_ee_pos[0] = pos_0_T_EE[0] + SCALE_FACTOR*(quest_position[0] - last_quest_position[0])
                target_ee_pos[1] = pos_0_T_EE[1] - SCALE_FACTOR*(quest_position[1] - last_quest_position[1])
                target_ee_pos[2] = pos_0_T_EE[2] + SCALE_FACTOR*(quest_position[2] - last_quest_position[2])

                # Calculate ROTATION of end-effector A' = (B*Binv)*A
                EE_homogeneous = np.eye(4)
                EE_homogeneous[:3,:3] = quaternion_matrix(rot_0_T_EE)[:-1, :-1]
                EE_homogeneous[:3,3] = pos_0_T_EE

                quest_homogeneous = np.eye(4)
                quest_homogeneous[:3,:3] = quaternion_matrix(quest_rotation)[:-1, :-1]
                quest_homogeneous[:3,3] = quest_position


                rotate_ee = np.matmul(quest_homogeneous, last_quest_homogeneous_inv)
                rotate_ee = self.scale_rotation(rotate_ee)
                target_ee = np.matmul(rotate_ee, EE_homogeneous)
                target_ee_rot = quaternion_from_matrix(target_ee)
 
                # Publish EEF pose for the conttroller
                self.publish_eef_target(target_ee_pos, np.asarray(target_ee_rot))

                # Publish command for gripper opening/closing
                if self.gripper_open != last_gripper_state:
                    if last_gripper_state == False:
                        # If the trigger button was pressed and the gripper was previously closed
                        action_result = self.gripper.move(0.4,0.4) # Open gripper with width
                        if action_result == True:
                            print("Gripper was successfully opened")
                        else:
                            print("Gripper failed to open")

                    elif last_gripper_state == True:
                        result_grasp = self.gripper.grasp()
                        if result_grasp == True:
                            print("Gripper was successfully closed")
                        else:
                            print("Gripper failed to close")

                last_gripper_state = self.gripper_open # Storing to take the difference in future loops
                # Storing values from the QUEST controller and creating matrices
                last_quest_position, last_quest_rotation = self.get_pose_info(self.quest_last_pose)
                last_quest_homogeneous = np.eye(4)
                last_quest_homogeneous[:3, 3] = copy.deepcopy(last_quest_position)
                last_quest_homogeneous[:3, :3] = quaternion_matrix(copy.deepcopy(last_quest_rotation))[:-1, :-1]
                last_quest_homogeneous_inv = np.eye(4)
                last_quest_homogeneous_inv[:3,:3] = np.linalg.inv(last_quest_homogeneous[:3,:3])
                last_quest_homogeneous_inv[:3, 3] = -np.matmul(np.linalg.inv(last_quest_homogeneous[:3,:3]),last_quest_homogeneous[:3, 3])

                # Storing values from the end-effector and creating matrix
                pos_0_T_EE, rot_0_T_EE = self.get_pose_info(self.ee_pose)
                last_EE_homogeneous = np.eye(4)
                last_EE_homogeneous[:3, 3] = copy.deepcopy(pos_0_T_EE)
                last_EE_homogeneous[:3,:3] = quaternion_matrix(copy.deepcopy(rot_0_T_EE))[:-1, :-1]

                last_quest_position, last_quest_rotation = self.get_pose_info(self.quest_last_pose)

                self.rate.sleep()
            if not self.teleop_on and not self.recorder.recording_stopped:
                # Stop recording when Teleop is switched off by the controller
                self.recorder.stop_recording()
                if self.video_recorder:
                    self.video_recorder.stop_recording()




@hydra.main(version_base=None, config_path="/opt/ros_ws/src/franka_teleop/config", config_name="recorder.yaml")
def main(cfg: DictConfig):
    # Initialize Teleop with config parameters
    teleop = Teleop(cfg)
    teleop.startup_procedure()
    teleop.run()

if __name__ == "__main__":
    main()

