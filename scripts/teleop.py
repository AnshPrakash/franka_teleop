#!/usr/bin/env python3
"""
franka_teleop node for teleoperating a Franka Emika Panda robot using a Meta Quest Controller.
Logic is to map the controller's pose changes to the robot's end-effector pose changes, with scaling.
The teleoperation can be toggled on/off via the home button, and the gripper can be opened/closed via the trigger button.
Check "https://github.com/AnshPrakash/franka_teleop/tree/main?tab=readme-ov-file#3-finally-how-to-teleoperate" for more details.
"""

import copy
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# controller manager services and controller/action interfaces
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
from control_msgs.action import FollowJointTrajectory

# Franka
from franka_msgs.msg import FrankaState

# tf2
import tf_transformations
import tf2_ros
from rclpy.duration import Duration

from util import go_to, attempt_to_go_to_joints
from grasping import Gripper

# Macro variables -> now declared as node parameters by default
DEFAULT_VIVE_CONTROLLER = 1 # If the left controller is being used for teleop, then 0. If right, then 1.
DEFAULT_SCALE_FACTOR = 10  # Indicates the multiplier for the delta of translation between vr controller and robot

class Teleop(Node):
    def __init__(self):
        super().__init__('franka_teleop')

        # Parameters (user can override via ROS 2 params)
        self.declare_parameter('vive_controller', DEFAULT_VIVE_CONTROLLER)
        self.declare_parameter('scale_factor', DEFAULT_SCALE_FACTOR)

        self.VIVE_CONTROLLER = self.get_parameter('vive_controller').value
        self.SCALE_FACTOR = self.get_parameter('scale_factor').value

        ############## Global variables #################
        self.gripper = Gripper()       # Define gripper class
        self.ee_pose = Pose()          # Pose of Franka's end effector
        self.gripper_open = False      # State of Franka's gripper

        self.vive_last_pose = Pose()   # Pose of VR controller
        self.vive_button_state = Joy() # State of ALL buttons from VR controller

        # The state change for each specific button will be defined via the button callback function
        self.vive_side_button = 0
        self.vive_home_button = 0
        self.vive_trigger_button = 0
        self.vive_pad_button = 0

        self.teleop_on = False         # When home button is pressed, teleop stops/starts

        ############## TF ###############################
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        ############## Publishers #######################
        qos_profile = 10  # simple convenience; change to explicit QoSProfile if you need reliability/durability
        self.pub = self.create_publisher(PoseStamped, '/cartesian_impedance_controller/desired_pose', qos_profile)
        self.pub_gripper = self.create_publisher(PointStamped, '/cartesian_impedance_controller/desired_gripper_state', qos_profile)

        ############## Subscribers ######################
        # VIVE controller pose and button states
        if self.VIVE_CONTROLLER == 0:
            # Left controller
            self.vive_left_subscriber = self.create_subscription(Pose, '/oculus/my_left_controller_1_Pose', self.vive_pose_cb, qos_profile)
            self.button_left_subscriber = self.create_subscription(Joy, '/oculus/my_left_controller_1/joy', self.button_cb, qos_profile)

        elif self.VIVE_CONTROLLER == 1:
            # Right controller
            self.vive_right_subscriber = self.create_subscription(Pose, '/oculus/my_right_controller_1_Pose', self.vive_pose_cb, qos_profile)
            self.button_right_subscriber = self.create_subscription(Joy, '/oculus/my_right_controller_1/joy', self.button_cb, qos_profile)

        # Franka end effector pose (arrives on franka_state_controller in the system)
        self.subscriber_ee_pose = self.create_subscription(FrankaState, '/franka_state_controller/franka_states', self.__process_ee_pose, 1)

        # Service clients (controller manager)
        self._list_controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._load_controller_client = self.create_client(LoadController, '/controller_manager/load_controller')
        self._unload_controller_client = self.create_client(UnloadController, '/controller_manager/unload_controller')
        self._switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # Note: Action client for following joint trajectories (used in startup_procedure)
        action_name = 'effort_joint_trajectory_controller/follow_joint_trajectory'
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, action_name)

        self.get_logger().info('Teleop node initialized')

    ################################# ROS callback functions #####################################################
    def vive_pose_cb(self, msg: Pose):
        # Saves vive controller pose to instance variable
        self.vive_last_pose = msg

    def button_cb(self, button_data: Joy):
        # Collects the state for each of the buttons from the VR controller
        self.vive_button_state = button_data

        # Add logic for starting/stopping teleop in case button was pressed
        if self.vive_home_button == 0 and len(button_data.buttons) > 0 and button_data.buttons[0] == 1:
            self.teleop_on = not self.teleop_on
            self.get_logger().info(f'Teleop toggled: {self.teleop_on}')

        # Add logic for closing and opening gripper
        if self.vive_trigger_button == 0 and len(button_data.buttons) > 1 and button_data.buttons[1] == 1:
            self.gripper_open = not self.gripper_open
            self.get_logger().info(f'Gripper open: {self.gripper_open}')

        # Add logic for moving the robot home
        if self.vive_pad_button == 0 and len(button_data.buttons) > 2 and button_data.buttons[2] == 1:
            self.move_home()

        # Storing button states
        self.vive_home_button = button_data.buttons[0]
        self.vive_trigger_button = button_data.buttons[1]
        self.vive_pad_button = button_data.buttons[2]
        self.vive_side_button = button_data.buttons[3]

    def __process_ee_pose(self, msg: FrankaState):
        # Callback to get EE pose (converts O_T_EE array to position + quaternion)
        try:
            # msg.O_T_EE is expected to be a flat list/array of length 16 (row-major or column-major?)
            # The original ROS1 code reshaped and transposed. Keep same logic.
            mat = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
            initial_quaternion = tf_transformations.quaternion_from_matrix(mat)
            initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)

            self.ee_pose.orientation.x = float(initial_quaternion[0])
            self.ee_pose.orientation.y = float(initial_quaternion[1])
            self.ee_pose.orientation.z = float(initial_quaternion[2])
            self.ee_pose.orientation.w = float(initial_quaternion[3])

            # Indices 12,13,14 after reshape+transpose correspond to translation (same as original)
            self.ee_pose.position.x = float(msg.O_T_EE[12])
            self.ee_pose.position.y = float(msg.O_T_EE[13])
            self.ee_pose.position.z = float(msg.O_T_EE[14])
        except Exception as e:
            self.get_logger().warn(f'Failed to process FrankaState O_T_EE: {e}')

    ########################################## Util functions #######################################################
    def get_NE_pose(self):
        # Use tf2 buffer to lookup transform 'panda_link0' <- 'panda_NE' (different way than from franka_state_controller)
        # (tries until success)
        while rclpy.ok():
            try:
                # lookup_transform(target_frame, source_frame, time)
                trans_stamped = self.tf_buffer.lookup_transform('panda_link0', 'panda_NE', rclpy.time.Time())
                t = trans_stamped.transform.translation
                r = trans_stamped.transform.rotation
                trans = (t.x, t.y, t.z)
                rot = (r.x, r.y, r.z, r.w)
                return trans, rot
            except Exception:
                # lookup may throw LookupException, ConnectivityException, ExtrapolationException
                # Sleep a bit then retry
                time.sleep(0.1)
                continue

    def get_pose_info(self, pose: Pose):
        # Gets information from Pose msg into lists
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z

        rot_x = pose.orientation.x
        rot_y = pose.orientation.y
        rot_z = pose.orientation.z
        rot_w = pose.orientation.w

        return [x, y, z], [rot_x, rot_y, rot_z, rot_w]

    ############################################### Desired pose/state functions ############################################
    def publish_eef_target(self, pos, quat):
        # Receives position and quaternion and converts to PoseStamped message (the one used for publishing the target pose)
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'panda_link0'

        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        msg.pose.orientation.x = float(quat[0])
        msg.pose.orientation.y = float(quat[1])
        msg.pose.orientation.z = float(quat[2])
        msg.pose.orientation.w = float(quat[3])

        # then apply the action!
        self.pub.publish(msg)

    def publish_gripper_target(self, gripper_action):
        # publish desired gripper state
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = self.get_clock().now().to_msg()
        msg_gripper.point.x = gripper_action

        self.pub_gripper.publish(msg_gripper)

    ############################################# ROS controllers functions #############################################

    def list_controllers(self, timeout_sec=2.0):
        if not self._list_controllers_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('ListControllers service not available')
        req = ListControllers.Request()
        fut = self._list_controllers_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    def load_controller(self, controller_name, timeout_sec=2.0):
        if not self._load_controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('LoadController service not available')
        req = LoadController.Request()
        req.name = controller_name
        fut = self._load_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    def unload_controller(self, controller_name, timeout_sec=2.0):
        if not self._unload_controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('UnloadController service not available')
        req = UnloadController.Request()
        req.name = controller_name
        fut = self._unload_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    def switch_controller(self, start_controllers, stop_controllers, timeout_sec=5.0):
        if not self._switch_controller_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('SwitchController service not available')
        req = SwitchController.Request()
        req.start_controllers = start_controllers
        req.stop_controllers = stop_controllers
        # strictness: 2 -> STRICT, 1 -> BEST_EFFORT (kept as 2 to mimic ROS1 strict behavior)
        try:
            req.strictness = 2
        except Exception:
            # some versions may have different naming; ignore if not present
            pass
        # start_asap and timeout exist in newer service definitions; set defaults similar to the ROS1 usage
        try:
            req.start_asap = False
            req.timeout = 0.0
        except Exception:
            pass

        fut = self._switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    ############################################ Robot motion scripts ###############################################
    def wait_for_message(self, topic, msg_type, timeout_sec=5.0):
        """Utility that waits for a single message on a topic (emulates rospy.wait_for_message)
        by creating a temporary subscription and spinning the node until the message arrives.
        """
        container = {'msg': None}
        event = threading.Event()

        def _cb(msg):
            container['msg'] = msg
            event.set()

        sub = self.create_subscription(msg_type, topic, _cb, 10)

        start = time.time()
        while rclpy.ok() and (time.time() - start) < timeout_sec:
            if event.wait(timeout=0.1):
                break
            # spin once to allow callbacks
            rclpy.spin_once(self, timeout_sec=0.1)

        # cleanup
        try:
            self.destroy_subscription(sub)
        except Exception:
            pass

        if container['msg'] is None:
            raise RuntimeError(f"Timeout waiting for message on {topic}")
        return container['msg']

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
        for c in list_controller_res.controller:
            if (c.name == "cartesian_pose_impedance_controller"):
                self.cartesian_pose_impedance_controller_loaded = True
                if (c.state == "running"):
                    self.cartesian_pose_impedance_controller_running = True

            if (c.name == "effort_joint_trajectory_controller"):
                self.effort_joint_trajectory_controller_loaded = True
                if (c.state == "running"):
                    self.effort_joint_trajectory_controller_running = True

        action = 'effort_joint_trajectory_controller/follow_joint_trajectory'
        # wait for the action server to be available
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"Action server '{action}' not available")

        topic = 'franka_state_controller/joint_states'
        self.get_logger().info(f"move_to_start: Waiting for message on topic '{topic}'")
        joint_state = self.wait_for_message(topic, JointState, timeout_sec=5.0)
        initial_pose = dict(zip(joint_state.name, joint_state.position))

        # Open gripper first
        action_result = self.gripper.move(0.04, 0.04)
        if action_result == True:
            self.get_logger().info("Gripper opened successfully")
            self.gripper_open = True
        else:
            self.get_logger().warn("Gripper failed to open")

        if not (self.cartesian_pose_impedance_controller_loaded and self.effort_joint_trajectory_controller_running):
            time.sleep(0.5)
            # if we do not start up for the first time - we first need to switch back!
            self.switch_controller(["effort_joint_trajectory_controller"], ["cartesian_pose_impedance_controller"])

        if (desired_joint_config is None):
            desired_joint_config = np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                             0.0032928755527341326, 1.994446315732633, 0.7839058620188021])

        # attempt_to_go_to_joints is kept external (same as in ROS1). It should be adapted for ROS2 if it
        # used rospy internals before.
        attempt_to_go_to_joints(self.trajectory_client, topic, desired_joint_config, duration=5)

        # After moving to the desired pose, switch the controller to the pose impedance controller
        if not (self.cartesian_pose_impedance_controller_loaded):
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
        self.get_logger().info("Robot moved to home position")

    ############################################ Teloperation loop ###############################################
    def run(self):
        """
        Runs the teleoperation. Some notes:
            - The teleoperation can be started/stopped by pressing the home button in the vive controller
            - The gripper can closed/opened by pressing the trigger in the vive controller
            - The robot can be moved to home position if the pad button is pressed
            - For updating the translation of the end-effector, a simple scale factor is used sucha as:
                A' = A + alpha(B'-B)
            - For updating the rotation, a matrix multiplication for quaternions is done:
                A' = (B*Binv)*A
        """
        rate_hz = 50.0
        sleep_dt = 1.0 / rate_hz

        def pose_to_homogeneous(position, quaternion):
            """Create a 4x4 homogeneous transform from position and quaternion."""
            H = np.eye(4)
            H[:3, :3] = tf_transformations.quaternion_matrix(quaternion)[:-1, :-1]
            H[:3, 3] = position
            return H

        def homogeneous_inverse(H):
            """Compute inverse of a rigid-body homogeneous transform H efficiently."""
            R = H[:3, :3]
            p = H[:3, 3]
            R_inv = np.linalg.inv(R)
            H_inv = np.eye(4)
            H_inv[:3, :3] = R_inv
            H_inv[:3, 3] = -R_inv.dot(p)
            return H_inv

        # --- initialize baselines ---
        last_gripper_state = self.gripper_open

        last_vive_pos, last_vive_quat = self.get_pose_info(self.vive_last_pose)
        last_vive_H = pose_to_homogeneous(last_vive_pos, last_vive_quat)
        last_vive_H_inv = homogeneous_inverse(last_vive_H)

        last_EE_pos, last_EE_quat = self.get_pose_info(self.ee_pose)
        last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)

        # main loop
        while rclpy.ok():
            # If teleop is disabled: keep baselines up-to-date and sleep
            if not self.teleop_on:
                last_vive_pos, last_vive_quat = self.get_pose_info(self.vive_last_pose)
                last_vive_H = pose_to_homogeneous(last_vive_pos, last_vive_quat)
                last_vive_H_inv = homogeneous_inverse(last_vive_H)

                last_EE_pos, last_EE_quat = self.get_pose_info(self.ee_pose)
                last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)

                last_gripper_state = self.gripper_open
                time.sleep(sleep_dt)
                continue

            # Teleop enabled: read current sensor/controller states
            try:
                trans_EE, rot_EE = self.get_NE_pose()
                self.get_logger().info(f"EE Position: {trans_EE}, EE Rotation: {rot_EE}")
            except Exception:
                trans_EE, rot_EE = (None, None)

            vive_pos, vive_quat = self.get_pose_info(self.vive_last_pose)
            EE_pos, EE_quat = self.get_pose_info(self.ee_pose)

            # --- Translation: simple scaled delta from baseline Vive pose ---
            delta_vive = np.array(vive_pos) - np.array(last_vive_pos)
            target_pos = np.array(EE_pos) + self.SCALE_FACTOR * delta_vive

            # --- Rotation: apply relative Vive rotation to current EE orientation ---
            # target_H = vive_H * last_vive_H_inv * EE_H
            EE_H = pose_to_homogeneous(EE_pos, EE_quat)
            vive_H = pose_to_homogeneous(vive_pos, vive_quat)

            target_H = vive_H.dot(last_vive_H_inv).dot(EE_H)
            target_quat = tf_transformations.quaternion_from_matrix(target_H)

            # Publish desired pose
            self.publish_eef_target(target_pos, np.asarray(target_quat))

            # --- Gripper: respond to changes in gripper_open state ---
            if self.gripper_open != last_gripper_state:
                if last_gripper_state is False:
                    # previously closed -> open
                    action_result = self.gripper.move(0.4, 0.4)
                    if action_result:
                        self.get_logger().info("Gripper was successfully opened")
                    else:
                        self.get_logger().warn("Gripper failed to open")
                else:
                    # previously open -> close/grasp
                    result_grasp = self.gripper.grasp()
                    if result_grasp:
                        self.get_logger().info("Gripper was successfully closed")
                    else:
                        self.get_logger().warn("Gripper failed to close")

            # --- update baselines for next iteration ---
            last_gripper_state = self.gripper_open

            last_vive_pos = vive_pos
            last_vive_quat = vive_quat
            last_vive_H = pose_to_homogeneous(last_vive_pos, last_vive_quat)
            last_vive_H_inv = homogeneous_inverse(last_vive_H)

            last_EE_pos = EE_pos
            last_EE_quat = EE_quat
            last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)

            time.sleep(sleep_dt)
    




def main(args=None):
    rclpy.init(args=args)
    node = Teleop()

    # Start the teleoperation loop in a background thread so that rclpy.spin can
    # process ROS callbacks in the main thread.
    import threading
    run_thread = threading.Thread(target=node.run, daemon=True)
    run_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    


