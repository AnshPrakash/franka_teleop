#!/usr/bin/env python3
"""
franka_teleop node for teleoperating a Franka Emika Panda robot using a Meta Quest Controller.
Logic is to map the controller's pose changes to the robot's end-effector pose changes, with scaling.
The teleoperation can be toggled on/off via the home button, and the gripper can be opened/closed via the trigger button.
Check "https://github.com/AnshPrakash/franka_teleop/tree/main?tab=readme-ov-file#3-finally-how-to-teleoperate" for more details.
"""

from typing import Optional
import copy
import time
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from sensor_msgs.msg import JointState, Joy
from franka_msgs.msg import FrankaState

from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
from control_msgs.action import FollowJointTrajectory

import tf_transformations
import tf2_ros

# Config
import hydra
from omegaconf import DictConfig

# Functions from scripts
from util import go_to, attempt_to_go_to_joints
from grasping import Gripper
from RosbagControlledRecorder import RosbagControlledRecorder

import enum

# Defaults (can be overridden via cfg)
DEFAULT_QUEST_CONTROLLER = 1
DEFAULT_SCALE_FACTOR = 7
DEFAULT_SCALE_FACTOR_ROTATION = 7

class Oculus_button(enum.Enum):
    A = 0
    B = 1
    RTr = 2
    RG = 3

class Teleop(Node):
    def __init__(self, cfg: DictConfig):
        super().__init__('franka_teleop')

        # config from hydra
        self.cfg = cfg

        # allow overriding the controller choice via cfg, otherwise use default
        self.QUEST_CONTROLLER = getattr(cfg, 'quest_controller', DEFAULT_QUEST_CONTROLLER)
        self.SCALE_FACTOR = getattr(cfg, 'scale_factor', DEFAULT_SCALE_FACTOR)
        self.SCALE_FACTOR_ROTATION = getattr(cfg, 'scale_factor_rotation', DEFAULT_SCALE_FACTOR_ROTATION)

        # robot/tools
        self.gripper = Gripper()
        self.ee_pose = Pose()
        self.gripper_open = False

        # quest controller state
        self.quest_last_pose = Pose()
        self.quest_button_state = Joy()
        self.quest_side_button = 0
        self.quest_home_button = 0
        self.quest_trigger_button = 0
        self.quest_pad_button = 0

        self.teleop_on = False

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # publishers
        qos_profile = 10
        self.pub = self.create_publisher(PoseStamped, '/cartesian_impedance_controller/desired_pose', qos_profile)
        self.pub_gripper = self.create_publisher(PointStamped, '/cartesian_impedance_controller/desired_gripper_state', qos_profile)

        # subscribers
        if self.QUEST_CONTROLLER == 0:
            self.quest_left_subscriber = self.create_subscription(Pose, '/oculus/my_left_controller_1_Pose', self.quest_pose_cb, qos_profile)
            self.button_left_subscriber = self.create_subscription(Joy, '/oculus/my_left_controller_1/joy', self.button_cb, qos_profile)
        else:
            self.quest_right_subscriber = self.create_subscription(Pose, '/oculus/my_right_controller_1_Pose', self.quest_pose_cb, qos_profile)
            self.button_right_subscriber = self.create_subscription(Joy, '/oculus/my_right_controller_1/joy', self.button_cb, qos_profile)

        self.subscriber_ee_pose = self.create_subscription(FrankaState, '/franka_state_controller/franka_states', self.__process_ee_pose, 1)

        # controller manager service clients
        self._list_controllers_client = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self._load_controller_client = self.create_client(LoadController, '/controller_manager/load_controller')
        self._unload_controller_client = self.create_client(UnloadController, '/controller_manager/unload_controller')
        self._switch_controller_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        # action client for joint trajectory
        action_name = 'effort_joint_trajectory_controller/follow_joint_trajectory'
        self.trajectory_client = ActionClient(self, FollowJointTrajectory, action_name)

        # Recorder initialization using cfg
        topics_only = [list(item.keys())[0] for item in cfg.topics]
        self.recorder = RosbagControlledRecorder(
            save_folder=cfg.save_folder,
            topics=topics_only
        )
        self.video_recorder: Optional[RosbagControlledRecorder] = None
        if getattr(cfg, 'video_topics', None) is not None:
            video_topics_only = [list(item.keys())[0] for item in cfg.video_topics]
            self.video_recorder = RosbagControlledRecorder(
                save_folder=cfg.save_folder,
                topics=video_topics_only,
                is_video=True,
                complementary_recorder=self.recorder
            )

        self.get_logger().info('Teleop (recorder) node initialized')

    ################################# callbacks #################################
    def quest_pose_cb(self, msg: Pose):
        self.quest_last_pose = msg

    def button_cb(self, button_data: Joy):
        # Collects the state for each of the buttons from the VR controller
        self.quest_button_state = button_data

        # Toggle teleop with A
        if self.quest_home_button == 0 and len(button_data.buttons) > Oculus_button.A.value and button_data.buttons[Oculus_button.A.value] == 1:
            self.teleop_on = not self.teleop_on
            self.get_logger().info(f'Teleop toggled: {self.teleop_on}')

        # Gripper toggle with B
        if self.quest_trigger_button == 0 and len(button_data.buttons) > Oculus_button.B.value and button_data.buttons[Oculus_button.B.value] == 1:
            self.gripper_open = not self.gripper_open
            self.get_logger().info(f'Gripper open: {self.gripper_open}')

        # Move home (only when teleop off)
        if self.quest_pad_button == 0 and len(button_data.buttons) > Oculus_button.RTr.value and button_data.buttons[Oculus_button.RTr.value] == 1 and not self.teleop_on:
            self.move_home()

        # store button states (safely)
        if len(button_data.buttons) > 0:
            self.quest_home_button = button_data.buttons[0]
        if len(button_data.buttons) > 1:
            self.quest_trigger_button = button_data.buttons[1]
        if len(button_data.buttons) > 2:
            self.quest_pad_button = button_data.buttons[2]
        if len(button_data.buttons) > 3:
            self.quest_side_button = button_data.buttons[3]

    def __process_ee_pose(self, msg: FrankaState):
        try:
            mat = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))
            q = tf_transformations.quaternion_from_matrix(mat)
            q = q / np.linalg.norm(q)
            self.ee_pose.orientation.x = float(q[0])
            self.ee_pose.orientation.y = float(q[1])
            self.ee_pose.orientation.z = float(q[2])
            self.ee_pose.orientation.w = float(q[3])
            self.ee_pose.position.x = float(msg.O_T_EE[12])
            self.ee_pose.position.y = float(msg.O_T_EE[13])
            self.ee_pose.position.z = float(msg.O_T_EE[14])
        except Exception as e:
            self.get_logger().warn(f'Failed to process FrankaState: {e}')

    ################################# utilities #################################
    def get_NE_pose(self):
        # lookup transform panda_link0 <- panda_NE
        while rclpy.ok():
            try:
                trans_stamped = self.tf_buffer.lookup_transform('panda_link0', 'panda_NE', rclpy.time.Time())
                t = trans_stamped.transform.translation
                r = trans_stamped.transform.rotation
                return (t.x, t.y, t.z), (r.x, r.y, r.z, r.w)
            except Exception:
                time.sleep(0.05)
                continue

    def get_pose_info(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        rx = pose.orientation.x
        ry = pose.orientation.y
        rz = pose.orientation.z
        rw = pose.orientation.w
        return [x, y, z], [rx, ry, rz, rw]

    def _skew_symmetric_matrix(self, v: np.array):
        return np.array([[0, -v[2], v[1]],[v[2], 0, -v[0]],[-v[1], v[0], 0]])

    def rotation_matrix_from_axis_angle(self, axis, angle):
        axis = axis / np.linalg.norm(axis)
        cos_t = np.cos(angle)
        sin_t = np.sin(angle)
        one_minus = 1 - cos_t
        outer = np.outer(axis, axis)
        R = cos_t * np.eye(3) + one_minus * outer + sin_t * self._skew_symmetric_matrix(axis)
        return R

    def scale_rotation(self, R):
        euler = tf_transformations.euler_from_matrix(R, 'rxyz')
        euler = [self.SCALE_FACTOR_ROTATION * angle for angle in euler]
        scaled = tf_transformations.euler_matrix(-euler[0], -euler[1], euler[2], 'rxyz')
        return scaled

    ################################# publish helpers #################################
    def publish_eef_target(self, pos, quat):
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
        self.pub.publish(msg)

    def publish_gripper_target(self, gripper_action):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x = float(gripper_action)
        self.pub_gripper.publish(msg)

    ################################# controller manager helpers #################################
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
        try:
            req.strictness = 2
            req.start_asap = False
            req.timeout = 0.0
        except Exception:
            pass
        fut = self._switch_controller_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout_sec)
        return fut.result()

    ################################# motion scripts #################################
    def wait_for_message(self, topic, msg_type, timeout_sec=5.0):
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
            rclpy.spin_once(self, timeout_sec=0.1)
        try:
            self.destroy_subscription(sub)
        except Exception:
            pass
        if container['msg'] is None:
            raise RuntimeError(f"Timeout waiting for message on {topic}")
        return container['msg']

    def startup_procedure(self, desired_joint_config=None, initial_config_pose=None):
        self.cartesian_pose_impedance_controller_loaded = False
        self.cartesian_pose_impedance_controller_running = False
        self.effort_joint_trajectory_controller_loaded = False
        self.effort_joint_trajectory_controller_running = False

        list_controller_res = self.list_controllers()
        for c in list_controller_res.controller:
            if c.name == 'cartesian_pose_impedance_controller':
                self.cartesian_pose_impedance_controller_loaded = True
                if c.state == 'running':
                    self.cartesian_pose_impedance_controller_running = True
            if c.name == 'effort_joint_trajectory_controller':
                self.effort_joint_trajectory_controller_loaded = True
                if c.state == 'running':
                    self.effort_joint_trajectory_controller_running = True

        action = 'effort_joint_trajectory_controller/follow_joint_trajectory'
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(f"Action server '{action}' not available")

        topic = 'franka_state_controller/joint_states'
        self.get_logger().info(f"move_to_start: Waiting for message on topic '{topic}'")
        joint_state = self.wait_for_message(topic, JointState, timeout_sec=5.0)
        initial_pose = dict(zip(joint_state.name, joint_state.position))

        # Open gripper first
        action_result = self.gripper.move(0.04, 0.04)
        if action_result:
            self.get_logger().info('Gripper opened successfully')
            self.gripper_open = True
        else:
            self.get_logger().warn('Gripper failed to open')

        if not (self.cartesian_pose_impedance_controller_loaded and self.effort_joint_trajectory_controller_running):
            time.sleep(0.5)
            self.switch_controller(['effort_joint_trajectory_controller'], ['cartesian_pose_impedance_controller'])

        if desired_joint_config is None:
            desired_joint_config = np.array([0.004286136549292948, 0.23023615878924988, -0.003981800034836296, -1.7545947008261213,
                                             0.0032928755527341326, 1.994446315732633, 0.7839058620188021])

        attempt_to_go_to_joints(self.trajectory_client, topic, desired_joint_config, duration=5)

        if not self.cartesian_pose_impedance_controller_loaded:
            self.load_controller('cartesian_pose_impedance_controller')
            time.sleep(0.5)

        self.switch_controller(['cartesian_pose_impedance_controller'], ['effort_joint_trajectory_controller'])

    def move_home(self):
        self.teleop_on = False
        self.switch_controller(['effort_joint_trajectory_controller'], ['cartesian_pose_impedance_controller'])
        time.sleep(0.5)
        self.startup_procedure()

    ################################# teleop loop (with recording) #################################
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
        rate_hz = 50.0
        dt = 1.0 / rate_hz

        def pose_to_homogeneous(position, quaternion):
            H = np.eye(4)
            H[:3, :3] = tf_transformations.quaternion_matrix(quaternion)[:-1, :-1]
            H[:3, 3] = position
            return H

        def homogeneous_inverse(H):
            R = H[:3, :3]
            p = H[:3, 3]
            R_inv = np.linalg.inv(R)
            H_inv = np.eye(4)
            H_inv[:3, :3] = R_inv
            H_inv[:3, 3] = -R_inv.dot(p)
            return H_inv

        # initialize baselines
        last_gripper_state = self.gripper_open
        last_q_pos, last_q_quat = self.get_pose_info(self.quest_last_pose)
        last_q_H = pose_to_homogeneous(last_q_pos, last_q_quat)
        last_q_H_inv = homogeneous_inverse(last_q_H)
        last_EE_pos, last_EE_quat = self.get_pose_info(self.ee_pose)
        last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)

        while rclpy.ok():
            if not self.teleop_on:
                # refresh baselines
                last_q_pos, last_q_quat = self.get_pose_info(self.quest_last_pose)
                last_q_H = pose_to_homogeneous(last_q_pos, last_q_quat)
                last_q_H_inv = homogeneous_inverse(last_q_H)
                last_EE_pos, last_EE_quat = self.get_pose_info(self.ee_pose)
                last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)
                last_gripper_state = self.gripper_open
                time.sleep(dt)
                # Stop recording if it was running and teleop just turned off
                if hasattr(self, 'recorder') and not getattr(self.recorder, 'recording_stopped', True):
                    try:
                        self.recorder.stop_recording()
                        if self.video_recorder is not None:
                            self.video_recorder.stop_recording()
                        self.get_logger().info('Recording stopped')
                    except Exception as e:
                        self.get_logger().warn(f'Failed to stop recorder: {e}')
                continue

            # When teleop is on -> ensure recorder running
            if hasattr(self, 'recorder') and not getattr(self.recorder, 'recording_started', False):
                try:
                    self.recorder.start_recording()
                    if self.video_recorder is not None:
                        self.video_recorder.start_recording()
                    self.get_logger().info('Recording started')
                except Exception as e:
                    self.get_logger().warn(f'Failed to start recorder: {e}')

            # read current states
            try:
                trans_EE, rot_EE = self.get_NE_pose()
            except Exception:
                trans_EE, rot_EE = (None, None)

            q_pos, q_quat = self.get_pose_info(self.quest_last_pose)
            EE_pos, EE_quat = self.get_pose_info(self.ee_pose)

            # translation (note sign change on y to match original code)
            delta_q = np.array(q_pos) - np.array(last_q_pos)
            target_pos = np.array(EE_pos) + np.array([ self.SCALE_FACTOR*delta_q[0],
                                                       -self.SCALE_FACTOR*delta_q[1],
                                                        self.SCALE_FACTOR*delta_q[2]])
            target_pos = target_pos.tolist()

            # rotation (scale/compose)
            EE_H = pose_to_homogeneous(EE_pos, EE_quat)
            q_H = pose_to_homogeneous(q_pos, q_quat)
            rotate_part = q_H.dot(last_q_H_inv)
            rotate_part = self.scale_rotation(rotate_part)
            target_H = rotate_part.dot(EE_H)
            target_quat = tf_transformations.quaternion_from_matrix(target_H)

            # publish
            self.publish_eef_target(target_pos, np.asarray(target_quat))

            # gripper
            if self.gripper_open != last_gripper_state:
                if last_gripper_state is False:
                    res = self.gripper.move(0.4, 0.4)
                    if res:
                        self.get_logger().info('Gripper opened')
                    else:
                        self.get_logger().warn('Gripper failed to open')
                else:
                    res = self.gripper.grasp()
                    if res:
                        self.get_logger().info('Gripper closed')
                    else:
                        self.get_logger().warn('Gripper failed to close')

            # update baselines
            last_gripper_state = self.gripper_open
            last_q_pos = q_pos
            last_q_quat = q_quat
            last_q_H = pose_to_homogeneous(last_q_pos, last_q_quat)
            last_q_H_inv = homogeneous_inverse(last_q_H)
            last_EE_pos = EE_pos
            last_EE_quat = EE_quat
            last_EE_H = pose_to_homogeneous(last_EE_pos, last_EE_quat)

            time.sleep(dt)


@hydra.main(version_base=None, config_path="/opt/ros_ws/src/franka_teleop/config", config_name="recorder.yaml")
def main(cfg: DictConfig):
    rclpy.init()
    node = Teleop(cfg)

    # Run startup and then run loop in background thread
    try:
        node.startup_procedure()
    except Exception as e:
        node.get_logger().warn(f'startup_procedure failed: {e}')

    run_thread = threading.Thread(target=node.run, daemon=True)
    run_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down')
        try:
            node.recorder.stop_recording()
        except Exception:
            pass
        if node.video_recorder is not None:
            try:
                node.video_recorder.stop_recording()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
