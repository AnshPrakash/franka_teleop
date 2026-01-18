#!/usr/bin/env python3
"""
ROS 1 -> ROS 2 (Humble) conversion of oculus_tranforms_vrpolicy.py

This node wraps VRPolicyFranka and republishes the policy outputs as
`geometry_msgs/Pose` and `sensor_msgs/Joy` topics.

Notes:
- The original script created publishers on each publish call; in ROS 2 we
  create them once in the node constructor and use a timer to poll the policy.
- The timer period is set to 10 ms by default (100 Hz). Adjust if you need
  higher frequency.
"""

from typing import Dict, Any
import rclpy
from rclpy.node import Node

import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

from franka_teleop.oculus_controller import VRPolicyFranka
from droid.misc.transformations import euler_to_quat


class OculusReaderVRPolicy:
    """
    Wrapper class for the Oculus Reader Policy, formatting Oculus Reader actions
    """

    def __init__(
        self,
        right_controller: bool = True,
        max_lin_vel: float = 1,
        max_rot_vel: float = 1,
        max_gripper_vel: float = 1,
        spatial_coeff: float = 1,
        pos_action_gain: float = 5,
        rot_action_gain: float = 2,
        gripper_action_gain: float = 3,
        rmat_reorder: list | None = None,
    ):
        if rmat_reorder is None:
            rmat_reorder = [-2, -1, -3, 4]

        self._vr_policy = VRPolicyFranka(
            right_controller=right_controller,
            max_lin_vel=max_lin_vel,
            max_rot_vel=max_rot_vel,
            max_gripper_vel=max_gripper_vel,
            spatial_coeff=spatial_coeff,
            pos_action_gain=pos_action_gain,
            rot_action_gain=rot_action_gain,
            gripper_action_gain=gripper_action_gain,
            rmat_reorder=rmat_reorder,
        )

        state_dict = {"cartesian_position": np.zeros(6), "gripper_position": 0.0}
        self._obs_dict = {"robot_state": state_dict}

    def get_update(self):
        """
        returns (active: bool, linear_twist, angular_twist, info)
        where `info` contains keys such as `target_cartesian_position` and
        `target_gripper_position`.
        """
        action, info = self._vr_policy.forward(self._obs_dict, include_info=True)

        active = np.any(action != np.zeros(7))
        if active:
            self._obs_dict["robot_state"]["cartesian_position"] = info["target_cartesian_position"]
            self._obs_dict["robot_state"]["gripper_position"] = info["target_gripper_position"]
            return True, action[:3], action[3:6], info
        else:
            return False, None, None, None


class OculusTransformsNode(Node):
    """ROS 2 node that republishes VR policy outputs as ROS topics."""

    def __init__(self, timer_period: float = 0.01):
        super().__init__("oculus_reader")

        # Create the VR policy wrapper
        self._oculus_reader = OculusReaderVRPolicy()

        # Publishers
        qos = 10
        self._pose_pub = self.create_publisher(Pose, "/oculus/my_right_controller_1_Pose", qos)
        self._joy_pub = self.create_publisher(Joy, "/oculus/my_right_controller_1/joy", qos)

        # Buttons we expect in the info dict
        self._control_buttons = ["A", "B", "RTr", "RG", "rightJS"]

        # Timer to poll the VR policy
        self._timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("OculusTransformsNode started (publishing pose and joy)")

    def timer_callback(self):
        # Poll the policy
        _, _, _, info = self._oculus_reader.get_update()
        if info is None:
            return

        # Extract pose and button info
        try:
            right_controller_buttons = {button: info[button] for button in self._control_buttons}
            right_controller_pose = info["target_cartesian_position"]
        except Exception as e:
            self.get_logger().warn(f"Malformed info from VR policy: {e}")
            return

        # Debug logging
        self.get_logger().debug(f"Buttons: {right_controller_buttons}")
        self.get_logger().debug(f"Pose: {right_controller_pose}")

        self.publish_transform(right_controller_pose)
        self.publish_buttons(right_controller_buttons)

    def publish_transform(self, transform: Any):
        """Publish geometry_msgs/Pose built from the policy transform."""
        pose_msg = Pose()

        translation = transform[:3]
        pose_msg.position.x = float(translation[0])
        pose_msg.position.y = float(translation[1])
        pose_msg.position.z = float(translation[2])

        quat = euler_to_quat(transform[3:])
        pose_msg.orientation.x = float(quat[0])
        pose_msg.orientation.y = float(quat[1])
        pose_msg.orientation.z = float(quat[2])
        pose_msg.orientation.w = float(quat[3])

        self._pose_pub.publish(pose_msg)

    def publish_buttons(self, buttons: Dict[str, Any]):
        joy_msg = Joy()
        joy_msg.header.stamp = self.get_clock().now().to_msg()
        joy_msg.header.frame_id = "world"

        # Axes - joystick info
        right_js = buttons.get("rightJS", [])
        joy_msg.axes = list(right_js)

        # Buttons (ensure ordering matches Oculus_button mapping)
        joy_msg.buttons = [
            int(buttons.get("A", 0)),
            int(buttons.get("B", 0)),
            int(buttons.get("RTr", 0)),
            int(buttons.get("RG", 0)),
        ]

        self._joy_pub.publish(joy_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OculusTransformsNode(timer_period=0.01)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down OculusTransformsNode")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
