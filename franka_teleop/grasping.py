#!/usr/bin/env python3
"""
ROS2 (Humble) Gripper helper using rclpy action clients.

Usage:
    import rclpy
    from rclpy.node import Node
    from your_pkg.gripper_ros2 import Gripper

    rclpy.init()
    node = Node('test_node')
    gripper = Gripper(node)                # or Gripper(node, gripper_ns='panda_gripper')
    ok = gripper.move(0.02, 0.02)          # open/close (width = finger1 + finger2)
    ok = gripper.grasp()                   # perform grasp (blocking)
    rclpy.shutdown()
"""
from __future__ import annotations
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ROS action/message types provided by franka ROS 2 packages
from franka_msgs.action import Grasp as GraspAction
from franka_msgs.action import Move as MoveAction
from franka_msgs.msg import GraspEpsilon

import time


class Gripper:
    def __init__(self, node: Node, gripper_ns: str = '/franka_gripper', wait_timeout: float = 5.0):
        """
        node: rclpy.node.Node that owns the communications
        gripper_ns: namespace where gripper action servers live (default '/franka_gripper')
        wait_timeout: seconds to wait for action servers on init
        """
        self.node = node
        self.gripper_ns = gripper_ns.rstrip('/')
        self._move_action_name = f'{self.gripper_ns}/move'
        self._grasp_action_name = f'{self.gripper_ns}/grasp'

        # Action clients
        self._move_client = ActionClient(self.node, MoveAction, self._move_action_name)
        self._grasp_client = ActionClient(self.node, GraspAction, self._grasp_action_name)

        # Wait for servers (best-effort)
        ok_move = self._move_client.wait_for_server(timeout_sec=wait_timeout)
        ok_grasp = self._grasp_client.wait_for_server(timeout_sec=wait_timeout)
        if not ok_move:
            self.node.get_logger().warn(f'Move action server not available at {self._move_action_name}')
        if not ok_grasp:
            self.node.get_logger().warn(f'Grasp action server not available at {self._grasp_action_name}')

    def _send_goal_and_wait(self, client: ActionClient, goal_msg, timeout: float = 5.0) -> Optional[object]:
        """
        Send a goal (goal_msg should be an instance of client.action.Goal) and wait for a result.
        Returns the result object on success, or None on failure/timeout.
        """
        # send_goal_async returns a future for a GoalHandle
        send_goal_future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=timeout)
        if not send_goal_future.done():
            self.node.get_logger().warn('Sending goal timed out')
            return None

        goal_handle = send_goal_future.result()
        if goal_handle is None:
            self.node.get_logger().warn('Goal handle is None (goal not accepted?)')
            return None

        if not goal_handle.accepted:
            self.node.get_logger().warn('Goal rejected by server')
            return None

        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, get_result_future, timeout_sec=timeout)
        if not get_result_future.done():
            self.node.get_logger().warn('Waiting for result timed out')
            return None

        result_response = get_result_future.result()
        # result_response is the GetResult.Response; actual result payload is result_response.result
        return result_response.result

    def move(self, finger1_y: float, finger2_y: float, speed: float = 0.1, timeout: float = 5.0) -> bool:
        """
        Move the gripper fingers to a width.

        finger1_y, finger2_y: inputs - legacy code used sum of two numbers for width
        speed: motion speed [m/s]
        timeout: seconds to wait for completion
        returns: bool success (from action result)
        """
        width = float(finger1_y + finger2_y)
        goal = MoveAction.Goal()  # type: ignore[attr-defined]
        # In ROS2 the Action goal type is e.g. MoveAction.Goal (this works with generated python classes)
        goal.width = width
        goal.speed = float(speed)

        self.node.get_logger().info(f'Gripper MOVE goal: width={goal.width:.4f}, speed={goal.speed:.4f}')
        result = self._send_goal_and_wait(self._move_client, goal, timeout=timeout)
        if result is None:
            return False
        # result has fields (success, error) according to action definition
        return bool(getattr(result, 'success', False))

    def grasp(self,
              width: float = 0.045,
              epsilon_inner: float = 0.01,
              epsilon_outer: float = 0.01,
              speed: float = 0.1,
              force: float = 5.0,
              timeout: float = 10.0) -> bool:
        """
        Execute a grasp action.

        width: target width [m]
        epsilon_inner/epsilon_outer: grasp epsilon tolerances
        speed: m/s
        force: N
        timeout: seconds to wait for action result
        returns: bool success
        """
        goal = GraspAction.Goal()
        goal.width = float(width)
        eps = GraspEpsilon()
        eps.inner = float(epsilon_inner)
        eps.outer = float(epsilon_outer)
        goal.epsilon = eps
        goal.speed = float(speed)
        goal.force = float(force)

        self.node.get_logger().info(
            'Executing grasp: width=%.4f, epsilon=(%.4f,%.4f), speed=%.3f, force=%.3f' %
            (goal.width, epsilon_inner, epsilon_outer, goal.speed, goal.force)
        )

        result = self._send_goal_and_wait(self._grasp_client, goal, timeout=timeout)
        if result is None:
            return False
        return bool(getattr(result, 'success', False))
