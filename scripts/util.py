import copy
import time
import rospy
import actionlib
import numpy as np
from threading import Lock
from typing import List, Union
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryResult
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from typing import Optional


def go_to(trajectory_client: actionlib.SimpleActionClient, joint_names: List[str],
          init_positions: Union[np.ndarray, List[float]], target_positions: Union[np.ndarray, List[float]],
          duration: float = None, max_vel: float = None, wait: bool = True, time_offset: float = 0.0):
    if duration is None:
        if max_vel is None:
            raise RuntimeError("If duration is not specified, the maximum allowed joint velocity must be specified")

        duration = max(0.5, np.linalg.norm(np.array(init_positions) - np.array(target_positions)) / max_vel)

    # Create a trajectory in which we simply move J0 by 20 degrees
    traj = JointTrajectory()
    traj.header.stamp = rospy.Time.now() + rospy.Duration.from_sec(time_offset)
    traj.joint_names = joint_names

    traj_point = JointTrajectoryPoint()
    traj_point.time_from_start = rospy.Duration(duration)
    traj_point.positions = target_positions
    traj_point.velocities = [0] * len(target_positions)
    traj.points = [traj_point]

    traj_goal = FollowJointTrajectoryGoal()
    traj_goal.trajectory = traj
    if wait:
        trajectory_client.send_goal_and_wait(traj_goal)
        return trajectory_client.get_result()
    else:
        trajectory_client.send_goal(traj_goal)


def attempt_to_go_to_joints(client, topic, desired_config, duration=5):
    '''
    This function implements the behavior to go to a certain desired joint configuration
    '''
    # get current joint state
    joint_state = rospy.wait_for_message(topic, JointState)
    # try at maximum 5 times
    for i in range(5):
        # then go to default pose
        result = go_to(client, joint_state.name[:7], joint_state.position[:7], desired_config, duration=duration)
        print(result)
        if (result.error_code == FollowJointTrajectoryResult.SUCCESSFUL):
            print("The robot was successfully moved and initialized")
            break
        else:
            # roserror message
            rospy.logerr("The robot was not able to move and initialized")
            time.sleep(1)

