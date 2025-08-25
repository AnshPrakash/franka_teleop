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
from rosbags.image import message_to_cvimage


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


def pose_to_numpy(msg):
    """
    Convert geometry_msgs/Pose to a NumPy array.
    
    Message structure (Pose.msg):contentReference[oaicite:3]{index=3}:contentReference[oaicite:4]{index=4}:
      - position: geometry_msgs/Point (float64 x, y, z):contentReference[oaicite:5]{index=5}  
      - orientation: geometry_msgs/Quaternion (float64 x, y, z, w):contentReference[oaicite:6]{index=6}  

    Returns:
      np.ndarray of shape (7,) as [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w].
    """
    return np.array([
        msg.position.x, msg.position.y, msg.position.z,
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
    ], dtype=np.float64)


def pose_stamped_to_numpy(msg):
    """
    Convert geometry_msgs/PoseStamped to a NumPy array.
    
    Message structure (PoseStamped.msg):contentReference[oaicite:8]{index=8}:
      - header: std_msgs/Header (ignored in conversion)  
      - pose: geometry_msgs/Pose (position and orientation as above)  

    Returns:
      np.ndarray of shape (7,) as [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w].
    """
    # Reuse pose_to_numpy on the embedded pose field
    return pose_to_numpy(msg.pose)

def joint_state_to_numpy(msg):
    """
    Convert sensor_msgs/JointState to a NumPy array.
    
    Message structure (JointState.msg):contentReference[oaicite:10]{index=10}:
      - header: std_msgs/Header (ignored here)  
      - name: string[] (joint names, ignored in numeric output)  
      - position: float64[] (joint positions)  
      - velocity: float64[] (joint velocities, same length or empty)  
      - effort: float64[] (joint efforts, same length or empty)  

    Returns:
      np.ndarray of shape (3*N,) where N = number of joints. For joint i,
      the output contains [positions.., velocities.., efforts..] in sequence.
    """
    positions = np.array(msg.position)
    velocities = np.array(msg.velocity) if msg.velocity is not None else []
    efforts = np.array(msg.effort) if msg.effort is not None else []
    
    arr = np.concatenate([positions, velocities, efforts])
    
    return arr

# def joy_to_numpy(msg):
#     """
#     Convert sensor_msgs/Joy to a NumPy array.
    
#     Message structure (Joy.msg):contentReference[oaicite:12]{index=12}:
#       - header: std_msgs/Header (ignored)  
#       - axes: float32[] (axis values)  
#       - buttons: int32[] (button values)  

#     Returns:
#       np.ndarray containing [axes..., buttons...] as floats.
#     """
#     # Convert axes and buttons to numpy arrays
#     axes = np.array(msg.axes, dtype=np.float32) if msg.axes else np.array([], dtype=np.float32)
#     buttons = np.array(msg.buttons, dtype=np.float32) if msg.buttons else np.array([], dtype=np.float32)
#     return np.concatenate([axes, buttons])

def image_to_numpy(msg):
    """
    Convert sensor_msgs/Image to a NumPy array.
    
    Message structure (Image.msg):contentReference[oaicite:15]{index=15}:contentReference[oaicite:16]{index=16}:
      - header: std_msgs/Header (ignored)  
      - height: uint32 (rows)  
      - width: uint32 (columns)  
      - encoding: string (e.g., 'rgb8', 'bgr8', 'mono8')  
      - is_bigendian: uint8 (byte order, ignored)  
      - step: uint32 (row length in bytes, can verify data size)  
      - data: uint8[] (pixel buffer, size = step*height)  
        Uses rosbags-image.message_to_cvimage to turn a deserialized sensor_msgs/Image
        into a numpy (OpenCV) array. Handles bgra8, bgr8, rgb8, mono8, rgb16, etc.
    
    Returns:
      np.ndarray with shape (height, width, C) or (height, width), dtype determined by encoding.
    """
    img = message_to_cvimage(msg)   # returns numpy ndarray (H,W[,C])
    return img

# def grasp_action_goal_to_numpy(msg):
#     """
#     Convert franka_gripper/GraspActionGoal to a NumPy array.
    
#     Action goal definition (Grasp.action):contentReference[oaicite:19]{index=19}:
#       - width: float64 (target gripper width in meters)  
#       - epsilon: GraspEpsilon (tolerance window)  
#            - inner: float64:contentReference[oaicite:20]{index=20}  
#            - outer: float64:contentReference[oaicite:21]{index=21}  
#       - speed: float64 (m/s)  
#       - force: float64 (N)  

#     Returns:
#       np.ndarray [width, epsilon.inner, epsilon.outer, speed, force].
#     """
#     goal = msg.goal  # Goal fields inside the ActionGoal
#     inner = goal.epsilon.inner
#     outer = goal.epsilon.outer
#     return np.array([goal.width, inner, outer, goal.speed, goal.force], dtype=np.float64)

def msg_to_numpy(msg):
    """
    Convert a ROS message to a NumPy array based on its type.

    Determines msg._type and calls the corresponding conversion function.
    """
    # Mapping of message type to conversion function
    converters = {
        'geometry_msgs/msg/Pose': pose_to_numpy,
        'geometry_msgs/msg/PoseStamped': pose_stamped_to_numpy,
        'sensor_msgs/msg/JointState': joint_state_to_numpy,
        'sensor_msgs/msg/Image': image_to_numpy,
        # 'sensor_msgs/msg/Joy': joy_to_numpy,
        # 'franka_gripper/msg/GraspActionGoal': grasp_action_goal_to_numpy, # correct this later
    }
    # from ipdb import set_trace as bp; bp()
    msg_type = getattr(msg, '__msgtype__', None)
    if msg_type in converters:
        return converters[msg_type](msg)
    else:
        raise ValueError(f"No converter implemented for message type '{msg_type}'")