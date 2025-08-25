import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from controller_manager_msgs.srv import SwitchController, LoadController, UnloadController, ListControllers
from actionlib import SimpleActionClient
from control_msgs.msg import FollowJointTrajectoryAction
from sensor_msgs.msg import JointState, Joy
from sensor_msgs.msg import Image

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

# Lock to protect concurrent access to latest messages
import threading


from util import go_to, attempt_to_go_to_joints, msg_to_numpy
from grasping import Gripper

class PolicyController:
    def __init__(self, target_frequency: float = 11):
        """
            Initialize the PolicyController
            Subscribe to the ROS topic for observations
            Initialize the policy network
        """
        
        # Initialize ROS node
        rospy.init_node("policy_controller")
        self.target_frequency = target_frequency
        self._last_obs_time = 0.0
        
        self.rate = rospy.Rate(50)     # Setting rate to 50Hz, we will downsample later to target_frequency
        
        self.topics = {
            # "/cartesian_impedance_controller/desired_pose": PoseStamped,
            "/franka_gripper/joint_states": JointState,
            "/franka_state_controller/O_T_EE": PoseStamped,
            # "/franka_state_controller/ee_pose": Pose,
            # "/franka_state_controller/franka_states": FrankaState,
            "/franka_state_controller/joint_states": JointState,
            # "/franka_state_controller/joint_states_desired": JointState,
            "/zedA/zed_node_A/left/image_rect_color": Image,
            "/zedB/zed_node_B/left/image_rect_color": Image,
        }
        
        self._msg_lock = threading.Lock()
        
        # Storage for the latest message and its timestamp (seconds) for each topic
        # latest_msgs stores deep copies of the last received message
        self.latest_msgs = {topic: None for topic in self.topics.keys()}
        # latest_times stores the timestamp (float seconds) of the last received message
        self.latest_times = {topic: 0.0 for topic in self.topics.keys()}

        
        # Keep subscriber objects so they don't get garbage-collected
        self.subscribers = {}
        
        # Create subscribers
        for topic_name, msg_type in self.topics.items():
            try:
                sub = rospy.Subscriber(topic_name, msg_type, self._make_callback(topic_name), queue_size=1)
                # store handle
                self.subscribers[topic_name] = sub
                rospy.loginfo_throttle(30.0, f"[PolicyController] Subscribed to {topic_name} as {msg_type.__name__}")
            except Exception as e:
                rospy.logwarn_throttle(30.0, f"[PolicyController] Failed to subscribe to {topic_name} ({msg_type}): {e}")

        rospy.loginfo("[PolicyController] Initialization complete. Topics subscribed: " + ", ".join(self.topics.keys()))
    
    
    # Callback factory to capture the topic name and update self.latest_* maps
    def _make_callback(self, topic_name):
        def _cb(msg):
            try:
                # Prefer header.stamp if present
                stamp = None
                if hasattr(msg, "header") and hasattr(msg.header, "stamp"):
                    try:
                        stamp = float(msg.header.stamp.to_sec())
                    except Exception:
                        stamp = None
                if stamp is None:
                    # fallback for messages without header (e.g. geometry_msgs/Pose)
                    stamp = float(rospy.get_time())

                # store a deepcopy to avoid accidental mutation by rospy internals
                with self._msg_lock:
                    self.latest_msgs[topic_name] = copy.deepcopy(msg)
                    self.latest_times[topic_name] = stamp
            except Exception as e:
                rospy.logwarn_throttle(10.0, f"[PolicyController] callback error for {topic_name}: {e}")
        return _cb


    def select_action(self, observation):
        pass


    def get_obeservation(self):
        """
        Get the observation from the ROS topics at the target frequency.

        Wait until ALL topics have a latest observation timestamp >=
        (self._last_obs_time + 1/self.target_frequency), then return a dict:
        {
            'timestamp': float,            # conservative timestamp (min of per-topic times)
            'data': {topic: np.array or None, ...},
            'times': {topic: float, ...}
        }

        No raw ROS messages are returned or stored here — only NumPy arrays via msg_to_numpy.
        """
        # Validate frequency and compute dt
        target_freq = self.target_frequency
        if target_freq is None:
            rospy.logerr("[PolicyController] Invalid target_frequency")
            
        target_dt = 1.0 / float(target_freq)
        tol = 1e-6

        # Normalize topic list
        if isinstance(self.topics, dict):
            topic_names = list(self.topics.keys())
        else:
            topic_names = list(self.topics)

        # Compute the threshold we require each topic to exceed
        wait_threshold = self._last_obs_time + target_dt - tol

        while not rospy.is_shutdown():
            triggered = False
            snapshot_msgs = {}
            snapshot_times = {}

            # Atomically check whether all topics have timestamps >= threshold and snapshot msgs
            with self._msg_lock:
                # gather current times (0.0 if never seen)
                current_times = {t: float(self.latest_times.get(t, 0.0)) for t in topic_names}

                # require every topic to have a non-zero timestamp >= wait_threshold
                all_ready = all((current_times[t] and (current_times[t] + tol) >= wait_threshold) for t in topic_names)

                if all_ready:
                    # conservative observation timestamp = min of the available times
                    triggering_time = min(current_times[t] for t in topic_names)
                    # snapshot messages and times (deepcopy under lock to avoid races)
                    snapshot_msgs = {t: copy.deepcopy(self.latest_msgs.get(t)) for t in topic_names}
                    snapshot_times = {t: float(current_times[t]) for t in topic_names}
                    # update last accepted observation time
                    self._last_obs_time = float(triggering_time)
                    triggered = True

            if triggered:
                # Convert each snapshot message to numpy array (outside lock)
                data = {}
                for t in topic_names:
                    msg = snapshot_msgs.get(t)
                    if msg is None:
                        data[t] = None
                    else:
                        try:
                            data[t] = msg_to_numpy(msg)
                        except Exception as e:
                            rospy.logwarn_throttle(10.0, f"[PolicyController] msg_to_numpy failed for {t}: {e}")
                            data[t] = None

                return {
                    "timestamp": float(triggering_time),
                    "data": data,
                    "times": snapshot_times,
                }

            # Not ready -> sleep briefly and re-evaluate
            rospy.sleep(0.005)
            wait_threshold = self._last_obs_time + target_dt - tol

        # ROS is shutting down
        return None

    
    
    def safety_check(self, action):
        """
            Check if the action is safe to execute
            If not, modify the action to be safe
            and log WARNING
        """
        # TODO: EE position should be within workspace
        pass
    

    def get_video_prompt(self):
        """
            Get the video prompt from the human
        """
        pass
    
    
    def run(self):
        """
            Main loop to run the policy controller
            Get observation -> Select action -> Execute action
        """
        while not rospy.is_shutdown():
            #TODO: 
            pass
    
    
        

@hydra.main(version_base=None, config_path="/opt/ros_ws/src/mimic_play/config", config_name="mimic_play_recorder.yaml")
def main(cfg: DictConfig):
    pass

if __name__ == "__main__":
    main()
