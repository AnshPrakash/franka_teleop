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
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix, euler_matrix, quaternion_from_euler

# Lock to protect concurrent access to latest messages
import threading


from util import go_to, attempt_to_go_to_joints, msg_to_numpy
from grasping import Gripper
from RosbagControlledRecorder import RosbagControlledRecorder


# Mimicplay policy imports
import mimicplay.utils.file_utils as FileUtils
import robomimic.utils.torch_utils as TorchUtils
import robomimic.utils.tensor_utils as TensorUtils
import robomimic.utils.obs_utils as ObsUtils

class PolicyController:
    def __init__(self, ckpt_path: str, video_prompt_path: str, target_frequency: float = 11):
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
                                
        ############## Publishers #######################
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.pub_gripper = rospy.Publisher('/cartesian_impedance_controller/desired_gripper_state', PointStamped, queue_size=0)

        ############## TF ###############################
        self.tf_listener = tf.TransformListener()

        ############# Recorder #########################
        
        ## For recording our results
        
        video_topics_only = [
            "/zedA/zed_node_A/left/image_rect_color", 
            "/zedB/zed_node_B/left/image_rect_color"
            ]
        
        save_folder = "eval_recordings"
        self.video_recorder = RosbagControlledRecorder(
            save_folder=save_folder,
            topics=video_topics_only,
            is_video=True,
            complementary_recorder=None
            )
        
        self.ckpt_path = ckpt_path
        self.video_prompt = video_prompt_path
        self.init_policy()

        rospy.loginfo("[PolicyController] Initialization complete. Topics subscribed: " + ", ".join(self.topics.keys()))
    
    
    def init_policy(self):
        """
            Initialize the policy network
        """
        ckpt_path = self.ckpt_path
        video_prompt = self.video_prompt

        # device
        device = TorchUtils.get_torch_device(try_to_use_cuda=True)

        # restore policy
        policy, ckpt_dict = FileUtils.policy_from_checkpoint(ckpt_path=ckpt_path, device=device, verbose=False)
        # get video prompt
        policy.policy.load_eval_video_prompt(video_prompt)
        
        self.policy = policy
        self.device = device

    
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

    ############################################# MimicPlay Model functions #############################################
    
    def get_action(self, observation_dict: dict) -> np.ndarray:
        """
            Get the action from the policy network (RolloutPolicy)
            Input: observation_dict
            Output: action as a numpy array (Mean action of the action distribution)
        """
        act = self.policy.get_action(ob=observation_dict, goal=None)
        return act 


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

        
    def action_to_ee_quaternion(self, action: np.ndarray):
        """
        Convert the action's Euler orientation to an end-effector quaternion.

        Expected action layout (for EE part):
            [ x, y, z, ox, oy, oz, gripper_open ]
        where ox, oy, oz are Euler angles (roll, pitch, yaw) in radians.

        Returns:
            np.ndarray of shape (4,) -> (qx, qy, qz, qw)
        """
        if action is None:
            raise ValueError("action is None")


        # Extract Euler angles (roll, pitch, yaw)
        roll = float(action[3])
        pitch = float(action[4])
        yaw = float(action[5])

        # roll, pitch, yaw = map(np.deg2rad, (roll, pitch, yaw))
        
        quat = np.asarray(quaternion_from_euler(roll, pitch, yaw), dtype=float) 

        return quat  # (qx, qy, qz, qw)


    def safety_check(self, action: np.ndarray) -> bool:
        """
        Check if the end-effector target position from `action` is inside the allowed task-space.
        If it's outside, publish a PoseStamped to /policy_controller/target_pose so the user can
        inspect it in RViz, then pause and prompt the user to continue or abort.

        Returns:
            bool: True if it is safe (or user approved) to continue, False to abort.
        """
        try:

            target_pos = action[0:3]  # [x, y, z]

            # Workspace bounds:
            # (tweak these sensible defaults to match your robot/setup)
            workspace_lower = np.array([0.15, -0.6, 0.0], dtype=float)
            workspace_upper = np.array([0.85,  0.6, 1.0], dtype=float)


            # Check if within bounds (inclusive)
            inside = np.all(target_pos >= workspace_lower) and np.all(target_pos <= workspace_upper)

            if inside:
                # safe to proceed
                return True

            # Outside workspace: publish visualization pose so user can inspect it in RViz
            rospy.logwarn(
                "[PolicyController] Requested EE target position is OUTSIDE the workspace bounds.\n"
                f"  target_pos = {target_pos.tolist()}\n"
                f"  allowed x = [{workspace_lower[0]}, {workspace_upper[0]}], "
                f"y = [{workspace_lower[1]}, {workspace_upper[1]}], "
                f"z = [{workspace_lower[2]}, {workspace_upper[2]}]"
            )

            # Create publisher on first use (latched so RViz can pick it up)
            if not hasattr(self, "_target_pose_pub"):
                self._target_pose_pub = rospy.Publisher(
                    "/policy_controller/target_pose", PoseStamped, queue_size=1, latch=True
                )
                # small sleep to allow registration (non-blocking)
                rospy.sleep(0.05)

            # Build PoseStamped from action (use action orientation if available)
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = getattr(self, "base_frame", "world")  # let user override if needed

            # fill position
            pose_msg.pose.position.x = float(target_pos[0])
            pose_msg.pose.position.y = float(target_pos[1])
            pose_msg.pose.position.z = float(target_pos[2])

            # orientation: if action contains euler angles use them, otherwise leave identity
            if action.size >= 6:
                try:
                    quat = self.action_to_ee_quaternion(action)  # expects (x,y,z,w) numpy array
                    # quaternion_from_euler returns [x, y, z, w]
                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])
                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"[PolicyController] failed to convert Euler->quat for debug pose: {e}")
                    # leave default orientation (0,0,0,1)

            # publish the debug pose for RViz visualization
            try:
                self._target_pose_pub.publish(pose_msg)
                rospy.loginfo("[PolicyController] Published target pose to /policy_controller/target_pose for inspection in RViz.")
            except Exception as e:
                rospy.logwarn(f"[PolicyController] Could not publish target pose: {e}")

            # Pause and ask the user to confirm. Allow ROS shutdown to break out.
            prompt = (
                "Action is outside workspace. Inspect target in RViz.\n"
                "Type 'c' or 'y' to CONTINUE and execute this action anyway, or 'a' or 'n' to ABORT: "
            )

            # Loop until valid input or ROS shutdown
            while not rospy.is_shutdown():
                try:
                    # Use input() so it works with python3
                    user = input(prompt).strip().lower()
                except EOFError:
                    # In some runtime contexts (no stdin), abort
                    rospy.logwarn("[PolicyController] No stdin available to confirm safety; aborting action.")
                    return False
                except Exception as e:
                    rospy.logwarn(f"[PolicyController] Input error: {e}; aborting.")
                    return False

                if user in ("c", "y", "yes", "continue"):
                    rospy.loginfo("[PolicyController] User approved executing out-of-bounds action.")
                    return True
                if user in ("a", "n", "no", "abort"):
                    rospy.loginfo("[PolicyController] User aborted action due to safety.")
                    return False

                # otherwise, loop again (invalid answer)
                rospy.loginfo("Please type 'c' to continue or 'a' to abort.")

            # If rospy is shutting down, abort
            rospy.loginfo("[PolicyController] rospy shutting down while waiting for safety confirmation -> aborting")
            return False

        except Exception as e:
            rospy.logwarn(f"[PolicyController] safety_check exception: {e}; aborting action.")
            return False

    
    def check_over(self, action_guidance) -> bool:
        """
            Check if the task is over
            if the robot has reached the goal and opened the gripper
            Goal is estimated from the action guidance
            And ask the user to confirm
            Returns True if the task is over, False otherwise
        """
        pass
    
    def run(self):
        """
            Main loop to run the policy controller
            Get observation -> Select action -> Execute action
            Get the video prompt from the human
            Also resets after every evaluation
        """
        while not rospy.is_shutdown():
            # Get latent prompt from human video
            action_guidance, latent_plan = self.get_video_prompt()
            policy_controller_run = True
            # Keep running until it reaches the goal and opens the gripper
            while policy_controller_run:
                # Get observation
                obs = self.get_obeservation()
                
                action = self.get_action(obs)
                
                quat_action = action[3:7]
                
                # Safety check
                if not self.safety_check(action):
                    rospy.loginfo("[PolicyController] Action aborted by safety check.")
                    policy_controller_run = False
                    continue
                
                target_ee_pos = action[0:3]
                target_ee_ori = quat_action
                
                self.publish_eef_target(target_ee_pos, target_ee_ori)
                if self.check_over(action_guidance):
                    rospy.loginfo("[PolicyController] Task completed successfully.")
                    policy_controller_run = False
                    continue
                
                self.rate.sleep()
                
            rospy.loginfo("[PolicyController] Resetting for next evaluation.")
            
            # Reset the environment
            
            self.move_home()
            
            rospy.sleep(2.0)
        
    
    
        

@hydra.main(version_base=None, config_path="/opt/ros_ws/src/mimic_play/config", config_name="mimic_play_recorder.yaml")
def main(cfg: DictConfig):
    pass

if __name__ == "__main__":
    main()
