# from reader import OculusReader
from tf.transformations import quaternion_from_matrix
import rospy
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg
from oculus_controller import VRPolicyFranka
import numpy as np
from droid.misc.transformations import euler_to_quat

class OculusReaderVRPolicy:
    """
    Wrapper class for the Oculus Reader Policy, formatting Oculus Reader actions

    Apr 2024,
    Author: Sophie Lueth
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
            rmat_reorder: list = [-2, -1, -3, 4]
        ):
        self._vr_policy = VRPolicyFranka(right_controller=right_controller,
                                   max_lin_vel=max_lin_vel,
                                   max_rot_vel=max_rot_vel,
                                   max_gripper_vel=max_gripper_vel,
                                   spatial_coeff=spatial_coeff,
                                   pos_action_gain=pos_action_gain,
                                   rot_action_gain=rot_action_gain,
                                   gripper_action_gain=gripper_action_gain,
                                   rmat_reorder=rmat_reorder)
      
        # self._dt = dt
        state_dict = {"cartesian_position": np.zeros(6), "gripper_position": 0.0}
        self._obs_dict = {"robot_state": state_dict}
      
    def get_update(self):
        """
        returns twist (position and euler angles), gripper position and whether button A was pressed m
        """ 
        action, info = self._vr_policy.forward(self._obs_dict, include_info=True)
        
        active = np.any(action != np.zeros(7))
        if active:

            # action /= self._dt
            self._obs_dict["robot_state"]["cartesian_position"] = info["target_cartesian_position"] 
            self._obs_dict["robot_state"]["gripper_position"] = info["target_gripper_position"]
            
            return active, action[:3], action[3:6], info
        
        else:
            return False, None, None, None
        

def publish_transform(transform):
    pose_publisher = rospy.Publisher('/oculus/my_right_controller_1_Pose', geometry_msgs.msg.Pose, queue_size=0)

    translation = transform[:3]

    # br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.Pose()

    # t.header.stamp = rospy.Time.now()
    # t.header.frame_id = 'world'
    # t.child_frame_id = name

    t.position.x = translation[0]
    t.position.y = translation[1]
    t.position.z = translation[2]

    quat = euler_to_quat(transform[3:])
    t.orientation.x = quat[0]
    t.orientation.y = quat[1]
    t.orientation.z = quat[2]
    t.orientation.w = quat[3]

    # br.sendTransform(t)
    
    pose_publisher.publish(t)

def publish_buttons(buttons):
    buttons_publisher = rospy.Publisher('/oculus/my_right_controller_1/joy', sensor_msgs.msg.Joy, queue_size=0)
    b = sensor_msgs.msg.Joy()

    # Header
    b.header.stamp = rospy.Time.now()
    b.header.frame_id = 'world'
    
    # Axes - Joy Stick Info
    b.axes = list(buttons['rightJS'])

    # Buttons
    b.buttons = [ int(buttons['A']), 
                  int(buttons['B']), 
                #   int(buttons['rightTrig'][0]), 
                #   int(buttons['rightGrip'][0])
                  int(buttons['RTr']),
                  int(buttons['RG'])
                  ]
    # t.child_frame_id = name
    # t.transform.translation.x = translation[0]
    # t.transform.translation.y = translation[1]
    # t.transform.translation.z = translation[2]

    # quat = quaternion_from_matrix(transform)
    # t.transform.rotation.x = quat[0]
    # t.transform.rotation.y = quat[1]
    # t.transform.rotation.z = quat[2]
    # t.transform.rotation.w = quat[3]

    buttons_publisher.publish(b)


def main():
    oculus_reader : OculusReaderVRPolicy = OculusReaderVRPolicy()
    control_buttons = ["A", "B", "RTr", "RG", "rightJS"]
    rospy.init_node('oculus_reader')
    rospy.loginfo("Oculus Publisher started")

    while not rospy.is_shutdown():
        rospy.sleep(0.001)
        
        _, _, _, info = oculus_reader.get_update()
        
        if info:

            right_controller_buttons = dict( [(button , info[button]) for button in control_buttons]) 
            right_controller_pose = info["target_cartesian_position"]

            print("Buttons", right_controller_buttons)
            print("Pose", right_controller_pose)

            publish_transform(right_controller_pose)
            publish_buttons(right_controller_buttons)
        

if __name__ == '__main__':
    main()
