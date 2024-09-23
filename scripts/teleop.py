import rospy
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
from controller_manager_msgs.srv import ListControllers
import numpy as np
import tf
import time

class Teleop():

    def __init__(self):
        ############## Start ROS Node ###################
        rospy.init_node('franka_teleop')

        ############## Global variables #################
        self.vive_last_right_pose = Pose()
        
        self.ee_pos = None
        self.ee_quat = None

        self.gripper_open = None
        ############## Subscribers ######################

        # VIVE controller pose
        self.vive_right_subscriber = rospy.Subscriber('/vive/my_right_controller_1_Pose', Pose, self.vive_pose_cb)

        # Franka end effector pose (for some reason can't read topic)
        self.subscribe_ee_pose = rospy.Subscriber('/franka_state_controller/franka_states/O_T_EE', PoseStamped, self.__process_ee_pose, queue_size=1)

        ############## Publishers #######################
        self.pub = rospy.Publisher('/cartesian_impedance_controller/desired_pose', PoseStamped, queue_size=0)
        self.pub_gripper = rospy.Publisher('/cartesian_impedance_controller/desired_gripper_state', PointStamped, queue_size=0)

        ############## TF ###############################
        self.tf_listener = tf.TransformListener()

    ################################# Controller Pose #####################################################
    def vive_pose_cb(self, msg):
        self.vive_last_right_pose = msg

    def get_NE_pose(self): 
        # Function to get EE pose from tf
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('panda_link0', 'panda_NE', rospy.Time(0))
                if (trans != None) and (rot != None):
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans, rot


    def __process_ee_pose(self, data):
        # Callback to get EE pose
        self.ee_pos = np.asarray([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.ee_quat = np.asarray([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        print("EE pose:",self.ee_pos)

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

    
    def publish_eef_target(self, pos, quat):
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

    def open_gripper(self):
        self.gripper_open = True

    def close_gripper(self):
        self.gripper_open = False

    def open_gripper_w_msg(self):
        # function is needed as otherwise the desired gripper state is only sent upon the node being active!
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        self.gripper_open = True
        msg_gripper.point.x = float(not (self.gripper_open))
        self.pub_gripper.publish(msg_gripper)
        time.sleep(1)


    def close_gripper_w_msg(self):
        # function is needed as otherwise the desired gripper state is only sent upon the node being active!
        msg_gripper = PointStamped()
        msg_gripper.header.stamp = rospy.Time.now()
        self.gripper_open = False
        msg_gripper.point.x = float(not (self.gripper_open))
        self.pub_gripper.publish(msg_gripper)
        time.sleep(1)


    def run(self):
        while not rospy.is_shutdown():

            # Get current EE pose
            trans_EE, rot_EE = self.get_NE_pose()
            print("EE position:", trans_EE)
            print("---------------------------------------------------")

            # Get VIVE controller pose
            vive_position, vive_orientation = self.get_pose_info(self.vive_last_right_pose)
            print("VIVE position:", vive_position)




if __name__ == '__main__':
    teleop = Teleop()
    teleop.run()
    


