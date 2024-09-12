import rospy
from geometry_msgs.msg import Pose
import tf

class Teleop():

    def __init__(self):
        ############## Global variables #################
        self.vive_last_right_pose = Pose()

        ############## Subscribers ######################
        self.vive_right_subscriber = rospy.Subscriber('/vive/my_right_controller_1_Pose', Pose, self.pose_cb)

        ############## TF ###############################
        self.transform_listener = tf.TransformListener()

    ################################# Controller Pose #####################################################
    def pose_cb(self, msg):
        self.vive_last_right_pose = msg
        print("Latest vive controller position:", self.vive_last_right_pose.position)
        print("Latest vive controller orientation:", self.vive_last_right_pose.orientation)

    def get_robot_pose(self): 
        while True:
            try:
                (trans, rot) = self.tf_listener.lookupTransform('panda_link_0', 'gripper_right_grasping_frame', rospy.Time(0))
                if (trans != None) and (rot != None):
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return trans, rot



if __name__ == '__main__':
    rospy.init_node('franka_teleop')
    teleop = Teleop()
    rospy.spin()
    


