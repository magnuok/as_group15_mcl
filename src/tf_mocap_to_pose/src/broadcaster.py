#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PoseStamped

odometry = PoseStamped()

def callback_odometry(odom_msg):
    """
    callback function used by a ros subscriber. Receives the odometry.
    :param pose: ros pose
    """
    # Create a new pose
    new_pose = PoseStamped()
    new_pose.header.frame_id = "mocap"


# TODO: Transform old to new pose. Add TRANSFORM
    new_pose.pose.position.x = odom_msg.pose.position.x - 0.00172006478533
    new_pose.pose.position.y = odom_msg.pose.position.y - 0.00965558644384
    new_pose.pose.position.z = odom_msg.pose.position.z - 0.270615190268

    quaternion = odom_msg.pose.orientation
    euler = tf.transformations.euler_from_quaternion([quaternion.x, quaternion.y, quaternion.z, quaternion.w], axes='sxyz')
    euler_trans = tf.transformations.euler_from_quaternion([0.00107793998905, 0.00787599850446, 0.372645974159, 0.927939653397], axes='sxyz')

    euler_list = list(euler)

    euler_list[2] = euler[2] - euler_trans[2]

    quaternion = tf.transformations.quaternion_from_euler(euler_list[0], euler_list[1], euler_list[2], 'sxyz')

    new_pose.pose.orientation.x = quaternion[0]
    new_pose.pose.orientation.y = quaternion[1]
    new_pose.pose.orientation.z = quaternion[2]
    new_pose.pose.orientation.w = quaternion[3]

    global odometry
    odometry = new_pose


if __name__ == '__main__':

    # initialize node
    rospy.init_node('pose_to_mocap_pose', anonymous=True)
    rospy.Subscriber("/vrpn_client_node/pioneer/pose", PoseStamped, callback_odometry)
    publisher = rospy.Publisher('/PoseStamped', PoseStamped, queue_size=10)

    while not rospy.is_shutdown():
        publisher.publish(odometry)