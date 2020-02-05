#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy as np
import roslib
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

pub = []

br = tf.TransformBroadcaster()

# first_pose = [459899.33863061824, 4483274.307344821, 624.3099975585938]
first_pose = [0, 0, 0]
old_pose = [0, 0, 0]
first_time = True
heading_from_poses = True
reset_odometry = True
custom_initial_pose = False
translation_to_frame = False
first_x = 0.0
first_y = 0.0
first_z = 0.0
loam_active = False
send_tf = False
add_noise_cov = False
tf_parent = ''
tf_child = ''

def odom_cb(msg):
    global pub, first_pose, first_time, loam_active, reset_odometry, heading_from_poses, old_pose, tf_parent, tf_child, send_tf, first_x, first_y, first_z, custom_initial_pose

    new_odom = Odometry()
    new_odom.header.frame_id = tf_parent
    new_odom.child_frame_id = tf_child
    if loam_active:
        x = msg.pose.pose.position.z
        y = msg.pose.pose.position.x
        z = -1*msg.pose.pose.position.y
        qx = msg.pose.pose.orientation.z
        qy = -1*msg.pose.pose.orientation.x
        qz = -1*msg.pose.pose.orientation.y
        qw = msg.pose.pose.orientation.w
        R, P, Y = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
        yaw_q = tf.transformations.quaternion_from_euler(R, P, -1*Y)
    else:

        if first_time and reset_odometry:
            if custom_initial_pose:
                first_pose = [first_x, first_y, first_z]
            else:
                first_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
            print('***FIRST POSE***\n', first_pose)
            first_time = False

        x = msg.pose.pose.position.x - first_pose[0]
        y = msg.pose.pose.position.y - first_pose[1]
        z = msg.pose.pose.position.z - first_pose[2]
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        R, P, Y = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
        if heading_from_poses:
            Y = np.arctan2(y - old_pose[1], x - old_pose[0])
            P = np.arctan2(z - old_pose[2], np.sqrt((x-old_pose[0])**2 + (y-old_pose[1])**2))
            old_pose = [x, y, z]
        if translation_to_frame:
            print(Y)
            x_trans = -1.405
            y_trans = 0.32
            x = x + x_trans*np.cos(Y) + y_trans*np.sin(Y)
            y = y + x_trans*np.sin(Y) + y_trans*np.cos(Y)
        yaw_q = tf.transformations.quaternion_from_euler(R, P, Y)#0.106
    new_odom.pose.pose.position.x = x
    new_odom.pose.pose.position.y = y
    new_odom.pose.pose.position.z = z
    new_odom.pose.pose.orientation.x = yaw_q[0]
    new_odom.pose.pose.orientation.y = yaw_q[1]
    new_odom.pose.pose.orientation.z = yaw_q[2]
    new_odom.pose.pose.orientation.w = yaw_q[3]

    new_odom.pose.covariance = msg.pose.covariance

    if add_noise_cov >= 0.0:
        yaw_cov = 0.05
        new_odom.pose.pose.position.x = np.random.normal(new_odom.pose.pose.position.x, add_noise_cov)
        new_odom.pose.pose.position.y = np.random.normal(new_odom.pose.pose.position.y, add_noise_cov)
        Y = np.random.normal(Y, yaw_cov)
        yaw_q = tf.transformations.quaternion_from_euler(R, P, Y)
        new_odom.pose.pose.orientation.x = yaw_q[0]
        new_odom.pose.pose.orientation.y = yaw_q[1]
        new_odom.pose.pose.orientation.z = yaw_q[2]
        new_odom.pose.pose.orientation.w = yaw_q[3]
        new_odom.pose.covariance = [add_noise_cov, 0, 0, 0, 0, 0, 0, add_noise_cov, 0, 0, 0, 0, 0, 0, add_noise_cov, 0, 0, 0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, yaw_cov]

    new_odom.header.stamp = msg.header.stamp

    pub.publish(new_odom)


    if send_tf:
        br.sendTransform([x, y, z], yaw_q, rospy.Time.now(), tf_child, tf_parent)


if __name__ == '__main__':

    rospy.init_node('rotate_odometry')
    prefix = '~'

    send_tf = rospy.get_param(prefix + 'send_tf', False) # 
    heading_from_poses = rospy.get_param(prefix + 'heading_from_poses', False) # 
    add_noise_cov = rospy.get_param(prefix + 'add_noise_cov', -1.0) # 
    reset_odometry = rospy.get_param(prefix + 'reset_odometry', False) # 
    translation_to_frame = rospy.get_param(prefix + 'translation_to_frame', False) # 
    custom_initial_pose = rospy.has_param(prefix + 'first_x')
    first_x = rospy.get_param(prefix + 'first_x', False) # 
    first_y = rospy.get_param(prefix + 'first_y', False) # 
    first_z = rospy.get_param(prefix + 'first_z', False) # 
    loam_active = rospy.get_param(prefix + 'loam_active', False) # 
    input_type = rospy.get_param(prefix + 'input_type', 0) # 0: Odometry 1:PoseWithCovarianceStamped
    input_topic = rospy.get_param(prefix + 'input_topic', 'odometry') 
    output_topic = rospy.get_param(prefix + 'output_topic', 'odometry_transformed') 
    tf_parent = rospy.get_param(prefix + 'tf_parent', 'map') 
    tf_child = rospy.get_param(prefix + 'tf_child', 'os1_lidar') 
    pub = rospy.Publisher(output_topic, Odometry, queue_size=1)

    if input_type == 0:
        odom_sub = rospy.Subscriber(input_topic, Odometry, odom_cb)
    else:
        pose_sub = rospy.Subscriber(input_topic, PoseWithCovarianceStamped, odom_cb)

    rospy.spin()
