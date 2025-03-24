#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def publish_new_frame():
    rospy.init_node('frame_combiner')

    tf_listener = tf.TransformListener()
    tf_broadcaster = tf2_ros.TransformBroadcaster()

    rate = rospy.Rate(200.0)  # 200 Hz

    frame1 = 'odom_torso'
    frame2 = 'torso_link'
    new_frame = 'height_map_2'
    parent_frame = 'odom_torso'  # Replace with the actual parent frame if needed

    while not rospy.is_shutdown():
        try:
            # Get the transformation from parent_frame to frame1
            (trans1, rot1) = tf_listener.lookupTransform(parent_frame, frame1, rospy.Time(0))
            # Get the transformation from parent_frame to frame2
            (trans2, rot2) = tf_listener.lookupTransform(parent_frame, frame2, rospy.Time(0))

            # Create a TransformStamped message
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = parent_frame
            t.child_frame_id = new_frame

            # Use the position from frame2
            t.transform.translation.x = trans2[0]
            t.transform.translation.y = trans2[1]
            t.transform.translation.z = trans2[2]

            # Use the orientation from frame2
            (roll, pitch, yaw) = euler_from_quaternion (rot2[:4])
            new_quat = quaternion_from_euler(0, 0, yaw)
            t.transform.rotation.x = new_quat[0]
            t.transform.rotation.y = new_quat[1]
            t.transform.rotation.z = new_quat[2]
            t.transform.rotation.w = new_quat[3]

            # Publish the new frame
            tf_broadcaster.sendTransform(t)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_new_frame()
    except rospy.ROSInterruptException:
        pass
