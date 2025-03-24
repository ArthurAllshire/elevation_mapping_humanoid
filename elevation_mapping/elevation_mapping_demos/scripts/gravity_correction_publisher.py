#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_multiply,
    rotation_matrix,
    euler_from_quaternion,
    quaternion_from_euler
)

def rotation_matrix(axis, angle, is_homogeneous=True):
    """
    Constructs a homogeneous 4x4 rotation matrix for rotation about the given axis.
    
    Parameters:
        axis (str): The axis to rotate about ('x', 'y', or 'z').
        angle (float): The rotation angle in radians.
        
    Returns:
        np.ndarray: A 4x4 homogeneous transformation matrix representing the rotation.
        
    Raises:
        ValueError: If the axis is not one of 'x', 'y', or 'z'.
    """
    c = np.cos(angle)
    s = np.sin(angle)
    
    if axis == 'x':
        # Rotation about the x-axis
        R = np.array([
            [1,  0,   0, 0],
            [0,  c,  -s, 0],
            [0,  s,   c, 0],
            [0,  0,   0, 1]
        ])
    elif axis == 'y':
        # Rotation about the y-axis
        R = np.array([
            [ c, 0, s, 0],
            [ 0, 1, 0, 0],
            [-s, 0, c, 0],
            [ 0, 0, 0, 1]
        ])
    elif axis == 'z':
        # Rotation about the z-axis
        R = np.array([
            [c, -s, 0, 0],
            [s,  c, 0, 0],
            [0,  0, 1, 0],
            [0,  0, 0, 1]
        ])
    else:
        raise ValueError("Invalid axis. Expected one of 'x', 'y', or 'z'.")
    
    if is_homogeneous:
        return R
    else:
        return R[:3, :3]

class GravityCorrectionPublisher:
    def __init__(self):
        rospy.init_node('gravity_correction_publisher')
        
        # TF listener and broadcaster
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Subscribe to IMU data
        self.imu_sub = rospy.Subscriber('/livox/imu', Imu, self.imu_callback)
        self.gravity_queue = []

        # self.latest_accel = None
        self.measured_gravity = None
        
        # Frame names
        self.lidar_frame = 'lidar_link'
        self.odom_frame = 'odom_torso'
        self.lidar_to_odom_trans = None
        self.lidar_to_odom_rot = None

        self.corrected_frame = 'odom_corrected'
        self.published = False

        self.n_average_linvel = 10
        
        # Publishing rate
        self.rate = rospy.Rate(100)  # 100 Hz

        # Wait for transforms to become available
        rospy.loginfo("Waiting for transforms to become available...")
        try:
            self.tf_listener.waitForTransform(
                self.odom_frame,
                self.lidar_frame,
                rospy.Time(),
                rospy.Duration(5.0)
            )
            rospy.loginfo("Transforms are now available!")
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.logwarn("Transform not available after waiting! Will keep trying during execution...")
        
    def imu_callback(self, msg):
        """Process incoming IMU messages."""
        if self.measured_gravity is not None:
            self.imu_sub.unregister()
            print(f'unsubscribed from IMU data')
            return

        latest_accel_recv = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        rospy.logdebug(f"Received IMU data. Acceleration: {latest_accel_recv}")

        self.gravity_queue.append(latest_accel_recv)
        if len(self.gravity_queue) > self.n_average_linvel:

            average_linvel = np.mean(self.gravity_queue, axis=0)


            # transform from IMU frame to lidar frame.
            # Trial and error requried to find this, not documented AFAIK :( 
            R = np.array([
                [-1, 0, 0],
                [0, -1, 0],
                [0, 0, -1]
            ])

            self.measured_gravity = np.dot(R, average_linvel)

            # print(f'latest_accel_recv: {latest_accel_recv}')
            # print(f'latest_accel: {self.latest_accel}')

    def compute_gravity_vector(self):
        """Compute rotation to align with gravity vector."""
        if self.measured_gravity is None:
            rospy.logwarn_throttle(1.0, "No IMU data received yet!")
            return None

        # Normalize acceleration vector
        accel_norm = np.linalg.norm(self.measured_gravity)
        if accel_norm < 1e-6:
            rospy.logwarn_throttle(1.0, "Acceleration magnitude too small!")
            return None

        accel_normalized = self.measured_gravity / accel_norm
        print(f"Normalized acceleration vector: {accel_normalized}")

        return accel_normalized


        # # Desired gravity direction in lidar frame (pointing down)
        # gravity_target = np.array([0.0, 0.0, -1.0])

        # # Compute rotation axis (cross product)
        # rotation_axis = np.cross(accel_normalized, gravity_target)
        # rotation_axis_norm = np.linalg.norm(rotation_axis)

        # if rotation_axis_norm < 1e-6:
        #     rospy.logdebug("Vectors already aligned, no rotation needed")
        #     return quaternion_from_euler(0, 0, 0)  # Identity quaternion

        # rotation_axis = rotation_axis / rotation_axis_norm

        # # Compute rotation angle
        # cos_angle = np.dot(accel_normalized, gravity_target)
        # angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
        # print(f'angle: {np.degrees(angle)}')
        # rospy.logdebug(f"Correction angle: {np.degrees(angle)} degrees")

        # # Create quaternion for gravity correction
        # return quaternion_from_euler(
        #     angle * rotation_axis[0],
        #     angle * rotation_axis[1],
        #     angle * rotation_axis[2]
        # )
    
    def get_odom_to_lidar_transform(self):

        try:
            # Get transform from odom_torso to lidar_link
            # Use a slightly older timestamp to avoid extrapolation
            timestamp = rospy.Time.now() - rospy.Duration(0.1)  # 100ms in the past
            
            try:
                self.tf_listener.waitForTransform(
                    self.odom_frame,
                    self.lidar_frame,
                    timestamp,
                    rospy.Duration(0.1)
                )
            except tf.Exception as e:
                rospy.logwarn_throttle(1.0, f"Wait for transform failed: {str(e)}")
                return None, None
            
            (trans, rot) = self.tf_listener.lookupTransform(
                self.odom_frame,
                self.lidar_frame,
                timestamp
            )
            rospy.logdebug(f"Current transform - Translation: {trans}, Rotation: {rot}")
            return trans, rot

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"Failed to lookup transform: {str(e)}")
            return None, None
    def publish_corrected_frame(self):
        """Publish the gravity-corrected frame."""
        if not self.measured_gravity is not None:
            rospy.logwarn_throttle(1.0, "Waiting for IMU data...")
            return
        
        if self.lidar_to_odom_trans is None or self.lidar_to_odom_rot is None:
            trans, rot = self.get_odom_to_lidar_transform()
            if trans is None or rot is None:
                rospy.logwarn_throttle(1.0, "Failed to get transform")
                return
            self.lidar_to_odom_trans = trans
            self.lidar_to_odom_rot = rot

        # Compute gravity correction in lidar frame
        gravity_vector = self.compute_gravity_vector()
        if gravity_vector is None:
            return
        print(f"OG gravity vector: {gravity_vector}")
        
        # first, get the rotation matrix from the rotation quaternion
        rot_mat = quaternion_matrix(self.lidar_to_odom_rot)
        print(f"Rotation matrix: {rot_mat}")

        local_gravity_vector = np.dot(rot_mat, np.concatenate((gravity_vector, np.array([1.0]))))[0:3]
        print(f"Local gravity vector: {local_gravity_vector}")

        local_gravity_vector = local_gravity_vector / np.linalg.norm(local_gravity_vector)
        print(f"Normalized local gravity vector: {local_gravity_vector}")

        # Desired gravity direction in lidar frame (pointing down)
        gravity_target = np.array([0.0, 0.0, -1.0])

        # Compute rotation axis (cross product)
        rotation_axis = np.cross(local_gravity_vector, gravity_target)
        rotation_axis_norm = np.linalg.norm(rotation_axis)

        if rotation_axis_norm < 1e-6:
            rospy.logdebug("Vectors already aligned, no rotation needed")
            return quaternion_from_euler(0, 0, 0)  # Identity quaternion

        rotation_axis = rotation_axis / rotation_axis_norm

        # Compute rotation angle
        cos_angle = np.dot(local_gravity_vector, gravity_target)
        angle = -np.arccos(np.clip(cos_angle, -1.0, 1.0))
        print(f'angle: {np.degrees(angle)}')

        # Create quaternion for gravity correction
        gravity_correction_quat = quaternion_from_euler(
            angle * rotation_axis[0],
            angle * rotation_axis[1],
            angle * rotation_axis[2]
        )

        corrected_rot_mat = quaternion_matrix(gravity_correction_quat)
        print(f"Corrected rotation matrix: {corrected_rot_mat}")

        final_trans = corrected_rot_mat[:3, 3]
        final_rot = quaternion_from_matrix(corrected_rot_mat)


        # # Convert current transform to matrix
        # current_mat = quaternion_matrix(rot)
        # current_mat[:3, 3] = trans

        # # Convert gravity correction to matrix
        # correction_mat = quaternion_matrix(gravity_correction_quat)

        # # Combine transformations
        # final_mat = np.dot(current_mat, correction_mat)

        # # Extract final rotation and translation
        # final_trans = final_mat[:3, 3]
        # final_rot = quaternion_from_matrix(final_mat)

        # Create and publish transform message
        print(f'gonna publish transform')
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.corrected_frame


        t.transform.translation.x = final_trans[0]
        t.transform.translation.y = final_trans[1]
        t.transform.translation.z = final_trans[2]

        t.transform.rotation.x = final_rot[0]
        t.transform.rotation.y = final_rot[1]
        t.transform.rotation.z = final_rot[2]
        t.transform.rotation.w = final_rot[3]

        self.tf_broadcaster.sendTransform(t)
        print("Published corrected transform")
        self.published = True

    def run(self):
        """Main run loop."""
        rospy.loginfo("Starting gravity correction publisher...")
        # while not rospy.is_shutdown():
        # i = 0
        # while i < 100:
        # while not self.published:
        while not rospy.is_shutdown():
            self.publish_corrected_frame()
            self.rate.sleep()
        rospy.spin()

if __name__ == '__main__':
    try:
        node = GravityCorrectionPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass 