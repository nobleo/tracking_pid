#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import math
import rospy
import tf
import unittest

from tracking_pid.path_interpolator import SectionInterpolation


class TestSectionInterpolator(unittest.TestCase):
    def quaternion_to_yaw(self, quaternion_in):
        quaternion = (
            quaternion_in.x,
            quaternion_in.y,
            quaternion_in.z,
            quaternion_in.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        return (euler[2] + math.pi) % (2 * math.pi) - math.pi  # wrap

    def test_interpolate_path_translation(self):
        """ Test interpolation for translation
        Accelerate at 1 m/s2 to 1m/s, keep 1s and deccelerate to standing still
        Total movement should be 2m.
        Trajectory is tested at different times
        """
        self._target_x_vel = 1.0
        self._target_x_acc = 1.0
        self._target_yaw_vel = 1.0
        self._target_yaw_acc = 1.0

        path = Path()
        pose1 = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose1.pose.orientation.x = quaternion[0]
        pose1.pose.orientation.y = quaternion[1]
        pose1.pose.orientation.z = quaternion[2]
        pose1.pose.orientation.x = quaternion[3]
        path.poses.append(pose1)

        pose2 = PoseStamped()
        pose2.pose.position.x = 2.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose2.pose.orientation.x = quaternion[0]
        pose2.pose.orientation.y = quaternion[1]
        pose2.pose.orientation.z = quaternion[2]
        pose2.pose.orientation.w = quaternion[3]
        path.poses.append(pose2)

        current_section = SectionInterpolation(pose1, pose2, rospy.Time(0.0),
                                               self._target_x_vel, self._target_x_acc,
                                               self._target_yaw_vel, self._target_yaw_acc)

        # Acceleration phase x/theta = 0.5*a*t^2, v/dtheta=a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(0.0))
        self.assertEqual(tp.pose.pose.position.x, 0.0)
        self.assertEqual(tp.velocity.linear.x, 0.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(0.5))
        self.assertAlmostEqual(tp.pose.pose.position.x, 0.125)
        self.assertAlmostEqual(tp.velocity.linear.x, 0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(1.0))
        self.assertAlmostEqual(tp.pose.pose.position.x, 0.5)
        self.assertAlmostEqual(tp.velocity.linear.x, 1.0)

        # Constant velocity phase x/theta = 0.5 + a*t, v/dtheta = 1.0
        tp = current_section.interpolate_with_acceleration(rospy.Time(1.5))
        self.assertAlmostEqual(tp.pose.pose.position.x, 1.0)
        self.assertAlmostEqual(tp.velocity.linear.x, 1.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(2.0))
        self.assertAlmostEqual(tp.pose.pose.position.x, 1.5)
        self.assertAlmostEqual(tp.velocity.linear.x, 1.0)

        # Decceleration phase x/theta = 1.5 + 1.0*t - 0.5*a*t^2, v/dtheta=1.0-a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(2.5))
        self.assertAlmostEqual(tp.pose.pose.position.x, 1.875)
        self.assertAlmostEqual(tp.velocity.linear.x, 0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(3.0))
        self.assertAlmostEqual(tp.pose.pose.position.x, 2.0)
        self.assertAlmostEqual(tp.velocity.linear.x, 0.0)

    def test_interpolate_path_rotation_left(self):
        """ Test interpolation for rotation to the left
        Accelerate at 1 rad/s2 to 1rad/s to the left, keep 1s and deccelerate to standing still
        Total movement should be 2 rad.
        Trajectory is tested at different times
        """
        self._target_x_vel = 1.0
        self._target_x_acc = 1.0
        self._target_yaw_vel = 1.0
        self._target_yaw_acc = 1.0

        path = Path()
        pose1 = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose1.pose.orientation.x = quaternion[0]
        pose1.pose.orientation.y = quaternion[1]
        pose1.pose.orientation.z = quaternion[2]
        pose1.pose.orientation.x = quaternion[3]
        path.poses.append(pose1)

        pose2 = PoseStamped()
        pose2.pose.position.x = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 2.0)
        pose2.pose.orientation.x = quaternion[0]
        pose2.pose.orientation.y = quaternion[1]
        pose2.pose.orientation.z = quaternion[2]
        pose2.pose.orientation.w = quaternion[3]
        path.poses.append(pose2)

        current_section = SectionInterpolation(pose1, pose2, rospy.Time(0.0),
                                               self._target_x_vel, self._target_x_acc,
                                               self._target_yaw_vel, self._target_yaw_acc)

        # Acceleration phase x/theta = 0.5*a*t^2, v/dtheta=a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(0.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertEqual(yaw, 0.0)
        self.assertEqual(tp.velocity.angular.z, 0.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(0.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 0.125)
        self.assertAlmostEqual(tp.velocity.angular.z, 0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(1.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 0.5)
        self.assertAlmostEqual(tp.velocity.angular.z, 1.0)

        # Constant velocity phase x/theta = 0.5 + a*t, v/dtheta = 1.0
        tp = current_section.interpolate_with_acceleration(rospy.Time(1.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 1.0)
        self.assertAlmostEqual(tp.velocity.angular.z, 1.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(2.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 1.5)
        self.assertAlmostEqual(tp.velocity.angular.z, 1.0)

        # Decceleration phase x/theta = 1.5 + 1.0*t - 0.5*a*t^2, v/dtheta=1.0-a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(2.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 1.875)
        self.assertAlmostEqual(tp.velocity.angular.z, 0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(3.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, 2.0)
        self.assertAlmostEqual(tp.velocity.angular.z, 0.0)

    def test_interpolate_path_rotation_right(self):
        """ Test interpolation for rotation to the right
        Accelerate at 1 rad/s2 to 1rad/s to the right, keep 1s and deccelerate to standing still
        Total movement should be -2 rad.
        Trajectory is tested at different times
        """
        self._target_x_vel = 1.0
        self._target_x_acc = 1.0
        self._target_yaw_vel = 1.0
        self._target_yaw_acc = 1.0

        path = Path()
        pose1 = PoseStamped()
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, 0.0)
        pose1.pose.orientation.x = quaternion[0]
        pose1.pose.orientation.y = quaternion[1]
        pose1.pose.orientation.z = quaternion[2]
        pose1.pose.orientation.x = quaternion[3]
        path.poses.append(pose1)

        pose2 = PoseStamped()
        pose2.pose.position.x = 0.0
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, -2.0)
        pose2.pose.orientation.x = quaternion[0]
        pose2.pose.orientation.y = quaternion[1]
        pose2.pose.orientation.z = quaternion[2]
        pose2.pose.orientation.w = quaternion[3]
        path.poses.append(pose2)

        current_section = SectionInterpolation(pose1, pose2, rospy.Time(0.0),
                                               self._target_x_vel, self._target_x_acc,
                                               self._target_yaw_vel, self._target_yaw_acc)

        # Acceleration phase x/theta = 0.5*a*t^2, v/dtheta=a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(0.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertEqual(yaw, 0.0)
        self.assertEqual(tp.velocity.angular.z, 0.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(0.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -0.125)
        self.assertAlmostEqual(tp.velocity.angular.z, -0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(1.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -0.5)
        self.assertAlmostEqual(tp.velocity.angular.z, -1.0)

        # Constant velocity phase x/theta = 0.5 + a*t, v/dtheta = 1.0
        tp = current_section.interpolate_with_acceleration(rospy.Time(1.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -1.0)
        self.assertAlmostEqual(tp.velocity.angular.z, -1.0)

        tp = current_section.interpolate_with_acceleration(rospy.Time(2.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -1.5)
        self.assertAlmostEqual(tp.velocity.angular.z, -1.0)

        # Decceleration phase x/theta = 1.5 + 1.0*t - 0.5*a*t^2, v/dtheta=1.0-a*t
        tp = current_section.interpolate_with_acceleration(rospy.Time(2.5))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -1.875)
        self.assertAlmostEqual(tp.velocity.angular.z, -0.5)

        tp = current_section.interpolate_with_acceleration(rospy.Time(3.0))
        yaw = self.quaternion_to_yaw(tp.pose.pose.orientation)
        self.assertAlmostEqual(yaw, -2.0)
        self.assertAlmostEqual(tp.velocity.angular.z, 0.0)


if __name__ == '__main__':
    unittest.main()
