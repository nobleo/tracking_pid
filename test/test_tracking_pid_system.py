#!/usr/bin/env python
PKG='tracking_pid'

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from tracking_pid.msg import traj_point, PidDebug
import math
import numpy as np
import rospy
import sys
import tf
import unittest


class TestTrackingPID(unittest.TestCase):

    def __init__(self, *args):
        super(TestTrackingPID, self).__init__(*args)

    def setUp(self):
        rospy.init_node("rostest_tracking_pid_node")
        self.trajectory_finished_sub = rospy.Subscriber("trajectory_finished", Bool, self.trajectory_finished_callback, queue_size=1)
        self.listener = tf.TransformListener()
        self.trajectory_finished = False


    def trajectory_finished_callback(self, trajectory_finished_msg):
        rospy.loginfo("Trajectory finished message received on topic")
        self.trajectory_finished = trajectory_finished_msg.data

    def quaternion_to_yaw(self, quat):
        euler = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
        return euler[2]

    def test_tracking_pid(self):
        """ Several checks are done:
        - Test that interpolator point and robot start moving
        - Test that error at all times is bounded
        - Test that after some time final goal is reached
        A path that does not start along the y-axis is expected
        """
        p1_msg=rospy.wait_for_message("trajectory", traj_point, timeout=5)
        self.listener.waitForTransform('/map', '/base_link', rospy.Time(0),rospy.Duration(1.0))
        (trans1,rot1) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        rospy.sleep(0.5)
        p2_msg=rospy.wait_for_message("trajectory", traj_point, timeout=5)
        (trans2,rot2) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
        self.assertNotEqual(p1_msg.pose.pose.position.x, p2_msg.pose.pose.position.x,"Trajectory point has not moved")
        self.assertNotEqual(trans1[0], trans2[0],"Robot has not moved")


        rospy.loginfo("Wait max 30 seconds for reaching goal")
        test_start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - test_start_time < 30.0:
            self.debug_msg=rospy.wait_for_message("debug", PidDebug, timeout=5)
            error_vec = (
                self.debug_msg.error.linear.x,
                self.debug_msg.error.linear.y,
                self.debug_msg.error.linear.z)
            error = np.linalg.norm(error_vec)
            self.assertLess(error , 1.0, "Linear error greater than 1.0 m")
            if self.trajectory_finished == True:
                break

        self.assertTrue(self.trajectory_finished,"Trajectory not finished in 30 seconds")


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'rostest_tracking_pid_node', TestTrackingPID)
