#!/usr/bin/env python

"""Accept a ROS navmsgs/Path and publish traj_point to tracking_pid interpolated along the given Path"""

import rospy
from tracking_pid.path_interpolator import InterpolatorNode


if __name__ == "__main__":
    rospy.init_node("path_interpolator")
    try:
        interpolator_node = InterpolatorNode()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
