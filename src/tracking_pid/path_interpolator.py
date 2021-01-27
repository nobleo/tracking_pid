#!/usr/bin/env python

"""Accept a ROS navmsgs/Path and publish traj_point to tracking_pid interpolated along the given Path"""

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tracking_pid.cfg import TargetVelocityConfig
from tracking_pid.msg import traj_point, FollowPathAction, FollowPathGoal, FollowPathResult, FollowPathFeedback
from visualization_msgs.msg import Marker
import actionlib
import dynamic_reconfigure.client
import math
import numpy as np
import rospy
import tf

X, Y, Z = 0, 1, 2
axes = {"X": 0, "Y": 1, "Z": 2}


class SectionInterpolation(object):
    """
    SectionInterpolation keeps track and interpolates poses between a start and end PoseStamped, with given
    x and yaw velocities Based on the difference between the start and end pose and the velocities,
    a duration for the section is calculated

    the interpolate method then determines an intermediate point along the section given some progress along the section
    """
    def __init__(self, from_, to, start_time, x_vel, x_acc, yaw_vel, yaw_acc):
        """
        Interpolate over the given section with some x and yaw velocities
        :param from_: start of the section
        :type from_: PoseStamped
        :param to: end of the section
        :type to: PoseStamped
        :param x_vel: translational velocity to move the trajectory target pose
        :type x_vel: float
        :param x_acc: translational acceleration to move the trajectory target pose
        :type x_acc: float
        :param yaw_vel: rotational velocity to rotate the trajectory target pose
        :type yaw_vel: float
        :param yaw_acc: rotational acceleration to rotate the trajectory target pose
        :type yaw_acc: float
        """
        self._x_vel = x_vel  # type: float
        self._x_acc_decc = x_acc
        self._yaw_vel = yaw_vel
        self._yaw_acc_decc = yaw_acc



        self.duration_on_section = rospy.Duration(0)

        self.section_start_pose_stamped = from_  # type: PoseStamped
        self.section_end_pose_stamped = to  # type: PoseStamped

        self._start_xyz = np.array([self.section_start_pose_stamped.pose.position.x,
                                    self.section_start_pose_stamped.pose.position.y,
                                    self.section_start_pose_stamped.pose.position.z])
        self._end_xyz = np.array([self.section_end_pose_stamped.pose.position.x,
                                  self.section_end_pose_stamped.pose.position.y,
                                  self.section_end_pose_stamped.pose.position.z])


        quaternion = (
                    self.section_start_pose_stamped.pose.orientation.x,
                    self.section_start_pose_stamped.pose.orientation.y,
                    self.section_start_pose_stamped.pose.orientation.z,
                    self.section_start_pose_stamped.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self._start_yaw = euler[2]

        quaternion = (
                    self.section_end_pose_stamped.pose.orientation.x,
                    self.section_end_pose_stamped.pose.orientation.y,
                    self.section_end_pose_stamped.pose.orientation.z,
                    self.section_end_pose_stamped.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self._end_yaw = euler[2]

        # Warning! These calculations are only valid for yaw. So not to be used in 3D

        self._delta = self._end_xyz - self._start_xyz
        self._delta_yaw = self._end_yaw - self._start_yaw
        self._delta_yaw = ( self._delta_yaw + np.pi) % (2 * np.pi ) - np.pi



        self.length_of_section = np.linalg.norm(self._delta)
        self.length_of_section_ang = np.linalg.norm(self._delta_yaw)

        self.time_x_acc_decc = self._x_vel/self._x_acc_decc  # Time during (de) and acceleration phases t = v/a
        self.length_x_acc_decc =  0.5*self._x_acc_decc*(self.time_x_acc_decc*self.time_x_acc_decc) # Translation during acceleration phase x = 0.5a*t^2
        self.length_x_vel = self.length_of_section - self.length_x_acc_decc - self.length_x_acc_decc # Translation during constant velocity phase
        self.time_x_vel  = self.length_x_vel/self._x_vel  # Time during constant velocity phase t = v/a
        self._x_vel_adjusted = self._x_vel

        if self.time_x_vel < 0:  # No constant acceleration phase. Recompute (de)-acceleration phase
            self.length_x_acc_decc = 0.5*self.length_of_section
            self.time_x_acc_decc = math.sqrt(2*self.length_x_acc_decc/self._x_acc_decc) # x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
            self._x_vel_adjusted = self._x_acc_decc*self.time_x_acc_decc
            self.length_x_vel = 0.0
            self.time_x_vel = 0.0

        self.time_yaw_acc_decc = self._yaw_vel/self._yaw_acc_decc  # Time during acceleration phase t = v/a
        self.length_yaw_acc_decc =  0.5*self._yaw_acc_decc*(self.time_yaw_acc_decc*self.time_yaw_acc_decc) # Translation during acceleration phase x = 0.5a*t^2
        self.length_yaw_vel = self.length_of_section_ang - self.length_yaw_acc_decc - self.length_yaw_acc_decc # Translation during constant velocity phase
        self.time_yaw_vel  = self.length_yaw_vel/self._yaw_vel  # Time during constant velocity phase t = v/a
        self._yaw_vel_adjusted = self._yaw_vel

        if self.time_yaw_vel < 0:  # No constant acceleration phase. Recompute (de)-acceleration phase
            self.length_yaw_acc_decc = 0.5*self.length_of_section_ang
            self.time_yaw_acc_decc = math.sqrt(2*self.length_yaw_acc_decc/self._yaw_acc_decc) # x = 0.5a*t^2 -> 2x/a = t^2 -> t = sqrt(2x/a)
            self._yaw_vel_adjusted = self._yaw_acc_decc*self.time_yaw_acc_decc
            self.length_yaw_vel = 0.0
            self.time_yaw_vel = 0.0


        self.duration_for_section_x = self.time_x_acc_decc + self.time_x_vel + self.time_x_acc_decc
        self.duration_for_section_yaw = self.time_yaw_acc_decc + self.time_yaw_vel + self.time_yaw_acc_decc
        self.duration_for_section = rospy.Duration(max(self.duration_for_section_x,self.duration_for_section_yaw))

        rospy.logdebug("self._x_acc_decc: %f",self._x_acc_decc)
        rospy.logdebug("self._x_vel: %f",self._x_vel)
        rospy.logdebug("self.length_of_section: %f",self.length_of_section)
        rospy.logdebug("self.time_x_acc_decc: %f",self.time_x_acc_decc)
        rospy.logdebug("self.time_x_vel: %f",self.time_x_vel)
        rospy.logdebug("self.length_x_acc_decc: %f",self.length_x_acc_decc)
        rospy.logdebug("self.length_x_vel: %f",self.length_x_vel)
        rospy.logdebug("self.duration_for_section_x: %f",self.duration_for_section_x)

        rospy.logdebug("self._yaw_acc_decc: %f",self._yaw_acc_decc)
        rospy.logdebug("self._yaw_vel: %f",self._yaw_vel)
        rospy.logdebug("self.length_of_section_ang: %f",self.length_of_section_ang)
        rospy.logdebug("self.time_yaw_acc_decc: %f",self.time_yaw_acc_decc)
        rospy.logdebug("self.time_yaw_vel: %f",self.time_yaw_vel)
        rospy.logdebug("self.length_yaw_acc_decc: %f",self.length_yaw_acc_decc)
        rospy.logdebug("self.length_yaw_vel: %f",self.length_yaw_vel)
        rospy.logdebug("self.duration_for_section_yaw: %f",self.duration_for_section_yaw)


        rospy.logdebug("self.duration_for_section: %f",self.duration_for_section.to_sec())



        self.section_start_time = start_time
        self.x_progress = 0.0
        self.yaw_progress = 0.0
        self.current_x_vel = 0.0
        self.current_yaw_vel = 0.0
        self.section_end_time = self.section_start_time + self.duration_for_section

    def interpolate(self, progress_ratio):
        """
        Calculate where we should be along the section given a ratio of progress.
        0.0 means we're at the start, 1.0 means finished
        :param progress_ratio: How far along the section are we?
        :type progress_ratio: float
        :return: an interpolation between the Section's start and end
        :rtype: PoseStamped
        """
        # target_x = start_x + delta_x * progress_on_section
        next_xyz = self._start_xyz + self._delta * progress_ratio
        next_yaw = self._start_yaw + self._delta_yaw * progress_ratio
        next_pose = PoseStamped()
        # next_pose.header.stamp is to be filled by the caller
        next_pose.header.frame_id = self.section_start_pose_stamped.header.frame_id
        next_pose.pose.position.x = next_xyz[X]
        next_pose.pose.position.y = next_xyz[Y]
        next_pose.pose.position.z = next_xyz[Z]
        # Compute orientation. PID can use it for holonomic robots
        quaternion = tf.transformations.quaternion_from_euler(0, 0, next_yaw)
        next_pose.pose.orientation.x = quaternion[0]
        next_pose.pose.orientation.y = quaternion[1]
        next_pose.pose.orientation.z = quaternion[2]
        next_pose.pose.orientation.w = quaternion[3]



        return next_pose

    def interpolate_with_acceleration(self, current_time):
        """
        Calculate where we should be along the section given a ratio of progress.
        0.0 means we're at the start, 1.0 means finished
        :param progress_ratio: How far along the section are we?
        :type progress_ratio: float
        :return: an interpolation between the Section's start and end
        :rtype: PoseStamped
        """

        current_section_time = (current_time - self.section_start_time)
        t = current_section_time.to_sec()


        if  t < self.time_x_acc_decc:
            tr = t
            self.x_progress =0.5*self._x_acc_decc*tr*tr
            self.current_x_vel = self._x_acc_decc*tr
            if self.x_progress > self.length_x_acc_decc:
                self.current_x_vel = self._x_vel_adjusted
                self.x_progress = self.length_x_acc_decc
        elif t < (self.time_x_acc_decc + self.time_x_vel):
            tr = (t - self.time_x_acc_decc)
            self.x_progress = self.length_x_acc_decc + self.current_x_vel*tr
            self.current_x_vel = self._x_vel_adjusted
            if self.x_progress > (self.length_x_acc_decc+self.length_x_vel):
                self.x_progress = (self.length_x_acc_decc+self.length_x_vel)
        elif t < (self.time_x_acc_decc + self.time_x_vel + self.time_x_acc_decc):
            tr = (t - self.time_x_acc_decc - self.time_x_vel)
            self.x_progress = (self.length_x_acc_decc + self.length_x_vel) + self._x_vel_adjusted*tr - 0.5*self._x_acc_decc*tr*tr
            self.current_x_vel = self._x_vel_adjusted - self._x_acc_decc*tr
            if self.x_progress > self.length_of_section:
                self.current_x_vel = 0.0
                self.x_progress = self.length_of_section
        else:
            self.current_x_vel = 0.0
            self.x_progress = self.length_of_section


        if  t < self.time_yaw_acc_decc:
            tr = t
            self.yaw_progress =0.5*self._yaw_acc_decc*tr*tr
            self.current_yaw_vel = self._yaw_acc_decc*tr
            if self.yaw_progress > self.length_yaw_acc_decc:
                self.current_yaw_vel = self._yaw_vel_adjusted
                self.yaw_progress = self.length_yaw_acc_decc
        elif t < (self.time_yaw_acc_decc + self.time_yaw_vel):
            tr = (t - self.time_yaw_acc_decc)
            self.yaw_progress = self.length_yaw_acc_decc + self.current_yaw_vel*tr
            self.current_yaw_vel = self._yaw_vel_adjusted
            if self.yaw_progress > (self.length_yaw_acc_decc+self.length_yaw_vel):
                self.yaw_progress = (self.length_yaw_acc_decc+self.length_yaw_vel)
        elif t < (self.time_yaw_acc_decc + self.time_yaw_vel + self.time_yaw_acc_decc):
            tr = (t - self.time_yaw_acc_decc - self.time_yaw_vel)
            self.yaw_progress = (self.length_yaw_acc_decc + self.length_yaw_vel) + self._yaw_vel_adjusted*tr - 0.5*self._yaw_acc_decc*tr*tr
            self.current_yaw_vel = self._yaw_vel_adjusted - self._yaw_acc_decc*tr
            if self.yaw_progress > self.length_of_section_ang:
                self.current_yaw_vel = 0.0
                self.yaw_progress = self.length_of_section_ang
        else:
            self.current_yaw_vel = 0.0
            self.yaw_progress = self.length_of_section_ang

        if self.length_of_section > 0:
            x_progress_ratio = self.x_progress/self.length_of_section
        else:
            x_progress_ratio = 1.0

        if self.length_of_section_ang > 0:
            yaw_progress_ratio = self.yaw_progress/self.length_of_section_ang
        else:
            yaw_progress_ratio = 1.0
        # target_x = start_x + delta_x * progress_on_section
        next_xyz = self._start_xyz + self._delta * x_progress_ratio
        next_yaw = self._start_yaw + self._delta_yaw * yaw_progress_ratio
        tp = traj_point()
        tp.pose = PoseStamped()
        # next_pose.header.stamp is to be filled by the caller
        tp.pose.header.frame_id = self.section_start_pose_stamped.header.frame_id
        tp.pose.pose.position.x = next_xyz[X]
        tp.pose.pose.position.y = next_xyz[Y]
        tp.pose.pose.position.z = next_xyz[Z]
        tp.velocity.linear.x = self.current_x_vel
        # Compute orientation. PID can use it for holonomic robots
        quaternion = tf.transformations.quaternion_from_euler(0, 0, next_yaw)
        tp.pose.pose.orientation.x = quaternion[0]
        tp.pose.pose.orientation.y = quaternion[1]
        tp.pose.pose.orientation.z = quaternion[2]
        tp.pose.pose.orientation.w = quaternion[3]
        tp.velocity.angular.z = np.sign(self._delta_yaw) * self.current_yaw_vel




        return tp

    def __repr__(self):
        return "Section(from_={}, to={}, x_vel={})".format(self._start_xyz, self._end_xyz, self._x_vel)

    @property
    def delta(self):
        return self._delta


class InterpolatorNode(object):
    def __init__(self):
        self._path_poses = None

        self.trajectory_pub = rospy.Publisher("trajectory", traj_point, queue_size=1)
        self.enable_srv = rospy.ServiceProxy("enable_control", SetBool)
        self.reconfigure_client = None  # type: dynamic_reconfigure.client.Client

        self._visualization_pub = rospy.Publisher("interpolator_viz", Marker, queue_size=1)
        self._pub_finished = rospy.Publisher("trajectory_finished", Bool, queue_size=1)

        self.robot_frame = 'base_link'

        self.listener = tf.TransformListener()

        self._sections = None  # List[Tuple[PoseStamped]]
        self._current_section = None  # type: SectionInterpolation

        self._rate = rospy.get_param("~rate", 50.0)
        self._timer = None  # type: rospy.Timer

        self._latest_subgoal_pose = None  # type: PoseStamped

        self._paused = False
        self._pause_sub = rospy.Subscriber("pause", Bool, self._process_pause, queue_size=1)

        self._target_x_vel = 1.0 # To be overridden by parameters
        self._target_x_acc = 0.2
        self._target_yaw_vel = 0.1
        self._target_yaw_acc = 0.1

        self._server = Server(TargetVelocityConfig, self._process_velocity)

        self._latest_path_msg = None
        self._path_sub = rospy.Subscriber("path", Path, self._accept_path_from_topic, queue_size=1)
        self._path_pub = rospy.Publisher("path/viz", Path, queue_size=1, latch=True)
        self._as = actionlib.SimpleActionServer("follow_path", FollowPathAction, auto_start=False)
        self._as.register_goal_callback(self._accept_goal)
        self._as.start()

        rospy.loginfo("%s initialized", rospy.get_name())

    def start_path(self):
        rospy.logdebug("start_path()")
        self._timer = rospy.Timer(rospy.Duration(1.0/self._rate), self._update_target)

    def stop_path(self):
        rospy.logdebug("stop_path()")
        self._timer.shutdown()
        self._timer = None

    def continue_path(self, start_time=None):
        # if we're un-paused and we were busy with a section: we resume the path from the last goal send:
        if not self._paused and self._current_section and self._latest_subgoal_pose:
            # When we're no longer paused, continue where we left off
            # This is: from the latest sent subgoal, to the same finish point of the current section
            # With the same velocity so the end time will be re-calculated based on the start_time
            rospy.loginfo("Resuming path with velocities %0.3f m/s %0.3f rad/s and accelerations %0.3f m/s2 %0.3f rad/s2", self._target_x_vel, self._target_yaw_vel, self._target_x_acc, self._target_yaw_acc)

            if start_time is None:
                start_time = self._latest_subgoal_pose.header.stamp

            self._current_section = SectionInterpolation(self._latest_subgoal_pose,
                                                         self._current_section.section_end_pose_stamped,
                                                         start_time,
                                                         self._target_x_vel, self._target_x_acc,
                                                         self._target_yaw_vel, self._target_yaw_acc)

    def _process_pause(self, bool_msg):
        if bool_msg.data and bool_msg.data != self._paused:
            rospy.loginfo("Pausing path_interpolator")
            rospy.logwarn("No acceleration limits implemented when pausing!")
            self._paused = bool_msg.data
        elif not bool_msg.data and bool_msg.data != self._paused:
            rospy.loginfo("Unpausing path_interpolator")
            resume_time = rospy.Time.now() - rospy.Duration(1.0 / self._rate)  # Prevent sending last goal again
            self._paused = bool_msg.data
            self.continue_path(start_time=resume_time)

    def _process_velocity(self, config, _):
        target_x_vel = config.target_x_vel
        target_yaw_vel = config.target_yaw_vel

        if target_x_vel == 0.0 or target_yaw_vel == 0.0:
            rospy.logwarn("Ignoring ~target_x_vel of {}, ~target_yaw_vel of {}, keeping {}, {}, consider using the pause function".format(target_x_vel, target_yaw_vel,self._target_x_vel, self._target_yaw_vel))
        else:
            self._target_x_vel = target_x_vel
            self._target_x_acc = config.target_x_acc
            self._target_yaw_vel = target_yaw_vel
            self._target_yaw_acc = config.target_yaw_acc


        self.continue_path()
        return config

    def _accept_path_from_topic(self, path_msg):
        """
        When receiving a path on a topic, just call our own action server!
        This ensures canceling/aborting/preemption etc behave as they should
        :return: None
        """
        rospy.loginfo("Path received on topic, calling action to execute")
        client = actionlib.SimpleActionClient("follow_path", FollowPathAction)
        client.wait_for_server()
        client.send_goal(FollowPathGoal(path=path_msg))

    def _accept_goal(self):
        rospy.logdebug("Accept goal")
        goal = self._as.accept_new_goal()
        self._process_path(goal.path)

    def _preempt_goal(self):
        rospy.logdebug("Preempt goal")
        self.stop_path()
        self._path_poses = None
        self._sections = None
        self._current_section = None
        self._as.set_preempted()

    def _process_path(self, path_msg):
        """Accept and store the path"""
        rospy.logdebug("_process_path(...). Path has {} poses".format(len(path_msg.poses)))
        # Assert that all poses are defined in the same frame
        # TODO: it would be nice to interpolate between poses defines in different frames but that is out of scope
        # TODO: Tracking_pid completely disregards the frame_id and just does everything in the frame
        #   it was configured for, even though it listens to Pose*Stamped*s

        if not path_msg.poses:
            rospy.logwarn("There are no poses in the given path with header {}".format(path_msg.header))
            return

        # If empty frame_ids are supplied, use the global headers frame_id
        undefined_poses = (p for p in path_msg.poses if not p.header.frame_id)

        for pose in undefined_poses:
            pose.header.frame_id = path_msg.header.frame_id

        # All the header_ids are the same, so set cardinality is 1
        # TODO: Fails for /map and map
        # assert(len(set(pose.header.frame_id for pose in path_msg.poses)) == 1)

        self._path_pub.publish(path_msg)

         # Path is valid, so lets store it
        self._latest_path_msg = path_msg

        target_x_vel = rospy.get_param("~target_x_vel", 1.0)
        target_x_acc = rospy.get_param("~target_x_acc", 1.0)
        target_yaw_vel = rospy.get_param("~target_yaw_vel", 1.0)
        target_yaw_acc = rospy.get_param("~target_yaw_acc", 1.0)
        if target_x_vel == 0.0 or target_yaw_vel == 0.0:
            rospy.logwarn("Ignoring ~target_x_vel of {}, ~target_yaw_vel of {}, keeping {}, {}, consider using the pause function".format(target_x_vel, target_yaw_vel, self._target_x_vel, self._target_yaw_vel))
        else:
            self._target_x_vel = target_x_vel
            self._target_x_acc = target_x_acc
            self._target_yaw_vel = target_yaw_vel
            self._target_yaw_acc = target_yaw_acc

        self.flip_for_axis = rospy.get_param("~flip_for_axis", None)
        if self.flip_for_axis:
            assert self.flip_for_axis in ["X", "Y", "Z"]

        if self.flip_for_axis and not self.reconfigure_client:
            self.reconfigure_client = dynamic_reconfigure.client.Client("controller", timeout=1)

        self._path_poses = path_msg.poses
        self._sections = list(zip(self._path_poses, self._path_poses[1:]))

        if not self._timer:
            self.start_path()

    def _update_target(self, event):
        """
        Called by self._timer and determines & publishes a interpolated pose along the received Path

        :param event: supplied by rospy.Timer, the time at which the callback method is called
        :type event: rospy.TimerEvent (see https://wiki.ros.org/rospy/Overview/Time#Timer)
        :return:
        """
        # event is supplied by the timer and contains info like the current time and time since last tick
        # based on the time, the start time of the subsection and the target_velocities, we can calculate where the target should be:
        # >>> duration_on_section = (current_time - section_start_time)
        # >>> duration_for_section = (length_of_section / target_(x_yaw)velocity)
        # >>> progress_on_section = (duration_on_section / duration_for_section)
        # >>> target_x = start_x + delta_x * progress_on_section

        # Service preempt before other early-outs
        if self._as.is_preempt_requested():
            self._preempt_goal()
            return

        if not self._path_poses:
            rospy.logdebug_throttle(1.0, "No path poses set")
            return

        if self._paused:
            rospy.logdebug_throttle(5.0, "Path_interpolator is paused")
            return

        if not self._current_section or rospy.Time.now() > self._current_section.section_end_time:  # or when past end time of current section, go to next
            try:
                start, end = self._sections.pop(0)
                self._current_section = SectionInterpolation(start, end, event.current_real, self._target_x_vel, self._target_x_acc, self._target_yaw_vel, self._target_yaw_acc)
                rospy.loginfo("Starting new {}. duration_for_section = {}".format(self._current_section, self._current_section.duration_for_section.to_sec()))

                if self.flip_for_axis:
                    # Have the control point in front if we are to drive forwards, control point in the back when driving backwards
                    # This makes the robot always keep point up, for Emma that always needs to keep pointing up
                    self._set_controller_direction(sign=np.sign(self._current_section.delta[axes[self.flip_for_axis]]))
            except IndexError:
                rospy.logdebug("Path ended")

                loop = rospy.get_param("~loop", 0)
                if loop != 0:
                    rospy.loginfo("~loop = {}, starting path again".format(loop))
                    rospy.set_param("~loop", loop-1)
                    self._process_path(self._latest_path_msg)
                else:
                    rospy.logdebug("No loop requested or remaining, finishing up")
                    self._pub_finished.publish(True)
                    if self._as.is_active():
                        self._as.set_succeeded()
                    self.stop_path()
                return

        duration_on_section = event.current_real - self._current_section.section_start_time

        # Distance between duplicated poses is 0, so we can't do the division below.
        # If the duration is 0, then we're done immediately.
        # This will still be valid when this node starts taking rotation and angular velocity into account
        # Then, even when the pose.position does not change while pose.orientation does change,
        #  the angular velocity will make the section have a nonzero duration
        if not self._current_section.duration_for_section.is_zero():
            progress_on_section = (duration_on_section / self._current_section.duration_for_section)
        else:
            rospy.loginfo("Instantaneous completion of 0-length section")
            progress_on_section = 1

        tp = self._current_section.interpolate_with_acceleration(rospy.Time.now())
        tp.pose.header.stamp = event.current_real


        # TODO: Rotate in the corners, using controller mode 3 tp.controller.data = 3

        # Remember the last interpolated sub-goal on our way to the next waypoint in the Path
        self._latest_subgoal_pose = tp.pose

        self.__publish_marker(tp.pose)
        self.trajectory_pub.publish(tp)

    def __publish_marker(self, pose_stamped):
        """
        Indicate what the current interpolated goal along the path is
        :param pose_stamped:
        :return:
        """
        m = Marker()
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 1.0
        m.pose = pose_stamped.pose
        m.header = pose_stamped.header
        m.type = Marker.SPHERE
        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2
        m.action = Marker.ADD
        m.ns = "interpolated"
        self._visualization_pub.publish(m)

    def _set_controller_direction(self, sign):
        """
        Flip the sign of the tracking_pid controller
        :param sign: optional. If *not* given, the sign of the control distance is flipped. If given, the given sign is applied, regardless of the current sign
        """

        rospy.loginfo("Flipping control point distance. Given sign: {}".format(sign))
        self.enable_srv(False)
        # Query the controller configuration to retrieve current 'l' parameter (control distance/length)
        controller_config = self.reconfigure_client.get_configuration()

        if controller_config is None:
            rospy.logfatal("Could not get config to update. Incorrect controller settings are dangerous, exiting to stop driving")
            exit(-1)
        else:
            new_l = abs(controller_config['l']) * sign

            rospy.logdebug("New control point distance: {}".format(new_l))
            self.reconfigure_client.update_configuration({"l": new_l})

            self.enable_srv(True)
