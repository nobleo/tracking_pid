// Subscribe to a topic about the state of a dynamic system and calculate feedback to
// stabilize it.

#include "controller/controller_node.h"
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>


void publishMarkers()
{
  p.x = tfCurPose.translation.x;
  p.y = tfCurPose.translation.y;
  p.z = tfCurPose.translation.z;
  mkCurPose.points.push_back(p);

  p.x = tfControlPose.getOrigin().x();
  p.y = tfControlPose.getOrigin().y();
  p.z = tfControlPose.getOrigin().z();
  mkControlPose.points.push_back(p);

  p.x = tfGoalPose.getOrigin().x();
  p.y = tfGoalPose.getOrigin().y();
  p.z = tfGoalPose.getOrigin().z();
  mkGoalPose.points[0] = p;

  marker_pub.publish(mkCurPose);
  marker_pub.publish(mkControlPose);
  marker_pub.publish(mkGoalPose);
}

bool enableCallback(std_srvs::SetBool::Request& req,   // NOLINT
                    std_srvs::SetBool::Response& res)  // NOLINT
{
  controller_enabled = req.data;
  waiting_for_setpoint = false;
  // TODO(Nobleo): Add wait dfor new Setp
  pid_controller.setEnabled(controller_enabled);
  res.success = true;
  if (controller_enabled)
    res.message = "Controller enabled";
  else
    res.message = "Controller disabled";
  return true;
}

bool enableAndWaitCallback(std_srvs::SetBool::Request& req,   // NOLINT
                           std_srvs::SetBool::Response& res)  // NOLINT
{
  controller_enabled = req.data;
  waiting_for_setpoint = true;
  // TODO(Nobleo): Add wait dfor new Setp
  pid_controller.setEnabled(controller_enabled);
  res.success = true;
  if (controller_enabled)
    res.message = "Controller enabled. Wait for Trajectory";
  else
    res.message = "Controller disabled";
  return true;
}

void trajectory_callback(const tracking_pid::traj_point& goalPointMsg)
{
  goalPoint = goalPointMsg;
  try
  {
    goalPoint.pose = tf_buffer.transform(goalPoint.pose, map_frame);
    waiting_for_setpoint =  false;
  }
  catch (tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }
  tfGoalPose = tf::Transform(
          tf::Quaternion(goalPoint.pose.pose.orientation.x, goalPoint.pose.pose.orientation.y,
                        goalPoint.pose.pose.orientation.z, goalPoint.pose.pose.orientation.w),
          tf::Vector3(goalPoint.pose.pose.position.x, goalPoint.pose.pose.position.y, goalPoint.pose.pose.position.z));
  if (goalPoint.controller.data != controlType)
  {
    controlType = goalPoint.controller.data;
    pid_controller.selectMode((tracking_pid::ControllerMode)controlType);
  }
}

void poseCallback()
{
  try
  {
    tf_buffer.canTransform(base_link_frame, map_frame, ros::Time(0), ros::Duration(10.0));
  }
  catch (tf2::TransformException ex)
  {
    ROS_ERROR("Received an exception trying to transform: %s", ex.what());
  }

  // calculate delta_t
  if (!prev_time.isZero())  // Not first time through the program
  {
    delta_t = ros::Time::now() - prev_time;
    prev_time = ros::Time::now();
    if (0 == delta_t.toSec())
    {
      ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu at time: %f", ros::Time::now().toSec());
      return;
    }
  }
  else
  {
    ROS_DEBUG("prev_time is 0, doing nothing");
    prev_time = ros::Time::now();
    return;
  }

  if (!waiting_for_setpoint)
  {
    geometry_msgs::Twist cmd_vel;
    tracking_pid::PidDebug pid_debug;
    geometry_msgs::TransformStamped tfCurPoseStamped;
    tfCurPoseStamped = tf_buffer.lookupTransform(map_frame, base_link_frame, ros::Time(0));
    tfCurPose = tfCurPoseStamped.transform;
    cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, delta_t, &pid_debug);

    if (controller_enabled)
    {
      control_effort_pub.publish(cmd_vel);
    }

    if (controller_debug_enabled)
    {
      debug_pub.publish(pid_debug);
    }
  }
}

void reconfigure_callback(const tracking_pid::PidConfig& config)
{
  pid_controller.configure(config);
  controller_debug_enabled = config.controller_debug_enabled;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "pid_controller");
  ros::NodeHandle node;
  ros::NodeHandle node_priv("~");

  // Get params if specified in launch file or as params on command-line, set defaults
  node_priv.param<std::string>("map_frame", map_frame, "map");
  node_priv.param<std::string>("base_link_frame", base_link_frame, "base_link");
  node_priv.param<bool>("holonomic_robot", holonomic_robot, false);
  pid_controller.setHolonomic(holonomic_robot);
  node_priv.param<bool>("track_base_link", track_base_link, false);
  pid_controller.setTrackBaseLink(track_base_link);
  node_priv.param<bool>("enabled_on_boot", enabled_on_boot, true);
  controller_enabled = enabled_on_boot;
  waiting_for_setpoint = true;

  // instantiate publishers & subscribers
  control_effort_pub = node.advertise<geometry_msgs::Twist>("move_base/cmd_vel", 1);
  sub_trajectory = node.subscribe("local_trajectory", 1, trajectory_callback);
  enable_service = node.advertiseService("enable_control", enableCallback);
  enable_and_wait_service = node.advertiseService("enable_control_and_wait", enableAndWaitCallback);

  marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  debug_pub = node.advertise<tracking_pid::PidDebug>("debug", 1);

  // configure dynamic reconfiguration
  dynamic_reconfigure::Server<tracking_pid::PidConfig> config_server;
  dynamic_reconfigure::Server<tracking_pid::PidConfig>::CallbackType f;
  f = boost::bind(&reconfigure_callback, _1);
  config_server.setCallback(f);

  // configure rviz visualization
  mkCurPose.header.frame_id = mkControlPose.header.frame_id = mkGoalPose.header.frame_id = map_frame;
  mkCurPose.header.stamp = mkControlPose.header.stamp = mkGoalPose.header.stamp = ros::Time::now();
  mkCurPose.ns = "axle point";
  mkControlPose.ns = "control point";
  mkGoalPose.ns = "goal point";
  mkCurPose.action = mkControlPose.action = mkGoalPose.action = visualization_msgs::Marker::ADD;
  mkCurPose.pose.orientation.w = mkControlPose.pose.orientation.w = mkGoalPose.pose.orientation.w = 1.0;
  mkCurPose.id = 0;
  mkControlPose.id = 1;
  mkGoalPose.id = 2;
  mkCurPose.type = mkControlPose.type = mkGoalPose.type = visualization_msgs::Marker::POINTS;
  mkCurPose.scale.x = 0.1;
  mkCurPose.scale.y = 0.1;
  mkControlPose.scale.x = 0.02;
  mkControlPose.scale.y = 0.02;
  mkGoalPose.scale.x = 0.1;
  mkGoalPose.scale.y = 0.1;
  mkCurPose.color.b = 1.0;
  mkCurPose.color.a = 1.0;
  mkControlPose.color.g = 1.0f;
  mkControlPose.color.a = 1.0;
  mkGoalPose.color.r = 1.0;
  mkGoalPose.color.a = 1.0;
  mkGoalPose.points.resize(1);

  tf2_ros::TransformListener tf_listener(tf_buffer);

  double loop_rate;
  node_priv.param("loop_rate", loop_rate, 20.0);
  ros::Rate rate(loop_rate);

  // Wait for frames to be available before starting node
  while (!tf_buffer.canTransform(base_link_frame,  map_frame, ros::Time(0), ros::Duration(10.0)))
    ROS_INFO("Waiting for transform between %s and %s", map_frame.c_str(), base_link_frame.c_str());
  ROS_INFO("Transform between %s and %s found", map_frame.c_str(), base_link_frame.c_str());

  while (ros::ok())
  {
    // Get current robot pose and spin controller
    poseCallback();

    // Publish some poses as markers to be visualized in Rviz
    publishMarkers();

    // Spin
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
