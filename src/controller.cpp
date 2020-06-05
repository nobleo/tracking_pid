//
// Created by nobleo on 11-9-18.
//

#include "controller/controller.h"

namespace tracking_pid
{
Controller::Controller()
{
  holonomic_robot_enable = false;
}

geometry_msgs::Twist Controller::update(const geometry_msgs::Transform current, const tf::Transform goal,
                                        const ros::Duration sample_time, tracking_pid::PidDebug* pid_debug)
{
  // Compute location of the point to be controlled
  double theta_rp = tf::getYaw(current.rotation);
  tf::Vector3 newOrigin;
  newOrigin.setX(current.translation.x + l * cos(theta_rp));
  newOrigin.setY(current.translation.y + l * sin(theta_rp));
  newOrigin.setZ(0);
  tf::Transform newCurrent;
  newCurrent.setOrigin(newOrigin);
  tf::Quaternion cur_rot(current.rotation.x, current.rotation.y, current.rotation.z, current.rotation.w);
  newCurrent.setRotation(cur_rot);

  tf::Transform newGoal;
  if (track_base_link_enabled)
  {
    double theta_goal = tf::getYaw(goal.getRotation());
    tf::Vector3 newGoalOrigin;
    newGoalOrigin.setX(goal.getOrigin().x() + l * cos(theta_goal));
    newGoalOrigin.setY(goal.getOrigin().y() + l * sin(theta_goal));
    newGoalOrigin.setZ(0);
    newGoal.setOrigin(newGoalOrigin);
    newGoal.setRotation(goal.getRotation());
  }
  else
  {
    newGoal = goal;
  }
  // Compute errorPose between controlPose and goalPose
  tf::Transform error = newCurrent.inverseTimes(newGoal);

  // Populate debug.raw_error output (used by boat_controller)
  pid_debug->raw_error.linear.x = error.getOrigin().x();
  pid_debug->raw_error.linear.y = error.getOrigin().y();
  pid_debug->raw_error.linear.z = 0;
  pid_debug->raw_error.angular.x = 0;
  pid_debug->raw_error.angular.y = 0;
  pid_debug->raw_error.angular.z = atan2(error.getOrigin().y(), error.getOrigin().x());

  //***** Feedback control *****//
  if (!((Kp_long <= 0. && Ki_long <= 0. && Kd_long <= 0.) ||
        (Kp_long >= 0. && Ki_long >= 0. && Kd_long >= 0.)))  // All 3 gains should have the same sign
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  if (!((Kp_lat <= 0. && Ki_lat <= 0. && Kd_lat <= 0.) ||
        (Kp_lat >= 0. && Ki_lat >= 0. && Kd_lat >= 0.)))  // All 3 gains should have the same sign
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");
  if (!((Kp_ang <= 0. && Ki_ang <= 0. && Kd_ang <= 0.) ||
        (Kp_ang >= 0. && Ki_ang >= 0. && Kd_ang >= 0.)))  // All 3 gains should have the same sign
    ROS_WARN("All three gains (Kp, Ki, Kd) should have the same sign for stability.");

  error_long.at(2) = error_long.at(1);
  error_long.at(1) = error_long.at(0);
  error_long.at(0) = error.getOrigin().x();  // Current error goes to slot 0
  error_lat.at(2) = error_lat.at(1);
  error_lat.at(1) = error_lat.at(0);
  error_lat.at(0) = error.getOrigin().y();  // Current error goes to slot 0
  error_ang.at(2) = error_ang.at(1);
  error_ang.at(1) = error_ang.at(0);


  tf::Transform origin;
  tf::transformMsgToTF(current, origin);
  tf::Transform error_ang_tf = origin.inverseTimes(newGoal);
  // Current error goes to slot 0
  error_ang.at(0) = angles::normalize_angle(atan2(error_ang_tf.getOrigin().y(), error_ang_tf.getOrigin().x()));


  // Populate debug output
  pid_debug->error.linear.x = error_long.at(0);
  pid_debug->error.linear.y = error_lat.at(0);
  pid_debug->error.angular.z = error_ang.at(0);

  // integrate the error
  error_integral_long += error_long.at(0) * sample_time.toSec();
  error_integral_lat += error_lat.at(0) * sample_time.toSec();
  error_integral_ang += error_ang.at(0) * sample_time.toSec();

  // Apply windup limit to limit the size of the integral term
  if (error_integral_long > fabsf(windup_limit))
    error_integral_long = fabsf(windup_limit);
  if (error_integral_long < -fabsf(windup_limit))
    error_integral_long = -fabsf(windup_limit);
  if (error_integral_lat > fabsf(windup_limit))
    error_integral_lat = fabsf(windup_limit);
  if (error_integral_lat < -fabsf(windup_limit))
    error_integral_lat = -fabsf(windup_limit);
  if (error_integral_ang > fabsf(windup_limit))
    error_integral_ang = fabsf(windup_limit);
  if (error_integral_ang < -fabsf(windup_limit))
    error_integral_ang = -fabsf(windup_limit);

  // My filter reference was Julius O. Smith III, Intro. to Digital Filters With Audio Applications.
  if (cutoff_frequency_long != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_long * 6.2832) * sample_time.toSec() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_long = 1 / tan_filt;
  }
  if (cutoff_frequency_lat != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_lat * 6.2832) * sample_time.toSec() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_lat = 1 / tan_filt;
  }
  if (cutoff_frequency_ang != -1)
  {
    // Check if tan(_) is really small, could cause c = NaN
    tan_filt = tan((cutoff_frequency_ang * 6.2832) * sample_time.toSec() / 2);

    // Avoid tan(0) ==> NaN
    if ((tan_filt <= 0.) && (tan_filt > -0.01))
      tan_filt = -0.01;
    if ((tan_filt >= 0.) && (tan_filt < 0.01))
      tan_filt = 0.01;

    c_ang = 1 / tan_filt;
  }

  filtered_error_long.at(2) = filtered_error_long.at(1);
  filtered_error_long.at(1) = filtered_error_long.at(0);
  filtered_error_long.at(0) = (1 / (1 + c_long * c_long + 1.414 * c_long)) *
                              (error_long.at(2) + 2 * error_long.at(1) + error_long.at(0) -
                               (c_long * c_long - 1.414 * c_long + 1) * filtered_error_long.at(2) -
                               (-2 * c_long * c_long + 2) * filtered_error_long.at(1));

  filtered_error_lat.at(2) = filtered_error_lat.at(1);
  filtered_error_lat.at(1) = filtered_error_lat.at(0);
  filtered_error_lat.at(0) =
    (1 / (1 + c_lat * c_lat + 1.414 * c_lat)) * (error_lat.at(2) + 2 * error_lat.at(1) + error_lat.at(0) -
        (c_lat * c_lat - 1.414 * c_lat + 1) * filtered_error_lat.at(2) -
        (-2 * c_lat * c_lat + 2) * filtered_error_lat.at(1));

  filtered_error_ang.at(2) = filtered_error_ang.at(1);
  filtered_error_ang.at(1) = filtered_error_ang.at(0);
  filtered_error_ang.at(0) =
    (1 / (1 + c_ang * c_ang + 1.414 * c_ang)) * (error_ang.at(2) + 2 * error_ang.at(1) + error_ang.at(0) -
        (c_ang * c_ang - 1.414 * c_ang + 1) * filtered_error_ang.at(2) -
        (-2 * c_ang * c_ang + 2) * filtered_error_ang.at(1));

  // Take derivative of error, first the raw unfiltered data:
  error_deriv_long.at(2) = error_deriv_long.at(1);
  error_deriv_long.at(1) = error_deriv_long.at(0);
  error_deriv_long.at(0) = (error_long.at(0) - error_long.at(1)) / sample_time.toSec();
  filtered_error_deriv_long.at(2) = filtered_error_deriv_long.at(1);
  filtered_error_deriv_long.at(1) = filtered_error_deriv_long.at(0);
  filtered_error_deriv_long.at(0) = (1 / (1 + c_long * c_long + 1.414 * c_long)) *
                                    (error_deriv_long.at(2) + 2 * error_deriv_long.at(1) + error_deriv_long.at(0) -
                                     (c_long * c_long - 1.414 * c_long + 1) * filtered_error_deriv_long.at(2) -
                                     (-2 * c_long * c_long + 2) * filtered_error_deriv_long.at(1));

  error_deriv_lat.at(2) = error_deriv_lat.at(1);
  error_deriv_lat.at(1) = error_deriv_lat.at(0);
  error_deriv_lat.at(0) = (error_lat.at(0) - error_lat.at(1)) / sample_time.toSec();
  filtered_error_deriv_lat.at(2) = filtered_error_deriv_lat.at(1);
  filtered_error_deriv_lat.at(1) = filtered_error_deriv_lat.at(0);
  filtered_error_deriv_lat.at(0) = (1 / (1 + c_lat * c_lat + 1.414 * c_lat)) *
                                   (error_deriv_lat.at(2) + 2 * error_deriv_lat.at(1) + error_deriv_lat.at(0) -
                                    (c_lat * c_lat - 1.414 * c_lat + 1) * filtered_error_deriv_lat.at(2) -
                                    (-2 * c_lat * c_lat + 2) * filtered_error_deriv_lat.at(1));

  error_deriv_ang.at(2) = error_deriv_ang.at(1);
  error_deriv_ang.at(1) = error_deriv_ang.at(0);
  error_deriv_ang.at(0) = (error_ang.at(0) - error_ang.at(1)) / sample_time.toSec();
  filtered_error_deriv_ang.at(2) = filtered_error_deriv_ang.at(1);
  filtered_error_deriv_ang.at(1) = filtered_error_deriv_ang.at(0);
  filtered_error_deriv_ang.at(0) = (1 / (1 + c_ang * c_ang + 1.414 * c_ang)) *
                                   (error_deriv_ang.at(2) + 2 * error_deriv_ang.at(1) + error_deriv_ang.at(0) -
                                    (c_ang * c_ang - 1.414 * c_ang + 1) * filtered_error_deriv_ang.at(2) -
                                    (-2 * c_ang * c_ang + 2) * filtered_error_deriv_ang.at(1));

  // calculate the control effort
  proportional_long = Kp_long * filtered_error_long.at(0);
  integral_long = Ki_long * error_integral_long;
  derivative_long = Kd_long * filtered_error_deriv_long.at(0);

  proportional_lat = Kp_lat * filtered_error_lat.at(0);
  integral_lat = Ki_lat * error_integral_lat;
  derivative_lat = Kd_lat * filtered_error_deriv_lat.at(0);

  proportional_ang = Kp_ang * filtered_error_ang.at(0);
  integral_ang = Ki_ang * error_integral_ang;
  derivative_ang = Kd_ang * filtered_error_deriv_ang.at(0);

  // Populate debug output
  pid_debug->proportional.linear.x = proportional_long;
  pid_debug->proportional.linear.y = proportional_lat;
  pid_debug->proportional.angular.z = proportional_ang;

  pid_debug->integral.linear.x = integral_long;
  pid_debug->integral.linear.y = integral_lat;
  pid_debug->integral.angular.z = integral_ang;

  pid_debug->derivative.linear.x = derivative_long;
  pid_debug->derivative.linear.y = derivative_lat;
  pid_debug->derivative.angular.z = derivative_ang;

  //  //***** Feedforward control *****//
  //  // Transform trajectory velocities from map frame to control-point frame
  //  theta_cp = tf::getYaw(tfControlPose.getRotation());
  //  xvel = cos(theta_cp) * goalPoint.velocity.x + sin(theta_cp) * goalPoint.velocity.y;
  //  yvel = -sin(theta_cp) * goalPoint.velocity.x + cos(theta_cp) * goalPoint.velocity.y;
  //  thvel = goalPoint.velocity.z;
  //  feedforward_long = xvel;
  //  feedforward_lat = yvel;
  //  feedforward_ang = thvel;

  // Populate debug output
  pid_debug->feedforward.linear.x = feedforward_long;
  pid_debug->feedforward.linear.y = feedforward_lat;
  pid_debug->feedforward.angular.z = feedforward_ang;

  //***** Overall control *****//
  // Controller logic && overall control effort
  control_effort_long = 0;
  control_effort_lat = 0;
  control_effort_ang = 0;
  if (feedback_long_enabled)
    control_effort_long = control_effort_long + proportional_long + integral_long + derivative_long;
  if (feedforward_long_enabled)
    control_effort_long = control_effort_long + feedforward_long;
  if (feedback_lat_enabled)
    control_effort_lat = control_effort_lat + proportional_lat + integral_lat + derivative_lat;
  if (feedforward_lat_enabled)
    control_effort_lat = control_effort_lat + feedforward_lat;
  if (feedback_ang_enabled)
    control_effort_ang = control_effort_ang + proportional_ang + integral_ang + derivative_ang;
  if (feedforward_ang_enabled)
    control_effort_ang = control_effort_ang + feedforward_ang;

  // Apply saturation limits
  if (control_effort_long > upper_limit)
    control_effort_long = upper_limit;
  else if (control_effort_long < lower_limit)
    control_effort_long = lower_limit;

  if (control_effort_lat > upper_limit)
    control_effort_lat = upper_limit;
  else if (control_effort_lat < lower_limit)
    control_effort_lat = lower_limit;

  if (control_effort_ang > ang_upper_limit)
    control_effort_ang = ang_upper_limit;
  else if (control_effort_ang < ang_lower_limit)
    control_effort_ang = ang_lower_limit;


  // Couple angular loop with forwards loop
  if (coupling_ang_long_enabled)
  {
    double scale_long_control_1min = (fabs(error_ang.at(0)) - dead_zone_yaw_error_cal)
                                     / (max_yaw_error_cal - dead_zone_yaw_error_cal);
    if (scale_long_control_1min < 0.0) scale_long_control_1min = 0.0;
    if (scale_long_control_1min > 1.0) scale_long_control_1min = 1.0;
    scale_long_control = 1.0 - scale_long_control_1min;
  }
  else
  {
    scale_long_control = 1.0;
  }


  pid_debug->scale_long_control = scale_long_control;

  geometry_msgs::Twist output_combined;
  // Generate twist message
  if (holonomic_robot_enable)
  {
    output_combined.linear.x = scale_long_control * control_effort_long;
    output_combined.linear.y = control_effort_lat;
    output_combined.linear.z = 0;
    output_combined.angular.x = 0;
    output_combined.angular.y = 0;
    output_combined.angular.z = control_effort_ang;
  }
  else
  {
    output_combined.linear.x = scale_long_control * control_effort_long;
    output_combined.linear.y = 0;
    output_combined.linear.z = 0;
    output_combined.angular.x = 0;
    output_combined.angular.y = 0;
    output_combined.angular.z =
      copysign(1.0, l) * control_effort_ang;  // Take the sign of l for the lateral control effort
  }

  // Publish control effort if controller enabled
  if (!enabled)  // Do nothing and reset integral action
  {
    error_integral_long = 0.0;
    error_integral_lat = 0.0;
    error_integral_ang = 0.0;
  }
  // ROS_INFO("errors (in cm/deg): (%.2f, %.2f, %.2f)", error_x*100, error_y*100,error_th);
  return output_combined;
}

geometry_msgs::Twist Controller::update(const geometry_msgs::Transform current, const geometry_msgs::Pose goal,
                                        const ros::Duration sample_time, tracking_pid::PidDebug* pid_debug)
{
  tf::Pose tfGoal;
  tf::poseMsgToTF(goal, tfGoal);
  return update(current, (tf::Transform)tfGoal, sample_time, pid_debug);
}

void Controller::selectMode(ControllerMode mode)
{
  switch (mode)
  {
  case ControllerMode::frontAxleLateral:
    // Front axle lateral controller (default)
    l = 0.5;
    feedback_long_enabled = true;
    feedback_lat_enabled = true;
    feedback_ang_enabled = false;
    feedforward_long_enabled = true;
    feedforward_lat_enabled = true;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::rearAxleLateral:
    // Rear axle lateral control
    l = 0.0;  // To prevent singular configuration
    feedback_long_enabled = true;
    feedback_lat_enabled = true;
    feedback_ang_enabled = false;
    feedforward_long_enabled = true;
    feedforward_lat_enabled = true;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::rearAxleAngular:
    // Rear axle angular controller
    l = 0.0;
    feedback_long_enabled = true;
    feedback_lat_enabled = false;
    feedback_ang_enabled = true;
    feedforward_long_enabled = false;
    feedforward_lat_enabled = false;
    feedforward_ang_enabled = false;
    break;
  case ControllerMode::fixOrientation:
    // Fix orientation controller
    l = 0.0;
    feedback_long_enabled = false;
    feedback_lat_enabled = false;
    feedback_ang_enabled = true;
    feedforward_long_enabled = false;
    feedforward_lat_enabled = false;
    feedforward_ang_enabled = true;
    break;
  }

  printParameters();
}

void Controller::printParameters()
{
  ROS_INFO("CONTROLLER PARAMETERS");
  ROS_INFO("-----------------------------------------");
  ROS_INFO("Controller enabled: %i", enabled);
  ROS_INFO("Controller DEBUG enabled: %i", debug_enabled);
  ROS_INFO("Distance L: %f", l);
  ROS_INFO("Feedback (long, lat, ang): (%i, %i, %i)", feedback_long_enabled, feedback_lat_enabled,
           feedback_ang_enabled);
  ROS_INFO("Feedforward (long, lat, ang): (%i, %i, %i)", feedforward_long_enabled, feedforward_lat_enabled,
           feedforward_ang_enabled);
  ROS_INFO("Longitudinal gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_long, Ki_long, Kd_long);
  ROS_INFO("Lateral gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_lat, Ki_lat, Kd_lat);
  ROS_INFO("Angular gains: (Kp: %f, Ki, %f, Kd, %f)", Kp_ang, Ki_ang, Kd_ang);

  ROS_INFO("Robot type (holonomic): (%i)", holonomic_robot_enable);

  ROS_INFO("Track base link: (%i)", track_base_link_enabled);

  if (cutoff_frequency_long == -1)  // If the cutoff frequency was not specified by the user
    ROS_INFO("LPF cutoff frequency: 1/4 of sampling rate");
  else
    ROS_INFO("LPF cutoff frequency: %f", cutoff_frequency_long);

  ROS_INFO("Integral-windup limit: %f", windup_limit);
  ROS_INFO("Saturation limits xy: %f/%f", upper_limit, lower_limit);
  ROS_INFO("Saturation limits ang: %f/%f", ang_upper_limit, ang_lower_limit);
  ROS_INFO("map frame: %s", map_frame.c_str());
  ROS_INFO("base_link frame: %s", base_link_frame.c_str());
  ROS_INFO("-----------------------------------------");
}

void Controller::configure(const tracking_pid::PidConfig& config)
{
  // Erase all queues when config changes
  std::fill(error_long.begin(), error_long.end(), 0);
  std::fill(filtered_error_long.begin(), filtered_error_long.end(), 0);
  std::fill(error_deriv_long.begin(), error_deriv_long.end(), 0);
  std::fill(filtered_error_deriv_long.begin(), filtered_error_deriv_long.end(), 0);

  std::fill(error_lat.begin(), error_lat.end(), 0);
  std::fill(filtered_error_lat.begin(), filtered_error_lat.end(), 0);
  std::fill(error_deriv_lat.begin(), error_deriv_lat.end(), 0);
  std::fill(filtered_error_deriv_lat.begin(), filtered_error_deriv_lat.end(), 0);

  std::fill(error_ang.begin(), error_ang.end(), 0);
  std::fill(filtered_error_ang.begin(), filtered_error_ang.end(), 0);
  std::fill(error_deriv_ang.begin(), error_deriv_ang.end(), 0);
  std::fill(filtered_error_deriv_ang.begin(), filtered_error_deriv_ang.end(), 0);

  Kp_long = config.Kp_long;
  Ki_long = config.Ki_long;
  Kd_long = config.Kd_long;
  Kp_lat = config.Kp_lat;
  Ki_lat = config.Ki_lat;
  Kd_lat = config.Kd_lat;
  Kp_ang = config.Kp_ang;
  Ki_ang = config.Ki_ang;
  Kd_ang = config.Kd_ang;

  l = config.l;
  debug_enabled = config.controller_debug_enabled;
  feedforward_long_enabled = config.feedforward_long;
  feedforward_lat_enabled = config.feedforward_lat;
  feedforward_ang_enabled = config.feedforward_ang;
  feedback_long_enabled = config.feedback_long;
  feedback_lat_enabled = config.feedback_lat;
  feedback_ang_enabled = config.feedback_ang;

  max_yaw_error_cal = config.max_yaw_error_cal;
  dead_zone_yaw_error_cal = config.dead_zone_yaw_error_cal;
  coupling_ang_long_enabled = config.coupling_ang_long;

  printParameters();
}

void Controller::setEnabled(bool value)
{
  ROS_DEBUG("Controller::setEnabled(%d)", value);
  enabled = value;
}

void Controller::setHolonomic(bool value)
{
  ROS_DEBUG("Controller::setHolonomic(%d)", value);
  holonomic_robot_enable = value;
}
void Controller::setTrackBaseLink(bool value)
{
  ROS_DEBUG("Controller::setTrackBaseLink(%d)", value);
  track_base_link_enabled = value;
}

}  // namespace tracking_pid
