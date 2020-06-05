//
// Created by nobleo on 11-9-18.
//
#include <angles/angles.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <string>
#include <tf/tf.h>
#include <tracking_pid/PidConfig.h>
#include <tracking_pid/PidDebug.h>
#include <vector>

#ifndef CONTROLLER_CONTROLLER_H
#define CONTROLLER_CONTROLLER_H

namespace tracking_pid
{
enum ControllerMode
{
  frontAxleLateral = 0,
  rearAxleLateral = 1,
  rearAxleAngular = 2,
  fixOrientation = 3,
};

class Controller
{
public:
  Controller();

  /**
   * Run one iteration of a PID controller
   * @param current Where is the robot now?
   * @param goal must the robot go?
   * @param sample_time controller sample time
   * @param pid_debug variable to store debuging data
   * @return direction in which to drive to reduce the error
   */
  geometry_msgs::Twist update(const geometry_msgs::Transform current, const tf::Transform goal,
                              const ros::Duration sample_time, tracking_pid::PidDebug* pid_debug);
  geometry_msgs::Twist update(const geometry_msgs::Transform current, const geometry_msgs::Pose goal,
                              const ros::Duration sample_time, tracking_pid::PidDebug* pid_debug);

  /**
   * Select mode for the controller
   * @param mode
   */
  void selectMode(ControllerMode mode);

  /**
   * Set parameters for the PID controller
   * @param config
   */
  void configure(const tracking_pid::PidConfig& config);

  /**
   * Set whether the controller is enabled
   * @param value
   */
  void setEnabled(bool value);

  /**
   * Set whether the robot is holonomic
   * @param value
   */
  void setHolonomic(bool value);

  /**
   * Set whether the robot should track the point with base_link
   * @param value
   */
  void setTrackBaseLink(bool value);

protected:
  /**
   * Output some debug information about the current parameters
   */
  void printParameters();

  // Frame names
  std::string map_frame;
  std::string base_link_frame;

  double control_effort_long = 0.0;  // output of pid controller
  double control_effort_lat = 0.0;   // output of pid controller
  double control_effort_ang = 0.0;   // output of pid controller

  bool enabled = true;
  bool feedback_long_enabled = false;
  bool feedback_lat_enabled = false;
  bool feedback_ang_enabled = false;
  bool feedforward_long_enabled = false;
  bool feedforward_lat_enabled = false;
  bool feedforward_ang_enabled = false;
  bool holonomic_robot_enable = false;
  bool track_base_link_enabled = false;
  bool coupling_ang_long_enabled = false;

  // feedforward controller

  double feedforward_long = 0;
  double feedforward_lat = 0;
  double feedforward_ang = 0;
  double xvel = 0.0;
  double yvel = 0.0;
  double thvel = 0.0;
  double theta_cp = 0.0;

  bool debug_enabled = false;

  // Primary feedback controller parameters
  double Kp_long = 0.0f;
  double Ki_long = 0.0f;
  double Kd_long = 0.0f;
  double Kp_lat = 0.0f;
  double Ki_lat = 0.0f;
  double Kd_lat = 0.0f;
  double Kp_ang = 0.0f;
  double Ki_ang = 0.0f;
  double Kd_ang = 0.0f;
  double l = 0.0f;

  // Coupling between lat/ang loops and longitudinal loop
  double max_yaw_error_cal = 0.0;
  double dead_zone_yaw_error_cal = 0.0;
  double scale_long_control = 1.0;

  // Cutoff frequency for the derivative calculation in Hz.
  // Negative -> Has not been set by the user yet, so use a default.
  double cutoff_frequency_long = -1.0f;
  double cutoff_frequency_lat = -1.0f;
  double cutoff_frequency_ang = -1.0f;

  // Upper and lower saturation limits
  double upper_limit = 10.0f;
  double lower_limit = -10.0f;

  double ang_upper_limit = 10.0f;
  double ang_lower_limit = -10.0f;

  // Anti-windup term. Limits the absolute value of the integral term.
  double windup_limit = 1000.0f;

  // Initialize filter data with zeros
  std::vector<double> error_long = decltype(error_long)(3, 0);
  std::vector<double> filtered_error_long = decltype(filtered_error_long)(3, 0);
  std::vector<double> error_deriv_long = decltype(error_deriv_long)(3, 0);
  std::vector<double> filtered_error_deriv_long = decltype(filtered_error_deriv_long)(3, 0);
  std::vector<double> error_lat = decltype(error_lat)(3, 0);
  std::vector<double> filtered_error_lat = decltype(filtered_error_lat)(3, 0);
  std::vector<double> error_deriv_lat = decltype(error_deriv_lat)(3, 0);
  std::vector<double> filtered_error_deriv_lat = decltype(filtered_error_deriv_lat)(3, 0);
  std::vector<double> error_ang = decltype(error_ang)(3, 0);
  std::vector<double> filtered_error_ang = decltype(filtered_error_ang)(3, 0);
  std::vector<double> error_deriv_ang = decltype(error_deriv_ang)(3, 0);
  std::vector<double> filtered_error_deriv_ang = decltype(filtered_error_deriv_ang)(3, 0);

  // Temporary variables
  double proportional_long = 0;  // proportional term of output
  double integral_long = 0;      // integral term of output
  double derivative_long = 0;    // derivative term of output
  double proportional_lat = 0;   // proportional term of output
  double integral_lat = 0;       // integral term of output
  double derivative_lat = 0;     // derivative term of output
  double proportional_ang = 0;   // proportional term of output
  double integral_ang = 0;       // integral term of output
  double derivative_ang = 0;     // derivative term of output
  double error_integral_lat = 0;
  double error_integral_long = 0;
  double error_integral_ang = 0;

  // Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
  // 1/4 of the sample rate.
  double c_long = 1.;
  double c_lat = 1.;
  double c_ang = 1.;

  // Used to check for tan(0)==>NaN in the filter calculation
  double tan_filt = 1.;
};
}  // namespace tracking_pid

#endif  // CONTROLLER_CONTROLLER_H
