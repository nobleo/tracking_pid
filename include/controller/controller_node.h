#ifndef CONTROLLER_CONTROLLER_NODE_H
#define CONTROLLER_CONTROLLER_NODE_H

#include "controller/controller.h"
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_srvs/SetBool.h>
#include <stdio.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tracking_pid/PidConfig.h>
#include <tracking_pid/PidDebug.h>
#include <tracking_pid/traj_point.h>
#include <tracking_pid/trajectory.h>
#include <vector>
#include <visualization_msgs/Marker.h>

// Declare functions
void trajectory_callback(const tracking_pid::traj_point& goalPoint);
void curOdomCallback(const nav_msgs::Odometry& odom_msg);
void controller();
void reconfigure_callback(const tracking_pid::PidConfig& config, uint32_t level);
void print_parameters();
void printPoses();
double dist2D(double x_err, double y_err);
void controllerSelector(int type);


// Generic pose variables
geometry_msgs::Transform tfCurPose;
tf::Transform tfControlPose = tf::Transform::getIdentity();
tf::Transform tfGoalPose = tf::Transform::getIdentity();
tf::Transform tfErrorPose = tf::Transform::getIdentity();
tf::StampedTransform tfMapToOdom;
tf2_ros::Buffer tf_buffer;

geometry_msgs::Pose controlPose;
geometry_msgs::Pose goalPose;
tracking_pid::traj_point goalPoint;
tf::Vector3 newOrigin;

// Frame names
std::string map_frame;
std::string base_link_frame;

// Errors
double error_x = 0;
double error_y = 0;
double error_th = 0;

// For timing
ros::Time prev_time;
ros::Duration delta_t;

// Controller output
double control_effort_long = 0.0;  // output of pid controller
double control_effort_lat = 0.0;   // output of pid controller
double control_effort_ang = 0.0;   // output of pid controller
geometry_msgs::Twist output_combined;

// Controller logic
int controlType = 0;
bool controller_enabled = true;
bool enabled_on_boot =  true;
bool waiting_for_setpoint = false;
bool feedback_long_enabled = false;
bool feedback_lat_enabled = false;
bool feedback_ang_enabled = false;
bool feedforward_long_enabled = false;
bool feedforward_lat_enabled = false;
bool feedforward_ang_enabled = false;
bool controller_debug_enabled = false;
bool holonomic_robot = false;
bool track_base_link = false;


// Primary feedback controller parameters
double Kp_long = 0, Ki_long = 0, Kd_long = 0;
double Kp_lat = 0, Ki_lat = 0, Kd_lat = 0;
double Kp_ang = 0, Ki_ang = 0, Kd_ang = 0;
double l = 0.0;

// feedforward controller

double feedforward_long = 0;
double feedforward_lat = 0;
double feedforward_ang = 0;
double xvel = 0.0;
double yvel = 0.0;
double thvel = 0.0;
double theta_cp = 0.0;

// Parameters for error calc. with disconinuous input
bool angle_error = false;
double angle_wrap = 2.0 * 3.14159;

// Cutoff frequency for the derivative calculation in Hz.
// Negative -> Has not been set by the user yet, so use a default.
double cutoff_frequency_long = -1;
double cutoff_frequency_lat = -1;
double cutoff_frequency_ang = -1;

// Used in filter calculations. Default 1.0 corresponds to a cutoff frequency at
// 1/4 of the sample rate.
double c_long = 1.;
double c_lat = 1.;
double c_ang = 1.;

// Used to check for tan(0)==>NaN in the filter calculation
double tan_filt = 1.;

// Upper and lower saturation limits
double upper_limit = 1000, lower_limit = -1000;

// Anti-windup term. Limits the absolute value of the integral term.
double windup_limit = 1000;

// Initialize filter data with zeros
std::vector<double> error_long(3, 0), filtered_error_long(3, 0), error_deriv_long(3, 0),
    filtered_error_deriv_long(3, 0);
std::vector<double> error_lat(3, 0), filtered_error_lat(3, 0), error_deriv_lat(3, 0), filtered_error_deriv_lat(3, 0);
std::vector<double> error_ang(3, 0), filtered_error_ang(3, 0), error_deriv_ang(3, 0), filtered_error_deriv_ang(3, 0);

// Temporary variables
double proportional_long = 0;           // proportional term of output
double integral_long = 0;               // integral term of output
double derivative_long = 0;             // derivative term of output
double proportional_lat = 0;            // proportional term of output
double integral_lat = 0;                // integral term of output
double derivative_lat = 0;              // derivative term of output
double proportional_ang = 0;            // proportional term of output
double integral_ang = 0;                // integral term of output
double derivative_ang = 0;              // derivative term of output
double steadySetPointThreshold = 0.01;  // Threshold in degrees on the linear velocity output
double error_integral_lat = 0;
double error_integral_long = 0;
double error_integral_ang = 0;
double theta_rp = 0.0;

// Topic and node names and message objects
ros::Publisher control_effort_pub;
ros::Subscriber sub_trajectory;
// ros::Subscriber subs_odom;
ros::ServiceServer enable_service;
ros::ServiceServer enable_and_wait_service;

// Debugging of controller
ros::Publisher debug_pub;
// Rviz visualization
ros::Publisher marker_pub;
visualization_msgs::Marker mkCurPose, mkControlPose, mkGoalPose;
geometry_msgs::Point p;

// For Screen output
std::string topic_from_controller, topic_from_plant, setpoint_topic, pid_enable_topic, node_name = "pid_node";

// Diagnostic objects
double min_loop_frequency = 1, max_loop_frequency = 1000;
int measurements_received = 0;

tracking_pid::Controller pid_controller;


#endif  // CONTROLLER_CONTROLLER_NODE_H
