//
// Created by nobleo on 25-9-18.
//

/*
 * Run tests for all of the common function except the print* functions that do not return anything testable
 * Most important here is the conversion function and a variant of A*. Each test is explained below
 *
 */

#include <gtest/gtest.h>

#include <ros/ros.h>
#include <controller/controller.h>


/**
 * Test output of controller when target is reached.
 * Output should be zero
 */
TEST(TestUpdateOutput, testZeroOutput)
{
  tracking_pid::Controller pid_controller;
  tracking_pid::PidConfig config = tracking_pid::PidConfig::__getDefault__();
  pid_controller.configure(config);
  tracking_pid::PidDebug pid_debug;
  geometry_msgs::Twist cmd_vel;
  ros::Duration sample_time(0.05);
  geometry_msgs::Transform tfCurPose;
  tf::Transform tfGoalPose;

  // Test tracking carrot
  tfCurPose.translation.x = 1.0;
  tfCurPose.translation.y = 1.0;
  tfCurPose.rotation = tf::createQuaternionMsgFromYaw(0.0);
  tfGoalPose.setOrigin(tf::Vector3(1.0 + config.l, 1.0, 0.0));  // Put carrot in front
  tfGoalPose.setRotation(tf::createQuaternionFromYaw(0.0));
  cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, sample_time, &pid_debug);
  ASSERT_EQ(cmd_vel.linear.x, 0.0);
  ASSERT_EQ(cmd_vel.linear.y, 0.0);
  ASSERT_EQ(cmd_vel.angular.z, 0.0);

  // Test tracking base_link
  pid_controller.setTrackBaseLink(true);
  tfGoalPose.setOrigin(tf::Vector3(1.0, 1.0, 0.0));
  cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, sample_time, &pid_debug);
  ASSERT_EQ(cmd_vel.linear.x, 0.0);
  ASSERT_EQ(cmd_vel.linear.y, 0.0);
  ASSERT_EQ(cmd_vel.angular.z, 0.0);
}

/**
 * Test tracking error for a fix target using a carrot.
 * A fixed target is given to the controller. After some iterations x/y tracking error
 * should tend to zero
 */
TEST(TestUpdateOutput, testErrorTrackingCarrot)
{
  tracking_pid::Controller pid_controller;
  tracking_pid::PidConfig config = tracking_pid::PidConfig::__getDefault__();
  pid_controller.configure(config);
  tracking_pid::PidDebug pid_debug;
  geometry_msgs::Twist cmd_vel;
  ros::Duration sample_time(0.05);
  geometry_msgs::Transform tfCurPose;
  tf::Transform tfGoalPose;

  // Test tracking carrot
  tfCurPose.translation.x = 0.0;
  tfCurPose.translation.y = 0.0;
  tfCurPose.rotation = tf::createQuaternionMsgFromYaw(0.0);
  tfGoalPose.setOrigin(tf::Vector3(1.0, 1.0, 0.0));
  tfGoalPose.setRotation(tf::createQuaternionFromYaw(0.0));
  for (int i = 0; i < 1000; i++)
  {
    cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, sample_time, &pid_debug);
    // Run plant model
    double theta = tf::getYaw(tfCurPose.rotation);
    tfCurPose.translation.x += cmd_vel.linear.x * cos(theta) * sample_time.toSec();
    tfCurPose.translation.y += cmd_vel.linear.x * sin(theta) * sample_time.toSec();
    tfCurPose.rotation = tf::createQuaternionMsgFromYaw(theta + cmd_vel.angular.z * sample_time.toSec());
  }
  // After some iterations error should reduce to less than 1%.
  ASSERT_LE(fabs(pid_debug.error.linear.x), fabs(0.01 * tfGoalPose.getOrigin().getX()));
  ASSERT_LE(abs(pid_debug.error.linear.y), fabs(0.01 * tfGoalPose.getOrigin().getY()));
}

/**
 * Test tracking error for a non-holonomic robot following a temporarily moving target using a base_link.
 * The target is moved for some time to let the angle tp be aligned. After some iterations x/y/theta
 * error with respect to target pose should be small enough
 */
TEST(TestUpdateOutput, testErrorTrackingBaseLink)
{
  tracking_pid::Controller pid_controller;
  tracking_pid::PidConfig config = tracking_pid::PidConfig::__getDefault__();
  pid_controller.configure(config);
  tracking_pid::PidDebug pid_debug;
  geometry_msgs::Twist cmd_vel;
  ros::Duration sample_time(0.05);
  geometry_msgs::Transform tfCurPose;
  tf::Transform tfGoalPose;

  // Test tracking base_link
  pid_controller.setTrackBaseLink(true);
  tfCurPose.translation.x = 0.0;
  tfCurPose.translation.y = 0.0;
  tfCurPose.rotation = tf::createQuaternionMsgFromYaw(1.0);
  double x_goal = 1.0;
  double y_goal = 1.0;
  tfGoalPose.setOrigin(tf::Vector3(x_goal, y_goal, 0.0));
  tfGoalPose.setRotation(tf::createQuaternionFromYaw(0.0));
  for (int i = 0; i < 1000; i++)
  {
    cmd_vel = pid_controller.update(tfCurPose, tfGoalPose, sample_time, &pid_debug);
    // Run plant model
    double theta = tf::getYaw(tfCurPose.rotation);
    tfCurPose.translation.x += (cmd_vel.linear.x * cos(theta) * sample_time.toSec()    // NOLINT
                                - cmd_vel.linear.y * sin(theta) * sample_time.toSec());
    tfCurPose.translation.y += (cmd_vel.linear.x * sin(theta) * sample_time.toSec()    // NOLINT
                                + cmd_vel.linear.y * cos(theta) * sample_time.toSec());
    tfCurPose.rotation = tf::createQuaternionMsgFromYaw(theta + cmd_vel.angular.z * sample_time.toSec());
    // Move goal forwards the fist 500 samples, then error for an non-holonomic robot tends to zero
    if (i < 500)
    {
      x_goal += cmd_vel.linear.x * sample_time.toSec();
      tfGoalPose.setOrigin(tf::Vector3(x_goal, y_goal, 0.0));
    }
  }
  tf::Transform tfCurTransform;
  tf::transformMsgToTF(tfCurPose, tfCurTransform);
  tf::Transform error = tfCurTransform.inverseTimes(tfGoalPose);
  // After some iterations error should reduce to less than 1cm/1degree
  ASSERT_LE(fabs(error.getOrigin().getX()), 0.01);
  ASSERT_LE(fabs(error.getOrigin().getY()), 0.01);
  ASSERT_LE(fabs(tf::getYaw(error.getRotation())), M_PI / 180);
}


// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
