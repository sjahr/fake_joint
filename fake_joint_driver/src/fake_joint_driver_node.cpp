/**
 * @file fake_joint_driver_node.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * Device driver node to fake loopback joints.
 */
#include "controller_manager/controller_manager.h"
#include "fake_joint_driver/fake_joint_driver.h"
#include "ros/ros.h"

/**
 * @brief Main function
 */
int main(int argc, char **argv) {
  // Init ROS node
  ros::init(argc, argv, "fake_joint_driver");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // Get control period
  double control_period;
  pnh.param<double>("control_period", control_period, 0.01);
  // Create hardware interface
  FakeJointDriver robot;
  // Connect to controller manager
  controller_manager::ControllerManager cm(&robot, nh);

  // Set loop rate
  int loop_rate;
  pnh.param<int>("main_loop_rate", loop_rate, 100);
  ros::Rate rate(loop_rate);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok())
  {
    robot.update();
    cm.update(ros::Time::now(), rate.expectedCycleTime());
    rate.sleep();
  }
  spinner.stop();

  return 0;
}
