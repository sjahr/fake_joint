/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include "fake_joint_driver/fake_joint_driver.h"
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <ros/ros.h>
#include <urdf/model.h>

FakeJointDriver::FakeJointDriver(void) {
  ros::NodeHandle pnh("~");
  std::set<std::string> joint_set;
  std::map<std::string, double> start_position_map;

  // Read parameters
  pnh.param<bool>("use_robot_description", use_description_, true);
  pnh.getParam("include_joints", include_joints_);
  pnh.getParam("exclude_joints", exclude_joints_);
  pnh.getParam("start_position", start_position_map);

  for (auto it = start_position_map.begin(); it != start_position_map.end();
       ++it) {
    ROS_DEBUG_STREAM("start_position: " << it->first << ": " << it->second);
  }

  for (auto i = 0; i < include_joints_.size(); ++i) {
    ROS_DEBUG_STREAM("include_joint[" << i << "]" << include_joints_[i]);
  }
  for (auto i = 0; i < exclude_joints_.size(); ++i) {
    ROS_DEBUG_STREAM("exclude_joint[" << i << "]" << exclude_joints_[i]);
  }
  // Read all joints in robot_description
  if (use_description_) {
    urdf::Model urdf_model;
    if (urdf_model.initParam("robot_description")) {
      for (auto it = urdf_model.joints_.begin(); it != urdf_model.joints_.end();
           ++it) {
        urdf::Joint joint = *it->second;
        // remove fixed and unknown joints
        if (joint.type == urdf::Joint::FIXED ||
            joint.type == urdf::Joint::UNKNOWN) {
          continue;
        }
        joint_set.insert(joint.name);
      }
    } else {
      ROS_WARN("We cannot find the parameter robot_description.");
    }
  }
  // Include joints into joint_set
  for (auto i = 0; i < include_joints_.size(); ++i) {
    joint_set.insert(include_joints_[i]);
  }
  // Exclude joints in joint_set
  for (auto i = 0; i < exclude_joints_.size(); ++i) {
    joint_set.erase(exclude_joints_[i]);
  }
  // Convert to vector (joint_names_)
  std::copy(joint_set.begin(), joint_set.end(),
            std::back_inserter(joint_names_));
  // Check the emptyness of joints
  if (joint_names_.size() == 0) {
    ROS_ERROR("No joints is specified. Please use include_joints parameters.");
    ros::shutdown();
  }
  // Resize members
  cmd_dis_.resize(joint_names_.size());
  cmd_dis_old_.resize(joint_names_.size());
  cmd_vel_.resize(joint_names_.size(), 0.0);
  act_dis_.resize(joint_names_.size());
  act_vel_.resize(joint_names_.size());
  act_eff_.resize(joint_names_.size());

  // Set start position
  for (auto it = start_position_map.begin(); it != start_position_map.end();
       ++it) {
    for (auto i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i] == it->first) {
        act_dis_[i] = it->second;
        cmd_dis_[i] = it->second;
        cmd_dis_old_[i] = cmd_dis_[i];
      }
    }
  }

  // Create joint_state_interface and position_joint_interface
  for (int i = 0; i < joint_names_.size(); ++i) {
    ROS_DEBUG_STREAM("joint[" << i << "]:" << joint_names_[i]);
    // Connect and register the joint_state_interface
    hardware_interface::JointStateHandle state_handle(
        joint_names_[i], &act_dis_[i], &act_vel_[i], &act_eff_[i]);
    joint_state_interface_.registerHandle(state_handle);

    // Connect and register the position_joint_interface
    hardware_interface::JointHandle pos_handle(
        joint_state_interface_.getHandle(joint_names_[i]), &cmd_dis_[i]);
    position_joint_interface_.registerHandle(pos_handle);

    // Connect and register the velocity_joint_interface
    hardware_interface::JointHandle vel_handle(
        joint_state_interface_.getHandle(joint_names_[i]), &cmd_vel_[i]);
    velocity_joint_interface_.registerHandle(vel_handle);
  }
  registerInterface(&joint_state_interface_);
  registerInterface(&position_joint_interface_);
  registerInterface(&velocity_joint_interface_);
}

/**
 * @brief Update function to call all of the update function of motors
 */
void FakeJointDriver::update(ros::Duration period) {

  // check if non zero velocity command is sent
  double vel_sum = 0.0;
  std::for_each(cmd_vel_.begin(), cmd_vel_.end(),
                [&vel_sum](double a) { return vel_sum += std::abs(a); });

  // if velocity commands are zero - use position interface
  if (vel_sum == 0.0) {
    // TODO enforce position and velocity limits
    // only do loopback
    act_dis_ = cmd_dis_;
    std::fill(cmd_vel_.begin(), cmd_vel_.end(), 0.0);
    act_vel_ = cmd_vel_;
  } else {
    // TODO enforce position, velocity and acceleration limits
    // if there is non zero velocity commands - use velocity interface
    act_vel_ = cmd_vel_;
    // create a lambda integration functor
    std::function<double(double, double)> integrator =
        [&period](double a, double b) { return a + b * period.toSec(); };
    // integrate position
    std::transform(act_dis_.begin(), act_dis_.end(), cmd_vel_.begin(),
                   act_dis_.begin(), integrator);
    // if velocity commands are not used in the next control loop iteration make
    // sure first position command is actual state of joints
    cmd_dis_ = act_dis_;
  }
}

/**
 * @brief Update function to call all of the update function of motors
 */
void FakeJointDriver::update(void) {
  // only do loopback
  act_dis_ = cmd_dis_;
}
