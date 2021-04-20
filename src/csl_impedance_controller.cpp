// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <csl_controllers/csl_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "franka_msgs/SetJointImpedance.h"

namespace csl_controllers {

bool CSLImpedanceController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "CSLImpedanceController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("CSLImpedanceController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("CSLImpedanceController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "CSLImpedanceController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
  for (size_t i = 0; i < q_start.size(); i++) {
    if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
      ROS_ERROR_STREAM(
          "CSLImpedanceController: Robot is not in the expected starting position for "
          "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
          "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
      return false;
    }
  }

  // double publish_rate(30.0);
  // rate_trigger_ = franka_hw::TriggerRate(publish_rate);
  // current_phase_pub_.init(node_handle, "current_phase", 1);


  ros::ServiceClient client = node_handle.serviceClient<franka_msgs::SetJointImpedance>("/franka_control/set_joint_impedance");
  franka_msgs::SetJointImpedance setJP_request;
  setJP_request.request.joint_stiffness = {{750, 750, 750, 625, 625, 500, 500}};
  if (client.call(setJP_request))
  {
    if (setJP_request.response.success) {
      ROS_INFO("Let's go home!\n");
    }
  }
  else
  {
    ROS_ERROR("STAY\n");
  }


  return true;
}

void CSLImpedanceController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    initial_pose_[i] = position_joint_handles_[i].getPosition();
  }
  elapsed_time_ = ros::Duration(0.0);
}

void CSLImpedanceController::update(const ros::Time& /*time*/,
                                            const ros::Duration& period) {
  elapsed_time_ += period;

  double delta_angle = M_PI / 16 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec())) * 0.2;

  // if (rate_trigger_() && current_phase_pub_.trylock()) {
  //   current_phase_pub_.msg_.data = position_d(1);
  //   current_phase_pub_.unlockAndPublish();
  // }

  for (size_t i = 0; i < 7; ++i) {
    if (i == 4) {
      position_joint_handles_[i].setCommand(initial_pose_[i] - delta_angle);
    } else {
      position_joint_handles_[i].setCommand(initial_pose_[i] + delta_angle);
    }
  }
}

}  // namespace csl_example_controllers

PLUGINLIB_EXPORT_CLASS(csl_controllers::CSLImpedanceController,
                       controller_interface::ControllerBase)
