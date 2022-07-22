#include <franka_arm_ros/pd_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>
#include <iostream>

namespace franka_arm_ros {

bool PDController::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {
    //checking to see if the default parameters can be access through node handle
    //and also getting information and interfaces  
    // std::string arm_id;
    // if (!node_handle.getParam("arm_id", arm_id)) {
    //     ROS_ERROR("JointImpedanceController: Could not read parameter arm_id");
    //     return false;
    // }
    // std::vector<std::string> joint_names;
    // if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    //     ROS_ERROR(
    //         "JointImpedanceController: Invalid or no joint_names parameters provided, aborting "
    //         "controller init!");
    //     return false;
    // }

    // double publish_rate(30.0);
    // if (!node_handle.getParam("publish_rate", publish_rate)) {
    //     ROS_INFO_STREAM("JointImpedanceController: publish_rate not found. Defaulting to "
    //                     << publish_rate);
    // }
   
    // auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    // if (effort_joint_interface == nullptr) {
    //     ROS_ERROR_STREAM(
    //         "JointImpedanceController: Error getting effort joint interface from hardware");
    //     return false;
    // }
    // for (size_t i = 0; i < 7; ++i) {
    //     try {
    //     joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    //     } catch (const hardware_interface::HardwareInterfaceException& ex) {
    //     ROS_ERROR_STREAM(
    //         "JointImpedanceController: Exception getting joint handles: " << ex.what());
    //     return false;
    //     }
    // }

    return true;
}

void PDController::starting(const ros::Time& /*time*/) {
  int i = 0;
}

void PDController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  int i = 0;
}

}  // namespace franka_arm_ros

PLUGINLIB_EXPORT_CLASS(franka_arm_ros::PDController, controller_interface::ControllerBase)