#include <franka_arm_ros/pd_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>  

namespace franka_arm_ros {

bool PDController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  effort_joint_interface_ = robot_hardware->get<hardware_interface::EffortJointInterface>();
  
  // check if got effort_joint_interface 
  if (effort_joint_interface_ == nullptr) {
    ROS_ERROR(
        "PDController: Error getting effort joint interface from hardware!");
    return false;
  }
  
  // check if got joint_names
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("PDController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("PDController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
    return false;
  }

  // check if joint_handle got each joint (crucial step of avoiding could not switch controller error)
  effort_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      effort_joint_handles_[i] = effort_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "PDController: Exception getting joint handles: " << e.what());
      return false;
    }
  }

  return true;
}

void PDController::starting(const ros::Time& /* time */) {
    int i = 0;
}

void PDController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  for (size_t i = 0; i < 7; ++i) {
    effort_joint_handles_[i].setCommand(0.01);
  }
}

}  // namespace franka_arm_ros

PLUGINLIB_EXPORT_CLASS(franka_arm_ros::PDController, controller_interface::ControllerBase)