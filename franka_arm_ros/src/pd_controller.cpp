#include <franka_arm_ros/pd_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   

namespace franka_arm_ros {

bool PDController::init(hardware_interface::RobotHW* robot_hw,
                                          ros::NodeHandle& node_handle) {
  // check if got arm_id
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("PDController: Could not read parameter arm_id");
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
  
  // check if model_handle_ works
  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PDController: Error getting model interface from hardware");
    return false;
  }

  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PDController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  // check if state_handle_ works
  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PDController: Error getting state interface from hardware");
    return false;
  }
  
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "PDController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  // check if joint_handles_ got each joint (crucial step of avoiding could not switch controller error)
  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "PDController: Error getting effort joint interface from hardware");
    return false;
  }
  
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "PDController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  return true;
}

void PDController::starting(const ros::Time& /* time */) {
    int i = 0;
}

void PDController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // get state variables
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // get target joint angles
  std::array<double, 7> target_array = {0.5, -0.785398163, 0, -2.35619449, 0, 1.57079632679, 0.785398163397};
  Eigen::Map<Eigen::Matrix<double, 7, 1>> target(target_array.data());

  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  Eigen::Matrix<double, 7, 1> torques = coriolis + 10 * (target - q) - 10 * dq; 

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }
}

}  // namespace franka_arm_ros

PLUGINLIB_EXPORT_CLASS(franka_arm_ros::PDController, controller_interface::ControllerBase)