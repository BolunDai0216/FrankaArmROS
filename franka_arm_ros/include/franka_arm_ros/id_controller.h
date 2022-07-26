#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/model.hpp>

#include <array>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <franka_example_controllers/JointTorqueComparison.h>
#include <franka_hw/franka_cartesian_command_interface.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <franka_hw/trigger_rate.h>

#include <Eigen/Dense>
#include<Eigen/Eigen>
#include <Eigen/Core>

namespace franka_arm_ros {

class IDController : public controller_interface::MultiInterfaceController<franka_hw::FrankaModelInterface, 
                                                                           hardware_interface::EffortJointInterface, 
                                                                           franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // pinocchio model & data
  pinocchio::Model model;
  pinocchio::Data data;

  // interface with franka_hw
  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;
  
  // publisher for data visualization
  ros::Publisher ee_measured_pub;
  ros::Publisher ee_desired_pub;

  // ROS messages for publishing data for visualization
  geometry_msgs::Vector3Stamped ee_desired_pos;
  geometry_msgs::Vector3Stamped ee_measured_pos;

  // end-effector frame id in pinocchio
  int ee_frame_id;

  // initial position of end-effector
  double x_0;
  double y_0;
  double z_0;

  // target position of end-effector
  double x_T;
  double y_T;
  double z_T;

  // time duration for planned trajectory
  double terminal_time;

  // current time in planned trajectory
  double t;

  // clock time when the controller started to go into effect 
  double start_t;

  // coefficient matrix for computing planned trajectory
  Eigen::Matrix<double, 6, 6> time_mat;

  // conditions for computing planned trajectory
  Eigen::Matrix<double, 6, 1> x_traj_cond_mat;
  Eigen::Matrix<double, 6, 1> y_traj_cond_mat;
  Eigen::Matrix<double, 6, 1> z_traj_cond_mat;

  // coefficients of the planned trajectory
  Eigen::Matrix<double, 6, 1> x_traj_coeff;
  Eigen::Matrix<double, 6, 1> y_traj_coeff;
  Eigen::Matrix<double, 6, 1> z_traj_coeff;

  // desired position of the end-effector at each time step
  double x_des;
  double y_des;
  double z_des;

  // desired velocity of the end-effector at each time step
  double vx_des;
  double vy_des;
  double vz_des;

  // desired accleration of the end-effector at each time step
  double ax_des;
  double ay_des;
  double az_des;

  // time matrix for computing desired position, velocity and acceleration
  Eigen::Matrix<double, 1, 6> position_time_mat; 
  Eigen::Matrix<double, 1, 6> velocity_time_mat; 
  Eigen::Matrix<double, 1, 6> acceleration_time_mat; 

  // vector of computed desired position, velocity and acceleration
  Eigen::Matrix<double, 3, 1> pos_desired;
  Eigen::Matrix<double, 3, 1> vel_desired;
  Eigen::Matrix<double, 3, 1> acc_desired;

  // error between desired and measured joint configurations
  Eigen::Matrix<double, 7, 1> delta_q;
  Eigen::Matrix<double, 7, 1> delta_dq;

  // desired joint accleration
  Eigen::Matrix<double, 7, 1> ddq_desired;

  // end-effector Jacobian
  Eigen::Matrix<double, 6, 7> J;

  // positional portion of the Jacobian
  Eigen::Matrix<double, 3, 7> Jp;

  // time deriative of end-effector Jacobian 
  Eigen::Matrix<double, 6, 7> dJ;

  // positional portion of the time deriative of end-effector Jacobian
  Eigen::Matrix<double, 3, 7> dJp;

  // Kp and Kd gains in control
  double kp_delta_q;
  double kd_delta_dq;
  double kd_dq;

  // applied torque
  Eigen::Matrix<double, 7, 1> torques;

  // bool value for determining if update() was called before or not
  bool notFirstUpdate;
};

}  // namespace franka_arm_ros