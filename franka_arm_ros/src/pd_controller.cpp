#include <franka_arm_ros/pd_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h> 
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>   
#include <geometry_msgs/Vector3Stamped.h>

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

  // build pin_robot from urdf
  std::string urdf_filename = "/home/bolun/catkin_ws/src/FrankaArmROS/franka_arm_ros/robots/panda_pin.urdf";
  pinocchio::urdf::buildModel(urdf_filename, model);
  data = pinocchio::Data(model);

  // publish data for rqt_plot
  ee_measured_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_measured_pos", 1000);
  ee_desired_pub = node_handle.advertise<geometry_msgs::Vector3Stamped>("/ee_desired_pos", 1000);

  return true;
}

void PDController::starting(const ros::Time& /* time */) {
    // get intial robot state
    franka::RobotState initial_state = state_handle_->getRobotState();

    // get intial end-effector position
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());    

    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(initial_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::updateFramePlacements(model, data);

    ee_frame_id = model.getFrameId("panda_fingertip");
    
    // get initial end-effector position
    x_0 = data.oMf[ee_frame_id].translation()(0);
    y_0 = data.oMf[ee_frame_id].translation()(1);
    z_0 = data.oMf[ee_frame_id].translation()(2);

    // get target end-effector position
    ros::param::get("/x_target", x_T);
    ros::param::get("/y_target", y_T);
    ros::param::get("/z_target", z_T);
    ros::param::get("/terminal_time", terminal_time);

    // get pd gains
    ros::param::get("/kp_delta_q", kp_delta_q);
    ros::param::get("/kd_delta_dq", kd_delta_dq);
    ros::param::get("/kd_dq", kd_dq);

    // solve for trajctory coefficients
    time_mat << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 0.0,
                1.0, terminal_time, pow(terminal_time, 2.0), pow(terminal_time, 3.0), pow(terminal_time, 4.0), pow(terminal_time, 5.0),
                0.0, 1.0, 2.0*terminal_time, 3.0*pow(terminal_time, 2.0), 4.0*pow(terminal_time, 3.0), 5.0*pow(terminal_time, 4.0),
                0.0, 0.0, 2.0, 6.0*terminal_time, 12.0*pow(terminal_time, 2.0), 20.0*pow(terminal_time, 3.0);
    
    x_traj_cond_mat << x_0, 0.0, 0.0, x_T, 0.0, 0.0; 
    y_traj_cond_mat << y_0, 0.0, 0.0, y_T, 0.0, 0.0; 
    z_traj_cond_mat << z_0, 0.0, 0.0, z_T, 0.0, 0.0; 

    x_traj_coeff = time_mat.colPivHouseholderQr().solve(x_traj_cond_mat);
    y_traj_coeff = time_mat.colPivHouseholderQr().solve(y_traj_cond_mat);
    z_traj_coeff = time_mat.colPivHouseholderQr().solve(z_traj_cond_mat);

    notFirstUpdate = false;
}

void PDController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  // get joint angles and angular velocities
  franka::RobotState robot_state = state_handle_->getRobotState();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());

  // update pinocchio robot model
  pinocchio::forwardKinematics(model, data, q, dq);
  pinocchio::updateFramePlacements(model, data);

  // get time
  if (!notFirstUpdate) {
    notFirstUpdate = true;
    start_t = ros::Time::now().toSec();
  }
  t = ros::Time::now().toSec() - start_t;


  // get state variables
  std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

  // get Jacobian matrix
  pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);

  if (t <= terminal_time) {
    position_time_mat << 1.0, t, pow(t, 2.0), pow(t, 3.0), pow(t, 4.0), pow(t, 5.0);
    velocity_time_mat << 0.0, 1.0, 2.0*t, 3.0*pow(t, 2.0), 4.0*pow(t, 3.0), 5.0*pow(t, 4.0);
    acceleration_time_mat << 0.0, 0.0, 2.0, 6.0*t, 12.0*pow(t, 2.0), 20.0*pow(t, 3.0);

    // get desired position
    x_des = position_time_mat * x_traj_coeff;
    y_des = position_time_mat * y_traj_coeff;
    z_des = position_time_mat * z_traj_coeff;
    pos_desired << x_des, y_des, z_des;
    
    // get desired velocity
    vx_des = velocity_time_mat * x_traj_coeff;
    vy_des = velocity_time_mat * y_traj_coeff;
    vz_des = velocity_time_mat * z_traj_coeff;
    vel_desired << vx_des, vy_des, vz_des;

    // get desired acceleration
    ax_des = acceleration_time_mat * x_traj_coeff;
    ay_des = acceleration_time_mat * y_traj_coeff;
    az_des = acceleration_time_mat * z_traj_coeff;
    acc_desired << ax_des, ay_des, az_des;
  } else {
    pos_desired << x_T, y_T, z_T;
    vel_desired << 0.0, 0.0, 0.0;
    acc_desired << 0.0, 0.0, 0.0;
  }

  // get positional Jacobian
  std::vector<int> row_idx{0, 1, 2};
  std::vector<int> col_idx{0, 1, 2, 3, 4, 5, 6};
  Jp = J(row_idx, col_idx);

  // get time derivative of positional Jacobian
  dJ = pinocchio::computeJointJacobiansTimeVariation(model, data, q, dq);
  dJp = dJ(row_idx, col_idx);

  // get errors
  delta_q = Jp.colPivHouseholderQr().solve(pos_desired - data.oMf[ee_frame_id].translation());
  delta_dq = Jp.colPivHouseholderQr().solve(vel_desired) - dq;

  // get desired joint acceleration
  ddq_desired = Jp.colPivHouseholderQr().solve(acc_desired - dJp * dq);

  // get mass matrix
  std::array<double, 49> mass_array = model_handle_->getMass();
  Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data()); 

  // compute PD controller
  torques = M * (ddq_desired + kp_delta_q * delta_q + kd_delta_dq * delta_dq) + coriolis - kd_dq * dq; 

  // set torque
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(torques[i]);
  }

  // publish tracking data to topics
  ee_desired_pos.vector.x = x_des;
  ee_desired_pos.vector.y = y_des;
  ee_desired_pos.vector.z = z_des;
  ee_desired_pos.header.stamp = ros::Time::now();

  ee_measured_pos.vector.x = data.oMf[ee_frame_id].translation()(0);
  ee_measured_pos.vector.y = data.oMf[ee_frame_id].translation()(1);
  ee_measured_pos.vector.z = data.oMf[ee_frame_id].translation()(2);
  ee_measured_pos.header.stamp = ros::Time::now();

  ee_desired_pub.publish(ee_desired_pos);
  ee_measured_pub.publish(ee_measured_pos);
}

}  // namespace franka_arm_ros

PLUGINLIB_EXPORT_CLASS(franka_arm_ros::PDController, controller_interface::ControllerBase)