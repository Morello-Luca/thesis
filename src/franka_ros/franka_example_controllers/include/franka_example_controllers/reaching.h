// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

namespace franka_example_controllers {

class reaching : public controller_interface::MultiInterfaceController<
                                                franka_hw::FrankaModelInterface,
                                                hardware_interface::EffortJointInterface,
                                                franka_hw::FrankaStateInterface> {
 public:
  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
  void starting(const ros::Time&) override;
  void update(const ros::Time&, const ros::Duration& period) override;

 private:
  // Saturation
  Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

  std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
  std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
  std::vector<hardware_interface::JointHandle> joint_handles_;

    double filter_params_{0.005};
  double nullspace_stiffness_{20.0};
  double nullspace_stiffness_target_{20.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  std::mutex position_and_orientation_d_target_mutex_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  
  // CARTESIAN PARAM
  	Eigen::Matrix<double, 6, 6> damping_c;
  	Eigen::Matrix<double, 6, 6> impendance_K_c;
  	Eigen::Matrix<double, 7, 7> C; 
  	Eigen::Vector3d ActualPosition;
  // FORCE PARAM
  	double force_error_integral;
  	double ee_f_d;
  	double ee_f_d_target; 
  	double kp_ {1.5};          // Proportional gain
	double kd_ {0.};           // Derivative   gain
	double ki_ {0.3};          // Integral     gain
  // MODULARITY
	Eigen::Vector3d ee_p_cr; 
  // RESIDUI
	Eigen::Matrix<double, 7, 1> r;
	Eigen::Matrix<double, 7, 1> I;
	Eigen::Matrix<double, 7, 1> q_old;
        Eigen::Matrix<double, 7, 1> dq_old;
        Eigen::Matrix<double, 7, 7> C_old;
        Eigen::Matrix<double, 7, 1> r_old;
        Eigen::Matrix<double, 7, 1> tau_d_old;
        Eigen::Matrix<double, 7, 7> M;
  // ADAPTATION POLICY
  	double impendance_rho; 
	double delta_imp_; 
  	double delta_force_; 
   	double force_rho;
  // JACOBIANS
        Eigen::Matrix<double, 6, 6> J_cr;  
        Eigen::Matrix<double, 6, 7> J; 

  // Dynamic reconfigure
  std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>
      dynamic_server_compliance_param_;
  ros::NodeHandle dynamic_reconfigure_compliance_param_node_;
  void complianceParamCallback(franka_example_controllers::compliance_paramConfig& config,
                               uint32_t level);
                               



  
  
  // PUBLISHERS
	  ros::Publisher pub_endeffector_pose_;
	  ros::Publisher pub_stima_;
	  ros::Publisher pub_force_error;


// SUBSCRIBERS
  	// Equilibrium pose subscriber
  		ros::Subscriber sub_equilibrium_pose_;
  		void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  	// force
  		ros::Subscriber sub_desired_force_;
  		void desiredforceCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  
};

}  // namespace franka_example_controllers
