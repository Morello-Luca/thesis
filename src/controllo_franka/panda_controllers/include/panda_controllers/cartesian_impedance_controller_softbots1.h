// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <memory>
#include <string>
#include <vector>
#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <eigen3/Eigen/Dense>

//#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <sensor_msgs/JointState.h>

namespace panda_controllers {

class CartesianImpedanceControllerSoftbots : public controller_interface::MultiInterfaceController<
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


	double stiffness_K_EndEffector_Traslation = 500;
	double stiffness_K_EndEffector_Rotation   = 31;



  double filter_params_{0.1};
  double nullspace_stiffness_{5.0};
  double nullspace_stiffness_target_{5.0};
  const double delta_tau_max_{1.0};
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_;
  Eigen::Matrix<double, 6, 6> cartesian_stiffness_target_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_;
  Eigen::Matrix<double, 6, 6> cartesian_damping_target_;
  Eigen::Matrix<double, 7, 1> q_d_nullspace_;
  Eigen::Vector3d position_d_;
  Eigen::Quaterniond orientation_d_;
  Eigen::Vector3d position_d_target_;
  Eigen::Quaterniond orientation_d_target_;
  Eigen::Quaterniond orientation_d_trans_;
  Eigen::Matrix<double, 6, 6> impendance_K_c;
    // disturb force
  Eigen::Matrix<double, 6, 1> force_inc_target ;

  bool first_cycle = true;

  ros::Time nowTime;
  ros::Time lastTime;
  ros::Duration deltaT;


  // Stiffness profile subscriber
  //ros::Subscriber sub_desired_stiffness_;
  //void desiredStiffnessCallback(const geometry_msgs::Vector3StampedConstPtr& msg);

  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Equilibrium pose subscriber
  ros::Publisher pub_endeffector_pose_;
    ros::Publisher pub_tau_residuo_;
      ros::Publisher pub_tau_disturbo_;
      ros::Publisher pub_stima_;
      ros::Publisher pub_error_;
      ros::Publisher pub_tau_d_;
      ros::Publisher pub_force_error;
      ros::Publisher pub_activator;
      ros::Publisher pub_q;
  
    // Stiffness profile subscriber
  ros::Subscriber sub_desired_force_;
  void desiredforceCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
  
  // Disturb force
    ros::Subscriber sub_disturb_force_;
  void disturbforceCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  
  

  //cartesian param
  double impendance_rho;                  // Activation controller function
  double delta_imp_;   

  // Force controller
  double force_error_integral;
    double kp_ {1.5};          // Proportional gain
  double kd_ {0.};           // Derivative   gain
  double ki_ {0.3};          // Integral     gain
  double delta_force_; // Threshold activation controller [m]
   double force_rho;
   Eigen::Matrix<double, 7, 1> tau_ext_initial_;
  Eigen::Matrix<double, 6, 6> damping_c;
  double ee_f_d {0}; 
  double ee_f_d_target;  

  
  // Jacobian
            	Eigen::Matrix<double, 6, 6> J_cr;   // Jacobian rotation center
		Eigen::Matrix<double, 6, 7> J;      // Jacobian
            
  // Object
	    	Eigen::Vector3d ee_p_cr;  	     // Position vector from end effector to object's centre
	    	
	    	
 // CARTESIAN CONTROLLER
    // position error
          Eigen::Matrix<double, 6, 1> error;
          Eigen::Matrix<double, 6, 1> error_prev;
          Eigen::Matrix<double, 6, 1> dot_error_filt;
          Eigen::Vector3d ActualPosition;
    // tau
    	
 // RESIDUO
	// r
		Eigen::Matrix<double, 7, 1> r;
	// i
		Eigen::Matrix<double, 7, 1> I;
	// old variables
               Eigen::Matrix<double, 7, 1> q_old;
               Eigen::Matrix<double, 7, 1> dq_old;
               Eigen::Matrix<double, 7, 7> C_old;
               Eigen::Matrix<double, 7, 1> r_old;
               Eigen::Matrix<double, 7, 1> tau_d_old;
               Eigen::Matrix<double, 7, 7> C;
               Eigen::Matrix<double, 7, 7> M;
		
ros::Publisher pub_tauD;

};

}  // namespace franka_example_controllers
