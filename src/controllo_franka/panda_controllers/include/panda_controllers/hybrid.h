#pragma once

#include <memory>
#include <string>
#include <vector>

#include <array>
#include <string>
#include <vector>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <controller_interface/multi_interface_controller.h>
#include <dynamic_reconfigure/server.h>

//#include <franka_example_controllers/compliance_paramConfig.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>

#include <ros/console.h>
#include <ros/node_handle.h>
#include <ros/time.h>

#include <franka/robot_state.h>

//Ros Message
#include <sensor_msgs/JointState.h>

//Message
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>




#define     DEBUG   0      

namespace panda_controllers
{

class hybrid : public controller_interface::MultiInterfaceController<   franka_hw::FrankaModelInterface,
                                                                        hardware_interface::EffortJointInterface, 
                                                                        franka_hw::FrankaStateInterface>
{ 
public: 
    bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle &node_handle)override;
    void starting(const ros::Time&)override;
    void update(const ros::Time&, const ros::Duration& period)override;

private:
      // Saturation
  	Eigen::Matrix<double, 7, 1> saturateTorqueRate(
      const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
      const Eigen::Matrix<double, 7, 1>& tau_J_d);  // NOLINT (readability-identifier-naming)

    std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
    std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
    std::vector<hardware_interface::JointHandle> joint_handles_;
  
    const double delta_tau_max_{1.0};
    
    /* Definig the timing */
    double dt;
    
    //______________ PARAMETERS ______________________

    // DYNAMIC PARAMS
    Eigen::Matrix<double, 7, 7> M;      // Mass
    Eigen::Matrix<double, 6, 6> M_c;    // Mass in operation space
    Eigen::Matrix<double, 6, 6> M_imp;  // Mass in cartesian space

    Eigen::Matrix<double, 7, 7> C;      // Coriolis
    Eigen::Matrix<double, 6, 6> C_c;    // Coriolis in operation space
    Eigen::Matrix<double, 6, 6> C_imp;  // Coriolis in cartesian space

    Eigen::Matrix<double, 6, 6> J_cr;   // Jacobian rotation center
    Eigen::Matrix<double, 6, 7> J;      // Jacobian 
    //Eigen::Matrix<double, 6, 6> Jee;    // Jacobian robotic arm 

    // CARTESIAN PARAMS
    Eigen::Matrix<double, 6, 6> impendance_K;   // Stiffness
    Eigen::Matrix<double, 6, 6> impendance_K_c; // Stiffness frame corretto
    Eigen::Matrix<double, 6, 6> impendance_D;   // Damping
    double impendance_rho {1};                  // Activation controller function
    double delta_imp_ {0.01};          	  // Threshold activation controller [m]
    bool flag = true;


    // FORCE PARAMS
    double kp_ {1.5};          // Proportional gain
    double kd_ {0.};           // Derivative   gain
    double ki_ {0.3};          // Integral     gain
    double force_rho {0};                // Activation controller function
    double delta_force_ {0.05}; // Threshold activation controller [m]
    double tau_error_integral;
    Eigen::Matrix<double, 7, 1> tau_ext_initial_;

    // TORQUE 
    Eigen::Matrix<double, 7, 1> tau_imp; // Input torque motion impendance control 
    Eigen::Matrix<double, 7, 1> tau_frc; // Input torque contact force control
    Eigen::Matrix<double, 7, 1> tau_g;   // Gravity compensation torque
    
    // POSITIONS
    // x_d oggetto
        Eigen::Vector3d position_d_;              // Posizione desiderata oggetto
        Eigen::Quaterniond orientation_d_;        // Orientazione desiderata oggetto
        
        Eigen::Vector3d dot_position_d_;          // Velocità desiderata oggetto
        Eigen::Vector3d dot_orientation_d_;    // Orientazione desiderata oggetto

        Eigen::Vector3d ddot_position_d_;         // Accelerazione desiderata oggetto
        Eigen::Vector3d ddot_orientation_d_;   // Orientazione desiderata oggetto
        
        
        Eigen::Vector3d ee_p_cr;            // vettore posizione punto di contatto -> centro di rotazione in terna end - effector
        Eigen::Matrix<double, 3, 1> ActualPosition;   // Posizione centro di massa attuale
        Eigen::Quaterniond ActualOrientation;         // Orientazione centri di rotazione
        Eigen::Matrix<double, 6, 1> x_dot;            // Velocità centro di massa attuale
        Eigen::Vector3d position_d_target_;
        Eigen::Quaterniond orientation_d_target_;

    // x 
        Eigen::Matrix<double, 7, 1> q_curr;          // Posizione dei giunti, x = J(q)q
    //________________________ END PARAMETERS _______________________________
      

    bool first_cycle = true;

  ros::Time nowTime;
  ros::Time lastTime;
  ros::Duration deltaT;




  // Equilibrium pose subscriber
  ros::Subscriber sub_equilibrium_pose_;
  void equilibriumPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  // Equilibrium pose subscriber
  ros::Publisher pub_endeffector_pose_;
};

}
