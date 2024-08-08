// Copyright (c) 2023 Franka Robotics GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
// INCLUDE SECTION
            #include <franka_example_controllers/reaching.h>
            #include <cmath>
            #include <memory>
            #include <iostream>
            #include <controller_interface/controller_base.h>
            #include <franka/robot_state.h>
            #include <pluginlib/class_list_macros.h>
            #include <ros/ros.h>
            #include <franka_example_controllers/pseudo_inversion.h>
            #include <franka_example_controllers/get_CoriolisMatrix.h>
            #include "franka_example_controllers/pseudo_inversion_moore.h"

namespace franka_example_controllers {

bool reaching::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
  std::vector<double> cartesian_stiffness_vector;
  std::vector<double> cartesian_damping_vector;

// SUBSCRIBERS
  // Franka pose subscriber
            sub_equilibrium_pose_ = node_handle.subscribe("equilibrium_pose", 1, &reaching::equilibriumPoseCallback, this,ros::TransportHints().reliable().tcpNoDelay());
  // Desired Contact force subscriber
            sub_desired_force_ = node_handle.subscribe("desired_force", 1, &reaching::desiredforceCallback, this,ros::TransportHints().reliable().tcpNoDelay());
// PUBLISHERS
  // Pose ee
            pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("franka_ee_pose", 1);
  // RESIDUAL MEASURE CHECK
            // Residual measure
                  pub_stima_ = node_handle.advertise<geometry_msgs::WrenchStamped>("stima_forza", 1);
  // Force control check          
            pub_force_error = node_handle.advertise<geometry_msgs::Vector3Stamped>("force_control_errors", 1);

  pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("franka_ee_pose", 1);


  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceExampleController: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceExampleController: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
        model_interface->getHandle(arm_id + "_model"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
        state_interface->getHandle(arm_id + "_robot"));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceExampleController: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceExampleController: Exception getting joint handles: " << ex.what());
      return false;
    }
  }

  dynamic_reconfigure_compliance_param_node_ =
      ros::NodeHandle(node_handle.getNamespace() + "/dynamic_reconfigure_compliance_param_node");

  dynamic_server_compliance_param_ = std::make_unique<
      dynamic_reconfigure::Server<franka_example_controllers::compliance_paramConfig>>(

      dynamic_reconfigure_compliance_param_node_);
  dynamic_server_compliance_param_->setCallback(
      boost::bind(&reaching::complianceParamCallback, this, _1, _2));

// Positions 
                      position_d_.setZero();
                      orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
                      position_d_target_.setZero();
                      orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
          // Stiffness / damping
                      cartesian_stiffness_.setZero();
                      cartesian_damping_.setZero();
                      impendance_K_c.setIdentity();
                      damping_c.setIdentity();
          // Residual
                      r.setZero();           
                      I.setZero();  
          // force integral error 
                      force_error_integral=0.0;
          // Position Vector 
                      ee_p_cr = Eigen::Vector3d(0.0, 0.0, 0.21);
                      ee_f_d=0;

  return true;
}

void reaching::starting(const ros::Time& /*time*/) {
  // INIT PARAM
              franka::RobotState initial_state = state_handle_->getRobotState();
              std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
              std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
  // MAP TO EIGEN
              Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
              Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
              Eigen::Map<Eigen::Matrix<double, 7, 1>> dq_initial(initial_state.dq.data());
              Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  // RESIDUALS
              double Coriolis_matrix_array[49];
              get_CoriolisMatrix(q_initial.data(), dq_initial.data(), Coriolis_matrix_array);
              C = Eigen::Map<Eigen::Matrix<double, 7, 7>>(Coriolis_matrix_array);
  // SET EQUILIBRIUM POINT TO CURRENT STATE
              Eigen::Vector3d correction(-0.0, -0.65, 0.0);
              position_d_ = initial_transform.translation()+correction+initial_transform.rotation()*ee_p_cr;
              orientation_d_ = Eigen::Quaterniond(initial_transform.rotation());
              position_d_target_ = initial_transform.translation()+correction+initial_transform.rotation()*ee_p_cr;
              orientation_d_target_ = Eigen::Quaterniond(initial_transform.rotation());
  // ADAPTATION POLICY INIT
              impendance_rho =1;   // Launchs active impendance control
              delta_imp_ = 0.01;   // Threshold activation controller [m]
              force_rho = 0;       // Launchs inactive force control
              delta_force_ = 0.05; // Threshold activation controller [m]
  // dq NULLSPACE
              q_d_nullspace_ = q_initial;
  // STIFFNESS ROTATION FROM EE TO ROBOT
              Eigen::Matrix<double, 6, 6> EeToBase;
              EeToBase.setIdentity();
              EeToBase.topLeftCorner(3,3) = initial_transform.rotation();
              EeToBase.bottomRightCorner(3,3) = initial_transform.rotation();   
              impendance_K_c = EeToBase*cartesian_stiffness_*EeToBase.inverse();
              damping_c = EeToBase*cartesian_damping_*EeToBase.inverse();
  // RESIDUALS INIT
              q_old  = q_initial;
              dq_old  = dq_initial;
              C_old   = C;
              r_old   = r;
              tau_d_old.setZero();
}

void reaching::update(const ros::Time& /*time*/,
                                                 const ros::Duration& /*period*/) {
  // STATE VARIABLES
              franka::RobotState robot_state = state_handle_->getRobotState();
              std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
              std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
              std::array<double, 49> mass_array = model_handle_->getMass();   
  // MAP TO EIGEN
              Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
              Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
              Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
              Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
              Eigen::Map<Eigen::Matrix<double, 7, 7>> Mass (mass_array.data());
              Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());
              Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
              Eigen::Vector3d position(transform.translation());
              Eigen::Quaterniond orientation(transform.rotation());
              Eigen::MatrixXd jacobian_transpose_pinv;    
              pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
              Eigen::MatrixXd combinedMatrix(6, 3);   
  // JACOBIAN TIP EE TO OBJECT CENTER 
              Eigen::Matrix3d skewMatrix = (Eigen::Matrix3d() <<    
                               0,         -ee_p_cr(2),     ee_p_cr(1),
                               ee_p_cr(2), 0,             -ee_p_cr(0),
                              -ee_p_cr(1), ee_p_cr(0),     0).finished(); //matrice antisimmetrica del vettore posizione ee-cr
              J_cr.setIdentity();                      
              J_cr.topRightCorner(3,3) << skewMatrix.transpose();   
              J = J_cr*jacobian; 
  // CARTESIAN CONTROLLER 
              // Compute error to desired pose
              Eigen::Matrix<double, 6, 1> error;
                      // position error
                      Eigen::Vector3d correction(-0.0, -0.65, 0.0);
                      ActualPosition = position+correction+transform.rotation()*ee_p_cr;
                      error.head(3) << ActualPosition - position_d_;
              // Orientation error
                      if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
                          orientation.coeffs() << -orientation.coeffs();}
                      // "difference" quaternion
                      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
                      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                      // Transform to base frame
                      error.tail(3) << -transform.rotation() * error.tail(3);
              // Velocity error
                      Eigen::VectorXd x_dot(6), dot_position_d(6), dot_error(6);
                      x_dot = J * dq;
                      dot_position_d << 0.0,0.0,0.0,0.0,0.0,0.0;
                      dot_error = dot_position_d - x_dot; 
  // POSITION ERROR ALONG Z DIRECTION IN EE FRAME
              Eigen::Vector3d ee_x;
              ee_x = -transform.rotation().inverse()*error.head(3); 
              double ee_x_z = ee_x(2);
  // STIFFNESS ADAPTATION POLICY
              if (ee_x_z >= delta_imp_ ) {                      
                      impendance_rho = 1; } 
              else if (ee_x_z >= 0 && ee_x_z < delta_imp_ ) {
                      impendance_rho = 0.5*(1- cos(M_PI *(ee_x_z/delta_imp_)) ) ; }
              else {
                      impendance_rho = 0;}
              // Update Stiffness 
              cartesian_stiffness_(2,2)  = impendance_rho*cartesian_stiffness_(0,0);
              cartesian_damping_(2, 2) = 2 * sqrt(cartesian_stiffness_(2,2));
  // ROTATION STIFFNESS FROM EE TO BASE
              Eigen::Matrix<double, 6, 6> EeToBase;
              EeToBase.setIdentity();
              EeToBase.topLeftCorner(3,3) = transform.rotation();
              EeToBase.bottomRightCorner(3,3) = transform.rotation();
              impendance_K_c = EeToBase*cartesian_stiffness_*EeToBase.inverse(); // Stiffness Rotation
              //std::cout<<"cartesian"<<std::endl<<cartesian_stiffness_<<std::endl;
              
              damping_c = EeToBase*cartesian_damping_*EeToBase.inverse();        // Damping Rotation
  // INIT TORQUES
              Eigen::VectorXd tau_task(7),tau_nullspace(7),tau_d(7),tau_imp(7);
  // Cartesian PD control with damping ratio = 1
              tau_task << J.transpose() *(-impendance_K_c * error - damping_c * (jacobian * dq));
  // nullspace PD control with damping ratio = 1
              Eigen::MatrixXd J_transpose_pinv;
              pseudoInverse_cart(J.transpose(), J_transpose_pinv);
              tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                               J.transpose() * J_transpose_pinv) *
                               (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                               (2.0 * sqrt(nullspace_stiffness_)) * dq);
  // Desired torque
              tau_imp = tau_task + tau_nullspace + coriolis ;

  // Force Control
              Eigen::VectorXd tau_frc(7), force_measured_ee(3),stima(6);
  // Force Control
              Eigen::Matrix<double, 7, 1> tau_measured_bias = r ;
              Eigen::Matrix<double, 6, 1> force_measured = jacobian_transpose_pinv * tau_measured_bias;   // external force measured
    
              //force_measured_ee = transform.rotation().inverse()*force_measured.head(3);
              //double ee_f_ext_z = force_measured_ee(2);     
              double ee_f_ext_z = force_measured(1);
              // Force error
                            double ee_f_ext_tilde = ee_f_d + ee_f_ext_z ;    // Force error
                            force_error_integral += 0.001*(ee_f_ext_tilde);  // error integral
              // Controllo in Forza 
                            double ee_f_frc = ee_f_d + kp_ * ee_f_ext_tilde + ki_ * force_error_integral; // forza controllo
  
              // Rho Calculation
              if (fabs(ee_x_z) >= 2*delta_force_ ) {                                               // Determino la rho force  
                  force_rho = 0;
              } else if (fabs(ee_x_z) >= delta_force_ && fabs(ee_x_z) < 2*delta_force_ ) {
                  force_rho = 0.5*(1+ cos(M_PI *(ee_x_z/delta_force_ -1 )) ) ;
              } else {
                  force_rho = 1;}
              
              Eigen::Matrix3d eeR = orientation.normalized().toRotationMatrix();                   // Matrice rotazione end-effector (XYZ)
              // Force control torque
              combinedMatrix << eeR, Eigen::Matrix3d::Zero();          // Matrice 6x3 --> Rotazione[3x3] - Nulla[3x3]   
              Eigen::Vector3d vettore; 
              vettore << 0.0, 0.0 , force_rho*ee_f_frc;                            // Vettore forza 
              tau_frc = jacobian.transpose()*combinedMatrix*vettore;   // Coppia risultante del controllo forza     
  // OUTPUT TAU         
              tau_d << tau_imp+tau_frc;
              stima = jacobian_transpose_pinv * r;

        // Force estimate
              geometry_msgs::WrenchStamped msg_stima_;
              msg_stima_.wrench.force.x = stima(0);
              msg_stima_.wrench.force.y = stima(1);
              msg_stima_.wrench.force.z = stima(2);
              msg_stima_.wrench.torque.x = stima(3);
              msg_stima_.wrench.torque.y = stima(4);
              msg_stima_.wrench.torque.z = stima(5);
              pub_stima_.publish(msg_stima_);

              geometry_msgs::PoseStamped msg_endeffector_pose_;
              msg_endeffector_pose_.pose.position.x = ActualPosition(0);
              msg_endeffector_pose_.pose.position.y = ActualPosition(1);
              msg_endeffector_pose_.pose.position.z = ActualPosition(2);
              msg_endeffector_pose_.pose.orientation.x = orientation.x();
              msg_endeffector_pose_.pose.orientation.y = orientation.y();
              msg_endeffector_pose_.pose.orientation.z = orientation.z();
              msg_endeffector_pose_.pose.orientation.w = orientation.w();
              pub_endeffector_pose_.publish(msg_endeffector_pose_);

                      // force check
              geometry_msgs::Vector3Stamped msg_force_error;
              msg_force_error.vector.x    = ee_f_frc;               // risultato controllo in forza
              msg_force_error.vector.y    = ee_f_ext_tilde;         // errore istantaneo
              msg_force_error.vector.z    = force_error_integral;   // errore accumulato
              pub_force_error.publish(msg_force_error);















// RESIDUAL COMPUTATION
    //  Init param (vedere cosa portare in init )                                  
        // KO posso portarlo nell'init
            double K0 = 10;
        // Frequenza e tempo di campionamento
            double T = 0.001;                  
        // getting dynamic param for residual 
            // get Coriolis Matrix
                double Coriolis_matrix_array[49];
                get_CoriolisMatrix(q.data(), dq.data(), Coriolis_matrix_array);
                C = Eigen::Map<Eigen::Matrix<double, 7, 7>>(Coriolis_matrix_array);
    // Tustin i(k)-i(k-1)= T/2*(ek + ek-1)
        auto Beta_old =  - C_old.transpose()*dq_old;
        auto Beta = -C.transpose()*dq;
        auto p = Mass * dq;
        auto A_k = (T/2) * (tau_d - Beta + tau_d_old - Beta_old);                   
        r = (K0 * (p - I - A_k - (T/2)*r_old))/(1+(K0*T/2));
        I = I + A_k + (T/2)*(r + r_old);
    // Update parametri dinamici _old
        q_old  = q;
        dq_old = dq;
        C_old  = C;
        r_old  = r;
        tau_d_old = tau_d;

  // Saturate torque rate to avoid discontinuities
  tau_d << saturateTorqueRate(tau_d, tau_J_d);
  for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_d(i));
  }

  // update parameters changed online either through dynamic reconfigure or through the interactive
  // target by filtering
  cartesian_stiffness_ =
      filter_params_ * cartesian_stiffness_target_ + (1.0 - filter_params_) * cartesian_stiffness_;
  cartesian_damping_ =
      filter_params_ * cartesian_damping_target_ + (1.0 - filter_params_) * cartesian_damping_;
  nullspace_stiffness_ =
      filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
  ee_f_d = ee_f_d_target;
}

Eigen::Matrix<double, 7, 1> reaching::saturateTorqueRate(
    const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
    const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
  Eigen::Matrix<double, 7, 1> tau_d_saturated{};
  for (size_t i = 0; i < 7; i++) {
    double difference = tau_d_calculated[i] - tau_J_d[i];
    tau_d_saturated[i] =
        tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
  }
  return tau_d_saturated;
}

void reaching::complianceParamCallback(
    franka_example_controllers::compliance_paramConfig& config,
    uint32_t /*level*/) {
  cartesian_stiffness_target_.setIdentity();
  cartesian_stiffness_target_.topLeftCorner(3, 3)
      << config.translational_stiffness * Eigen::Matrix3d::Identity()*3;
  cartesian_stiffness_target_.bottomRightCorner(3, 3)
      << config.rotational_stiffness * Eigen::Matrix3d::Identity()*3;
  cartesian_damping_target_.setIdentity();
  // Damping ratio = 1
  cartesian_damping_target_.topLeftCorner(3, 3)
      << 2.0 * sqrt(config.translational_stiffness*3) * Eigen::Matrix3d::Identity();
  cartesian_damping_target_.bottomRightCorner(3, 3)
      << 2.0 * sqrt(config.rotational_stiffness*3) * Eigen::Matrix3d::Identity();
  nullspace_stiffness_target_ = config.nullspace_stiffness;
}

void reaching::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> position_d_target_mutex_lock(
      position_and_orientation_d_target_mutex_);
  position_d_target_ << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;
  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }
}

void reaching::desiredforceCallback(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  ee_f_d_target = msg->vector.z;}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::reaching,
                       controller_interface::ControllerBase)
