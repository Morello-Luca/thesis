// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

// Include section
    #include <panda_controllers/cartesian_impedance_controller_softbots2.h>
    #include <cmath>
    #include <controller_interface/controller_base.h>
    #include <franka/robot_state.h>
    #include <pluginlib/class_list_macros.h>
    #include <ros/ros.h>
    #include "panda_controllers/Torque.h"
    #include "utils/pseudo_inversion.h"
    #include "utils/pseudo_inversion_moore.h"
    #include <utils/get_CoriolisMatrix2.h>
    #include <iostream>
        using namespace std;

namespace panda_controllers {
bool CartesianImpedanceControllerSoftbots2::init(hardware_interface::RobotHW* robot_hw,
                                                ros::NodeHandle& node_handle) {

// SUBSCRIBERS

  // Franka pose subscriber
          sub_equilibrium_pose_ = node_handle.subscribe("equilibrium_pose", 1, &CartesianImpedanceControllerSoftbots2::equilibriumPoseCallback, this,ros::TransportHints().reliable().tcpNoDelay());
  
  // Desired ee force subcriber 
          sub_desired_force_ = node_handle.subscribe("desired_force", 1, &CartesianImpedanceControllerSoftbots2::desiredforceCallback, this,ros::TransportHints().reliable().tcpNoDelay());
  
  // Disturb force
          // applies a known force at the ee, this is used only to test and check the residual perfomance
          // do not use while the control is active
          sub_disturb_force_ = node_handle.subscribe("disturb_force", 1, &CartesianImpedanceControllerSoftbots2::disturbforceCallback, this,ros::TransportHints().reliable().tcpNoDelay());


// PUBLISHERS

  // Pose ee
          pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("franka_ee_pose", 1);
  // tau_d
          pub_tau_d_ = node_handle.advertise<geometry_msgs::WrenchStamped>("taud", 1);
  
  // RESIDUAL MEASURE CHECK
          // Disturbance torque applied with sub
              pub_tau_disturbo_ = node_handle.advertise<geometry_msgs::WrenchStamped>("disturbo", 1);
          // Residual measure
              pub_tau_residuo_  = node_handle.advertise<geometry_msgs::WrenchStamped>("residuo", 1);
          // Force estimate using residual measure
              pub_stima_ = node_handle.advertise<geometry_msgs::WrenchStamped>("stima_forza", 1);
          // Error
              pub_error_ = node_handle.advertise<geometry_msgs::WrenchStamped>("errore_stima", 1);
  // force control check
          pub_force_error = node_handle.advertise<geometry_msgs::Vector3Stamped>("force_control_errors", 1);
  // activators
          pub_activator = node_handle.advertise<geometry_msgs::Vector3Stamped>("activators", 1);
  // joint
          pub_q = node_handle.advertise<sensor_msgs::JointState> ("position", 1);



// HANDLERS
  std::string arm_id;
  if (!node_handle.getParam("arm_id", arm_id)) {
    ROS_ERROR_STREAM("CartesianImpedanceControllerSoftbots: Could not read parameter arm_id");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    ROS_ERROR(
        "CartesianImpedanceControllerSoftbots: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  }

  franka_hw::FrankaModelInterface* model_interface =
      robot_hw->get<franka_hw::FrankaModelInterface>();
  if (model_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerSoftbots: Error getting model interface from hardware");
    return false;
  }
  try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerSoftbots: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerSoftbots: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerSoftbots: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "CartesianImpedanceControllerSoftbots: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "CartesianImpedanceControllerSoftbots: Exception getting joint handles: " << ex.what());
      return false;
    }
  }



// INIT PARAM
  
    // Positions 
          position_d_.setZero();
          orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
          
          position_d_target_.setZero();
          orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;
      
    // Stiffness / damping
          // init 
                  cartesian_stiffness_.setIdentity();
                  cartesian_stiffness_target_.setIdentity();
                  impendance_K_c.setIdentity();

                  cartesian_damping_.setIdentity();
                  cartesian_damping_target_.setIdentity();
                  damping_c.setIdentity();
                  force_inc_target.setZero();

          // Setting
                  cartesian_stiffness_.topLeftCorner(3, 3) << 450*Eigen::Matrix3d::Identity();
                  cartesian_stiffness_.bottomRightCorner(3, 3) << 28*Eigen::Matrix3d::Identity();
                  cartesian_stiffness_target_ = cartesian_stiffness_;
                  
                  cartesian_damping_.topLeftCorner(3, 3) = 2.0 * sqrt (450)*Eigen::Matrix3d::Identity();
                  cartesian_damping_.bottomRightCorner(3, 3) = 2.0 * sqrt(28)*Eigen::Matrix3d::Identity();
                  cartesian_damping_target_ = cartesian_damping_;
      
    // Residual
          r.setZero();           
          I.setZero();  

    // force integral error 
          force_error_integral=0.0;

    // Position Vector 
          ee_p_cr = Eigen::Vector3d(0.0, 0.0, 0.21);
          double ee_f_d {0}; 

  return true;
}



void CartesianImpedanceControllerSoftbots2::starting(const ros::Time& /*time*/) {
  
// Init
    franka::RobotState initial_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();

    // Jacobian
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
      
    // map to eigen
        Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // Residual  
        double Coriolis_matrix_array[49];
        get_CoriolisMatrix2(q_initial.data(), dq_initial.data(), Coriolis_matrix_array);
        C = Eigen::Map<Eigen::Matrix<double, 7, 7>>(Coriolis_matrix_array);

// compute initial velocity with jacobian and set x_attractor and q_d_nullspace to initial configuration
    // set equilibrium point to current state
        Eigen::Vector3d correction(-0.0, 0.65, 0.0);
        position_d_ = initial_transform.translation()+ initial_transform.linear()*ee_p_cr+correction;// + initial_transform.linear()*ee_p_cr;
        orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
  
        position_d_target_ = initial_transform.translation()+ initial_transform.linear()*ee_p_cr+correction ;//+ initial_transform.linear()*ee_p_cr;;
        orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());
   
        // set nullspace equilibrium configuration to initial q
        q_d_nullspace_ = q_initial;


// controllers activation parameters 
    // Impendance
        impendance_rho =1;   // Launchs active impendance control
        delta_imp_ = 0.01;   // Threshold activation controller [m]
    // Force
        force_rho = 0;       // Launchs inactive force control
        delta_force_ = 0.15; // Threshold activation controller [m]

// Rotation stiffness and damping matrix
    // Rotation Matrix init
        Eigen::Matrix<double, 6, 6> EeToBase;
        EeToBase.setIdentity();
        EeToBase.topLeftCorner(3,3) = initial_transform.linear();
        EeToBase.bottomRightCorner(3,3) = initial_transform.linear();
    
    // Matrix rotation    
        impendance_K_c = EeToBase*cartesian_stiffness_*EeToBase.inverse();
        damping_c = EeToBase*cartesian_damping_*EeToBase.inverse();
          
// Residual first cycle
     q_old  = q_initial;
    dq_old  = dq_initial;
    C_old   = C;
    r_old   = r;
    tau_d_old.setZero();    
}



void CartesianImpedanceControllerSoftbots2::update(const ros::Time& /*time*/,
                                                  const ros::Duration& /*period*/) {

// INIT 

    // Get state variables
        franka::RobotState robot_state = state_handle_->getRobotState();
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        std::array<double, 49> mass_array = model_handle_->getMass();    

    // Map to Eigen
        Eigen::Map<Eigen::Matrix<double, 7,1>> coriolis(coriolis_array.data());
        Eigen::Map<Eigen::Matrix<double, 6,7>> jacobian(jacobian_array.data());
        Eigen::Map<Eigen::Matrix<double, 7,1>> q(robot_state.q.data());
        Eigen::Map<Eigen::Matrix<double, 7,1>> dq(robot_state.dq.data());
        Eigen::Map<Eigen::Matrix<double, 7,1>> tau_J_d(robot_state.tau_J_d.data());  
        Eigen::Map<Eigen::Matrix<double, 7, 7>> Mass (mass_array.data());
        Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
        Eigen::Vector3d position(transform.translation());
        Eigen::Quaterniond orientation(transform.linear());

    // Force control Parameters         
        // Jacobian Pseudoinverse
        Eigen::MatrixXd jacobian_transpose_pinv;    
        pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);
    // Rotation matrix init (f_frc)
        Eigen::MatrixXd combinedMatrix(6, 3);                                                          



    // Jacobian with position vector 
        // Object's rotation centre  
            // Vettore posizione end-effector centro di rotazione oggetto ASSUNTO NOTO PER IL MOMENTO
            Eigen::Matrix3d skewMatrix = (Eigen::Matrix3d() <<    
                               0,         -ee_p_cr(2),     ee_p_cr(1),
                               ee_p_cr(2), 0,             -ee_p_cr(0),
                              -ee_p_cr(1), ee_p_cr(0),     0).finished(); //matrice antisimmetrica del vettore posizione ee-cr
        J_cr.setIdentity();                      
        J_cr.topRightCorner(3,3) << skewMatrix.transpose();   
        J = J_cr*jacobian;  



// CARTESIAN CONTROLLER
    // compute error to desired pose
        // position error
            Eigen::Vector3d correction(-0.0, 0.65, 0.0);
            ActualPosition = position +transform.linear()*ee_p_cr+ correction;// +transform.linear()*ee_p_cr;
            error.head(3) << ActualPosition - position_d_;
        
        // orientation error
            if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
                orientation.coeffs() << -orientation.coeffs();}
            Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
            error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
            
            // Transform to base frame
                error.tail(3) << -transform.linear() * error.tail(3);
    
    // compute velocity error
        Eigen::VectorXd x_dot(6), dot_position_d(6), dot_error(6);
        x_dot = J * dq;
        dot_position_d << 0.0,0.0,0.0,0.0,0.0,0.0;
        dot_error = dot_position_d - x_dot; 

    // compute z error ee base
        Eigen::Vector3d ee_x;
        ee_x = -transform.linear().inverse()*error.head(3); 
        double ee_x_z = ee_x(2);

    // Stiffness variation
        // Rho impendance Calculation
              if (ee_x_z >= delta_imp_ ) {                      
                  impendance_rho = 1;
              } else if (ee_x_z >= 0 && ee_x_z < delta_imp_ ) {
                  impendance_rho = 0.5*(1- cos(M_PI *(ee_x_z/delta_imp_)) ) ;
              } else {
                  impendance_rho = 0;}
        
        // Update Stiffness 
              cartesian_stiffness_(2,2)  = impendance_rho*450;
              cartesian_damping_(2, 2) = 2 * sqrt(cartesian_stiffness_(2,2));
        
        // Rotation matrix from ee to base for impedance / damping
              Eigen::Matrix<double, 6, 6> EeToBase;
              EeToBase.setIdentity();
              EeToBase.topLeftCorner(3,3) = transform.linear();
              EeToBase.bottomRightCorner(3,3) = transform.linear();

              impendance_K_c = EeToBase*cartesian_stiffness_*EeToBase.inverse(); // Stiffness Rotation
              damping_c = EeToBase*cartesian_damping_*EeToBase.inverse();        // Damping Rotation


    Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7),tau_imp(7);
    // pseudoinverse for nullspace handling
      // kinematic pseuoinverse
          Eigen::MatrixXd J_transpose_pinv;
          pseudoInverse_cart(J.transpose(), J_transpose_pinv);
          tau_nullspace << (Eigen::MatrixXd::Identity(7, 7) -
                            J.transpose() * J_transpose_pinv) *
                           (nullspace_stiffness_ * (q_d_nullspace_ - q) -
                           (2.0 * sqrt(nullspace_stiffness_)) * dq);

    tau_task << J.transpose() * (- impendance_K_c *  error + damping_c * dot_error);
    tau_imp = tau_task + tau_nullspace;

// Force Control
    Eigen::VectorXd tau_frc(7), force_measured_ee(3);
    
    // Filter tau measure
    Eigen::Matrix<double, 7, 1> tau_measured_bias = r ;
    Eigen::Matrix<double, 6, 1> force_measured = jacobian_transpose_pinv * tau_measured_bias;   // external force measured
    

    force_measured_ee = transform.linear().inverse()*force_measured.head(3);
    //cout<<"forza ee base  "<<endl<<force_measured_ee<<endl;

    double ee_f_ext_z = force_measured_ee(2);       // external frc z component
    
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

            cout<<"errore lungo z panda2 "<<endl<<ee_x_z<<endl;
            //cout<<"panda1 force_rho "<<endl<<force_rho<<endl;
            //cout<<"panda1 impedance_rho "<<endl<<impendance_rho<<endl;

    Eigen::Matrix3d eeR = orientation.normalized().toRotationMatrix();                   // Matrice rotazione end-effector (XYZ)
    // Force control torque
        combinedMatrix << eeR, Eigen::Matrix3d::Zero();          // Matrice 6x3 --> Rotazione[3x3] - Nulla[3x3]   
        Eigen::Vector3d vettore; 


        vettore << 0.0, 0.0 , force_rho*ee_f_frc;                            // Vettore forza 
        //cout<<"vettore " <<endl<<vettore<<endl;        
        tau_frc = jacobian.transpose()*combinedMatrix*vettore;   // Coppia risultante del controllo forza     


    // Assembly the results         
      tau_d << tau_imp+tau_frc;
    // tau dist
      Eigen::VectorXd tau_dist(7), incognita(7), stima(6), riconversione(6), error_res(6);
      incognita = jacobian.transpose()*force_inc_target;
      
      riconversione = jacobian_transpose_pinv * incognita;
      
      // Conditioning product pinv(J^T)*J^T
      Eigen::JacobiSVD<Eigen::MatrixXd> svd1(jacobian_transpose_pinv*jacobian.transpose()); 
      double cond1 = svd1.singularValues()(0) / svd1.singularValues()(svd1.singularValues().size()-1); 
      // Conditioning J^T
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian.transpose()); 
      double cond = svd.singularValues()(0) / svd.singularValues()(svd.singularValues().size()-1); 
      // Rank J^T
      Eigen::FullPivLU<Eigen::Matrix<double, 7, 6>> lu_decomp(jacobian.transpose());
      auto rank = lu_decomp.rank();
      


      //cout<<"prodotto"<<endl<<jacobian_transpose_pinv*jacobian.transpose()<<endl;
      //cout<<"tau frc panda1"<<endl<<tau_frc<<endl;


      tau_dist = tau_d + incognita;

      stima = jacobian_transpose_pinv * r;

      error_res = stima - force_inc_target;


// PUBBLICO POSIZIONE E VELCITA
  sensor_msgs::JointState q_msg;
  q_msg.position.assign(robot_state.q.begin(), robot_state.q.end());
  q_msg.velocity.assign(robot_state.dq.begin(), robot_state.dq.end());
  
  q_msg.header.stamp = ros::Time::now();
  pub_q.publish(q_msg);

// PUBLISHERS
        
    // External distrurb torque
        geometry_msgs::WrenchStamped msg_dist_;
   
        msg_dist_.wrench.force.x = incognita(0);
        msg_dist_.wrench.force.y = incognita(1);
        msg_dist_.wrench.force.z = incognita(2);
        msg_dist_.wrench.torque.x = incognita(3);
        msg_dist_.wrench.torque.y = incognita(4);
        msg_dist_.wrench.torque.z = incognita(5);
   
        pub_tau_disturbo_.publish(msg_dist_);


    // External distrurb torque
        geometry_msgs::WrenchStamped msg_taud_;
   
        msg_taud_.wrench.force.x = tau_d(0);
        msg_taud_.wrench.force.y = tau_d(1);
        msg_taud_.wrench.force.z = tau_d(2);
        msg_taud_.wrench.torque.x = tau_d(3);
        msg_taud_.wrench.torque.y = tau_d(4);
        msg_taud_.wrench.torque.z = tau_d(5);
   
        pub_tau_d_.publish(msg_taud_);


    // Residuo       
        geometry_msgs::WrenchStamped msg_residuo_;
        
        msg_residuo_.wrench.force.x = r(0);
        msg_residuo_.wrench.force.y = r(1);
        msg_residuo_.wrench.force.z = r(2);
        msg_residuo_.wrench.torque.x = r(3);
        msg_residuo_.wrench.torque.y = r(4);
        msg_residuo_.wrench.torque.z = r(5);
        
        pub_tau_residuo_.publish(msg_residuo_);

        
    // Force estimate
        geometry_msgs::WrenchStamped msg_stima_;
            
        msg_stima_.wrench.force.x = stima(0);
        msg_stima_.wrench.force.y = stima(1);
        msg_stima_.wrench.force.z = stima(2);
        msg_stima_.wrench.torque.x = stima(3);
        msg_stima_.wrench.torque.y = stima(4);
        msg_stima_.wrench.torque.z = stima(5);
        
        pub_stima_.publish(msg_stima_);

    // Force error
        geometry_msgs::WrenchStamped msg_error_;
            
        msg_error_.wrench.force.x = error_res(0);
        msg_error_.wrench.force.y = error_res(1);
        msg_error_.wrench.force.z = error_res(2);
        msg_error_.wrench.torque.x = error_res(3);
        msg_error_.wrench.torque.y = error_res(4);
        msg_error_.wrench.torque.z = error_res(5);
        
        pub_error_.publish(msg_error_);




    // Pub ee pose
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



    // force check
        geometry_msgs::Vector3Stamped msg_activate;
    
        msg_activate.vector.x    = force_rho;              // force activator 
        msg_activate.vector.y    = impendance_rho;         // errore istantaneo
        msg_activate.vector.z    = 0;                      // null

        pub_activator.publish(msg_activate);


// RESIDUAL COMPUTATION

    //  Init param (vedere cosa portare in init )                                  
        // KO posso portarlo nell'init
            double K0 = 10;
        // Frequenza e tempo di campionamento
            double T = 0.001;                  

        // getting dynamic param for residual 
            // get Coriolis Matrix
                double Coriolis_matrix_array[49];
                get_CoriolisMatrix2(q.data(), dq.data(), Coriolis_matrix_array);
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
    
        tau_dist << saturateTorqueRate(tau_dist, tau_J_d);

// Set Commanded torque 
        for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_dist(i));}


        filter_params_ * nullspace_stiffness_target_ + (1.0 - filter_params_) * nullspace_stiffness_;
        position_d_ = filter_params_ * position_d_target_ + (1.0 - filter_params_) * position_d_;
  orientation_d_ = orientation_d_.slerp(filter_params_, orientation_d_target_);
        ee_f_d = ee_f_d_target;
}

Eigen::Matrix<double, 7, 1> CartesianImpedanceControllerSoftbots2::saturateTorqueRate(
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




void CartesianImpedanceControllerSoftbots2::desiredforceCallback(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  ee_f_d_target = msg->vector.z;
  
}


  void CartesianImpedanceControllerSoftbots2::disturbforceCallback(
    const geometry_msgs::TwistStampedConstPtr& msg){
      force_inc_target <<   msg->twist.linear.x,
                            msg->twist.linear.y,
                            msg->twist.linear.z,
                            msg->twist.angular.x,
                            msg->twist.angular.y,
                            msg->twist.angular.z; } 


void CartesianImpedanceControllerSoftbots2::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
    
    double assex = msg->pose.position.x;
    double assey = msg->pose.position.y;
    double assez = msg->pose.position.z;
  
    position_d_target_ << assex,assey,assez ;

    double coeffx=msg->pose.orientation.x;
    double coeffy=msg->pose.orientation.y;
    double coeffz=msg->pose.orientation.z;
    double coeffw=msg->pose.orientation.w;

    Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
    orientation_d_target_.coeffs() <<coeffx,coeffy,coeffz,coeffw;

    if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
      orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
    }

    Eigen::AngleAxisd aa_orientation_recieved(orientation_d_target_);

}

}  // namespace panda_controllers

PLUGINLIB_EXPORT_CLASS(panda_controllers::CartesianImpedanceControllerSoftbots2,
                       controller_interface::ControllerBase)


