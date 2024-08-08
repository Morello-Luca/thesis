//_________________ INCLUDE SECTION __________________________

#include <panda_controllers/hybrid.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include "utils/pseudo_inversion.h"
#include <franka/model.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <franka/duration.h>
#include <franka/exception.h>

#include <iostream>
using namespace std;

namespace panda_controllers {

//___________________________ INIT _________________________________

bool hybrid::init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) {

	// ____________ NODES CALLBACK ___________________________

	//___________________ EQUILIBRIUM POSE ___________________
		//_______________________ SUBSCRIBER _________________ 
	sub_equilibrium_pose_ = node_handle.subscribe(
      "equilibrium_pose", 1, &hybrid::equilibriumPoseCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());
		// ______________________ PUBLISHER __________________
   	pub_endeffector_pose_ = node_handle.advertise<geometry_msgs::PoseStamped>("franka_ee_pose", 1);
   	//_____________ ARM INIT _________________________________
   	//_________________ ARM_ID INIT __________________________ 
   	std::string arm_id;
  	if (!node_handle.getParam("arm_id", arm_id)) {
    	ROS_ERROR_STREAM("Hybrid: Could not read parameter arm_id");
    	return false;
  	}
  	//_________________ JOINTS INIT __________________________
  	std::vector<std::string> joint_names;
  	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
    	ROS_ERROR(
        "Hybrid: Invalid or no joint_names parameters provided, "
        "aborting controller init!");
    return false;
  	}

  	//_____________ HANDLERS _________________________________
  	franka_hw::FrankaModelInterface* model_interface =
    robot_hw->get<franka_hw::FrankaModelInterface>();
  	if (model_interface == nullptr) {
    	ROS_ERROR_STREAM(
        	"Hybrid: Error getting model interface from hardware");
    return false;
  	}

    try {
    model_handle_.reset(
        new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
  	} 
  	catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Hybrid: Exception getting model handle from interface: "
        << ex.what());
    return false;
  }

  franka_hw::FrankaStateInterface* state_interface =
      robot_hw->get<franka_hw::FrankaStateInterface>();
  if (state_interface == nullptr) {
    ROS_ERROR_STREAM(
        "Hybrid: Error getting state interface from hardware");
    return false;
  }
  try {
    state_handle_.reset(
        new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
  } catch (hardware_interface::HardwareInterfaceException& ex) {
    ROS_ERROR_STREAM(
        "Hybrid: Exception getting state handle from interface: "
        << ex.what());
    return false;
  }

  hardware_interface::EffortJointInterface* effort_joint_interface =
      robot_hw->get<hardware_interface::EffortJointInterface>();
  if (effort_joint_interface == nullptr) {
    ROS_ERROR_STREAM(
        "Hybrid: Error getting effort joint interface from hardware");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    try {
      joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
    } catch (const hardware_interface::HardwareInterfaceException& ex) {
      ROS_ERROR_STREAM(
          "Hybrid: Exception getting joint handles: " << ex.what());
      return false;
    }
  }
//_________________ END HANDLERS _____________________________
//_________________ INIT PARAMETERS __________________________
//______________________ POSITION ____________________________  
  position_d_.setZero();
  orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  position_d_target_.setZero();
  orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

  ddot_position_d_.setZero();
  ddot_orientation_d_.setZero();

  dot_position_d_.setZero();
  dot_orientation_d_.setZero();
  
  

//______________________ IMPENDANCE PARAM ____________________
  impendance_K.setIdentity(); // Init Stiffness
  impendance_D.setIdentity(); // Init Damping
  impendance_rho = 1;

  impendance_K.topLeftCorner(3, 3) 		<< 1000*Eigen::Matrix3d::Identity(); // Traslational stiffness
  impendance_K(2,2) = impendance_rho*1000;										 // Activation-z param
  impendance_K.bottomRightCorner(3, 3) 	<< 150*Eigen::Matrix3d::Identity();	 // Rotational stiffness


  impendance_D.topLeftCorner(3, 3) = 2 * sqrt(1000) *Eigen::Matrix3d::Identity();					// traslational damping
  impendance_D.bottomRightCorner(3, 3) = 2 * sqrt(150)*Eigen::Matrix3d::Identity();	// Rotational damping


  return true;
}
//___________________________ END INIT _________________________________

//___________________________ STARTING _________________________________

void hybrid::starting(const ros::Time& /*time*/) {
  // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
  // to initial configuration
  franka::RobotState initial_state = state_handle_->getRobotState();
  // get jacobian
  std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

  Eigen::Map<Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > dq_initial(initial_state.dq.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > q_initial(initial_state.q.data());
  Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

  // set equilibrium point to current state
  Eigen::Vector3d ee_p_cr(0.0,0.0, 0.01);
  Eigen::Quaterniond orientation(initial_transform.linear());                           // Orientazione end-effector
  auto eeR = orientation.normalized().toRotationMatrix();  
  //position_d_ = initial_transform.translation()+ eeR.transpose()*( ee_p_cr);
  position_d_ = initial_transform.translation();
  orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

  position_d_target_ = initial_transform.translation();
  orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

  //Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data()); portare in starting
  
  // STARTING FORCE CONTROL SECTION
    std::array<double, 7> gravity_array = model_handle_->getGravity(); //array gravità
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(initial_state.tau_J.data()); //tau misurata
    Eigen::Map<Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data()); // gravità
    // CORREZIONE BIAS PER LA COPPIA ESTERNA
    tau_ext_initial_ = tau_measured;
    tau_error_integral=0.0;
}
//___________________________ END STARTING _________________________________

//___________________________ UPDATE _______________________________________

void hybrid::update(const ros::Time& /*time*/,const ros::Duration& /*period*/) {
  
// GETTING STATE VARIABLES FROM ROBOT                                                          //---- Extract data from robot ----// 
	   franka::RobotState robot_state = state_handle_->getRobotState();                                       // Robot state
	   std::array<double, 49> mass_array = model_handle_->getMass();                                          // Mass data array
	   std::array<double, 7> coriolis_array = model_handle_->getCoriolis();                                   // Coriolis data array
       std::array<double, 42> jacobian_array = model_handle_->getZeroJacobian(franka::Frame::kEndEffector);   // Jacobian data array
       std::array<double, 7> gravity_data = model_handle_->getGravity();                                      // Gravity data array

// MAPPING STATE VARIABLES

// ROBOT DYNAMIC PARAMETER
  	    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass_array.data());        // Robot Mass
  	    Eigen::Map<Eigen::Matrix<double, 7, 7>> C(coriolis_array.data());    // Robot Coriolis
  	    Eigen::Map<Eigen::Matrix<double, 6, 7>>Jee(jacobian_array.data());   // Robot arm Jacobian
  	    Eigen::Map<Eigen::Matrix<double, 7, 1>> G(gravity_data.data());      // Robot Gravity
        Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());     // Robot joint position
  	    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());   // Robot joint velocity

// FORCE CONTROL PARAMETER
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_measured(robot_state.tau_J.data()); // Exteral torque measured 
        Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(robot_state.tau_J_d.data());    // Desired torque ??
        double ee_f_d {0};                                                             // Desired force along z-component
        Eigen::MatrixXd combinedMatrix(6, 3);                                           // Rotational matrix and zero matrix stacked
        Eigen::Vector3d vettore;                                                        // Result vector desired force
        Eigen::MatrixXd jacobian_transpose_pinv;                                        // Pseudoinversa Jacobiano trasposto
        pseudoInverse(Jee.transpose(), jacobian_transpose_pinv);

// POSITION AND ORIENTATION END EFFECTOR
  	     Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));	// Posa end effector
  	     Eigen::Vector3d position_(transform.translation());							// Posizione end-effector 
  	     Eigen::Quaterniond orientation(transform.linear());							// Orientazione end-effector
         auto eeR = orientation.normalized().toRotationMatrix();		                // Matrice rotazione end-effector (XYZ)
  	     //auto eeR = orientation.normalized().toRotationMatrix().eulerAngles(0, 1, 2);	// Matrice rotazione end-effector (XYZ)


  // PUBLISH END-EFFECTOR POSE
  	     geometry_msgs::PoseStamped msg_endeffector_pose_;
  	         msg_endeffector_pose_.pose.position.x = position_(0);
             msg_endeffector_pose_.pose.position.y = position_(1);
  	         msg_endeffector_pose_.pose.position.z = position_(2);
  	         msg_endeffector_pose_.pose.orientation.x = orientation.x();
  	         msg_endeffector_pose_.pose.orientation.y = orientation.y();
  	         msg_endeffector_pose_.pose.orientation.z = orientation.z();
  	         msg_endeffector_pose_.pose.orientation.w = orientation.w();
  	     pub_endeffector_pose_.publish(msg_endeffector_pose_);    

// JACOBIAN CENTER ROTATION OBJECT 
        Eigen::Vector3d ee_p_cr(0.0, 0.0, 0.01);   // Vettore posizione end-effector centro di rotazione oggetto ASSUNTO NOTO PER IL MOMENTO
        Eigen::Matrix3d skewMatrix = (Eigen::Matrix3d() <<    
                               0,         -ee_p_cr(2),     ee_p_cr(1),
                               ee_p_cr(2), 0,             -ee_p_cr(0),
                              -ee_p_cr(1), ee_p_cr(0),     0).finished(); //matrice antisimmetrica del vettore posizione ee-cr
        J_cr.setIdentity();                       // Identità 6x6
        J_cr.topRightCorner(3,3) << skewMatrix.transpose();   // inserisco matrice antisimmetrica nel jacobiano

// CONVERT MASS E CORIOLIS IN CARTESIAN SPACE
    // MASS
            M_c = (Jee*M.inverse()*Jee.transpose()).inverse();   // Proiezione Massa in spazio operativo [6x6] appunti robotica
            M_imp = J_cr.transpose().inverse()*M_c*J_cr.inverse();         // Proiezione Massa in spazio cartesiano [6x6]
    
    // CORIOLIS
            C_c = (Jee*C.inverse()*Jee.transpose()).inverse();   // Proiezione Coriolis in spazio operativo [6x6] appunti robotica
            C_imp = J_cr.transpose().inverse()*C_c*J_cr.inverse();         // Proiezione Coriolis in spazio cartesiano [6x6]

    // TEST
         /*   cout<< "M "<< M << endl;
            cout<< "M inversa " << M.inverse()<<endl;
        
            cout<< "C " << C<<endl;
            cout<< "C inversa " << C.inverse()<<endl;
        
            cout<< "jacobian Jee "<< Jee<<endl;
            cout<< "jacobian J_cr " << J_cr<< endl;*/


// COMPUTE OBJECT CENTER OF ROTATION
        J = J_cr*Jee;                                            // Jacobiano completo
        x_dot = J_cr*Jee*dq;                                     // Velocità centro rotazione 

        //ActualPosition = position_ + eeR*( ee_p_cr); // Posizione effettiva
        ActualPosition = position_;
      
        ActualOrientation = orientation;                         // Orientazione effettiva        

// COMPUTE POSITION ERROR
        Eigen::VectorXd error(6), dot_error(6);    // Error Init
            // COMPUTE TRANSLATIONAL ERROR
                    error.head(3) << position_d_ - ActualPosition;	// Compute Error 3x1 (traslational)
            
            // COMPUTE ORIIENTATION ERROR
                    Eigen::Quaterniond error_quaternion(ActualOrientation.inverse() * orientation_d_); // Compute error 3x1 (orientation)
                    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
                    error.tail(3) << -transform.linear() * error.tail(3);

// COMPUTE DOT ERROR
          // Translation error
                    dot_error.head(3) << dot_position_d_ - x_dot.head(3); // derivata errore x tilde 
          // Orientation error
                // convert dot_position_d_ from quaternion to euler
                    dot_error.tail(3) << dot_orientation_d_ - x_dot.tail(3);            // calcolo errore 

// EXTRACT Z-COMPONENT FROM ERROR
        Eigen::Vector3d ee_x = (eeR.transpose()*error.head(3));	   // Ruoto il vettore di errore nella base end-effector
        //cout<<"errore posizione"<<endl<< error.head(3)<<endl;
        //cout<<"position_d_ "<<endl<< position_d_<<endl;
        //cout<<"ActualPosition "<<endl<<ActualPosition<<endl;
        //cout<<"errore nella base end-effector"<<endl<<ee_x<<endl;


        double ee_x_z = ee_x[3];					   // Estraggo la componente z



// --------------------------------------------------------------------------------------------------------------------------
//_______________________________ BEGIN CARTESIAN IMPENDANCE CONTROL ________________________________________________________
// --------------------------------------------------------------------------------------------------------------------------

// RHO IMPENDANCE CALCULATION
   	    if (ee_x_z >= delta_imp_ ) {									    
            impendance_rho = 1;
        } else if (ee_x_z >= 0 && ee_x_z < delta_imp_ ) {
            impendance_rho = 0.5*(1- cos(M_PI *(ee_x_z/delta_imp_)) ) ;
        } else {
            impendance_rho = 0;}
        impendance_K(2,2)  = impendance_rho*1000;
        impendance_D(3, 3) = 2 * sqrt(impendance_K(2,2));

// TRASFORMO LA MATRICE DI STIFFNESS
        Eigen::Matrix<double,6, 6> rotational;
        rotational.setZero();
        rotational.topLeftCorner(3,3) = eeR;
        rotational.bottomRightCorner(3,3) = eeR;
        impendance_K_c = rotational*impendance_K;		// Calcolo la matrice di stiffness nel frame globale

// CALCOLO TAU_IMEDENZA
        Eigen::VectorXd ddot_desired_vector(6),dot_desired_vector(6); 
        
        ddot_desired_vector << ddot_position_d_ , ddot_orientation_d_;
        dot_desired_vector << dot_position_d_ , dot_orientation_d_;  
        
    /*   tau_imp = J.transpose()*(   impendance_K_c * error + 
                                    impendance_D * dot_error + 
                                    M_imp * ddot_desired_vector +
                                    C_imp * dot_desired_vector);  */

        tau_imp = Jee.transpose()*(   -impendance_K * error - 
                                    impendance_D * (Jee*dq) ); 

      /*  cout << "matrice rigidezza " << impendance_K_c << "        errore "<<error<<endl ;
        cout << "Matrice damping "<< impendance_D << "           dot_error"<<dot_error<<endl; 
        cout << "M_imp " << M_imp<<endl;
        cout << "C_imp " << C_imp<<endl;
        cout << "ddot_desired_vector "<<ddot_desired_vector<<endl;
        cout << "dot_desired_vector "<<dot_desired_vector<<endl;*/


        if (flag == true ){

            cout << "coppia impedenza  "<<endl << tau_imp<<endl;
            cout<<"errore "<<endl<< error<<endl;
            cout<< "x_dot"<<endl<<x_dot<<endl;
            cout<< "dot_position_d_"<<endl<<dot_position_d_<<endl;
            cout<<"dot errore "<<endl<< dot_error<<endl;
            cout<<"ee_x_z"<<endl<<ee_x_z<<endl;
            cout<<"Kc"<<endl<<impendance_K_c<<endl;
            cout<<"rotational"<<endl<<rotational<<endl;
            cout<<"k"<<endl<<impendance_K<<endl;
            cout<<"impendance_K_c * error" <<endl<<impendance_K_c * error<<endl;
            cout<<"impendance_D * dot_error" <<endl<<impendance_D * dot_error * error<<endl;
            cout<<"M_imp * ddot_desired_vector" <<endl<<M_imp * ddot_desired_vector<<endl;
            cout<<"C_imp * dot_desired_vector" <<endl<<C_imp * dot_desired_vector<<endl;
            cout<< "jacobian"<<endl<<J<<endl;

            flag=false;
        }




// --------------------------------------------------------------------------------------------------------------------------
// ------------------------   END CARTESIAN IMPENDANCE CONTROL  -------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// ------------------------   BEGIN FORCE CONTROL  --------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// CONVERTO COPPIA ESTERNA IN FORZA
        Eigen::Matrix<double, 7, 1> tau_measured_bias = tau_measured - tau_ext_initial_ ;
        Eigen::Matrix<double, 6, 1> force_measured = jacobian_transpose_pinv * tau_measured_bias;   // forza esterna misurata
        double ee_f_ext_z = force_measured(2);                                                      // forza esterna misurata lungo z

// CALCOLO ERRORE FORZA
        double ee_f_ext_tilde = ee_f_d + 0;    // errore forza 
        //cout << "forza misurata"<<endl<<force_measured<<endl;
        //cout<< "ee_f_ext_z"<<endl<<ee_f_ext_z<<endl;
        //cout << "ee_f_d "<<endl<< ee_f_d <<endl;
        double force_error_integral;
        force_error_integral += 0.001*(ee_f_ext_tilde);           // integrale errore

// CONTROLLO FORZA 
        double ee_f_frc = ee_f_d + kp_ * ee_f_ext_tilde + ki_ * force_error_integral; // forza controllo

// RHO CALCULATION
        if (fabs(ee_x_z) >= 2*delta_force_ ) {                                               // Determino la rho force  
            force_rho = 0;
        } else if (fabs(ee_x_z) >= delta_force_ && fabs(ee_x_z) < 2*delta_force_ ) {
            force_rho = 0.5*(1+ cos(M_PI *(ee_x_z/delta_force_ -1 )) ) ;
        } else {
            force_rho = 1;}

// DETERMINO LA COPPIA DEL CONTROLLO DI FORZA
        combinedMatrix << eeR.transpose(), Eigen::Matrix3d::Zero();     // Matrice 6x3 --> Rotazione[3x3] - Nulla[3x3]   
        //cout<< "combinedMatrix "<<endl<< combinedMatrix<<endl;
        vettore << 0.0, 0.0 , force_rho*ee_f_frc;      // Vettore forza 
        //cout<<"errore posizione z "<<endl<< ee_x_z<<endl;
        //cout<<"vettore forza "<<endl<<vettore<<endl;
        tau_frc = Jee.transpose()*combinedMatrix*vettore;   // Coppia risultante del controllo forza 

// --------------------------------------------------------------------------------------------------------------------------
// ------------------------   END FORCE CONTROL  --------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------
//_______________________________ BEGIN ASSEMBLY ____________________________________________________________________________
// --------------------------------------------------------------------------------------------------------------------------
 Eigen::VectorXd tau_task(7);
 //tau_task = tau_imp + tau_frc;
 tau_task << tau_imp;
 //cout << "coppia impedenza  "<<endl << tau_imp<<endl;
 //cout<<"coppia forza  "<<endl<<tau_frc<<endl;


/* cout << "tau_task joint 1 " << tau_task(1) << endl;
 cout << "tau_task joint 2 " << tau_task(2) << endl;
 cout << "tau_task joint 3 " << tau_task(3) << endl;
 cout << "tau_task joint 4 " << tau_task(4) << endl;
 cout << "tau_task joint 5 " << tau_task(5) << endl;
 cout << "tau_task joint 6 " << tau_task(6) << endl;
 cout << "tau_task joint 7 " << tau_task(7) << endl;
 
 cout << "tau_imp joint 1 " << tau_imp(1) << endl;
 cout << "tau_imp joint 2 " << tau_imp(2) << endl;
 cout << "tau_imp joint 3 " << tau_imp(3) << endl;
 cout << "tau_imp joint 4 " << tau_imp(4) << endl;
 cout << "tau_imp joint 5 " << tau_imp(5) << endl;
 cout << "tau_imp joint 6 " << tau_imp(6) << endl;
 cout << "tau_imp joint 7 " << tau_imp(7) << endl;

 cout << "tau_frc joint 1 " << tau_frc(1) << endl;
 cout << "tau_frc joint 2 " << tau_frc(2) << endl;
 cout << "tau_frc joint 3 " << tau_frc(3) << endl;
 cout << "tau_frc joint 4 " << tau_frc(4) << endl;
 cout << "tau_frc joint 5 " << tau_frc(5) << endl;
 cout << "tau_frc joint 6 " << tau_frc(6) << endl;
 cout << "tau_frc joint 7 " << tau_frc(7) << endl;

 cout << "tau_J_d joint 1 " << tau_J_d(1) << endl;
 cout << "tau_J_d joint 2 " << tau_J_d(2) << endl;
 cout << "tau_J_d joint 3 " << tau_J_d(3) << endl;
 cout << "tau_J_d joint 4 " << tau_J_d(4) << endl;
 cout << "tau_J_d joint 5 " << tau_J_d(5) << endl;
 cout << "tau_J_d joint 6 " << tau_J_d(6) << endl;
 cout << "tau_J_d joint 7 " << tau_J_d(7) << endl;*/


// Saturate torque rate to avoid discontinuities
    tau_task << saturateTorqueRate(tau_task, tau_J_d);
    for (size_t i = 0; i < 7; ++i) {
    joint_handles_[i].setCommand(tau_task(i));
 }
/*
 cout << "tau_task_sat joint 1 " << tau_task(1) << endl;
 cout << "tau_task_sat joint 2 " << tau_task(2) << endl;
 cout << "tau_task_sat joint 3 " << tau_task(3) << endl;
 cout << "tau_task_sat joint 4 " << tau_task(4) << endl;
 cout << "tau_task_sat joint 5 " << tau_task(5) << endl;
 cout << "tau_task_sat joint 6 " << tau_task(6) << endl;
 cout << "tau_task_sat joint 7 " << tau_task(7) << endl;
*/


// ------------------ parameters update --------------
  //position_d_ = 0.1 * position_d_target_ + (1.0 - 0.1) * position_d_;
  //orientation_d_ = orientation_d_.slerp(0.1, orientation_d_target_);
  //___________________________ END UPDATE ___________________________________
}
// ------------------ Saturate callback --------------
  Eigen::Matrix<double, 7, 1> hybrid::saturateTorqueRate(
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


void hybrid::equilibriumPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  position_d_target_ << msg->pose.position.y, msg->pose.position.y, msg->pose.position.z;

  Eigen::Quaterniond last_orientation_d_target(orientation_d_target_);
  orientation_d_target_.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y,
      msg->pose.orientation.z, msg->pose.orientation.w;

  if (last_orientation_d_target.coeffs().dot(orientation_d_target_.coeffs()) < 0.0) {
    orientation_d_target_.coeffs() << -orientation_d_target_.coeffs();
  }

  /*std::cout << "POSITION x= " << position_d_target_(0)
            << " y=" << position_d_target_(1)
            << " z=" << position_d_target_(2) << std::endl;*/


  Eigen::AngleAxisd aa_orientation_recieved(orientation_d_target_);
  // std::cout << "ORIENTATION AA RECEIVED x= " << aa_orientation_recieved.axis()(0)
  //           << " y=" << aa_orientation_recieved.axis()(1)
  //           << " z=" << aa_orientation_recieved.axis()(2)
  //           << " angle= " << aa_orientation_recieved.angle() << std::endl;
}

} // namespace panda

PLUGINLIB_EXPORT_CLASS(panda_controllers::hybrid, controller_interface::ControllerBase)
