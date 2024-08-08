//#include "/home/gianmacc/panda/src/gc_planner/src/gc_planner_header.h"
#include "/home/franka/test_dual_arm/src/gc_calibration/include/gc_planner_header.h"

template<typename _Matrix_Type_> 
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon()) 
{  
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);  
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);  
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint(); 
} 

bool get_tag_pose_{false};
int passo = 0;
int punto = 1;
int n_passi;
geometry_msgs::Pose pose_TAG;
ros::Publisher pub_T_RC;
Eigen::MatrixXf T_RT_N;
Eigen::MatrixXf T_CT_N;
Eigen::Matrix3f correction;
Eigen::Matrix4f correction_T;
std::ofstream Matrices;
std::ofstream Final_pose;

void callback_EE_pose(const geometry_msgs::PoseStamped);
void callback_TAG_pose(const apriltag_ros::AprilTagDetectionArray);

int main (int argc, char** argv) {
    ros::init (argc, argv, "calibration_T");
    ros::NodeHandle nh;
    Matrices.open("/home/franka/test_dual_arm/src/gc_calibration/workdir/calib_matrices.csv");
	correction << 0, 1, 0, 1, 0, 0, 0, 0, -1;
	//pre_rot << 1, 0, 0, 0, -1, 0, 0, 0, -1;
	correction_T << 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1;
    ros::Subscriber sub_EE_pose = nh.subscribe<geometry_msgs::PoseStamped>("/franka/cartesian_impedance_controller_softbots/franka_ee_pose", 1, callback_EE_pose);
    ros::Subscriber sub_TAG_pose = nh.subscribe<apriltag_ros::AprilTagDetectionArray>("/tag_detections", 1, callback_TAG_pose);
    get_tag_pose_ = false; 
    pub_T_RC = nh.advertise<geometry_msgs::PoseStamped>("/pose_robot_cam", 1);
    ros::spin();
    return 0;
}

void callback_TAG_pose(const apriltag_ros::AprilTagDetectionArray pose_TAG_msg){
	//std::cout << "enter tag cback" << std::endl;
	if (!get_tag_pose_) 
		return;
    pose_TAG = pose_TAG_msg.detections[0].pose.pose.pose;
    get_tag_pose_ = false;
    std::cout << "\n\nGET TAG POSE\n\n" << std::endl;
}

void callback_EE_pose(const geometry_msgs::PoseStamped pose_EE_msg){
  	geometry_msgs::Pose pose_EE;
	pose_EE = pose_EE_msg.pose;
	//std::cout << "enter franka cback" << std::endl;
	if(passo == 0){
		std::cout << "inserire numero di punti da cui calcolare la T_RC: \n";
		std::cin >> n_passi;
		int n_cols;
    	n_cols = n_passi*4;
		T_RT_N.resize(4, n_cols);
		T_CT_N.resize(4, n_cols);
		if(n_passi < 1)
			ROS_ERROR("inserire almeno un punto");
		std::cout << "end passo 0" << std::endl;
	}
	else if(passo % 2 != 0){//se il numero di passi è dispari
  		std::cout << "Portare EE nel punto numero " << punto << " e premere invio \n";
		char a;
  		std::cin >> a;
  		punto += 1;
  		get_tag_pose_ = true;
	}
	else{
		if (get_tag_pose_) 
			return;
		std::cout << "\n\nGET TAG POSE - DONE\n\n";
		Eigen::Quaternionf q_RT;
		q_RT.w() = pose_EE.orientation.w;
		q_RT.x() = pose_EE.orientation.x;
		q_RT.y() = pose_EE.orientation.y;
		q_RT.z() = pose_EE.orientation.z;
		Eigen::Quaternionf q_CT;
		q_CT.w() = pose_TAG.orientation.w;
		q_CT.x() = pose_TAG.orientation.x;
		q_CT.y() = pose_TAG.orientation.y;
		q_CT.z() = pose_TAG.orientation.z;
		Eigen::Matrix3f R_RT = q_RT.toRotationMatrix();
		Eigen::Matrix3f R_CT = q_CT.toRotationMatrix();
		//R_CT = correction * R_CT;
		//R_RT = correction * R_RT;
		Eigen::Vector3f pos_RT;
	    pos_RT(0) = pose_EE.position.x;
	    pos_RT(1) = pose_EE.position.y;
	    pos_RT(2) = pose_EE.position.z; 
	   	Eigen::Vector3f pos_CT;
	    pos_CT(0) = pose_TAG.position.x;
	    pos_CT(1) = pose_TAG.position.y;
	    pos_CT(2) = pose_TAG.position.z;

		Eigen::Matrix4f T_RT = Eigen::Matrix4f::Identity();
    	T_RT.block<3, 3>(0, 0) = R_RT;
    	T_RT.block<3, 1>(0, 3) = pos_RT.transpose();
    	Eigen::Matrix4f T_CT = Eigen::Matrix4f::Identity();
    	T_CT.block<3, 3>(0, 0) = R_CT;
    	T_CT.block<3, 1>(0, 3) = pos_CT.transpose();
		//T_CT = T_CT * correction_T;
    	int n_cols, col_start;
    	n_cols = passo*2;
    	col_start = n_cols-4;
    	T_RT_N.block<4, 4>(0, col_start) = T_RT;
    	T_CT_N.block<4, 4>(0, col_start) = T_CT;
    	Matrices.open("/home/franka/test_dual_arm/src/gc_calibration/workdir/calib_matrices.csv");
		Matrices << "T_RC\n" << EigToString(T_RT_N) << "\nT_CT\n" << EigToString(T_CT_N) << std::endl;
		Matrices.close();
	}
	if (passo == n_passi*2){
		Eigen::Matrix4f T_RC_stimata;
		Eigen::MatrixXf T_CT_N_pinv;
		T_CT_N_pinv.resize(n_passi*2, 4);
		T_CT_N_pinv = pseudoInverse(T_CT_N);
		T_RC_stimata = T_RT_N*T_CT_N_pinv;
		//std::cout << T_RC_stimata << std::endl;
		//std::cout << "\n" << std::endl;
		//Eigen::JacobiSVD<Eigen::Matrix3f> svd(T_RC_stimata.block<3, 3>(0, 0)); 
		//T_RC_stimata.block<3, 3>(0, 0) = svd.matrixU()*svd.matrixV().transpose();
		//Eigen::Quaternionf q(T_RC_stimata.block<3, 3>(0, 0));
		//q.normalize();
		//T_RC_stimata.block<3, 3>(0, 0) = q.toRotationMatrix();
		Eigen::Quaternionf q_RC(T_RC_stimata.block<3, 3>(0, 0));
		geometry_msgs::Pose T_RC;
		T_RC.orientation.x = q_RC.x();
		T_RC.orientation.y = q_RC.y();
		T_RC.orientation.z = q_RC.z();
		T_RC.orientation.w = q_RC.w();
		T_RC.position.x = T_RC_stimata(0,3);
		T_RC.position.y = T_RC_stimata(1,3);
		T_RC.position.z = T_RC_stimata(2,3);
		std::cout << T_RC_stimata << std::endl;
	    Final_pose.open("/home/franka/test_dual_arm/src/gc_calibration/workdir/camera_calib_debug.csv");
		Final_pose << EigToString(T_RC_stimata);
		// Final_pose << T_RC.position.x << "," << T_RC.position.y << 
		// 	"," << T_RC.position.z  << "," << T_RC.orientation.w  << 
		// 	"," << T_RC.orientation.x  << "," << T_RC.orientation.y << 
		// 	"," << T_RC.orientation.z << std::endl;
		Final_pose.close();
		std::cout << "FINE" << std::endl;
		system("rosnode kill calibration_T"); 
		while(1);
		std::cout << "end of calibration" << std::endl;
	}
	else{
		passo += 1;
	}
}
