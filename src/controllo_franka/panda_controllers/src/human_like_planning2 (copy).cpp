#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>

#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");


#define NUM_ROWS 24
#define NUM_COLS 128

std::string matrix_name = "/home/franka/test_dual_arm/src/controllo_franka/panda_controllers/src/matrice_pc.csv";
std::ifstream file_stream(matrix_name);
std::string data_row_str;
std::vector<std::string> data_row;
Eigen::MatrixXd data_matrix;

Eigen::Matrix3d pre_rot;

Eigen::MatrixXd ee_trajectory(6, NUM_COLS);
Eigen::MatrixXd ee_trajectory2(6, NUM_COLS);
Eigen::MatrixXd ee_trajectory1(6, NUM_COLS);


Eigen::VectorXd initial_EE_point_pd_1(6);
Eigen::VectorXd initial_EE_point_pd_2(6);
bool initial_pose_init_pd_1 = false;
bool initial_pose_init_pd_2 = false;
geometry_msgs::Pose msg_pose;

void load_fpc()
{
    if (!file_stream.is_open())
    {
        std::cout << "ERRORE APERTURA .csv\n";
    }

    data_matrix.resize(NUM_ROWS, NUM_COLS);

    for (int i = 0; i < NUM_ROWS; ++i)
    {
        std::getline(file_stream, data_row_str);
        boost::algorithm::split(data_row, data_row_str, boost::is_any_of(","), boost::token_compress_on);
        for (int j = 0; j < NUM_COLS; ++j)
        {
            data_matrix(i,j) = std::stod(data_row[j]);
        }
    }
}


Eigen::Vector3d ToEulerAngles(const tf::Quaternion& q) {

   Eigen::Vector3d angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();


    // roll (x-axis rotation)

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles[2] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)

    double sinp = std::sqrt(1 + 2 * (w * y - x *z));
    double cosp = std::sqrt(1 - 2 * (w * y - x * z));
    angles[1] = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles[0] = std::atan2(siny_cosp, cosy_cosp);
    return angles;
}

















void single_dof(double starting_point, double ending_point, double starting_vel, double ending_vel, int i, double dt)
{
    //Starting and ending values of fPCs
    double fPC0_start = data_matrix(i*4,0);
    double fPC1_start = data_matrix(i*4+1,0);
    double fPC2_start = data_matrix(i*4+2,0);
    double fPC3_start = data_matrix(i*4+3,0);
    double fPC0_end = data_matrix(i*4,NUM_COLS-1);
    double fPC1_end = data_matrix(i*4+1,NUM_COLS-1);
    double fPC2_end = data_matrix(i*4+2,NUM_COLS-1);
    double fPC3_end = data_matrix(i*4+3,NUM_COLS-1);

    //Starting and ending values of first derivative of fPCs
    double fPC0dot_start = (data_matrix(i*4,1) - data_matrix(i*4,0))/dt;
    double fPC1dot_start = (data_matrix(i*4+1,1) - data_matrix(i*4+1,0))/dt;
    double fPC2dot_start = (data_matrix(i*4+2,1) - data_matrix(i*4+2,0))/dt;
    double fPC3dot_start = (data_matrix(i*4+3,1) - data_matrix(i*4+3,0))/dt;
    double fPC0dot_end = (data_matrix(i*4,NUM_COLS-1) - data_matrix(i*4,NUM_COLS-2))/dt;
    double fPC1dot_end = (data_matrix(i*4+1,NUM_COLS-1) - data_matrix(i*4+1,NUM_COLS-2))/dt;
    double fPC2dot_end = (data_matrix(i*4+2,NUM_COLS-1) - data_matrix(i*4+2,NUM_COLS-2))/dt;
    double fPC3dot_end = (data_matrix(i*4+3,NUM_COLS-1) - data_matrix(i*4+3,NUM_COLS-2))/dt;

    //Definition of linear system for trajectory computation
    Eigen::Vector4d b;
    b << starting_point-fPC0_start, ending_point-fPC0_end, starting_vel-fPC0dot_start, ending_vel-fPC0dot_end;

    Eigen::Matrix4d A;
    Eigen::Matrix4d A_I;
    A << 1, fPC1_start, fPC2_start, fPC3_start,
         1, fPC1_end, fPC2_end, fPC3_end,
         0, fPC1dot_start, fPC2dot_start, fPC3dot_start,
         0, fPC1dot_end, fPC2dot_end, fPC3dot_end;
    
    A_I = A.inverse();

    //fPCs' weight computation
    Eigen::Vector4d x;
    x = A_I*b;

    //Single DoF trajectory computation
    Eigen::RowVectorXd trajectory_single_dof;
    trajectory_single_dof = x(0)*Eigen::RowVectorXd::Ones(NUM_COLS) + data_matrix.row(i*4) + x(1)*data_matrix.row(i*4+1) + x(2)*data_matrix.row(i*4+2) + x(3)*data_matrix.row(i*4+3);

    ee_trajectory.block(i,0,1,NUM_COLS) = trajectory_single_dof;
}



















// Convert xyzrpy vector to geometry_msgs Pose (PRESA DA PANDA-SOFTHAND -> TaskSequencer.cpp)
geometry_msgs::Pose convert_vector_to_pose(Eigen::VectorXd input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    rotation = pre_rot*rotation;
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);  //CONTROLLARE SE METTERE #include <eigen_conversions/eigen_msg.h>
    return output_pose;
}















geometry_msgs::Twist convert_vector_to_twist(Eigen::VectorXd input_vec)
{
    geometry_msgs::Twist output_twist;

    output_twist.linear.x = input_vec(0);
    output_twist.linear.y = input_vec(1);
    output_twist.linear.z = input_vec(2);
    output_twist.angular.x = input_vec(3);
    output_twist.angular.y = input_vec(4);
    output_twist.angular.z = input_vec(5);

    return output_twist;
}











void robotPoseCallback_pd_1(const geometry_msgs::PoseStamped& msg)
{
    if (!initial_pose_init_pd_1)
    {
        msg_pose = msg.pose;

        std::cout << "Message received" << std::endl;
        std::cout << msg << std::endl;
        
        Eigen::Affine3d input_affine;
        Eigen::Vector3d traslazione;
        Eigen::Vector3d rpy;
        Eigen::Matrix3d mat_rotazione;
        
        tf::poseMsgToEigen(msg_pose,input_affine);
        traslazione = input_affine.translation();
        mat_rotazione = input_affine.rotation();
        mat_rotazione = pre_rot*mat_rotazione;
                
        Eigen::Quaterniond quad(mat_rotazione);
        tf::Quaternion tfQuat;
        tfQuat.setW(quad.w());
        tfQuat.setX(quad.x());
        tfQuat.setY(quad.y());
        tfQuat.setZ(quad.z());
        rpy =ToEulerAngles(tfQuat);
        
        initial_EE_point_pd_1 << traslazione[0], traslazione[1], traslazione[2], rpy[2], rpy[1], rpy[0];




        std::cout << "Starting pose read from topic:" << std::endl;
        std::cout << initial_EE_point_pd_1 << std::endl;

        initial_pose_init_pd_1 = true;
    }   
}


void robotPoseCallback_pd_2(const geometry_msgs::PoseStamped& msg)
{
    if (!initial_pose_init_pd_2)
    {
        msg_pose = msg.pose;

        std::cout << "Message received" << std::endl;
        std::cout << msg << std::endl;
        
        Eigen::Affine3d input_affine;
        Eigen::Vector3d traslazione;
        Eigen::Vector3d rpy;
        Eigen::Matrix3d mat_rotazione;
        
        tf::poseMsgToEigen(msg_pose,input_affine);
        traslazione = input_affine.translation();
        mat_rotazione = input_affine.rotation();
        mat_rotazione = pre_rot*mat_rotazione;



        Eigen::Quaterniond quad(mat_rotazione);
        tf::Quaternion tfQuat;
        tfQuat.setW(quad.w());
        tfQuat.setX(quad.x());
        tfQuat.setY(quad.y());
        tfQuat.setZ(quad.z());
        rpy =ToEulerAngles(tfQuat);
        
        
        initial_EE_point_pd_2 << traslazione[0], traslazione[1], traslazione[2], rpy[2], rpy[1], rpy[0];

        std::cout << "Starting pose read from topic:" << std::endl;
        std::cout << initial_EE_point_pd_2 << std::endl;

        initial_pose_init_pd_2 = true;
    }   
}

bool vis = false;
Eigen::Vector3d cent;
void pox_Callback(const geometry_msgs::Vector3Stamped& msg)
{
    if (!vis){
    cent << msg.vector.x,msg.vector.y,msg.vector.z;
    cent(1) = cent(1)+0.65;
    vis = true;}
}














int main(int argc, char **argv)
{
    pre_rot << 1,  0,  0, 
               0, -1,  0, 
               0,  0, -1;
   
    //Initialize the node
    ROS_INFO("NODE INITIALIZATION");
    ros::init(argc, argv, "HL_planner");
    ros::NodeHandle node;

    //Initialize frame trajectory publisher
    ROS_INFO("PUBLISHER INITIALIZATION");
    ros::Publisher pub_pose1 = node.advertise<geometry_msgs::PoseStamped>("/combined_panda/cartesian_impedance_example_controller1/equilibrium_pose", 1);
    ros::Publisher pub_pose2 = node.advertise<geometry_msgs::PoseStamped>("/combined_panda/cartesian_impedance_example_controller2/equilibrium_pose", 1);

    




    ros::Publisher pub_twist = node.advertise<geometry_msgs::Twist>("/franka/equilibrium_twist", 1);
    ros::Publisher pub_time = node.advertise<std_msgs::Float64>("/franka/time",1);

    

    //Initialize starting pose subscriber
    ros::Subscriber robot_pose_sub1 = node.subscribe("/combined_panda/cartesian_impedance_example_controller1/franka_ee_pose", 1, robotPoseCallback_pd_1);
    ros::Subscriber robot_pose_sub2 = node.subscribe("/combined_panda/cartesian_impedance_example_controller2/franka_ee_pose", 1, robotPoseCallback_pd_2);
    ros::Subscriber pox = node.subscribe("/vector_topic", 1, pox_Callback);





    //Load the fPCs
    ROS_INFO("LOAD fPCs");
    load_fpc();

    ROS_INFO("Variable definitions");
    //Starting and ending times definition
    double t_start = 0;
    double t_end;


    std::cout << "Input visione" <<std::endl<<cent;



    std::cout << "Input time to perform trajectory [s]";
    std::cin >> t_end;

    //Time axis definition
    Eigen::VectorXd t;
    t = Eigen::VectorXd::LinSpaced(NUM_COLS, t_start, t_end);

    //Velocities constrains definition
    Eigen::VectorXd initial_velocity(6);
    Eigen::VectorXd final_velocity(6);

    initial_velocity << 0, 0, 0, 0, 0, 0;
    
    double aux;
    
    final_velocity << 0, 0, 0, 0, 0, 0;

    //Cartesian constrains definition

    Eigen::VectorXd final_EE_point (6);
    // std::cout << "Input final pose vector element by element (xyzrpy)";
    
    // for (int i = 0; i < 6; i++)
    // {
    //     std::cin >> aux;
    //     final_EE_point(i) = aux;
    // }
    
    
    ROS_INFO("INPUT COMPLETED");
    int choice;
    // ROS_INFO("which panda should move? \n 1: Panda 1 (left panda) \n 2: Panda 2 (right panda) \n 3: Multiarm");
    // std::cin >> choice;
    // std::cout << choice << std::endl;



    double dt;
    dt = t(2)-t(1);
    Eigen::VectorXd actual_pose;
    Eigen::VectorXd actual_pose2;

    geometry_msgs::Pose actual_pose_msg;
    geometry_msgs::Pose actual_pose_msg2;

    geometry_msgs::PoseStamped actual_posestamped_msg;
    geometry_msgs::PoseStamped actual_posestamped_msg2;

    Eigen::VectorXd actual_velocity;
    geometry_msgs::Twist actual_twist;

    Eigen::VectorXd actual_velocity2;
    geometry_msgs::Twist actual_twist2;


    std_msgs::Float64 computed_time;
    ros::Rate rate(1/dt);
    
    /**/

    while (ros::ok())
    {
                ros::spinOnce();
                /*UP*/
                if (initial_pose_init_pd_2)
                {
                    ROS_INFO("Trajectory Computation");
                    //
                    final_EE_point(0) = 0.5;
                    final_EE_point(1) = 0.2;
                    final_EE_point(2) = 0.25;
                    // adjust orientation panda 2
                    final_EE_point(3) = -1.5708;
                    final_EE_point(4) = 0;
                    final_EE_point(5) = 0;
                    //Trajectory computation panda 2
                    for (int i = 0; i < 6; i++){
                        single_dof(initial_EE_point_pd_2(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
                    }
                    ee_trajectory2 = ee_trajectory;


                    // adjust orientation panda 1
                    final_EE_point(0) = 0.5;
                    final_EE_point(1) = -0.2;
                    final_EE_point(2) = 0.25;
                    final_EE_point(3) = 1.5708;
                    final_EE_point(4) = 0;
                    final_EE_point(5) = 0;

                    //Trajectory computation panda 1
                    for (int i = 0; i < 6; i++){
                        single_dof(initial_EE_point_pd_1(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
                    }
                    ee_trajectory1 = ee_trajectory;
            
                    ROS_INFO("Trajectory Publishing ");
                    for (int i = 0; i < NUM_COLS; i++){

                        actual_pose  = ee_trajectory.col(i);
                        actual_pose2 = ee_trajectory2.col(i);

                        actual_pose_msg = convert_vector_to_pose(actual_pose);
                        actual_pose_msg2 = convert_vector_to_pose(actual_pose2);
                        
                        actual_posestamped_msg.pose = actual_pose_msg;
                        actual_posestamped_msg.header.stamp = ros::Time::now();
                        
                        actual_posestamped_msg2.pose = actual_pose_msg2;
                        actual_posestamped_msg2.header.stamp = ros::Time::now();


                        if (i<NUM_COLS-1){
                            actual_velocity = (ee_trajectory.col(i+1)-ee_trajectory.col(i))/dt;
                            actual_twist = convert_vector_to_twist(actual_velocity);

                            actual_velocity2 = (ee_trajectory2.col(i+1)-ee_trajectory2.col(i))/dt;
                            actual_twist2 = convert_vector_to_twist(actual_velocity2);
                        }
                

                        pub_pose1.publish(actual_posestamped_msg);
                        pub_pose2.publish(actual_posestamped_msg2);

                        pub_twist.publish(actual_twist);
                        pub_time.publish(computed_time);
                        
                        rate.sleep();
                    };
                ROS_INFO("Sto per entrare nel forward");
                
                /*FORWARD*/
                //  if (initial_pose_init_pd_2)
                // {
                    initial_EE_point_pd_2 = ee_trajectory2.col(NUM_COLS-1);
                    initial_EE_point_pd_1 = ee_trajectory1.col(NUM_COLS-1);
                    ros::spinOnce();

                    ROS_INFO("Trajectory Computation");
                    ROS_INFO("Prima del delirio");
                    getchar();
                    //Trajectory computation panda 2
                    final_EE_point(0) = cent(0);
                    final_EE_point(1) = cent(1);
                    final_EE_point(2) = cent(2);
                    final_EE_point(3) = -1.5708;
                    final_EE_point(4) = 0;
                    final_EE_point(5) = 0;


                    //Trajectory computation panda 2
                    for (int i = 0; i < 6; i++){
                        single_dof(initial_EE_point_pd_2(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
                    }

                    ee_trajectory2 = ee_trajectory;

                    // adjust orientation panda 1
                    final_EE_point(0) = cent(0);
                    final_EE_point(1) = cent(1);
                    final_EE_point(2) = cent(2);
                    final_EE_point(3) = 1.5708;
                    final_EE_point(4) = 0;
                    final_EE_point(5) = 0;


                    std::cout<<"cent "<<cent(1)<<std::endl;

                //     //Trajectory computation panda 1
                    for (int i = 0; i < 6; i++){
                        single_dof(initial_EE_point_pd_1(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
                    }

            
                    ROS_INFO("Trajectory Publishing ");
                    for (int i = 0; i < NUM_COLS; i++){

                        actual_pose  = ee_trajectory.col(i);
                        actual_pose2 = ee_trajectory2.col(i);

                        actual_pose_msg = convert_vector_to_pose(actual_pose);
                        actual_pose_msg2 = convert_vector_to_pose(actual_pose2);
                        
                        actual_posestamped_msg.pose = actual_pose_msg;
                        actual_posestamped_msg.header.stamp = ros::Time::now();
                        
                        actual_posestamped_msg2.pose = actual_pose_msg2;
                        actual_posestamped_msg2.header.stamp = ros::Time::now();


                        if (i<NUM_COLS-1){
                            actual_velocity = (ee_trajectory.col(i+1)-ee_trajectory.col(i))/dt;
                            actual_twist = convert_vector_to_twist(actual_velocity);

                            actual_velocity2 = (ee_trajectory2.col(i+1)-ee_trajectory2.col(i))/dt;
                            actual_twist2 = convert_vector_to_twist(actual_velocity2);
                        }
                

                        pub_pose1.publish(actual_posestamped_msg);
                        pub_pose2.publish(actual_posestamped_msg2);

                        pub_twist.publish(actual_twist);
                        pub_time.publish(computed_time);
                        
                        rate.sleep();
                    }
                break;
                }
                ROS_INFO("WAITING INITIAL POSE");   
            }


    /**/








    // switch (choice){

    //     case 1:
    //         while (ros::ok())
    //         {
    //             ros::spinOnce();
    //             if (initial_pose_init_pd_1)
    //             {
    //                 ROS_INFO("Trajectory Computation");
    //                 //Trajectory computation
    //                 for (int i = 0; i < 6; i++){
    //                     single_dof(initial_EE_point_pd_1(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
    //                 }
            
    //                 ROS_INFO("Trajectory Publishing panda 1 ");
    //                 for (int i = 0; i < NUM_COLS; i++){
    //                     actual_pose = ee_trajectory.col(i);
    //                     actual_pose_msg = convert_vector_to_pose(actual_pose);
    //                     actual_posestamped_msg.pose = actual_pose_msg;
    //                     actual_posestamped_msg.header.stamp = ros::Time::now();
                        
    //                     if (i<NUM_COLS-1){
    //                         actual_velocity = (ee_trajectory.col(i+1)-ee_trajectory.col(i))/dt;
    //                         actual_twist = convert_vector_to_twist(actual_velocity);
    //                     }
                
    //                     pub_pose1.publish(actual_posestamped_msg);
    //                     pub_twist.publish(actual_twist);
    //                     pub_time.publish(computed_time);
    //                     rate.sleep();
    //                 }
    //             break;
    //             }
    //             ROS_INFO("WAITING INITIAL POSE");   
    //         }
    //     break;

    //     case 2:
    //         while (ros::ok())
    //         {
    //             ros::spinOnce();
    //             if (initial_pose_init_pd_2)
    //             {
    //                 ROS_INFO("Trajectory Computation");
    //                 //Trajectory computation
    //                 for (int i = 0; i < 6; i++){
    //                     single_dof(initial_EE_point_pd_2(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
    //                 }
                    
    //                 ROS_INFO("Trajectory Publishing panda 2 ");
    //                 for (int i = 0; i < NUM_COLS; i++){
    //                     actual_pose = ee_trajectory.col(i);
    //                     actual_pose_msg = convert_vector_to_pose(actual_pose);
    //                     actual_posestamped_msg.pose = actual_pose_msg;
    //                     actual_posestamped_msg.header.stamp = ros::Time::now();
                        
    //                     if (i<NUM_COLS-1){
    //                         actual_velocity = (ee_trajectory.col(i+1)-ee_trajectory.col(i))/dt;
    //                         actual_twist = convert_vector_to_twist(actual_velocity);
    //                     }
                
    //                     pub_pose2.publish(actual_posestamped_msg);
    //                     pub_twist.publish(actual_twist);
    //                     pub_time.publish(computed_time);
    //                     rate.sleep();
    //                 }
    //             break;
    //             }
    //             ROS_INFO("WAITING INITIAL POSE");   
    //         }
    //     break;

    // case 3:
    //         while (ros::ok())
    //         {
    //             ros::spinOnce();
    //             if (initial_pose_init_pd_2)
    //             {
    //                 ROS_INFO("Trajectory Computation");
                    
    //                 // adjust orientation panda 2
    //                 final_EE_point(3) = -1.5708;
    //                 final_EE_point(4) = 0;
    //                 final_EE_point(5) = 0;

    //                 //Trajectory computation panda 2
    //                 for (int i = 0; i < 6; i++){
    //                     single_dof(initial_EE_point_pd_2(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
    //                 }

    //                 ee_trajectory2 = ee_trajectory;

    //                 // adjust orientation panda 1
    //                 final_EE_point(3) = 1.5708;
    //                 final_EE_point(4) = 0;
    //                 final_EE_point(5) = 0;

    //                 //Trajectory computation panda 1
    //                 for (int i = 0; i < 6; i++){
    //                     single_dof(initial_EE_point_pd_1(i), final_EE_point(i), initial_velocity(i), final_velocity(i), i, dt);
    //                 }

            
    //                 ROS_INFO("Trajectory Publishing ");
    //                 for (int i = 0; i < NUM_COLS; i++){

    //                     actual_pose  = ee_trajectory.col(i);
    //                     actual_pose2 = ee_trajectory2.col(i);

    //                     actual_pose_msg = convert_vector_to_pose(actual_pose);
    //                     actual_pose_msg2 = convert_vector_to_pose(actual_pose2);
                        
    //                     actual_posestamped_msg.pose = actual_pose_msg;
    //                     actual_posestamped_msg.header.stamp = ros::Time::now();
                        
    //                     actual_posestamped_msg2.pose = actual_pose_msg2;
    //                     actual_posestamped_msg2.header.stamp = ros::Time::now();


    //                     if (i<NUM_COLS-1){
    //                         actual_velocity = (ee_trajectory.col(i+1)-ee_trajectory.col(i))/dt;
    //                         actual_twist = convert_vector_to_twist(actual_velocity);

    //                         actual_velocity2 = (ee_trajectory2.col(i+1)-ee_trajectory2.col(i))/dt;
    //                         actual_twist2 = convert_vector_to_twist(actual_velocity2);
    //                     }
                

    //                     pub_pose1.publish(actual_posestamped_msg);
    //                     pub_pose2.publish(actual_posestamped_msg2);

    //                     pub_twist.publish(actual_twist);
    //                     pub_time.publish(computed_time);
                        
    //                     rate.sleep();
    //                 }
    //             break;
    //             }
    //             ROS_INFO("WAITING INITIAL POSE");   
    //         }
    // break;
    // }
    ROS_INFO("TASK COMPLETED");

}
