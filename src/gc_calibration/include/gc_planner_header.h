#include <eigen3/Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <tf/transform_broadcaster.h>



#include <apriltag_ros/AprilTagDetectionArray.h>

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <random>
#include <cstdlib>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
//#include <qb_interface/handRef.h>                DA RIMETTERE


#define NUM_ROWS 24
#define NUM_COLS 128

/*------------------------------------
custom function used to output Eigen library objects
to avoid overloaded operator << which is normally used by
output streams but also used by Eigen for value assignments
------------------------------------*/

static void copyMatrixToEig(Eigen::Matrix4d& mat, std::ifstream& stream){
  Eigen::MatrixXd data(4,4);
  for (int row = 0; row < 4; row++){
    std::string data_row_str;
    std::vector<std::string> data_row;
    std::getline(stream, data_row_str);
    boost::algorithm::split(data_row, data_row_str, boost::is_any_of(" "), boost::token_compress_on);
    for (int column = 0; column < 4; column++){
      data(row, column) = std::stod(data_row[column]);
    }
    mat.block<4, 4>(0, 0) = data;
    //std::cout << "TYPE" << typeid(data).name() << std:endl;
  }

}

// static void copyMatrixToEig(Eigen::Matrix4f& mat, std::ifstream& stream){
//    Eigen::MatrixXf data(4,4);
//    for (int row = 0; row < 4; row++){
//      std::string data_row_str;
//      std::vector<std::string> data_row;
//      std::getline(stream, data_row_str);
//      boost::algorithm::split(data_row, data_row_str, boost::is_any_of(","), boost::token_compress_on);
//      for (int column = 0; column < 4; column++){
//        data(row, column) = std::stof(data_row[column]);
//      }
//      mat.block<4, 4>(0, 0) = data;
//    }
//  }

static std::string EigToString(const Eigen::MatrixXd& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::MatrixXf& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Matrix4d& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Matrix4f& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Matrix3d& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Matrix3f& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::VectorXd& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::VectorXf& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Vector3d& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

static std::string EigToString(const Eigen::Vector3f& mat){
  std::stringstream ss;
  ss << mat;
  return ss.str();
}

/*------------------------------------------
The following function is working but we chose not to use it
Connection between two planned trajectories by finding the closest
point from the old trajectory to the new trajectory
The new trajectory will be started from that point

int where_to_start(void){
    std::cout << "interrupted in \n" << EigToString(prev_trajectory) << std::endl;
    int where_min_dist = 0;
    double min_dist = 0;
    double dist_norm = 0;
    for (int h = 0; h < NUM_COLS; h++){
        dist_norm = sqrt(pow(planned_trajectory(0,h) - prev_trajectory(0), 2) +
                    + pow(planned_trajectory(1,h) - prev_trajectory(1), 2) 
                    + pow(planned_trajectory(2,h) - prev_trajectory(2), 2));
        if (h == 0)
            min_dist = dist_norm;
        else if(dist_norm < min_dist){
            min_dist = dist_norm;
            where_min_dist = h;
        }
    }
    std::cout << where_min_dist << " is where will restart \n" 
        << planned_trajectory.col(where_min_dist) << std::endl;
    return where_min_dist;
}
------------------------------------------*/

/*------------------------------------------
THE FOLLOWING CODE IS TAKEN FROM eigen_conversions
DOCUMENTATION FROM ROS GITHUB
------------------------------------------*/

namespace tf {

void quaternionEigenToTF(const Eigen::Quaterniond& e, tf::Quaternion& t)
{
  t[0] = e.x();
  t[1] = e.y();
  t[2] = e.z();
  t[3] = e.w();
}

void pointMsgToEigen(const geometry_msgs::Point &m, Eigen::Vector3d &e)
{
  e(0) = m.x; 
  e(1) = m.y; 
  e(2) = m.z; 
}

void pointEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Point &m)
{
  m.x = e(0);
  m.y = e(1);
  m.z = e(2);
}

namespace {
  template<typename T>
  void poseMsgToEigenImpl(const geometry_msgs::Pose &m, T &e)
  {
    e = Eigen::Translation3d(m.position.x,
                             m.position.y,
                             m.position.z) *
      Eigen::Quaterniond(m.orientation.w,
                         m.orientation.x,
                         m.orientation.y,
                         m.orientation.z);
  }

  template<typename T>
  void poseEigenToMsgImpl(const T &e, geometry_msgs::Pose &m)
  {
    m.position.x = e.translation()[0];
    m.position.y = e.translation()[1];
    m.position.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.orientation.x = q.x();
    m.orientation.y = q.y();
    m.orientation.z = q.z();
    m.orientation.w = q.w();
    if (m.orientation.w < 0) {
      m.orientation.x *= -1;
      m.orientation.y *= -1;
      m.orientation.z *= -1;
      m.orientation.w *= -1;
    }
  }

  template<typename T>
  void transformMsgToEigenImpl(const geometry_msgs::Transform &m, T &e)
  {
    e = Eigen::Translation3d(m.translation.x,
                             m.translation.y,
                             m.translation.z) *
      Eigen::Quaterniond(m.rotation.w,
                         m.rotation.x,
                         m.rotation.y,
                         m.rotation.z);
  }

  template<typename T>
  void transformEigenToMsgImpl(const T &e, geometry_msgs::Transform &m)
  {
    m.translation.x = e.translation()[0];
    m.translation.y = e.translation()[1];
    m.translation.z = e.translation()[2];
    Eigen::Quaterniond q = (Eigen::Quaterniond)e.linear();
    m.rotation.x = q.x();
    m.rotation.y = q.y();
    m.rotation.z = q.z();
    m.rotation.w = q.w();
    if (m.rotation.w < 0) {
      m.rotation.x *= -1;
      m.rotation.y *= -1;
      m.rotation.z *= -1;
      m.rotation.w *= -1;
    }
  }
}

void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Affine3d &e)
{
  poseMsgToEigenImpl(m, e);
}

void poseMsgToEigen(const geometry_msgs::Pose &m, Eigen::Isometry3d &e)
{
  poseMsgToEigenImpl(m, e);
}

void poseEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Pose &m)
{
  poseEigenToMsgImpl(e, m);
}

void poseEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Pose &m)
{
  poseEigenToMsgImpl(e, m);
}

void quaternionMsgToEigen(const geometry_msgs::Quaternion &m, Eigen::Quaterniond &e)
{
  e = Eigen::Quaterniond(m.w, m.x, m.y, m.z);
}

void quaternionEigenToMsg(const Eigen::Quaterniond &e, geometry_msgs::Quaternion &m)
{
  m.x = e.x();
  m.y = e.y();
  m.z = e.z();
  m.w = e.w();
}

void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Affine3d &e)
{
  transformMsgToEigenImpl(m, e);
}

void transformMsgToEigen(const geometry_msgs::Transform &m, Eigen::Isometry3d &e)
{
  transformMsgToEigenImpl(m, e);
}

void transformEigenToMsg(const Eigen::Affine3d &e, geometry_msgs::Transform &m)
{
  transformEigenToMsgImpl(e, m);
}

void transformEigenToMsg(const Eigen::Isometry3d &e, geometry_msgs::Transform &m)
{
  transformEigenToMsgImpl(e, m);
}

void vectorMsgToEigen(const geometry_msgs::Vector3 &m, Eigen::Vector3d &e)
{
  e(0) = m.x; 
  e(1) = m.y; 
  e(2) = m.z; 
}

void vectorEigenToMsg(const Eigen::Vector3d &e, geometry_msgs::Vector3 &m)
{
  m.x = e(0);
  m.y = e(1);
  m.z = e(2);
}

/* void twistMsgToEigen(const geometry_msgs::Twist &m, Eigen::Matrix<double,6,1> &e)
{
  e[0] = m.linear.x;
  e[1] = m.linear.y;
  e[2] = m.linear.z;
  e[3] = m.angular.x;
  e[4] = m.angular.y;
  e[5] = m.angular.z;
}

void twistEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Twist &m)
{
  m.linear.x = e[0];
  m.linear.y = e[1];
  m.linear.z = e[2];
  m.angular.x = e[3];
  m.angular.y = e[4];
  m.angular.z = e[5];
}

void wrenchMsgToEigen(const geometry_msgs::Wrench &m, Eigen::Matrix<double,6,1> &e)
{
  e[0] = m.force.x;
  e[1] = m.force.y;
  e[2] = m.force.z;
  e[3] = m.torque.x;
  e[4] = m.torque.y;
  e[5] = m.torque.z;
}

void wrenchEigenToMsg(const Eigen::Matrix<double,6,1> &e, geometry_msgs::Wrench &m)
{
  m.force.x = e[0];
  m.force.y = e[1];
  m.force.z = e[2];
  m.torque.x = e[3];
  m.torque.y = e[4];
  m.torque.z = e[5];
} */

} // namespace
