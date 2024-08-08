#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <string.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>



ros::Publisher pub_panda1;
ros::Publisher pub_panda2;
ros::Publisher pub_force_panda1;
ros::Publisher pub_force_panda2; 








void chatterCallback(const geometry_msgs::PoseStampedConstPtr& msg)
 {

        


            // msg init
                    geometry_msgs::PoseStamped msg_obj_;
            // msg 
                    msg_obj_.pose.position.x    = msg->pose.position.x;
                    msg_obj_.pose.position.y    = msg->pose.position.y;
                    msg_obj_.pose.position.z    = msg->pose.position.z;
                    msg_obj_.pose.orientation.x = msg->pose.orientation.x;
                    msg_obj_.pose.orientation.y = msg->pose.orientation.y;
                    msg_obj_.pose.orientation.z = msg->pose.orientation.z;
                    msg_obj_.pose.orientation.w = msg->pose.orientation.w;
            // Publish 
                    pub_panda2.publish(msg_obj_);
                    pub_panda1.publish(msg_obj_);
 }

 void forceCallback(const geometry_msgs::Vector3StampedConstPtr& msg)
 {
            // msg init
                    geometry_msgs::Vector3Stamped msg_force_;
            // msg 
                    msg_force_.vector.x    = msg->vector.x;
                    msg_force_.vector.y    = msg->vector.y;
                    msg_force_.vector.z    = msg->vector.z;
            // Publish 
                    pub_force_panda2.publish(msg_force_);
                    pub_force_panda1.publish(msg_force_);
 }





int main(int argc, char **argv)
    {
        // init handlers
                ros::init(argc, argv, "Planning") ;
                ros::NodeHandle node_handle ;
        // Pubisher pandas
                // pose
                        pub_panda1 = node_handle.advertise<geometry_msgs::PoseStamped>("panda_1/panda1/equilibrium_pose", 1);
                        pub_panda2 = node_handle.advertise<geometry_msgs::PoseStamped>("panda_2/panda2/equilibrium_pose", 1);
                // force
                        pub_force_panda1 = node_handle.advertise<geometry_msgs::Vector3Stamped>("/combined_panda/force_panda1/desired_force", 1);
                        pub_force_panda2 = node_handle.advertise<geometry_msgs::Vector3Stamped>("/combined_panda/force_panda2/desired_force", 1);

        // subscribers 
                //object pose
                        ros::Subscriber sub = node_handle.subscribe ("pose", 1, chatterCallback,ros::TransportHints().reliable().tcpNoDelay()) ;
                // force
                        ros::Subscriber sub_force = node_handle.subscribe ("both_force", 1, forceCallback,ros::TransportHints().reliable().tcpNoDelay()) ;



        ros::spin();
        return 0;
}


 