#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include<mavros_msgs/RCIn.h>
#include<nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

mavros_msgs::RCIn rc;
nav_msgs::Odometry position_msg;
geometry_msgs::PoseStamped init_target_pos, joy_point;
mavros_msgs::State current_state;
float position_x, position_y, position_z,  current_yaw, targetpos_x, targetpos_y;
float pi = 3.14159265;
double rc1, rc2, rc3, rc4, rc7, rc8;
double offset_x, offset_y, offset_z;
int drone_id;
bool use_rviz, need_pub_goal = false;;
double control_dist = 10;//这个代表着控制点与无人机的距离

//read vehicle odometry
void position_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg.header.frame_id = std::to_string(drone_id);
    position_msg.pose.pose.position.x = msg->pose.pose.position.x + offset_x;
    position_msg.pose.pose.position.y = msg->pose.pose.position.y + offset_y;
    position_msg.pose.pose.position.z = msg->pose.pose.position.z + offset_z;
    position_msg.pose.pose.orientation = msg->pose.pose.orientation;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "offset_odom_pub");
	setlocale(LC_ALL,"");
	ros::NodeHandle nh("~");
    nh.param("drone_id", drone_id, 0);
    nh.param("offset_x", offset_x, 0.0);
    nh.param("offset_y", offset_y, 0.0);
    nh.param("offset_z", offset_z, 0.0);
    ros::Publisher new_odom_pub = nh.advertise<nav_msgs::Odometry>
	("/odom_" + std::to_string(drone_id), 1);
	ros::Subscriber position_sub=nh.subscribe<nav_msgs::Odometry>
    ("sub_odom",10,position_cb);
    std::cout<<"============ start pub odom offset==============="<<std::endl;
    ros::Rate rate(5); //

    while(ros::ok())
    { 
        new_odom_pub.publish(position_msg);
        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}