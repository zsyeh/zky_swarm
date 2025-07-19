#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Eigen>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#define PI 3.14159265358979
#define VELOCITY2D_CONTROL 0b011111000111 //设置好对应的掩码，从右往左依次对应PX/PY/PZ/VX/VY/VZ/AX/AY/AZ/FORCE/YAW/YAW-RATE
#define HEIGHT 1.5 //无人机飞行高度
//设置掩码时注意要用的就加上去，用的就不加，这里是用二进制表示，我需要用到VX/VY/VZ/YAW，所以这四个我给0，其他都是1.
/****************************/
class CXR_CTRL_V3
{
	public:
	//函数
		CXR_CTRL_V3();
		void rc_cb(const mavros_msgs::RCIn::ConstPtr& msg);
		void state_cb(const mavros_msgs::State::ConstPtr& msg);
		void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
		void allodom_cb(const nav_msgs::Odometry::ConstPtr&msg);
		double uav_to_goal(double x, double y, double z);
		void twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg);//ego的回调函数，读取ego轨迹前瞻点（即跟踪点）的位置和速度,并把前瞻点发布到rviz可视化出来
		double plan_yaw_rate(double des_yaw);//无人机航向角速度计算的函数，就是拿ego的yaw减去无人机的yaw得到角速度，并限制了最大角速度
		geometry_msgs::Point vel_command(double x, double y, double z);//无人机body系下的速度跟踪计算，并把ego的速度1作为前馈加上去，采取了前馈+PD进行跟踪
		void main_state(const ros::TimerEvent &e);//整体的运行逻辑部分的函数
		void pub_goal(double x, double y, double z);
		double limit_vel(double v, double max);
		void cmd_xyz();
		void update_flag();
        void set_offboard();
		void stop();
        void disarm();
        void arm();
	//ros
		ros::Timer status, zhuantai;
		ros::Subscriber state_sub, rc_sub, twist_sub, target_sub, odom_sub;
		ros::Subscriber allodom_sub;
		ros::Publisher local_pos_pub, target_pub;
		ros::Publisher servo_pub;
		ros::Time last_time;
		ros::ServiceClient disarm_client, set_mode_client;
	//msgs
		quadrotor_msgs::PositionCommand ego;
		visualization_msgs::Marker trackdrone, track_point_arrow, vision_point, trackpoint;
		tf::StampedTransform ts;//用来发布无人机当前位置的坐标系坐标轴
		tf::TransformBroadcaster tfBroadcasterPointer;	//广播坐标轴
		unsigned short velocity_mask = VELOCITY2D_CONTROL;
		mavros_msgs::PositionTarget current_goal;
		mavros_msgs::RCIn rc;
		int rc_value, exec_state = 0;
		nav_msgs::Odometry position_msg;
		mavros_msgs::State current_state;
		double position_x, position_y, position_z, now_x, now_y, currvx, currvy, currvz, current_yaw, targetpos_x, targetpos_y;
		double ego_pos_x, ego_pos_y, ego_pos_z,feedforward_x, feedforward_y, feedforward_z, last_setpoint_x, last_setpoint_y, i_pos, ego_yaw; //EGO planner information has position
		bool get_now_pos;//触发轨迹的条件判断
		geometry_msgs::Point vel;
		double des_vx, des_vy, des_vz, feed_gain, feed_gain_2, dt;
		/******************flag*************************/
		bool update = false, need_pub_goal = true, get_one = true, update_global = false;
		bool arm_flag = false;
		bool have_odom = false;
		bool ego_flag = false;
		//pd这块参数自己看着来，p一般1就行，d的话如果速度在3-4m/s的话，就设置0.2-0.3
		const double px = 1;
		const double dx = 0.0;
		const double py = 1;
		const double dy = 0.0;
		const double pz = 1;//z轴只需要p就可以了
		const double yaw_rate_gain = 2;//角速度p,设置3或者4也可以，根据自己需求来
		const double max_yaw_rate = 100;//最大角速度，建议别太低,如果速度在0-1.5m/s之间的话可以设置为80-90，当然100也没问题

		/**********init param************/
		double takeoff_alt, max_takeoff_vel, max_vel, offset_x, offset_y, offset_z;
		int drone_id;
		//jijianbizhang
		Eigen::Vector3d uav0, uav1, uav2, uav3;
		double dist_0_1, dist_0_2, dist_0_3, dist_1_2, dist_1_3, dist_2_3;
		double stop_x, stop_y, stop_z, bz_dist;
		std::vector<Eigen::Vector3d> uav;
		double distance_uav;
		bool bz_flag = false, jj_bz;
		double forward_dist;//forward fly dist
};

CXR_CTRL_V3::CXR_CTRL_V3()
{
	ros::NodeHandle nh("~");
	get_now_pos = false;
	/****************/
    disarm_client = nh.serviceClient<mavros_msgs::CommandLong> ("/mavros/cmd/command");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode> ("/mavros/set_mode");
	/*****************/
	/*******param*********/
	nh.param<int>("drone_id", drone_id, 0);
	nh.param<double>("takeoff_alt", takeoff_alt, 1.0);
	nh.param<double>("max_takeoff_vel", max_takeoff_vel, 1.0);
	nh.param<double>("max_vel", max_vel, 1.0);
	nh.param<double>("bz_dist", bz_dist, 2.0);
	nh.param<double>("forward_dist", forward_dist, 10.0);
	nh.param<double>("offset_x", offset_x, 0.0);
	nh.param<double>("offset_y", offset_y, 0.0);
	nh.param<double>("offset_z", offset_z, 0.0);
	nh.param<bool>("use_bizhang", jj_bz, false);
	/*********sub pub************/
	status = nh.createTimer(ros::Duration(0.02), &CXR_CTRL_V3::main_state, this);
	state_sub = nh.subscribe("state", 10, &CXR_CTRL_V3::state_cb, this);//读取飞控状态的话题
	rc_sub = nh.subscribe("rc/in",10, &CXR_CTRL_V3::rc_cb, this);//读取遥控器通道的话题，目前不需要
	twist_sub = nh.subscribe("position_cmd", 10, &CXR_CTRL_V3::twist_cb, this);//订阅egoplanner的规划指令话题的
	target_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal_"+std::to_string(drone_id), 1);
	odom_sub = nh.subscribe("odom", 10, &CXR_CTRL_V3::odom_cb, this);
	allodom_sub = nh.subscribe("/odom_new", 10, &CXR_CTRL_V3::allodom_cb, this);
	local_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("setpoint_raw/local", 1);
	// std::cout<<bz_dist<<std::endl;
}

void CXR_CTRL_V3::disarm()
{
    mavros_msgs::CommandLong disarm;
    disarm.request.broadcast = false;
    disarm.request.command = 400;
    disarm.request.param1 = 0;
    disarm.request.param2 = 21196;
    disarm_client.call(disarm);
}

void CXR_CTRL_V3::arm()
{
    mavros_msgs::CommandLong arm;
    arm.request.broadcast = false;
    arm.request.command = 400;
    arm.request.param1 = 1;
    arm.request.param2 = 21196;
    disarm_client.call(arm);
}

void CXR_CTRL_V3::stop()
{
	if(sqrt(currvx*currvx + currvy*currvy + currvz*currvz) < 0.25)
	{
		current_goal.velocity.x = limit_vel(vel_command(stop_x, stop_y, stop_z).x, 0.5);
		current_goal.velocity.y = limit_vel(vel_command(stop_x, stop_y, stop_z).y, 0.5);
		current_goal.velocity.z = limit_vel(vel_command(stop_x, stop_y, stop_z).z, 0.5);		
	}
	else
	{
		stop_x = position_x;
		stop_y = position_y;
		stop_z = position_z;
		current_goal.velocity.x = 0;
		current_goal.velocity.y = 0;
		current_goal.velocity.z = 0;
	}
	current_goal.yaw_rate = 0;
}

void CXR_CTRL_V3::set_offboard()
{
    mavros_msgs::SetMode set;
    set.request.custom_mode="OFFBOARD";
    set_mode_client.call(set);
}

 void CXR_CTRL_V3::rc_cb(const mavros_msgs::RCIn::ConstPtr&msg)
{
    rc = *msg;
    rc_value = rc.channels[4];
}

double CXR_CTRL_V3::limit_vel(double v, double max)
{
	double vel;
	if(v > max)
	{
		vel = max;
	}
	else if(v < -max)
	{
		vel = -max;
	}
	else
	{
		vel = v;
	}
	return vel;
}

void CXR_CTRL_V3::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
}

void CXR_CTRL_V3::allodom_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
	bz_flag = false;
	uav.resize(4);
	uav[std::stoi(msg->header.frame_id)](0) = msg->pose.pose.position.x;
	uav[std::stoi(msg->header.frame_id)](1) = msg->pose.pose.position.y;
	uav[std::stoi(msg->header.frame_id)](2) = msg->pose.pose.position.z;
	for(int i=0; i<uav.size(); i++)
	{
		distance_uav = (uav[i] - uav[drone_id]).norm();
		if(distance_uav < 0.1)
		{
			continue;
		}
		if(distance_uav < bz_dist)
		{
			if(i < drone_id)
			{
				bz_flag = true;
			}
		}
	}
	//std::cout<<"x= "<<uavpt[1](0)<<"\n"<<"y= "<<uavpt[1](1)<<"\n"<<"z= "<<uavpt[1][2]<<"\n"<<std::endl;
}

//read vehicle odometry
void CXR_CTRL_V3::odom_cb(const nav_msgs::Odometry::ConstPtr&msg)
{
    position_msg=*msg;
    ts.stamp_ = msg->header.stamp;
    ts.frame_id_ = "world";
    ts.child_frame_id_ = "drone_pos" + std::to_string(drone_id);
    ts.setRotation(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    ts.setOrigin(tf::Vector3(msg->pose.pose.position.x+offset_x, msg->pose.pose.position.y+offset_y, msg->pose.pose.position.z+offset_z));
    tfBroadcasterPointer.sendTransform(ts);
    if(!get_now_pos)
    {
        now_x = msg->pose.pose.position.x + offset_x;
        now_y = msg->pose.pose.position.y + offset_y;
		stop_x = msg->pose.pose.position.x + offset_x;
		stop_y = msg->pose.pose.position.y + offset_y;
		stop_z = msg->pose.pose.position.z + offset_z;
        get_now_pos = true;
    }
    position_x = msg->pose.pose.position.x + offset_x;
    position_y = msg->pose.pose.position.y + offset_y;
    position_z = msg->pose.pose.position.z + offset_z;
    currvx = position_msg.twist.twist.linear.x;
    currvy = position_msg.twist.twist.linear.y;
	currvz = position_msg.twist.twist.linear.z;
    tf2::Quaternion quat;
    tf2::convert(msg->pose.pose.orientation, quat); //把mavros/local_position/pose里的四元数转给tf2::Quaternion quat
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    current_yaw = yaw;
	have_odom = true;
}

double CXR_CTRL_V3::uav_to_goal(double x, double y, double z)
{
	double dist = sqrt(pow(x-position_x, 2) + pow(y-position_y, 2) + pow(z-position_z, 2));
	return dist;
}

void CXR_CTRL_V3::twist_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg)//ego的回调函数
{
	ego = *msg;
    ego_pos_x = ego.position.x;
	ego_pos_y = ego.position.y;
	ego_pos_z = ego.position.z;
    //速度可以作为前馈，根据需要用，这里把ego的速度转化到了机体body系下
    feedforward_x = ego.velocity.x * cos(current_yaw) + ego.velocity.y * sin(current_yaw);
	feedforward_y = -ego.velocity.x*sin(current_yaw) + ego.velocity.y*cos(current_yaw);
	feedforward_z = ego.velocity.z;
    ego_yaw = ego.yaw*180/PI;
	//ego_flag = true;
}

double CXR_CTRL_V3::plan_yaw_rate(double des_yaw)
{
	double the; // 极坐标系下的极角
    the = (des_yaw*PI/180.0) - current_yaw;
    // 限制极角的范围
    if (the > PI)
        the -= 2 * PI;
    else if (the < -PI)
        the += 2 * PI;
    if(the*180/PI > max_yaw_rate)
        the = max_yaw_rate*PI/180;
    else if(the*180/PI < -max_yaw_rate)
        the =-max_yaw_rate*PI/180;
    return yaw_rate_gain*the;
}

geometry_msgs::Point CXR_CTRL_V3::vel_command(double x, double y, double z)
{
    vel.x = (x - position_x) * cos(current_yaw) + (y - position_y) * sin(current_yaw);
    vel.y = -(x - position_x)*sin(current_yaw) + (y - position_y)*cos(current_yaw);
    vel.z = z - position_z;
    return vel;
}

void CXR_CTRL_V3::pub_goal(double x, double y, double z)
{
	geometry_msgs::PoseStamped tag;
	tag.pose.position.x = x;
	tag.pose.position.y = y;
	tag.pose.position.z = z;
	target_pub.publish(tag);
}

void CXR_CTRL_V3::cmd_xyz()
{
	current_goal.velocity.x = limit_vel(des_vx, max_vel);
	current_goal.velocity.y = limit_vel(des_vy, max_vel);
	current_goal.velocity.z = limit_vel(des_vz, max_vel);
}

void CXR_CTRL_V3::update_flag()
{
	need_pub_goal = true;
	get_one = true;
	last_time = ros::Time::now();
}

void CXR_CTRL_V3::main_state(const ros::TimerEvent &e)
{
	if(!have_odom)
	{
		return;
	}
	dt = ros::Time::now().toSec() - last_time.toSec();
	double Dx = 0.1;
	//std::cout<<dt<<std::endl;
	des_vx = feedforward_x + px * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).x - Dx * currvx;
	des_vy = feedforward_y + py * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).y - dy * currvy;
	des_vz = feedforward_z + pz * vel_command(ego_pos_x, ego_pos_y, ego_pos_z).z;
	current_goal.coordinate_frame = mavros_msgs::PositionTarget::FRAME_BODY_NED;
	current_goal.header.stamp = ros::Time::now();
	current_goal.type_mask = velocity_mask; 
	if(!ego_flag)
	{
		if(!arm_flag && current_state.mode == "OFFBOARD")
		{
			arm();
			arm_flag = true;
		}
		current_goal.velocity.x = limit_vel(vel_command(now_x, now_y, takeoff_alt).x, max_vel);
		current_goal.velocity.y = limit_vel(vel_command(now_x, now_y, takeoff_alt).y, max_vel);
		current_goal.velocity.z = limit_vel(1.5*vel_command(now_x, now_y, takeoff_alt).z, max_takeoff_vel);	
		current_goal.yaw_rate = plan_yaw_rate(0);//保持机头朝向不变	
		if(fabs(takeoff_alt - position_z) < 0.3)
		{
			if(need_pub_goal)
			{
				pub_goal(forward_dist+offset_x, 0+offset_y, HEIGHT+offset_z);
				need_pub_goal = false;
				update_flag();
				ego_flag = true;
			}
		}
	}
	else
	{
		if(uav_to_goal(forward_dist+offset_x, 0+offset_y, HEIGHT+offset_z) < 1)
		{
			if(need_pub_goal)
			{
				pub_goal(0+offset_x, 0+offset_y, HEIGHT+offset_z);
				need_pub_goal = false;
			}
		}

		if(jj_bz)
		{
			if(!bz_flag)
			{
				cmd_xyz();
				current_goal.yaw_rate = plan_yaw_rate(0);
			}
			else
			{
				stop();
			}
		}
		else
		{
			cmd_xyz();
			current_goal.yaw_rate = plan_yaw_rate(0);			
		}
	}
	local_pos_pub.publish(current_goal);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "main_ctrl");
	setlocale(LC_ALL,""); 
	CXR_CTRL_V3 cxr_ctrl_v3;
    std::cout<<"-----------------开始规划！--------------------"<<std::endl;
	ros::spin();
	return 0;
}
