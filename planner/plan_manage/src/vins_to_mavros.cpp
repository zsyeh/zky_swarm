#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于 tf2 和 geometry_msgs 之间的转换
#include <cmath> // 用于 M_PI

geometry_msgs::PoseStamped px4_pose;
ros::Publisher px4_pub;

// 回调函数，当收到 /Odometry 消息时被调用
void poseCb(const nav_msgs::Odometry::ConstPtr& msg) {
    // 1. 创建一个代表修正旋转的四元数
    // 传感器向前倾斜了25度，所以我们需要应用一个-25度的俯仰角（Pitch）来补偿。
    // setRPY的参数分别是 滚转(roll), 俯仰(pitch), 偏航(yaw)，单位是弧度。
    tf2::Quaternion correction_quaternion;
    correction_quaternion.setRPY(0, -25.0 * M_PI / 180.0, 0);

    // 2. 获取来自LIO的原始姿态四元数
    tf2::Quaternion original_quaternion;
    // 使用 tf2::fromMsg 将 geometry_msgs::Quaternion 转换为 tf2::Quaternion
    tf2::fromMsg(msg->pose.pose.orientation, original_quaternion);

    // 3. 将修正旋转应用到原始姿态上
    // 旋转的叠加通过四元数乘法实现。正确的顺序是 原始姿态 * 修正量
    tf2::Quaternion corrected_quaternion = original_quaternion * correction_quaternion;
    corrected_quaternion.normalize(); // 乘法后最好进行归一化，确保它是一个有效的旋转四元数

    // 4. 填充要发布的消息
    // header 和 LIO 的保持一致
    px4_pose.header = msg->header;
    // 位置信息 (position) 不需要改变，直接沿用
    px4_pose.pose.position = msg->pose.pose.position;
    // 姿态信息 (orientation) 使用我们修正后的结果
    // 使用 tf2::toMsg 将 tf2::Quaternion 转换回 geometry_msgs::Quaternion
    px4_pose.pose.orientation = tf2::toMsg(corrected_quaternion);

    // 5. 发布修正后的位姿
    px4_pub.publish(px4_pose);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh;

    // 发布给 MAVROS 的话题
    px4_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 1);
    // 订阅来自 Fast-LIO 的里程计话题
    ros::Subscriber px4_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 1, poseCb);

    ros::spin();

    return 0;
}