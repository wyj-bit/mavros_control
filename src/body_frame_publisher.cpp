#include <ros/ros.h>
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
{
    // 使用 tf2 的 Quaternion 类
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw); // 输入的角度单位为弧度

    // 转换为 ROS 的 geometry_msgs::Quaternion
    geometry_msgs::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();

    return quat_msg;
}
void poseCallback(const nav_msgs::Odometry &msg)
{
    // 创建 TF 广播器
    static tf2_ros::TransformBroadcaster br;

    // 创建 TransformStamped 消息
    geometry_msgs::TransformStamped tf_dt;
    geometry_msgs::TransformStamped tf_bd;
    geometry_msgs::TransformStamped tf_mo;

    // 使用接收到的消息时间戳
    tf_dt.header.stamp = msg.header.stamp;
    tf_bd.header.stamp = msg.header.stamp;
    tf_mo.header.stamp = msg.header.stamp;

    // 设置第一个变换的父帧和子帧
    tf_dt.header.frame_id = "camera_pose_frame";      // 父坐标系
    tf_dt.child_frame_id = "d400_link"; // 子坐标系

    double pitch1 = 20 * M_PI/180;
    geometry_msgs::Quaternion quat1 = eulerToQuaternion(0.0, pitch1, 0.0);

    // 设置第一个变换的平移和旋转
    tf_dt.transform.translation.x = -0.0089607;
    tf_dt.transform.translation.y = 0;
    tf_dt.transform.translation.z = -0.063682;
    tf_dt.transform.rotation.x = quat1.x;
    tf_dt.transform.rotation.y = quat1.y;
    tf_dt.transform.rotation.z = quat1.z;
    tf_dt.transform.rotation.w = quat1.w;


    // 设置第二个变换的父帧和子帧
    tf_bd.header.frame_id = "d400_link";             // 父坐标系
    tf_bd.child_frame_id = "base_link"; // 子坐标系
    double pitch2 = 3.5 * M_PI/180;
    geometry_msgs::Quaternion quat2 = eulerToQuaternion(0.0, pitch2, 0.0);

    // 设置第二个变换的平移和旋转
    tf_bd.transform.translation.x = -0.0813359;
    tf_bd.transform.translation.y = 0;
    tf_bd.transform.translation.z = -0.1133749;
    tf_bd.transform.rotation.x = quat2.x;
    tf_bd.transform.rotation.y = quat2.y;
    tf_bd.transform.rotation.z = quat2.z;
    tf_bd.transform.rotation.w = quat2.w;
    
    
    // 设置第三个变换的父帧和子帧
    tf_mo.header.frame_id = "camera_odom_frame";             // 父坐标系
    tf_mo.child_frame_id = "map"; // 子坐标系

    // 设置第三个变换的平移和旋转
    tf_mo.transform.translation.x = -0.05235;
    tf_mo.transform.translation.y = 0;
    tf_mo.transform.translation.z = -0.1815375;
    tf_mo.transform.rotation.x = 0;
    tf_mo.transform.rotation.y = 0;
    tf_mo.transform.rotation.z = 0;
    tf_mo.transform.rotation.w = 1;

    // 发布第一个变换
    br.sendTransform(tf_dt);


    // 发布第二个变换
    br.sendTransform(tf_bd);
    
    // 发布第三个变换
    br.sendTransform(tf_mo);

    // 日志输出
    ROS_INFO("Published transform: %s -> %s [x: %.2f, y: %.2f, z: %.2f]",
             tf_dt.header.frame_id.c_str(),
             tf_dt.child_frame_id.c_str(),
             tf_dt.transform.translation.x,
             tf_dt.transform.translation.y,
             tf_dt.transform.translation.z);

    // ROS_INFO("Published transform: %s -> %s [x: %.2f, y: %.2f, z: %.2f]",
    //          tf_bd.header.frame_id.c_str(),
    //          tf_bd.child_frame_id.c_str(),
    //          tf_bd.transform.translation.x,
    //          tf_bd.transform.translation.y,
    //          tf_bd.transform.translation.z);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "body_frame_publisher");

    // 创建节点句柄
    ros::NodeHandle nh;

    // 订阅 /mavros/local_position/pose 主题
    ros::Subscriber pose_sub = nh.subscribe("/camera/odom/sample", 10, poseCallback);

    // 循环等待回调
    ros::spin();

    return 0;
}
