/*
 * ROS1版本: Offboard 控制节点
 * 将原ROS2代码转换为ROS1 (roscpp) 实现
 */

#include <ros/ros.h>
#include <stdint.h>
#include <atomic>
#include <thread>
#include <chrono>
#include <iostream>

// MAVROS 消息与服务
#include <mavros_msgs/State.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/PositionTarget.h>

// ROS消息
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// TF2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

// Eigen
#include <Eigen/Eigen>

// 结束任务信号
static std::atomic<bool> is_finished(false);

namespace Controller
{

    class MainProcess
    {
    public:
        MainProcess();
        ~MainProcess();

    public:
        // --- 内部数据结构 ---
        struct StateInfo
        {
            uint64_t timestamp = 0;
            mavros_msgs::State px4_state;
            Eigen::Vector3d global_position_ori = Eigen::Vector3d::Zero();
            Eigen::Vector3d global_position = Eigen::Vector3d::Zero();
            Eigen::Vector3d local_position = Eigen::Vector3d::Zero();
            Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
            Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
            Eigen::Quaterniond attitude = Eigen::Quaterniond::Identity();
            Eigen::Vector3d euler_angle = Eigen::Vector3d::Zero();
            Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
        } state_info_;


        struct Config
        {
            int Init_Height = 2;
            int SubcribeFre = 50;
            int PublishFre = 50;
            int Fre_Rate = 1;
            bool Is_message_Timeout = true;
        } config_;

        // ROS 对象
        ros::NodeHandle nh_;
        ros::Subscriber sub_px4_state_;
        ros::ServiceClient client_set_mode_;
        ros::Timer timer_main_process_;

        // 回调与处理函数
        void stateCallback(const mavros_msgs::State::ConstPtr &msg);
        void CallbackMainProcess(const ros::TimerEvent &);
    };

    MainProcess::MainProcess()
    {
        // 参数
        nh_.param("subscribe_rate", config_.SubcribeFre, 50);
        nh_.param("publish_rate", config_.PublishFre, 50);
        nh_.param("flight_height", config_.Init_Height, 2);
        config_.Fre_Rate = std::ceil((double)config_.SubcribeFre / config_.PublishFre);


        // 订阅器
        sub_px4_state_ = nh_.subscribe("/mavros/state", 10, &MainProcess::stateCallback, this);

        // 客户端
        client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        // 定时器
        ros::Duration pub_period(1.0 / config_.PublishFre);
        timer_main_process_ = nh_.createTimer(pub_period, &MainProcess::CallbackMainProcess, this);

        ROS_INFO("ROS1 Offboard Control Node Initialized");
    }

    MainProcess::~MainProcess()
    {
        // nothing special
    }

    void MainProcess::stateCallback(const mavros_msgs::State::ConstPtr &msg)
    {
        state_info_.timestamp = ros::Time::now().toNSec();
        state_info_.px4_state = *msg;
    }

    void MainProcess::CallbackMainProcess(const ros::TimerEvent &)
    {
        static long long offboard_counter = 0;
        static long long first_flight_counter = 0;
        static bool first_flight_recorded = false;
        static bool is_return = false;
        static bool over = false;

        if (state_info_.px4_state.connected)
        {
            ROS_INFO_ONCE("Connected to PX4");

            // 切换到 OFFBOARD
            if (!over && !is_return && state_info_.px4_state.mode != "OFFBOARD" && offboard_counter > 50)
            {
                mavros_msgs::SetMode mode_srv;
                mode_srv.request.custom_mode = "OFFBOARD";
                client_set_mode_.call(mode_srv);
                ROS_INFO("Switching to OFFBOARD mode");
            }
            offboard_counter++;
        }
    }

} // namespace Controller


int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "mavros_px4_node");
    ros::NodeHandle nh;

    // 读取 delay_time 参数，默认值为 0
    int delay_time = 0;
    nh.param("delay_time", delay_time, 0);

    // 打印延迟时间
    ROS_INFO("Delay time before starting: %d seconds", delay_time);

    // 延迟启动
    if (delay_time > 0)
    {
        std::this_thread::sleep_for(std::chrono::seconds(delay_time));
    }

    ROS_INFO("Starting mavros_px4_node after delay...");

    // 启动主逻辑
    Controller::MainProcess node;

    // ROS 事件循环
    ros::spin();
    return 0;
}
