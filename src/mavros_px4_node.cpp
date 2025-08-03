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

        struct PlanningAttitude
        {
            Eigen::Matrix<double, 11, 1> data;
            PlanningAttitude()
            {
                data << 0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0;
            }
        } planning_attitude_;

        struct Config
        {
            double Delta = 0.0;
            double Init_Height = 3;
            int SubcribeFre = 50;
            int PublishFre = 50;
            int Fre_Rate = 1;
            bool Is_message_Timeout = true;
        } config_;

        // ROS 对象
        ros::NodeHandle nh_;
        ros::Publisher pub_position_target_;
        ros::Subscriber sub_px4_state_, sub_local_position_, sub_global_position_;
        ros::Subscriber sub_velocity_, sub_acceleration_, sub_attitude_, sub_planning_attitude_;
        ros::ServiceClient client_arm_, client_set_mode_;
        ros::Timer timer_main_process_, timer_planning_timeout_;
        ros::Time last_receive_time_;

        // 回调与处理函数
        void stateCallback(const mavros_msgs::State::ConstPtr &msg);
        void localPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
        void globalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg);
        void accelCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void attitudeCallback(const sensor_msgs::Imu::ConstPtr &msg);
        void planningCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
        void CallbackMainProcess(const ros::TimerEvent &);
        void checkTimeout(const ros::TimerEvent &);
    };

    MainProcess::MainProcess()
    {
        // 参数
        nh_.param("subscribe_rate", config_.SubcribeFre, 50);
        nh_.param("publish_rate", config_.PublishFre, 50);
        nh_.param("flight_height", config_.Init_Height, 3.0);
        config_.Fre_Rate = std::ceil((double)config_.SubcribeFre / config_.PublishFre);

        // 发布器
        pub_position_target_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);

        // 订阅器
        sub_px4_state_ = nh_.subscribe("/mavros/state", 10, &MainProcess::stateCallback, this);
        sub_local_position_ = nh_.subscribe("/mavros/local_position/pose", 10, &MainProcess::localPosCallback, this);
        sub_global_position_ = nh_.subscribe("/mavros/global_position/global", 10, &MainProcess::globalPosCallback, this);
        sub_velocity_ = nh_.subscribe("/mavros/local_position/velocity_local", 10, &MainProcess::velocityCallback, this);
        sub_acceleration_ = nh_.subscribe("/mavros/imu/data", 10, &MainProcess::accelCallback, this);
        sub_attitude_ = nh_.subscribe("/mavros/imu/data", 10, &MainProcess::attitudeCallback, this);
        sub_planning_attitude_ = nh_.subscribe("/yopo/pos_cmd", 10, &MainProcess::planningCallback, this);

        // 客户端
        client_arm_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        client_set_mode_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

        // 定时器
        ros::Duration pub_period(1.0 / config_.PublishFre);
        timer_main_process_ = nh_.createTimer(pub_period, &MainProcess::CallbackMainProcess, this);
        ros::Duration check_period((1.0 / config_.SubcribeFre) / 10.0);
        timer_planning_timeout_ = nh_.createTimer(check_period, &MainProcess::checkTimeout, this);

        last_receive_time_ = ros::Time::now();
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

    void MainProcess::localPosCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        state_info_.local_position = Eigen::Vector3d(msg->pose.position.x,
                                                     msg->pose.position.y,
                                                     msg->pose.position.z);
        tf2::Quaternion q(msg->pose.orientation.x,
                          msg->pose.orientation.y,
                          msg->pose.orientation.z,
                          msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        m.getRPY(state_info_.euler_angle[0], state_info_.euler_angle[1], state_info_.euler_angle[2]);
    }

    void MainProcess::globalPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        state_info_.global_position = Eigen::Vector3d(msg->latitude, msg->longitude, msg->altitude);
    }

    void MainProcess::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr &msg)
    {
        state_info_.velocity = Eigen::Vector3d(msg->twist.linear.x,
                                               msg->twist.linear.y,
                                               msg->twist.linear.z);
    }

    void MainProcess::accelCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        state_info_.acceleration = Eigen::Vector3d(msg->linear_acceleration.x,
                                                   msg->linear_acceleration.y,
                                                   msg->linear_acceleration.z);
    }

    void MainProcess::attitudeCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        state_info_.attitude = Eigen::Quaterniond(msg->orientation.w,
                                                  msg->orientation.x,
                                                  msg->orientation.y,
                                                  msg->orientation.z);
        state_info_.angular_velocity = Eigen::Vector3d(msg->angular_velocity.x,
                                                       msg->angular_velocity.y,
                                                       msg->angular_velocity.z);
    }

    void MainProcess::planningCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
    {
        planning_attitude_.data.head<3>() = Eigen::Vector3d(msg->position.x,
                                                            msg->position.y,
                                                            msg->position.z);
        planning_attitude_.data.segment<3>(3) = Eigen::Vector3d(msg->velocity.x,
                                                                msg->velocity.y,
                                                                msg->velocity.z);
        planning_attitude_.data.segment<3>(6) = Eigen::Vector3d(msg->acceleration_or_force.x,
                                                                msg->acceleration_or_force.y,
                                                                msg->acceleration_or_force.z);
        planning_attitude_.data.segment<2>(9) = Eigen::Vector2d(msg->yaw, msg->yaw_rate);
        last_receive_time_ = ros::Time::now();
        config_.Is_message_Timeout = false;
    }

    void MainProcess::checkTimeout(const ros::TimerEvent &)
    {
        if ((ros::Time::now() - last_receive_time_).toSec() > (1.0 / config_.SubcribeFre * 1.05))
        {
            config_.Is_message_Timeout = true;
            // ROS_WARN("Control message timeout, hovering...");
        }
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
            // 解锁
            if (!over && !is_return && !state_info_.px4_state.armed && offboard_counter > 250)
            {
                mavros_msgs::CommandBool arm_srv;
                arm_srv.request.value = true;
                client_arm_.call(arm_srv);
                ROS_INFO("Arming... ");
            }

            // 切换到 OFFBOARD
            if (!over && !is_return && state_info_.px4_state.mode != "OFFBOARD" && state_info_.px4_state.armed && offboard_counter > 250)
            {
                mavros_msgs::SetMode mode_srv;
                mode_srv.request.custom_mode = "OFFBOARD";
                client_set_mode_.call(mode_srv);
                ROS_INFO("Switching to OFFBOARD mode");
            }



            // 起飞阶段
            if (!over && !is_return && state_info_.local_position.z() > config_.Init_Height + config_.Delta - 0.3 &&
                state_info_.local_position.z() < config_.Init_Height + config_.Delta+ 0.3 &&
                state_info_.px4_state.mode == "OFFBOARD" && state_info_.px4_state.armed && !first_flight_recorded)
            {
                first_flight_counter = offboard_counter;
                first_flight_recorded = true;
                ROS_INFO("Reached takeoff altitude");
            }

            mavros_msgs::PositionTarget cmd;
            cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
            cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                            mavros_msgs::PositionTarget::IGNORE_AFY |
                            mavros_msgs::PositionTarget::IGNORE_AFZ |
                            mavros_msgs::PositionTarget::IGNORE_VX |
                            mavros_msgs::PositionTarget::IGNORE_VY |
                            mavros_msgs::PositionTarget::IGNORE_VZ |
                            mavros_msgs::PositionTarget::IGNORE_YAW |
                            mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

            if (!over && !is_return && !first_flight_recorded)
            {
                // 起飞保持 x=0,y=0,z=Init_Height
                cmd.position.x = 0;
                cmd.position.y = 0;
                cmd.position.z = config_.Init_Height+ config_.Delta;
                pub_position_target_.publish(cmd);
            }
            else if (first_flight_recorded)
            {
                if (is_finished)
                {
                    // RTL降落
                    if (!is_return)
                    {
                        mavros_msgs::SetMode mode_srv;
                        mode_srv.request.custom_mode = "POSCTL";
                        client_set_mode_.call(mode_srv);
                        ROS_INFO("Switching to POS mode");
                        is_return = true;
                    }
                    // 已降落并锁定
                    // if (state_info_.local_position.z() < 0.1 && !over)
                    // {
                    //     mavros_msgs::CommandBool disarm_srv;
                    //     disarm_srv.request.value = false;
                    //     client_arm_.call(disarm_srv);
                    //     ROS_INFO("Landed and disarmed");
                    //     over = true;
                    // }
                    // 发布保持位置
                    cmd.position.x = planning_attitude_.data(0);
                    cmd.position.y = planning_attitude_.data(1);
                    cmd.position.z = planning_attitude_.data(2)+ config_.Delta;
                    pub_position_target_.publish(cmd);
                }
                else if (config_.Is_message_Timeout)
                {
                    // 超时悬停
                    cmd.position.x = planning_attitude_.data(0);
                    cmd.position.y = planning_attitude_.data(1);
                    cmd.position.z = planning_attitude_.data(2)+ config_.Delta;
                    pub_position_target_.publish(cmd);
                }
                else
                {
                    // 按规划飞行
                    cmd.position.x = planning_attitude_.data(0);
                    cmd.position.y = planning_attitude_.data(1);
                    cmd.position.z = planning_attitude_.data(2)+ config_.Delta;
                    cmd.velocity.x = planning_attitude_.data(3);
                    cmd.velocity.y = planning_attitude_.data(4);
                    cmd.velocity.z = planning_attitude_.data(5);
                    cmd.acceleration_or_force.x = planning_attitude_.data(6);
                    cmd.acceleration_or_force.y = planning_attitude_.data(7);
                    cmd.acceleration_or_force.z = planning_attitude_.data(8);
                    cmd.yaw = planning_attitude_.data(9);
                    cmd.yaw_rate = planning_attitude_.data(10);
                    cmd.type_mask &= ~(mavros_msgs::PositionTarget::IGNORE_VX |
                                       mavros_msgs::PositionTarget::IGNORE_VY |
                                       mavros_msgs::PositionTarget::IGNORE_VZ |
                                       mavros_msgs::PositionTarget::IGNORE_AFX |
                                       mavros_msgs::PositionTarget::IGNORE_AFY |
                                       mavros_msgs::PositionTarget::IGNORE_AFZ |
                                       mavros_msgs::PositionTarget::IGNORE_YAW |
                                       mavros_msgs::PositionTarget::IGNORE_YAW_RATE);

                    pub_position_target_.publish(cmd);
                    ROS_INFO("Following Path ......");
                }
            }
            offboard_counter++;
        }
    }

} // namespace Controller

// 键盘监听
void keyboardListener()
{
    char c;
    while (std::cin >> c)
    {
        if (c == 'p')
        {
            is_finished = true;
            ROS_INFO("Key 'p' pressed, finishing mission");
            break;
        }
    }
}

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

    // 启动键盘监听线程
    std::thread listener(keyboardListener);

    // ROS 事件循环
    ros::spin();

    // 等待键盘监听线程结束
    listener.join();

    return 0;
}
