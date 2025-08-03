#!/usr/bin/env python3

import rospy
import csv
from mavros_msgs.msg import PositionTarget

def waypoint_publisher():
    # 初始化ROS节点
    rospy.init_node('waypoint_publisher', anonymous=True)
    
    # 创建发布者，发布到/yopo/pos_cmd话题，消息类型为PositionTarget
    pub = rospy.Publisher('/yopo/pos_cmd', PositionTarget, queue_size=10)
    
    # 设置发布频率（Hz）
    rate = rospy.Rate(50)  # 1Hz
    
    # CSV文件路径，根据实际情况修改
    csv_file = '/home/nx/workspace/src/mavros_control/src/trajectory.csv'
    
    try:
        # 打开CSV文件
        with open(csv_file, 'r') as file:
            reader = csv.reader(file)
            # 跳过标题行
            next(reader)
            
            # 遍历每一行数据
            for row in reader:
                if len(row) != 11:
                    rospy.logerr(f"无效的航点数据: {row}")
                    continue
                
                # 创建PositionTarget消息
                msg = PositionTarget()
                
                # 设置坐标帧和类型掩码
                msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
                # 配置需要使用的字段，这里假设所有字段都使用
                msg.type_mask = 0
                
                # 解析CSV数据
                x, y, z, vx, vy, vz, ax, ay, az, yaw, yaw_rate = map(float, row)
                
                # 设置位置
                msg.position.x = x
                msg.position.y = y
                msg.position.z = z
                
                # 设置速度
                msg.velocity.x = vx
                msg.velocity.y = vy
                msg.velocity.z = vz
                
                # 设置加速度/力
                msg.acceleration_or_force.x = ax
                msg.acceleration_or_force.y = ay
                msg.acceleration_or_force.z = az
                
                # 设置偏航角和偏航角速度
                msg.yaw = yaw
                msg.yaw_rate = yaw_rate
                
                # 发布消息
                pub.publish(msg)
                rospy.loginfo(f"发布航点: x={x}, y={y}, z={z}, vx={vx}, vy={vy}, vz={vz}")
                
                # 按照设定的频率休眠
                rate.sleep()
    
    except FileNotFoundError:
        rospy.logerr(f"找不到CSV文件: {csv_file}")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

if __name__ == '__main__':
    try:
        waypoint_publisher()
    except rospy.ROSInterruptException:
        pass
