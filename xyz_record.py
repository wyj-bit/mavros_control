#!/usr/bin/env python
import rospy
import csv
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import os

# 初始化数据存储
mavros_data = None
t265_data = None

# 获取脚本路径
script_path = os.path.dirname(os.path.realpath(__file__))

# 打开两个 CSV 文件并写入表头
local_csv = open(os.path.join(script_path, 'local_position.csv'), 'w')
t265_csv = open(os.path.join(script_path, 't265.csv'), 'w')

local_writer = csv.writer(local_csv)
t265_writer = csv.writer(t265_csv)

local_writer.writerow(['x', 'y', 'z'])
t265_writer.writerow(['x', 'y', 'z'])

# 回调函数：记录最新消息
def mavros_callback(msg):
    global mavros_data
    mavros_data = msg

def t265_callback(msg):
    global t265_data
    t265_data = msg

# 定时器回调：每 0.02 秒写入一次
def timer_callback(event):
    if mavros_data:
        pos = mavros_data.pose.position
        local_writer.writerow([pos.x, pos.y, pos.z])
    if t265_data:
        pos = t265_data.pose.pose.position
        t265_writer.writerow([pos.x, pos.y, pos.z])

def main():
    rospy.init_node('fixed_rate_logger', anonymous=True)

    # 订阅话题
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, mavros_callback)
    rospy.Subscriber('/t265/odom', Odometry, t265_callback)

    # 设置定时器：50Hz = 每隔0.02秒
    rospy.Timer(rospy.Duration(0.02), timer_callback)

    rospy.spin()

    # 程序结束时关闭文件
    local_csv.close()
    t265_csv.close()

if __name__ == '__main__':
    main()
