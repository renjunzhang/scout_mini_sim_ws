#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class MyPlanner:
    def __init__(self):
        # 1. 初始化节点
        rospy.init_node('my_mppi_planner', anonymous=True)

        # 2. 设置订阅者 (感知 & 定位)
        # 订阅激光雷达 (用于避障)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        # 订阅里程计 (用于知道车在哪)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 3. 设置发布者 (控制)
        # 发布速度指令给 Scout Mini
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 4. 算法参数
        self.rate = rospy.Rate(20) # 20Hz 控制频率
        self.current_scan = None
        self.current_odom = None

        rospy.loginfo("My Planner Started! Waiting for sensors...")

    def scan_callback(self, msg):
        """ 获取激光雷达数据 """
        self.current_scan = msg.ranges
        # 在这里处理雷达数据，比如转换成局部地图

    def odom_callback(self, msg):
        """ 获取里程计数据 """
        # 提取位置 x, y 和 朝向 (四元数转欧拉角)
        self.current_odom = msg.pose.pose

    def run_mppi_algo(self):
        """ 
        这里是 MPPI 算法的核心循环 
        以后你的 MPPI 代码主要填在这里
        """
        twist = Twist()

        # --- 简单的测试逻辑 (以后删除) ---
        # 如果有雷达数据，且正前方 0.5米没障碍，就直走
        if self.current_scan is not None:
            # 假设前方索引是 0 (取决于雷达安装方向，可能需要调整)
            front_dist = self.current_scan[0] 

            if front_dist > 0.5:
                twist.linear.x = 0.2  # 慢速前进
                twist.angular.z = 0.0
            else:
                twist.linear.x = 0.0
                rospy.loginfo("Obstacle detected! Stop.")
        # -------------------------------

        return twist

    def start(self):
        while not rospy.is_shutdown():
            # 1. 运行算法计算控制量
            cmd = self.run_mppi_algo()

            # 2. 发送指令
            self.cmd_pub.publish(cmd)

            # 3. 休眠等待下一帧
            self.rate.sleep()

if __name__ == '__main__':
    try:
        planner = MyPlanner()
        planner.start()
    except rospy.ROSInterruptException:
        pass