#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class InfraredObstacleAvoider:
    def __init__(self):
        # 初始化ROS节点（匿名=True避免节点名冲突）
        rospy.init_node('infrared_obstacle_avoider', anonymous=True)
        
        # 发布速度指令（话题：/cmd_vel，队列大小10）
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # 障碍物检测阈值（单位：米，可通过参数服务器修改）
        self.obstacle_threshold = rospy.get_param('~obstacle_threshold', 0.3)
        
        # 存储左/前/右红外传感器距离（初始值设为无穷大）
        self.left_dist = float('inf')
        self.front_dist = float('inf')
        self.right_dist = float('inf')
        
        # 订阅红外传感器话题（需与硬件发布的话题一致）
        rospy.Subscriber('/infrared/left', Range, self.left_cb)
        rospy.Subscriber('/infrared/front', Range, self.front_cb)
        rospy.Subscriber('/infrared/right', Range, self.right_cb)
        
        # 运动参数（可根据小车性能调整）
        self.linear_speed = 0.2    # 前进速度（m/s）
        self.angular_speed = 0.5   # 旋转速度（rad/s）
        
        # 循环频率（10Hz，每0.1秒执行一次避障逻辑）
        self.rate = rospy.Rate(10)
        
        rospy.loginfo("✅ 红外避障节点已启动！")

    # 左侧传感器回调函数（更新距离数据）
    def left_cb(self, msg):
        self.left_dist = msg.range

    # 前方传感器回调函数
    def front_cb(self, msg):
        self.front_dist = msg.range

    # 右侧传感器回调函数
    def right_cb(self, msg):
        self.right_dist = msg.range

    # 核心避障逻辑
    def avoid_obstacle(self):
        twist = Twist()  # 初始化速度指令消息
        
        # 前方有障碍物：停止前进，向空旷方向转向
        if self.front_dist < self.obstacle_threshold:
            rospy.logwarn(f"⚠️  前方检测到障碍物（距离：{self.front_dist:.2f}m）")
            twist.linear.x = 0.0  # 停止前进
            # 优先向左侧空旷方向转向，否则向右侧
            if self.left_dist > self.right_dist:
                rospy.loginfo("🔄 向左转向...")
                twist.angular.z = self.angular_speed
            else:
                rospy.loginfo("🔄 向右转向...")
                twist.angular.z = -self.angular_speed
        
        # 前方无障碍物：正常前进，侧面有障碍物则微调
        else:
            twist.linear.x = self.linear_speed  # 前进
            twist.angular.z = 0.0               # 不旋转
            
            # 左侧有近距离障碍物：轻微右转
            if self.left_dist < self.obstacle_threshold * 1.2:
                rospy.loginfo("⚠️  左侧有障碍物，轻微右转")
                twist.angular.z = -self.angular_speed * 0.5
            # 右侧有近距离障碍物：轻微左转
            elif self.right_dist < self.obstacle_threshold * 1.2:
                rospy.loginfo("⚠️  右侧有障碍物，轻微左转")
                twist.angular.z = self.angular_speed * 0.5
        
        self.cmd_vel_pub.publish(twist)  # 发布速度指令

    # 节点运行循环
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.avoid_obstacle()
                self.rate.sleep()
        except rospy.ROSInterruptException:
            rospy.loginfo("🛑 避障节点已停止")
        finally:
            # 程序结束时停止小车
            twist = Twist()
            self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    try:
        avoider = InfraredObstacleAvoider()
        avoider.run()
    except rospy.ROSInterruptException:
        pass
