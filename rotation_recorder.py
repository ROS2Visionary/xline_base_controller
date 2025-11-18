#!/usr/bin/env python3
"""
ROS2节点：控制机器人原地旋转并记录轨迹
功能：
1. 订阅/robot_pose和/imu话题
2. 发布/cmd_vel控制机器人旋转
3. 使用IMU计算旋转圈数
4. 实时记录位置到CSV
5. 绘制轨迹并计算半径
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import least_squares
import threading
import time
import math
from tf_transformations import euler_from_quaternion


class RotationRecorder(Node):
    def __init__(self):
        super().__init__('rotation_recorder')
        
        # 订阅话题
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/robot_pose',
            self.pose_callback,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10)
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        # 状态变量
        self.current_pose = None
        self.current_yaw = 0.0
        self.initial_yaw = None
        self.total_rotation = 0.0  # 累计旋转角度（弧度）
        self.last_yaw = None
        
        # 记录数据
        self.position_data = []
        self.csv_file = None
        self.csv_writer = None
        
        # 控制标志
        self.is_rotating = False
        self.target_rotations = 0.0
        
        self.get_logger().info('旋转记录节点已启动！')
        self.get_logger().info('等待话题数据...')
        
    def pose_callback(self, msg):
        """接收机器人位置"""
        self.current_pose = msg
        
        # 如果正在旋转，记录位置
        if self.is_rotating and self.csv_writer is not None:
            x = msg.pose.position.x
            y = msg.pose.position.y
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            # 写入CSV
            self.csv_writer.writerow([timestamp, x, y])
            self.csv_file.flush()
            
            # 保存到内存
            self.position_data.append([x, y])
    
    def imu_callback(self, msg):
        """接收IMU数据并计算旋转角度"""
        # 从四元数提取yaw角
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.current_yaw = yaw
        
        # 初始化
        if self.initial_yaw is None:
            self.initial_yaw = yaw
            self.last_yaw = yaw
            return
        
        # 计算角度变化（处理-π到π的跳变）
        if self.last_yaw is not None:
            delta_yaw = yaw - self.last_yaw
            
            # 处理角度跳变
            if delta_yaw > math.pi:
                delta_yaw -= 2 * math.pi
            elif delta_yaw < -math.pi:
                delta_yaw += 2 * math.pi
            
            self.total_rotation += abs(delta_yaw)
        
        self.last_yaw = yaw
    
    def start_rotation(self, num_rotations, angular_speed=0.5):
        """开始旋转指定圈数"""
        if self.is_rotating:
            self.get_logger().warn('已经在旋转中！')
            return
        
        # 重置状态
        self.is_rotating = True
        self.target_rotations = num_rotations
        self.total_rotation = 0.0
        self.position_data = []
        
        # 创建CSV文件
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        csv_filename = f'rotation_data_{timestamp}.csv'
        self.csv_file = open(csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'x', 'y'])
        
        self.get_logger().info(f'开始旋转 {num_rotations} 圈，角速度: {angular_speed} rad/s')
        self.get_logger().info(f'数据记录到: {csv_filename}')
        
        # 发布旋转命令
        target_angle = num_rotations * 2 * math.pi
        rate = self.create_rate(50)  # 50Hz
        
        while self.is_rotating and rclpy.ok():
            # 检查是否完成
            if self.total_rotation >= target_angle:
                self.get_logger().info(f'完成旋转！总角度: {math.degrees(self.total_rotation):.2f}°')
                self.stop_rotation()
                break
            
            # 发布速度命令
            cmd = Twist()
            cmd.angular.z = angular_speed
            self.cmd_vel_pub.publish(cmd)
            
            # 显示进度
            progress = (self.total_rotation / target_angle) * 100
            rotations_completed = self.total_rotation / (2 * math.pi)
            self.get_logger().info(
                f'进度: {progress:.1f}% | 已旋转: {rotations_completed:.2f} 圈',
                throttle_duration_sec=0.5)
            
            rate.sleep()
    
    def stop_rotation(self):
        """停止旋转"""
        # 发送停止命令
        cmd = Twist()
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        self.is_rotating = False
        
        # 关闭CSV文件
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        
        self.get_logger().info('旋转已停止')
        
        # 分析数据
        if len(self.position_data) > 10:
            self.analyze_trajectory()
    
    def analyze_trajectory(self):
        """分析轨迹并绘制"""
        self.get_logger().info('分析轨迹数据...')
        
        data = np.array(self.position_data)
        x = data[:, 0]
        y = data[:, 1]
        
        # 计算圆心和半径
        center_x, center_y, radius = self.fit_circle(x, y)
        
        self.get_logger().info(f'拟合圆心: ({center_x:.4f}, {center_y:.4f})')
        self.get_logger().info(f'拟合半径: {radius:.4f} m')
        
        # 计算误差统计
        distances = np.sqrt((x - center_x)**2 + (y - center_y)**2)
        mean_radius = np.mean(distances)
        std_radius = np.std(distances)
        max_deviation = np.max(np.abs(distances - radius))
        
        self.get_logger().info(f'平均半径: {mean_radius:.4f} m')
        self.get_logger().info(f'半径标准差: {std_radius:.4f} m')
        self.get_logger().info(f'最大偏差: {max_deviation:.4f} m')
        
        # 绘制轨迹
        self.plot_trajectory(x, y, center_x, center_y, radius)
    
    def fit_circle(self, x, y):
        """最小二乘法拟合圆"""
        def calc_R(xc, yc):
            """计算到圆心的距离"""
            return np.sqrt((x - xc)**2 + (y - yc)**2)
        
        def f(c):
            """目标函数：最小化半径的方差"""
            Ri = calc_R(*c)
            return Ri - Ri.mean()
        
        # 初始猜测：数据的中心点
        center_estimate = np.mean(x), np.mean(y)
        result = least_squares(f, center_estimate)
        
        xc, yc = result.x
        Ri = calc_R(xc, yc)
        R = Ri.mean()
        
        return xc, yc, R
    
    def plot_trajectory(self, x, y, center_x, center_y, radius):
        """绘制轨迹图"""
        plt.figure(figsize=(10, 10))
        
        # 绘制轨迹
        plt.plot(x, y, 'b-', linewidth=2, label='机器人轨迹')
        plt.plot(x[0], y[0], 'go', markersize=10, label='起点')
        plt.plot(x[-1], y[-1], 'ro', markersize=10, label='终点')
        
        # 绘制拟合圆
        theta = np.linspace(0, 2*np.pi, 100)
        circle_x = center_x + radius * np.cos(theta)
        circle_y = center_y + radius * np.sin(theta)
        plt.plot(circle_x, circle_y, 'r--', linewidth=2, label=f'拟合圆 (R={radius:.4f}m)')
        
        # 绘制圆心
        plt.plot(center_x, center_y, 'r*', markersize=15, label='圆心')
        
        # 设置图形
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.xlabel('X (m)', fontsize=12)
        plt.ylabel('Y (m)', fontsize=12)
        plt.title('机器人原地旋转轨迹', fontsize=14, fontweight='bold')
        plt.legend(fontsize=10)
        
        # 保存图片
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f'trajectory_{timestamp}.png'
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        self.get_logger().info(f'轨迹图已保存: {filename}')
        
        plt.show()


def main():
    rclpy.init()
    node = RotationRecorder()
    
    # 在单独的线程中运行ROS节点
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # 等待接收到初始数据
    print("\n等待IMU和位置数据...")
    time.sleep(2)
    
    try:
        while rclpy.ok():
            print("\n" + "="*50)
            print("机器人旋转控制")
            print("="*50)
            
            try:
                num_rotations = float(input("请输入旋转圈数 (输入0退出): "))
                
                if num_rotations == 0:
                    print("退出程序...")
                    break
                
                if num_rotations < 0:
                    print("圈数必须为正数！")
                    continue
                
                angular_speed = float(input("请输入角速度 (rad/s, 默认0.5): ") or "0.5")
                
                if angular_speed <= 0:
                    print("角速度必须为正数！")
                    continue
                
                # 开始旋转
                node.start_rotation(num_rotations, angular_speed)
                
            except ValueError:
                print("输入无效，请输入数字！")
            except KeyboardInterrupt:
                print("\n检测到中断...")
                break
    
    finally:
        # 确保停止旋转
        if node.is_rotating:
            node.stop_rotation()
        
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()