#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile
import math


class RobotTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('robot_trajectory_publisher')
        
        # 创建Publisher，使用QoS深度为10
        qos_profile = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(
            PoseStamped,
            '/robot_pose',
            qos_profile
        )
        
        # 设置发布频率（20Hz）
        self.timer_period = 0.1  # 秒
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # IMU 发布器（100Hz）
        self.imu_publisher_ = self.create_publisher(Imu, '/imu', qos_profile)
        self.imu_timer = self.create_timer(0.01, self.imu_timer_callback)

        # 轨迹参数
        self.t = 0.0  # 时间参数
        self.radius = 6.0  # 圆形轨迹半径
        self.angular_velocity = 0.5  # 角速度

        self.get_logger().info('机器人轨迹发布节点已启动')
        self.get_logger().info('发布话题: /robot_pose, /imu')
    
    def timer_callback(self):
        """定时器回调函数，发布模拟轨迹"""
        msg = PoseStamped()
        
        # 设置时间戳
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # 生成圆形轨迹
        # x = r * cos(ωt)
        # y = r * sin(ωt)
        msg.pose.position.x = self.radius * math.cos(self.angular_velocity * self.t)
        msg.pose.position.y = self.radius * math.sin(self.angular_velocity * self.t)
        msg.pose.position.z = 0.0
        
        # 设置方向（朝向运动方向）
        # 计算切线方向的偏航角
        yaw = self.angular_velocity * self.t + math.pi / 2
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.orientation.w = math.cos(yaw / 2)
        
        # 发布消息
        self.publisher_.publish(msg)
        
        # 打印信息（每2秒打印一次）
        if int(self.t * 10) % 20 == 0:
            self.get_logger().info(
                f'发布位置: x={msg.pose.position.x:.2f}, '
                f'y={msg.pose.position.y:.2f}, '
                f'z={msg.pose.position.z:.2f}'
            )
        
        # 更新时间参数
        self.t += self.timer_period

    def imu_timer_callback(self):
        """IMU 定时器回调，100Hz 发布固定 IMU 数据"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # 固定线加速度（静止状态，仅重力分量）
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        # 固定角速度（静止）
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0

        # 固定姿态四元数（水平无偏转）
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        # 协方差设为已知（对角线填小值）
        msg.orientation_covariance[0] = 1e-6
        msg.orientation_covariance[4] = 1e-6
        msg.orientation_covariance[8] = 1e-6
        msg.angular_velocity_covariance[0] = 1e-6
        msg.angular_velocity_covariance[4] = 1e-6
        msg.angular_velocity_covariance[8] = 1e-6
        msg.linear_acceleration_covariance[0] = 1e-6
        msg.linear_acceleration_covariance[4] = 1e-6
        msg.linear_acceleration_covariance[8] = 1e-6

        self.imu_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    trajectory_publisher = RobotTrajectoryPublisher()
    
    try:
        rclpy.spin(trajectory_publisher)
    except KeyboardInterrupt:
        trajectory_publisher.get_logger().info('节点被用户中断')
    finally:
        trajectory_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()