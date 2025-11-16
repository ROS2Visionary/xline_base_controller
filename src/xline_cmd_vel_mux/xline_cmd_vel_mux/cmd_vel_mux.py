#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XLine命令速度复用节点（带智能障碍物避障）
订阅来自执行任务和平板操控的Twist消息，并根据障碍物检测结果智能转发到cmd_vel话题

功能特性：
1. 根据机器人运动状态智能判断需要关注的障碍物方向
2. 支持可配置的安全距离参数（仅用于原地旋转）
3. 当检测到障碍物时自动停止机器人
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math


class CmdVelMux(Node):
    """
    命令速度复用节点（带智能障碍物避障）
    订阅来自执行任务和平板操控的Twist消息，并根据障碍物检测智能转发
    """
    
    # 障碍物方向索引
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # 声明参数
        self.declare_parameter('obstacle_safe_distance', 0.1)  # 安全距离（米），仅用于原地旋转
        self.declare_parameter('enable_obstacle_detection', True)  # 是否启用障碍物检测
        self.declare_parameter('linear_velocity_threshold', 0.01)  # 线速度零值判断阈值（米/秒）
        self.declare_parameter('angular_velocity_threshold', 0.01)  # 角速度零值判断阈值（弧度/秒）
        
        # 获取参数值
        self.safe_distance = self.get_parameter('obstacle_safe_distance').value
        self.enable_obstacle_detection = self.get_parameter('enable_obstacle_detection').value
        self.linear_threshold = self.get_parameter('linear_velocity_threshold').value
        self.angular_threshold = self.get_parameter('angular_velocity_threshold').value
        
        # 障碍物距离数组 [前方, 后方, 左侧, 右侧]
        # 0.0 表示无障碍物或距离无穷大
        self.obstacle_distances = [0.0, 0.0, 0.0, 0.0]
        
        # 当前机器人的运动状态（最后实际发布的速度）
        self.current_cmd_vel = Twist()
        
        # 上一次的运动状态（用于检测状态变化）
        self.last_motion_state = 'stopped'
        
        # 是否已经因为障碍物停止
        self.is_stopped_by_obstacle = False
        
        # 创建发布者 - 发布到 cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # 订阅障碍物检测话题
        self.obstacle_subscription = self.create_subscription(
            Float32MultiArray,
            '/obstacle_detected',
            self.obstacle_callback,
            10
        )
        
        # 订阅执行任务的控制指令
        self.task_subscription = self.create_subscription(
            Twist,
            'task_cmd_vel',
            self.task_callback,
            10
        )
        
        # 订阅平板操控的指令
        self.tablet_subscription = self.create_subscription(
            Twist,
            'tablet_cmd_vel',
            self.tablet_callback,
            10
        )
        
        # 打印启动信息
        self.get_logger().info('=' * 60)
        self.get_logger().info('CmdVelMux节点已启动（智能障碍物避障）')
        self.get_logger().info('=' * 60)
        self.get_logger().info('订阅话题:')
        self.get_logger().info('  - task_cmd_vel: 执行任务控制指令')
        self.get_logger().info('  - tablet_cmd_vel: 平板操控指令')
        self.get_logger().info('  - /obstacle_detected: 障碍物检测数据')
        self.get_logger().info('发布话题:')
        self.get_logger().info('  - cmd_vel: 最终控制指令')
        self.get_logger().info('-' * 60)
        self.get_logger().info('障碍物检测参数:')
        self.get_logger().info(f'  - 启用状态: {self.enable_obstacle_detection}')
        self.get_logger().info(f'  - 安全距离(原地旋转): {self.safe_distance:.2f}m')
        self.get_logger().info('  - 检测规则:')
        self.get_logger().info('    * 直线前进: 只检查前方障碍物')
        self.get_logger().info('    * 直线后退: 只检查后方障碍物')
        self.get_logger().info('    * 转弯运动: 检查运动方向的障碍物')
        self.get_logger().info('    * 原地旋转: 检查四周，障碍物距离<安全距离时停止')
        self.get_logger().info('-' * 60)
        self.get_logger().info('速度判断阈值:')
        self.get_logger().info(f'  - 线速度零值阈值: {self.linear_threshold:.3f}m/s')
        self.get_logger().info(f'  - 角速度零值阈值: {self.angular_threshold:.3f}rad/s')
        self.get_logger().info(f'  - 说明: 速度绝对值小于阈值时判定为0')
        self.get_logger().info('=' * 60)
    
    def obstacle_callback(self, msg):
        """
        障碍物检测话题回调函数
        重要：每次收到障碍物数据时，都检查当前运动状态是否安全
        """
        if len(msg.data) >= 4:
            self.obstacle_distances = list(msg.data[:4])
            
            # 如果未启用障碍物检测，直接返回
            if not self.enable_obstacle_detection:
                return
            
            # 检查当前运动状态是否安全
            # 即使没有新的速度指令，也要持续保护机器人安全
            self.check_and_protect()
    
    def check_and_protect(self):
        """
        检查当前运动状态，如果检测到危险则主动停止机器人
        这个函数在两种情况下被调用：
        1. 收到新的障碍物数据时（obstacle_callback）
        2. 收到新的速度指令时（process_and_forward）
        """
        # 如果机器人已经停止，不需要重复检查
        current_motion_state = self.get_motion_state(self.current_cmd_vel)
        if current_motion_state == 'stopped':
            self.is_stopped_by_obstacle = False
            return
        
        # 检查当前运动状态下是否有危险
        should_stop, stop_reason = self.check_obstacle(self.current_cmd_vel, current_motion_state)
        
        if should_stop:
            # 检测到危险，主动发布零速度
            if not self.is_stopped_by_obstacle:
                # 只在第一次检测到危险时发布和打印日志
                stop_msg = Twist()
                self.cmd_vel_publisher.publish(stop_msg)
                
                # 更新当前状态
                self.current_cmd_vel = stop_msg
                self.is_stopped_by_obstacle = True
                
                self.get_logger().warn(
                    f'动态障碍物检测! 已紧急停止! '
                    f'状态:{current_motion_state}, 原因:{stop_reason}'
                )
        else:
            # 安全状态，重置标志
            if self.is_stopped_by_obstacle:
                self.get_logger().info('障碍物已清除，可以继续运动')
                self.is_stopped_by_obstacle = False
    
    def task_callback(self, msg):
        """执行任务话题回调函数"""
        self.get_logger().debug('收到执行任务指令')
        self.process_and_forward(msg, source='task')
    
    def tablet_callback(self, msg):
        """平板操控话题回调函数"""
        self.get_logger().debug('收到平板操控指令')
        self.process_and_forward(msg, source='tablet')
    
    def process_and_forward(self, cmd_vel_msg, source='unknown'):
        """
        处理速度指令并根据障碍物检测结果转发
        
        Args:
            cmd_vel_msg: 原始Twist消息
            source: 指令来源（'task'或'tablet'）
        """
        
        # 如果未启用障碍物检测，直接转发并更新状态
        if not self.enable_obstacle_detection:
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            self.current_cmd_vel = cmd_vel_msg
            return
        
        # 判断新指令的运动状态
        motion_state = self.get_motion_state(cmd_vel_msg)
        
        # 检查新指令是否会遇到障碍物
        should_stop, stop_reason = self.check_obstacle(cmd_vel_msg, motion_state)
        
        if should_stop:
            # 新指令有危险，发布零速度
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            
            # 更新当前状态为停止
            self.current_cmd_vel = stop_msg
            self.is_stopped_by_obstacle = True
            
            # 打印警告信息
            self.get_logger().warn(
                f'[{source}] 指令被阻止! '
                f'目标状态:{motion_state}, 原因:{stop_reason}'
            )
        else:
            # 新指令安全，正常转发
            self.cmd_vel_publisher.publish(cmd_vel_msg)
            
            # 更新当前状态为新指令
            self.current_cmd_vel = cmd_vel_msg
            self.is_stopped_by_obstacle = False
            
            # 打印调试信息（仅在状态变化时）
            if motion_state != self.last_motion_state:
                self.get_logger().debug(
                    f'[{source}] 运动状态变化: {self.last_motion_state} → {motion_state}'
                )
                self.last_motion_state = motion_state
    
    def get_motion_state(self, cmd_vel):
        """
        根据Twist消息判断机器人运动状态
        
        Args:
            cmd_vel: Twist消息
            
        Returns:
            str: 运动状态描述
        """
        linear_x = cmd_vel.linear.x
        angular_z = cmd_vel.angular.z
        
        # 判断线速度方向（使用阈值判断是否为0）
        if abs(linear_x) < self.linear_threshold:
            linear_state = 'stop'
        elif linear_x > 0:
            linear_state = 'forward'
        else:
            linear_state = 'backward'
        
        # 判断角速度方向（使用阈值判断是否为0）
        if abs(angular_z) < self.angular_threshold:
            angular_state = 'straight'
        elif angular_z > 0:
            angular_state = 'turn_left'
        else:
            angular_state = 'turn_right'
        
        # 组合状态
        if linear_state == 'stop' and angular_state == 'straight':
            return 'stopped'
        elif linear_state == 'stop' and angular_state == 'turn_left':
            return 'rotate_left'
        elif linear_state == 'stop' and angular_state == 'turn_right':
            return 'rotate_right'
        elif linear_state == 'forward' and angular_state == 'straight':
            return 'forward'
        elif linear_state == 'forward' and angular_state == 'turn_left':
            return 'forward_left'
        elif linear_state == 'forward' and angular_state == 'turn_right':
            return 'forward_right'
        elif linear_state == 'backward' and angular_state == 'straight':
            return 'backward'
        elif linear_state == 'backward' and angular_state == 'turn_left':
            return 'backward_left'
        elif linear_state == 'backward' and angular_state == 'turn_right':
            return 'backward_right'
        else:
            return 'unknown'
    
    def check_obstacle(self, cmd_vel, motion_state):
        """
        根据运动状态检查障碍物
        
        Args:
            cmd_vel: Twist消息
            motion_state: 运动状态
            
        Returns:
            tuple: (是否应该停止, 停止原因)
        """
        front_dist = self.obstacle_distances[self.FRONT]
        back_dist = self.obstacle_distances[self.BACK]
        left_dist = self.obstacle_distances[self.LEFT]
        right_dist = self.obstacle_distances[self.RIGHT]
        
        # 辅助函数：检查距离是否触发停止
        def is_dangerous(distance, use_safe_distance=False):
            """
            检查距离是否危险
            
            Args:
                distance: 障碍物距离（0表示无障碍物）
                use_safe_distance: 是否使用安全距离判断（True=原地旋转模式，False=普通运动模式）
            
            Returns:
                bool: 是否危险
                
            逻辑说明：
                - distance <= 0: 无障碍物，返回False
                - use_safe_distance=False（普通运动）: 只要有障碍物(distance>0)就返回True
                - use_safe_distance=True（原地旋转）: 只有当0<distance<safe_distance时返回True
            """
            if distance <= 0.0:  # 0.0表示无障碍物
                return False
            
            if use_safe_distance:
                # 原地旋转模式：只有当障碍物距离小于安全距离时才停止
                return distance < self.safe_distance
            else:
                # 普通运动模式：只要检测到障碍物（距离>0）就停止
                return True
        
        # 根据运动状态判断需要检查的方向
        if motion_state == 'stopped':
            # 静止状态，不需要检查
            return False, ''
        
        elif motion_state == 'forward':
            # 前进直线：只检查前方（有障碍物即停），忽略左右
            if is_dangerous(front_dist):
                return True, f'前方检测到障碍物{front_dist:.2f}m'
        
        elif motion_state == 'backward':
            # 后退直线：只检查后方（有障碍物即停），忽略左右
            if is_dangerous(back_dist):
                return True, f'后方检测到障碍物{back_dist:.2f}m'
        
        elif motion_state == 'forward_left':
            # 前进左转：检查前方、左侧（有障碍物即停）
            if is_dangerous(front_dist):
                return True, f'前方检测到障碍物{front_dist:.2f}m'
            if is_dangerous(left_dist):
                return True, f'左侧检测到障碍物{left_dist:.2f}m'
        
        elif motion_state == 'forward_right':
            # 前进右转：检查前方、右侧（有障碍物即停）
            if is_dangerous(front_dist):
                return True, f'前方检测到障碍物{front_dist:.2f}m'
            if is_dangerous(right_dist):
                return True, f'右侧检测到障碍物{right_dist:.2f}m'
        
        elif motion_state == 'backward_left':
            # 后退左转：检查后方、左侧（有障碍物即停）
            if is_dangerous(back_dist):
                return True, f'后方检测到障碍物{back_dist:.2f}m'
            if is_dangerous(left_dist):
                return True, f'左侧检测到障碍物{left_dist:.2f}m'
        
        elif motion_state == 'backward_right':
            # 后退右转：检查后方、右侧（有障碍物即停）
            if is_dangerous(back_dist):
                return True, f'后方检测到障碍物{back_dist:.2f}m'
            if is_dangerous(right_dist):
                return True, f'右侧检测到障碍物{right_dist:.2f}m'
        
        elif motion_state == 'rotate_left':
            # 原地左转：检查所有方向，使用安全距离判断
            # 原地旋转时，机器人会以自身中心为轴旋转
            # 只有当障碍物距离小于安全距离时才停止
            if is_dangerous(front_dist, use_safe_distance=True):
                return True, f'前方安全距离不足{front_dist:.2f}m'
            if is_dangerous(back_dist, use_safe_distance=True):
                return True, f'后方安全距离不足{back_dist:.2f}m'
            if is_dangerous(left_dist, use_safe_distance=True):
                return True, f'左侧安全距离不足{left_dist:.2f}m'
            if is_dangerous(right_dist, use_safe_distance=True):
                return True, f'右侧安全距离不足{right_dist:.2f}m'
        
        elif motion_state == 'rotate_right':
            # 原地右转：检查所有方向，使用安全距离判断
            # 原地旋转时，机器人会以自身中心为轴旋转
            # 只有当障碍物距离小于安全距离时才停止
            if is_dangerous(front_dist, use_safe_distance=True):
                return True, f'前方安全距离不足{front_dist:.2f}m'
            if is_dangerous(back_dist, use_safe_distance=True):
                return True, f'后方安全距离不足{back_dist:.2f}m'
            if is_dangerous(left_dist, use_safe_distance=True):
                return True, f'左侧安全距离不足{left_dist:.2f}m'
            if is_dangerous(right_dist, use_safe_distance=True):
                return True, f'右侧安全距离不足{right_dist:.2f}m'
        
        # 未触发任何停止条件
        return False, ''


def main(args=None):
    rclpy.init(args=args)
    
    cmd_vel_mux = CmdVelMux()
    
    try:
        rclpy.spin(cmd_vel_mux)
    except KeyboardInterrupt:
        cmd_vel_mux.get_logger().info('收到中断信号，正在关闭...')
    finally:
        cmd_vel_mux.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()