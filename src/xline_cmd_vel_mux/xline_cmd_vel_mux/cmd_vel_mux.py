#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XLine命令速度复用节点（带智能障碍物动态减速 - 四方向停止距离版 + 加速度限制）
订阅来自执行任务和平板操控的Twist消息，并根据障碍物检测结果智能减速转发到cmd_vel话题

功能特性：
1. 根据机器人运动状态智能判断需要关注的障碍物方向
2. 支持前后左右四个方向独立配置停止距离（分别针对平板操控和执行任务）
3. 根据障碍物距离动态计算减速比例，实现平滑减速
4. 在各方向设定的停止距离处完全停止机器人
5. 支持线速度和角速度的加速度限制，防止速度突变
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import time


class CmdVelMux(Node):
    """
    命令速度复用节点（带智能障碍物动态减速 - 四方向版 + 加速度限制）
    订阅来自执行任务和平板操控的Twist消息，并根据障碍物检测智能减速转发
    """
    
    # 障碍物方向索引
    FRONT = 0
    BACK = 1
    LEFT = 2
    RIGHT = 3
    
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # 声明参数 - 障碍物检测相关
        self.declare_parameter('enable_obstacle_detection',False)  # 是否启用障碍物检测
        self.declare_parameter('obstacle_detection_range', 0.7)  # 障碍物检测范围（米）
        self.declare_parameter('linear_velocity_threshold', 0.001)  # 线速度零值判断阈值（米/秒）
        self.declare_parameter('angular_velocity_threshold', 0.001)  # 角速度零值判断阈值（弧度/秒）
        
        # 声明参数 - 平板操控相关（四方向停止距离）
        self.declare_parameter('tablet_max_linear_speed', 1.0)  # 平板最大线速度（米/秒）
        self.declare_parameter('tablet_max_angular_speed', 1.5)  # 平板最大角速度（弧度/秒）
        self.declare_parameter('tablet_stop_distance_front', 0.2)   # 前方停止距离（米）
        self.declare_parameter('tablet_stop_distance_back', 0.2)    # 后方停止距离（米）
        self.declare_parameter('tablet_stop_distance_left', 0.1)   # 左侧停止距离（米）
        self.declare_parameter('tablet_stop_distance_right', 0.1)  # 右侧停止距离（米）
        self.declare_parameter('tablet_rotate_stop_distance', 0.1) # 原地旋转停止距离（米）
        
        # 声明参数 - 平板操控加速度限制
        self.declare_parameter('tablet_max_linear_acceleration', 1.0)   # 最大线加速度（米/秒²）
        self.declare_parameter('tablet_max_angular_acceleration', 1.5)  # 最大角加速度（弧度/秒²）
        
        # 声明参数 - 执行任务相关（四方向停止距离）
        self.declare_parameter('task_max_linear_speed', 0.4)  # 任务最大线速度（米/秒）
        self.declare_parameter('task_max_angular_speed', 1.5)  # 任务最大角速度（弧度/秒）
        self.declare_parameter('task_stop_distance_front', 0.05)    # 前方停止距离（米）
        self.declare_parameter('task_stop_distance_back', 0.05)     # 后方停止距离（米）
        self.declare_parameter('task_stop_distance_left', 0.05)    # 左侧停止距离（米）
        self.declare_parameter('task_stop_distance_right', 0.05)   # 右侧停止距离（米）
        self.declare_parameter('task_rotate_stop_distance', 0.05)   # 原地旋转停止距离（米）
        
        # 声明参数 - 执行任务加速度限制
        self.declare_parameter('task_max_linear_acceleration', 0.5)   # 最大线加速度（米/秒²）
        self.declare_parameter('task_max_angular_acceleration', 2.0)  # 最大角加速度（弧度/秒²）
        
        # 获取参数值 - 通用参数
        self.enable_obstacle_detection = self.get_parameter('enable_obstacle_detection').value
        self.detection_range = self.get_parameter('obstacle_detection_range').value
        self.linear_threshold = self.get_parameter('linear_velocity_threshold').value
        self.angular_threshold = self.get_parameter('angular_velocity_threshold').value
        
        # 获取参数值 - 平板操控参数（四方向）
        self.tablet_max_linear = self.get_parameter('tablet_max_linear_speed').value
        self.tablet_max_angular = self.get_parameter('tablet_max_angular_speed').value
        self.tablet_stop_dist = {
            'front': self.get_parameter('tablet_stop_distance_front').value,
            'back': self.get_parameter('tablet_stop_distance_back').value,
            'left': self.get_parameter('tablet_stop_distance_left').value,
            'right': self.get_parameter('tablet_stop_distance_right').value,
            'rotate': self.get_parameter('tablet_rotate_stop_distance').value
        }
        self.tablet_max_linear_accel = self.get_parameter('tablet_max_linear_acceleration').value
        self.tablet_max_angular_accel = self.get_parameter('tablet_max_angular_acceleration').value
        
        # 获取参数值 - 执行任务参数（四方向）
        self.task_max_linear = self.get_parameter('task_max_linear_speed').value
        self.task_max_angular = self.get_parameter('task_max_angular_speed').value
        self.task_stop_dist = {
            'front': self.get_parameter('task_stop_distance_front').value,
            'back': self.get_parameter('task_stop_distance_back').value,
            'left': self.get_parameter('task_stop_distance_left').value,
            'right': self.get_parameter('task_stop_distance_right').value,
            'rotate': self.get_parameter('task_rotate_stop_distance').value
        }
        self.task_max_linear_accel = self.get_parameter('task_max_linear_acceleration').value
        self.task_max_angular_accel = self.get_parameter('task_max_angular_acceleration').value
        
        # 障碍物距离数组 [前方, 后方, 左侧, 右侧]
        # 0.0 表示无障碍物或距离无穷大
        self.obstacle_distances = [0.0, 0.0, 0.0, 0.0]
        
        # 当前机器人的运动状态（最后实际发布的速度）
        self.current_cmd_vel = Twist()
        self.current_source = None  # 当前指令来源
        
        # 上一次实际发布的速度（用于加速度限制）
        self.last_published_cmd = Twist()
        
        # 上一次发布速度的时间戳
        self.last_publish_time = time.time()
        
        # 上一次的运动状态（用于检测状态变化）
        self.last_motion_state = 'stopped'
        
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
        self.print_startup_info()
    
    def print_startup_info(self):
        """打印启动信息"""
        self.get_logger().info('=' * 80)
        self.get_logger().info('CmdVelMux节点已启动（智能障碍物动态减速 - 四方向 + 加速度限制）')
        self.get_logger().info('=' * 80)
        self.get_logger().info('订阅话题:')
        self.get_logger().info('  - task_cmd_vel: 执行任务控制指令')
        self.get_logger().info('  - tablet_cmd_vel: 平板操控指令')
        self.get_logger().info('  - /obstacle_detected: 障碍物检测数据')
        self.get_logger().info('发布话题:')
        self.get_logger().info('  - cmd_vel: 最终控制指令')
        self.get_logger().info('-' * 80)
        self.get_logger().info('障碍物检测参数:')
        self.get_logger().info(f'  - 启用状态: {self.enable_obstacle_detection}')
        self.get_logger().info(f'  - 检测范围: {self.detection_range:.2f}m')
        self.get_logger().info('-' * 80)
        self.get_logger().info('平板操控参数:')
        self.get_logger().info(f'  - 最大线速度: {self.tablet_max_linear:.2f}m/s')
        self.get_logger().info(f'  - 最大角速度: {self.tablet_max_angular:.2f}rad/s')
        self.get_logger().info(f'  - 最大线加速度: {self.tablet_max_linear_accel:.2f}m/s²')
        self.get_logger().info(f'  - 最大角加速度: {self.tablet_max_angular_accel:.2f}rad/s²')
        self.get_logger().info(f'  - 停止距离 [前方]: {self.tablet_stop_dist["front"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [后方]: {self.tablet_stop_dist["back"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [左侧]: {self.tablet_stop_dist["left"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [右侧]: {self.tablet_stop_dist["right"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [旋转]: {self.tablet_stop_dist["rotate"]:.2f}m (四周)')
        self.get_logger().info('-' * 80)
        self.get_logger().info('执行任务参数:')
        self.get_logger().info(f'  - 最大线速度: {self.task_max_linear:.2f}m/s')
        self.get_logger().info(f'  - 最大角速度: {self.task_max_angular:.2f}rad/s')
        self.get_logger().info(f'  - 最大线加速度: {self.task_max_linear_accel:.2f}m/s²')
        self.get_logger().info(f'  - 最大角加速度: {self.task_max_angular_accel:.2f}rad/s²')
        self.get_logger().info(f'  - 停止距离 [前方]: {self.task_stop_dist["front"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [后方]: {self.task_stop_dist["back"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [左侧]: {self.task_stop_dist["left"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [右侧]: {self.task_stop_dist["right"]:.2f}m')
        self.get_logger().info(f'  - 停止距离 [旋转]: {self.task_stop_dist["rotate"]:.2f}m (四周)')
        self.get_logger().info('-' * 80)
        self.get_logger().info('动态减速策略:')
        self.get_logger().info('  - 障碍物距离 > 检测范围: 不减速 (100%速度)')
        self.get_logger().info('  - 检测范围 >= 距离 > 停止距离: 线性减速')
        self.get_logger().info('  - 距离 <= 停止距离: 完全停止 (0%速度)')
        self.get_logger().info('  - 减速公式: 速度比例 = (距离-停止距离)/(检测范围-停止距离)')
        self.get_logger().info('  - 各方向独立计算，取最严格限制')
        self.get_logger().info('-' * 80)
        self.get_logger().info('加速度限制策略:')
        self.get_logger().info('  - 防止速度突变，确保平滑加速/减速')
        self.get_logger().info('  - 线速度变化率 <= 最大线加速度 × 时间间隔')
        self.get_logger().info('  - 角速度变化率 <= 最大角加速度 × 时间间隔')
        self.get_logger().info('  - 障碍物消失后平滑恢复速度，而非瞬间加速')
        self.get_logger().info('-' * 80)
        self.get_logger().info('智能检测规则:')
        self.get_logger().info('  - 直线前进: 检查前方 (使用前方停止距离)')
        self.get_logger().info('  - 直线后退: 检查后方 (使用后方停止距离)')
        self.get_logger().info('  - 前进左转: 检查前方+左侧 (使用各自停止距离)')
        self.get_logger().info('  - 前进右转: 检查前方+右侧 (使用各自停止距离)')
        self.get_logger().info('  - 后退左转: 检查后方+左侧 (使用各自停止距离)')
        self.get_logger().info('  - 后退右转: 检查后方+右侧 (使用各自停止距离)')
        self.get_logger().info('  - 原地旋转: 检查四周 (使用旋转停止距离)')
        self.get_logger().info('=' * 80)
    
    def obstacle_callback(self, msg):
        """
        障碍物检测话题回调函数
        更新障碍物距离数据，并重新评估当前运动状态
        """
        if len(msg.data) >= 4:
            self.obstacle_distances = list(msg.data[:4])
            
            # 如果未启用障碍物检测，直接返回
            if not self.enable_obstacle_detection:
                return
            
            # 如果当前有运动指令，重新评估并调整
            if self.current_source is not None:
                self.recheck_current_motion()
    
    def recheck_current_motion(self):
        """
        重新检查当前运动状态，根据最新的障碍物数据调整速度
        """
        if self.current_source is None:
            return
        
        # 根据当前来源获取停止距离字典和加速度限制
        if self.current_source == 'task':
            stop_distances = self.task_stop_dist
            max_linear_accel = self.task_max_linear_accel
            max_angular_accel = self.task_max_angular_accel
        else:  # tablet
            stop_distances = self.tablet_stop_dist
            max_linear_accel = self.tablet_max_linear_accel
            max_angular_accel = self.tablet_max_angular_accel
        
        # 判断运动状态
        motion_state = self.get_motion_state(self.current_cmd_vel)
        
        # 计算减速比例
        scale = self.calculate_scale_factor(
            self.current_cmd_vel,
            motion_state,
            stop_distances
        )
        
        # 应用减速比例
        scaled_cmd = self.apply_scale(self.current_cmd_vel, scale)
        
        # 应用加速度限制
        final_cmd = self.apply_acceleration_limit(
            scaled_cmd,
            max_linear_accel,
            max_angular_accel
        )
        
        # 发布调整后的速度
        self.cmd_vel_publisher.publish(final_cmd)
        
        # 如果减速比例变化显著，打印日志
        if abs(scale - 1.0) > 0.1:  # 减速超过10%
            self.get_logger().debug(
                f'[动态调整] 来源:{self.current_source}, '
                f'状态:{motion_state}, 减速比例:{scale:.2f}'
            )
    
    def task_callback(self, msg):
        """执行任务话题回调函数"""
        self.process_and_forward_raw(msg, source='task')
    
    def tablet_callback(self, msg):
        """平板操控话题回调函数"""
        self.process_and_forward_raw(msg, source='tablet')

    # 发布调整后的速度
    def process_and_forward_raw(self, cmd_vel_msg, source='unknown'):
        self.cmd_vel_publisher.publish(cmd_vel_msg)
    
    def process_and_forward(self, cmd_vel_msg, source='unknown'):
        """
        处理速度指令并根据障碍物检测结果动态减速转发
        
        Args:
            cmd_vel_msg: 原始Twist消息
            source: 指令来源（'task'或'tablet'）
        """
        
        # 更新当前来源和原始指令
        self.current_source = source
        self.current_cmd_vel = cmd_vel_msg
        
        # 如果未启用障碍物检测，直接应用加速度限制后转发
        if not self.enable_obstacle_detection:
            # 仍然需要应用加速度限制
            if source == 'task':
                max_linear_accel = self.task_max_linear_accel
                max_angular_accel = self.task_max_angular_accel
            else:
                max_linear_accel = self.tablet_max_linear_accel
                max_angular_accel = self.tablet_max_angular_accel
            
            final_cmd = self.apply_acceleration_limit(
                cmd_vel_msg,
                max_linear_accel,
                max_angular_accel
            )
            self.cmd_vel_publisher.publish(final_cmd)
            return
        
        # 根据来源获取对应的停止距离字典和加速度限制
        if source == 'task':
            stop_distances = self.task_stop_dist
            max_linear_accel = self.task_max_linear_accel
            max_angular_accel = self.task_max_angular_accel
        else:  # tablet
            stop_distances = self.tablet_stop_dist
            max_linear_accel = self.tablet_max_linear_accel
            max_angular_accel = self.tablet_max_angular_accel
        
        # 判断运动状态
        motion_state = self.get_motion_state(cmd_vel_msg)
        
        # 计算减速比例
        scale = self.calculate_scale_factor(
            cmd_vel_msg,
            motion_state,
            stop_distances
        )
        
        # 应用减速比例
        scaled_cmd = self.apply_scale(cmd_vel_msg, scale)
        
        # 应用加速度限制
        final_cmd = self.apply_acceleration_limit(
            scaled_cmd,
            max_linear_accel,
            max_angular_accel
        )
        
        # 发布调整后的速度
        self.cmd_vel_publisher.publish(final_cmd)
        
        # 打印调试信息（状态变化或显著减速时）
        if motion_state != self.last_motion_state or abs(scale - 1.0) > 0.1:
            if scale < 0.01:  # 完全停止
                self.get_logger().warn(
                    f'[{source}] 障碍物过近，已停止! 状态:{motion_state}'
                )
            elif scale < 1.0:  # 减速中
                self.get_logger().info(
                    f'[{source}] 动态减速中，状态:{motion_state}, '
                    f'速度比例:{scale:.2%}'
                )
            else:  # 正常通过
                if motion_state != self.last_motion_state:
                    self.get_logger().debug(
                        f'[{source}] 状态变化: {self.last_motion_state} → {motion_state}'
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
    
    def calculate_scale_factor(self, cmd_vel, motion_state, stop_distances):
        """
        根据运动状态和障碍物距离计算速度缩放因子（支持四方向独立停止距离）
        
        Args:
            cmd_vel: Twist消息
            motion_state: 运动状态
            stop_distances: 停止距离字典，包含 'front', 'back', 'left', 'right', 'rotate' 键
            
        Returns:
            float: 速度缩放因子 [0.0, 1.0]
                  1.0 = 不减速
                  0.0 = 完全停止
        """
        front_dist = self.obstacle_distances[self.FRONT]
        back_dist = self.obstacle_distances[self.BACK]
        left_dist = self.obstacle_distances[self.LEFT]
        right_dist = self.obstacle_distances[self.RIGHT]
        
        # 辅助函数：计算单个方向的减速比例
        def calc_direction_scale(distance, target_stop_distance):
            """
            计算单个方向的减速比例
            
            Args:
                distance: 障碍物距离（0表示无障碍物）
                target_stop_distance: 目标停止距离
            
            Returns:
                float: 减速比例 [0.0, 1.0]
            
            逻辑：
                - distance <= 0: 无障碍物，返回1.0（不减速）
                - distance <= target_stop_distance: 返回0.0（完全停止）
                - target_stop_distance < distance <= detection_range: 线性减速
                - distance > detection_range: 返回1.0（不减速）
            """
            if distance <= 0.0:  # 无障碍物
                return 1.0
            
            if distance <= target_stop_distance:  # 太近，完全停止
                return 0.0
            
            if distance >= self.detection_range:  # 超出检测范围，不减速
                return 1.0
            
            # 在减速区间内，线性计算
            # scale = (distance - stop_distance) / (detection_range - stop_distance)
            scale = (distance - target_stop_distance) / (self.detection_range - target_stop_distance)
            return max(0.0, min(1.0, scale))  # 确保在[0, 1]范围内
        
        # 初始化缩放因子为1.0（不减速）
        scale = 1.0
        
        # 根据运动状态选择需要检查的方向（使用各方向独立的停止距离）
        if motion_state == 'stopped':
            # 静止状态，不需要减速
            return 1.0
        
        elif motion_state == 'forward':
            # 前进直线：只检查前方，使用前方停止距离
            scale = min(scale, calc_direction_scale(front_dist, stop_distances['front']))
        
        elif motion_state == 'backward':
            # 后退直线：只检查后方，使用后方停止距离
            scale = min(scale, calc_direction_scale(back_dist, stop_distances['back']))
        
        elif motion_state == 'forward_left':
            # 前进左转：检查前方、左侧，使用各自的停止距离
            scale = min(scale, calc_direction_scale(front_dist, stop_distances['front']))
            scale = min(scale, calc_direction_scale(left_dist, stop_distances['left']))
        
        elif motion_state == 'forward_right':
            # 前进右转：检查前方、右侧，使用各自的停止距离
            scale = min(scale, calc_direction_scale(front_dist, stop_distances['front']))
            scale = min(scale, calc_direction_scale(right_dist, stop_distances['right']))
        
        elif motion_state == 'backward_left':
            # 后退左转：检查后方、左侧，使用各自的停止距离
            scale = min(scale, calc_direction_scale(back_dist, stop_distances['back']))
            scale = min(scale, calc_direction_scale(left_dist, stop_distances['left']))
        
        elif motion_state == 'backward_right':
            # 后退右转：检查后方、右侧，使用各自的停止距离
            scale = min(scale, calc_direction_scale(back_dist, stop_distances['back']))
            scale = min(scale, calc_direction_scale(right_dist, stop_distances['right']))
        
        elif motion_state == 'rotate_left' or motion_state == 'rotate_right':
            # 原地旋转：检查所有方向，使用旋转停止距离（四周统一）
            rotate_stop = stop_distances['rotate']
            scale = min(scale, calc_direction_scale(front_dist, rotate_stop))
            scale = min(scale, calc_direction_scale(back_dist, rotate_stop))
            scale = min(scale, calc_direction_scale(left_dist, rotate_stop))
            scale = min(scale, calc_direction_scale(right_dist, rotate_stop))
        
        return scale
    
    def apply_scale(self, cmd_vel, scale):
        """
        应用缩放因子到速度指令
        
        Args:
            cmd_vel: 原始Twist消息
            scale: 缩放因子 [0.0, 1.0]
            
        Returns:
            Twist: 缩放后的Twist消息
        """
        scaled_cmd = Twist()
        scaled_cmd.linear.x = cmd_vel.linear.x * scale
        scaled_cmd.linear.y = cmd_vel.linear.y * scale
        scaled_cmd.linear.z = cmd_vel.linear.z * scale
        scaled_cmd.angular.x = cmd_vel.angular.x * scale
        scaled_cmd.angular.y = cmd_vel.angular.y * scale
        scaled_cmd.angular.z = cmd_vel.angular.z * scale
        return scaled_cmd
    
    def apply_acceleration_limit(self, target_cmd, max_linear_accel, max_angular_accel):
        """
        应用加速度限制，防止速度突变
        
        Args:
            target_cmd: 目标Twist消息（经过障碍物减速计算后的期望速度）
            max_linear_accel: 最大线加速度（米/秒²）
            max_angular_accel: 最大角加速度（弧度/秒²）
            
        Returns:
            Twist: 应用加速度限制后的Twist消息
        """
        # 计算时间间隔
        current_time = time.time()
        dt = current_time - self.last_publish_time
        
        # 防止时间间隔过大（例如第一次调用或长时间未更新）
        # 限制dt在合理范围内（0.01s ~ 1.0s）
        dt = max(0.01, min(dt, 1.0))
        
        # 计算允许的最大速度变化量
        max_linear_delta = max_linear_accel * dt
        max_angular_delta = max_angular_accel * dt
        
        # 创建输出消息
        limited_cmd = Twist()
        
        # 限制线速度变化（x方向）
        linear_delta = target_cmd.linear.x - self.last_published_cmd.linear.x
        if abs(linear_delta) > max_linear_delta:
            # 超过限制，按最大加速度调整
            if linear_delta > 0:
                limited_cmd.linear.x = self.last_published_cmd.linear.x + max_linear_delta
            else:
                limited_cmd.linear.x = self.last_published_cmd.linear.x - max_linear_delta
        else:
            # 未超过限制，使用目标速度
            limited_cmd.linear.x = target_cmd.linear.x
        
        # 限制线速度变化（y方向，对于全向移动机器人）
        linear_y_delta = target_cmd.linear.y - self.last_published_cmd.linear.y
        if abs(linear_y_delta) > max_linear_delta:
            if linear_y_delta > 0:
                limited_cmd.linear.y = self.last_published_cmd.linear.y + max_linear_delta
            else:
                limited_cmd.linear.y = self.last_published_cmd.linear.y - max_linear_delta
        else:
            limited_cmd.linear.y = target_cmd.linear.y
        
        # z方向通常不用，但为完整性也处理
        limited_cmd.linear.z = target_cmd.linear.z
        
        # 限制角速度变化（z方向 - 偏航角）
        angular_delta = target_cmd.angular.z - self.last_published_cmd.angular.z
        if abs(angular_delta) > max_angular_delta:
            # 超过限制，按最大角加速度调整
            if angular_delta > 0:
                limited_cmd.angular.z = self.last_published_cmd.angular.z + max_angular_delta
            else:
                limited_cmd.angular.z = self.last_published_cmd.angular.z - max_angular_delta
        else:
            # 未超过限制，使用目标速度
            limited_cmd.angular.z = target_cmd.angular.z
        
        # x和y方向角速度通常不用，直接复制
        limited_cmd.angular.x = target_cmd.angular.x
        limited_cmd.angular.y = target_cmd.angular.y
        
        # 更新上次发布的速度和时间
        self.last_published_cmd = limited_cmd
        self.last_publish_time = current_time
        
        return limited_cmd


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