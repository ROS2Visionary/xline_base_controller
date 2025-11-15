#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
XLine命令速度复用节点
订阅来自执行任务和平板操控的Twist消息，并转发到cmd_vel话题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    """
    命令速度复用节点
    订阅来自执行任务和平板操控的Twist消息，并转发到cmd_vel话题
    """
    
    def __init__(self):
        super().__init__('cmd_vel_mux')
        
        # 创建发布者 - 发布到 cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )
        
        # 订阅执行任务的控制指令
        self.task_subscription = self.create_subscription(
            Twist,
            'task_cmd_vel',  # 执行任务话题名称
            self.task_callback,
            10
        )
        
        # 订阅平板操控的指令
        self.tablet_subscription = self.create_subscription(
            Twist,
            'tablet_cmd_vel',  # 平板操控话题名称
            self.tablet_callback,
            10
        )
        
        self.get_logger().info('CmdVelMux节点已启动')
        self.get_logger().info('订阅话题: task_cmd_vel, tablet_cmd_vel')
        self.get_logger().info('发布话题: cmd_vel')
    
    def task_callback(self, msg):
        """执行任务话题回调函数 - 直接转发"""
        self.get_logger().debug('收到执行任务指令，正在转发...')
        self.cmd_vel_publisher.publish(msg)
    
    def tablet_callback(self, msg):
        """平板操控话题回调函数 - 直接转发"""
        self.get_logger().debug('收到平板操控指令，正在转发...')
        self.cmd_vel_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    cmd_vel_mux = CmdVelMux()
    
    try:
        rclpy.spin(cmd_vel_mux)
    except KeyboardInterrupt:
        pass
    finally:
        cmd_vel_mux.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
