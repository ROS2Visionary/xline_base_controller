#!/usr/bin/env python3
"""
ROS2机器人主动运动测试系统 - 简化版
解决rate.sleep()导致的阻塞问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import Imu
import math
import numpy as np
from collections import deque
import time
from enum import Enum
import json
from datetime import datetime
from rclpy.qos import qos_profile_sensor_data

class TestType(Enum):
    """测试类型枚举"""
    STRAIGHT_LINE = "直线行走"
    IN_PLACE_ROTATION = "原地旋转"
    CURVED_PATH = "转弯行走"
    SQUARE_PATH = "正方形路径"
    CIRCLE_PATH = "圆形路径"


class TestResult:
    """测试结果类"""
    def __init__(self, test_type, test_params):
        self.test_type = test_type
        self.test_params = test_params
        self.start_time = None
        self.end_time = None
        
        # 期望值
        self.expected_distance = 0.0
        self.expected_angle = 0.0
        self.expected_end_position = None
        
        # 实际值
        self.actual_distance = 0.0
        self.actual_angle = 0.0
        self.actual_end_position = None
        
        # 误差
        self.position_error = 0.0
        self.angle_error = 0.0
        self.distance_error = 0.0
        
        # 数据采集
        self.imu_data = []
        self.position_data = []
        self.cmd_vel_data = []
        
        # 结果判定
        self.passed = False
        self.error_messages = []


class RobotMotionTester(Node):
    def __init__(self):
        super().__init__('robot_motion_tester')
        
        # 发布速度命令
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 订阅传感器数据
        self.imu_sub = self.create_subscription(
            Imu, '/imu', self.imu_callback, qos_profile_sensor_data)
        self.position_sub = self.create_subscription(
            PointStamped, '/reflector_position', self.position_callback, 10)
        
        # 当前数据
        self.current_position = None
        self.current_imu_angle = 0.0
        self.start_position = None
        self.start_angle = 0.0
        
        # IMU角度累积
        self.accumulated_angle = 0.0
        self.last_imu_time = None
        
        # 测试数据收集
        self.test_running = False
        self.current_test_result = None
        
        # 测试配置
        self.position_tolerance = 0.1  # 位置误差容限 (m)
        self.angle_tolerance = 0.1     # 角度误差容限 (rad)
        
        # 命令发送频率
        self.cmd_publish_rate = 20  # Hz
        self.cmd_publish_interval = 1.0 / self.cmd_publish_rate  # 秒
        
        # 测试报告
        self.test_results = []
        
        self.get_logger().info('机器人运动测试系统已启动（简化版）')
        self.get_logger().info(f'命令发送频率: {self.cmd_publish_rate} Hz')
        self.get_logger().info('等待传感器数据...')
        
        # 等待传感器数据
        self._wait_for_sensors()
        
        # 初始停止（确保机器人静止）
        self.stop_robot()
        self.get_logger().info('✓ 系统初始化完成')
    
    def _wait_for_sensors(self):
        """等待传感器数据就绪"""
        self.get_logger().info('正在等待位置和IMU数据...')
        timeout = 10.0
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_position is not None:
                self.get_logger().info('✓ 传感器数据已就绪')
                return
        
        if self.current_position is None:
            self.get_logger().error('等待传感器数据超时！')
    
    def imu_callback(self, msg):
        """IMU数据回调"""
        current_time = self.get_clock().now()
        
        # 累积角速度得到角度
        if self.last_imu_time is not None:
            dt = (current_time - self.last_imu_time).nanoseconds / 1e9
            angular_vel_z = msg.angular_velocity.z
            self.accumulated_angle += angular_vel_z * dt
        
        self.last_imu_time = current_time
        
        # 如果测试正在运行，记录数据
        if self.test_running and self.current_test_result:
            self.current_test_result.imu_data.append({
                'time': current_time.nanoseconds / 1e9,
                'angular_velocity': msg.angular_velocity.z,
                'accumulated_angle': self.accumulated_angle
            })
    
    def position_callback(self, msg):
        """位置数据回调"""
        self.current_position = (msg.point.x, msg.point.y)
        
        # 如果测试正在运行，记录数据
        if self.test_running and self.current_test_result:
            current_time = self.get_clock().now()
            self.current_test_result.position_data.append({
                'time': current_time.nanoseconds / 1e9,
                'x': msg.point.x,
                'y': msg.point.y
            })
    
    def send_velocity(self, linear, angular):
        """发送速度命令"""
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_vel_pub.publish(msg)
        
        # 记录命令
        if self.test_running and self.current_test_result:
            current_time = self.get_clock().now()
            self.current_test_result.cmd_vel_data.append({
                'time': current_time.nanoseconds / 1e9,
                'linear': linear,
                'angular': angular
            })
    
    def stop_robot(self):
        """
        停止机器人 - 简化版
        使用简单的time.sleep()代替rate.sleep()
        """
        self.get_logger().info('🛑 正在停止机器人...')
        
        # 创建停止消息
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        # 持续发送停止命令 - 简化版（不使用rate）
        stop_duration = 2.0  # 秒
        interval = self.cmd_publish_interval  # 0.05秒 (20Hz)
        
        count = 0
        elapsed = 0.0
        
        self.get_logger().info(f'持续发送停止命令 {stop_duration} 秒...')
        
        while elapsed < stop_duration:
            # 发送停止命令
            self.cmd_vel_pub.publish(stop_msg)
            count += 1
            
            # 处理ROS消息
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # 简单的sleep
            time.sleep(interval)
            elapsed += interval
            
            # 每发送10次打印一次进度
            if count % 10 == 0:
                self.get_logger().info(f'  已发送 {count} 次停止命令...')
        
        self.get_logger().info(f'✓ 停止命令已发送 {count} 次')
        
        # 额外等待确保机器人完全静止
        self.get_logger().info('等待机器人完全静止...')
        time.sleep(0.5)
        
        # 再发送几次停止命令以确保
        for i in range(10):
            self.cmd_vel_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(0.05)
        
        self.get_logger().info('✓ 机器人已完全停止')
    
    def start_test(self, test_type, test_params):
        """开始测试"""
        # 确保机器人停止
        self.stop_robot()
        time.sleep(1.0)
        
        self.current_test_result = TestResult(test_type, test_params)
        self.current_test_result.start_time = time.time()
        
        # 记录起始位置和角度
        self.start_position = self.current_position
        self.start_angle = self.accumulated_angle
        
        self.test_running = True
        
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'开始测试: {test_type.value}')
        self.get_logger().info(f'参数: {test_params}')
        self.get_logger().info(f'起始位置: ({self.start_position[0]:.3f}, {self.start_position[1]:.3f})')
        self.get_logger().info(f'起始角度: {self.start_angle:.3f} rad')
        self.get_logger().info(f'{"="*70}\n')
    
    def end_test(self):
        """结束测试"""
        self.test_running = False
        
        # 停止机器人
        self.stop_robot()
        
        # 等待机器人完全停止
        self.get_logger().info('等待机器人完全静止以采样最终位置...')
        time.sleep(2.0)
        
        # 多次采样获取稳定的最终位置
        final_positions = []
        self.get_logger().info('采样最终位置...')
        for i in range(20):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.current_position:
                final_positions.append(self.current_position)
            time.sleep(0.1)
        
        if final_positions:
            final_x = np.mean([p[0] for p in final_positions])
            final_y = np.mean([p[1] for p in final_positions])
            self.current_test_result.actual_end_position = (final_x, final_y)
            self.get_logger().info(f'最终位置: ({final_x:.3f}, {final_y:.3f})')
        
        self.current_test_result.end_time = time.time()
        
        # 分析结果
        self._analyze_test_result()
        
        # 保存结果
        self.test_results.append(self.current_test_result)
        
        # 打印结果
        self._print_test_result(self.current_test_result)
    
    def _analyze_test_result(self):
        """分析测试结果"""
        result = self.current_test_result
        
        if result.actual_end_position is None or self.start_position is None:
            result.passed = False
            result.error_messages.append("缺少位置数据")
            return
        
        # 计算起点到终点的直线距离（弦长）
        dx = result.actual_end_position[0] - self.start_position[0]
        dy = result.actual_end_position[1] - self.start_position[1]
        straight_line_distance = math.sqrt(dx**2 + dy**2)

        # 默认将实际距离设置为直线距离
        actual_distance = straight_line_distance

        # 对圆弧路径，优先使用轨迹积分得到的路径长度，更接近真实行驶距离
        if result.test_type == TestType.CURVED_PATH and len(result.position_data) >= 2:
            path_length = 0.0
            prev = result.position_data[0]
            for p in result.position_data[1:]:
                pdx = p['x'] - prev['x']
                pdy = p['y'] - prev['y']
                path_length += math.sqrt(pdx**2 + pdy**2)
                prev = p
            actual_distance = path_length

        result.actual_distance = actual_distance
        
        # 计算实际旋转角度
        result.actual_angle = self.accumulated_angle - self.start_angle
        
        # 计算误差
        result.distance_error = abs(result.actual_distance - result.expected_distance)
        result.angle_error = abs(result.actual_angle - result.expected_angle)
        
        # 计算位置误差
        if result.expected_end_position:
            ex, ey = result.expected_end_position
            result.position_error = math.sqrt(
                (result.actual_end_position[0] - ex)**2 + 
                (result.actual_end_position[1] - ey)**2
            )
        
        # 判定是否通过
        result.passed = True
        
        if result.distance_error > self.position_tolerance:
            result.passed = False
            result.error_messages.append(
                f"距离误差过大: {result.distance_error:.3f}m (容限: {self.position_tolerance}m)")
        
        if result.angle_error > self.angle_tolerance:
            result.passed = False
            result.error_messages.append(
                f"角度误差过大: {result.angle_error:.3f}rad (容限: {self.angle_tolerance}rad)")
        
        if result.expected_end_position and result.position_error > self.position_tolerance:
            result.passed = False
            result.error_messages.append(
                f"位置误差过大: {result.position_error:.3f}m (容限: {self.position_tolerance}m)")
    
    def _print_test_result(self, result):
        """打印测试结果"""
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info(f'测试结果: {result.test_type.value}')
        self.get_logger().info(f'{"="*70}')
        
        # 期望值
        self.get_logger().info('\n【期望值】')
        self.get_logger().info(f'  移动距离: {result.expected_distance:.3f} m')
        self.get_logger().info(f'  旋转角度: {result.expected_angle:.3f} rad ({math.degrees(result.expected_angle):.1f}°)')
        if result.expected_end_position:
            self.get_logger().info(f'  目标位置: ({result.expected_end_position[0]:.3f}, {result.expected_end_position[1]:.3f})')
        
        # 实际值
        self.get_logger().info('\n【实际值】')
        self.get_logger().info(f'  移动距离: {result.actual_distance:.3f} m')
        self.get_logger().info(f'  旋转角度: {result.actual_angle:.3f} rad ({math.degrees(result.actual_angle):.1f}°)')
        self.get_logger().info(f'  最终位置: ({result.actual_end_position[0]:.3f}, {result.actual_end_position[1]:.3f})')
        
        # 误差
        self.get_logger().info('\n【误差分析】')
        self.get_logger().info(f'  距离误差: {result.distance_error:.3f} m ({result.distance_error/result.expected_distance*100 if result.expected_distance > 0 else 0:.1f}%)')
        self.get_logger().info(f'  角度误差: {result.angle_error:.3f} rad ({math.degrees(result.angle_error):.1f}°)')
        if result.expected_end_position:
            self.get_logger().info(f'  位置误差: {result.position_error:.3f} m')
        
        # 判定结果
        self.get_logger().info('\n【测试结果】')
        if result.passed:
            self.get_logger().info('  状态: ✓✓✓ 通过')
        else:
            self.get_logger().warn('  状态: ✗✗✗ 失败')
            for msg in result.error_messages:
                self.get_logger().warn(f'    - {msg}')
        
        duration = result.end_time - result.start_time
        self.get_logger().info(f'  测试时长: {duration:.1f} 秒')
        self.get_logger().info(f'{"="*70}\n')
    
    def test_straight_line(self, distance=1.0, velocity=0.3):
        """测试直线行走"""
        self.start_test(TestType.STRAIGHT_LINE, 
                       {'distance': distance, 'velocity': velocity})
        
        # 设置期望值
        self.current_test_result.expected_distance = distance
        self.current_test_result.expected_angle = 0.0
        
        # 计算期望终点
        current_angle = self.start_angle
        end_x = self.start_position[0] + distance * math.cos(current_angle)
        end_y = self.start_position[1] + distance * math.sin(current_angle)
        self.current_test_result.expected_end_position = (end_x, end_y)
        
        # 计算运行时间
        duration = distance / velocity
        
        self.get_logger().info(f'执行: 以 {velocity} m/s 前进 {distance} m (预计 {duration:.1f} 秒)')
        
        # 发送速度命令 - 简化版
        interval = self.cmd_publish_interval
        start_time = time.time()
        cmd_count = 0
        
        while (time.time() - start_time) < duration and rclpy.ok():
            self.send_velocity(velocity, 0.0)
            cmd_count += 1
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(interval)
            
            # 每秒打印一次进度
            elapsed = time.time() - start_time
            if cmd_count % self.cmd_publish_rate == 0:
                self.get_logger().info(f'  运动中... {elapsed:.1f}/{duration:.1f}秒')
        
        self.get_logger().info(f'✓ 运动完成，共发送 {cmd_count} 次命令')
        
        self.end_test()
    
    def test_in_place_rotation(self, angle_deg, angular_velocity=0.5):
        """测试原地旋转"""
        angle_rad = math.radians(angle_deg)
        
        self.start_test(TestType.IN_PLACE_ROTATION,
                       {'angle': angle_deg, 'angular_velocity': angular_velocity})
        
        # 设置期望值
        self.current_test_result.expected_distance = 0.0
        self.current_test_result.expected_angle = angle_rad
        self.current_test_result.expected_end_position = self.start_position
        
        # 计算运行时间
        duration = abs(angle_rad / angular_velocity)
        direction = 1 if angle_rad > 0 else -1
        
        self.get_logger().info(f'执行: 原地旋转 {angle_deg}° (预计 {duration:.1f} 秒)')
        
        # 发送速度命令 - 简化版
        interval = self.cmd_publish_interval
        start_time = time.time()
        cmd_count = 0
        
        while (time.time() - start_time) < duration and rclpy.ok():
            self.send_velocity(0.0, angular_velocity * direction)
            cmd_count += 1
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(interval)
            
            # 每秒打印一次进度
            elapsed = time.time() - start_time
            if cmd_count % self.cmd_publish_rate == 0:
                self.get_logger().info(f'  旋转中... {elapsed:.1f}/{duration:.1f}秒')
        
        self.get_logger().info(f'✓ 旋转完成，共发送 {cmd_count} 次命令')
        
        self.end_test()
    
    def test_curved_path(self, radius=1.0, angle_deg=90, linear_vel=0.3):
        """测试转弯（圆弧路径）"""
        angle_rad = math.radians(angle_deg)
        
        self.start_test(TestType.CURVED_PATH,
                       {'radius': radius, 'angle': angle_deg, 'velocity': linear_vel})
        
        # 计算角速度
        angular_vel = linear_vel / radius
        direction = 1.0 if angle_rad >= 0.0 else -1.0
        
        # 计算弧长
        arc_length = radius * abs(angle_rad)
        
        # 设置期望值
        self.current_test_result.expected_distance = arc_length
        self.current_test_result.expected_angle = angle_rad
        
        # 计算期望终点
        current_angle = self.start_angle
        # 根据转向方向确定圆心位置：左转在左侧，右转在右侧
        cx = self.start_position[0] - direction * radius * math.sin(current_angle)
        cy = self.start_position[1] + direction * radius * math.cos(current_angle)
        end_angle = current_angle + angle_rad
        end_x = cx + direction * radius * math.sin(end_angle)
        end_y = cy - direction * radius * math.cos(end_angle)
        self.current_test_result.expected_end_position = (end_x, end_y)
        
        # 计算运行时间
        duration = arc_length / linear_vel
        
        self.get_logger().info(f'执行: 半径 {radius}m 转弯 {angle_deg}° (预计 {duration:.1f} 秒)')
        
        # 发送速度命令 - 简化版
        interval = self.cmd_publish_interval
        start_time = time.time()
        cmd_count = 0
        
        while (time.time() - start_time) < duration and rclpy.ok():
            self.send_velocity(linear_vel, angular_vel * direction)
            cmd_count += 1
            rclpy.spin_once(self, timeout_sec=0.001)
            time.sleep(interval)
            
            # 每秒打印一次进度
            elapsed = time.time() - start_time
            if cmd_count % self.cmd_publish_rate == 0:
                self.get_logger().info(f'  转弯中... {elapsed:.1f}/{duration:.1f}秒')
        
        self.get_logger().info(f'✓ 转弯完成，共发送 {cmd_count} 次命令')
        
        self.end_test()
    
    def test_square_path(self, side_length=1.0, velocity=0.3):
        """测试正方形路径"""
        self.get_logger().info(f'\n{"#"*70}')
        self.get_logger().info(f'组合测试: 正方形路径 (边长 {side_length}m)')
        self.get_logger().info(f'{"#"*70}\n')
        
        for i in range(4):
            self.get_logger().info(f'\n--- 第 {i+1} 边 ---')
            self.test_straight_line(side_length, velocity)
            time.sleep(2)
            
            if i < 3:
                self.get_logger().info(f'\n--- 转角 {i+1} ---')
                self.test_in_place_rotation(90, 0.5)
                time.sleep(2)
        
        self.get_logger().info(f'\n{"#"*70}')
        self.get_logger().info('正方形路径测试完成')
        self.get_logger().info(f'{"#"*70}\n')
    
    def test_circle_path(self, radius=0.5, linear_vel=0.3):
        """测试圆形路径"""
        self.get_logger().info(f'\n{"#"*70}')
        self.get_logger().info(f'圆形路径测试 (半径 {radius}m)')
        self.get_logger().info(f'{"#"*70}\n')
        
        self.test_curved_path(radius, 360, linear_vel)
        
        self.get_logger().info(f'\n{"#"*70}')
        self.get_logger().info('圆形路径测试完成')
        self.get_logger().info(f'{"#"*70}\n')
    
    def generate_report(self, filename='test_report.json'):
        """生成测试报告"""
        report = {
            'test_time': datetime.now().isoformat(),
            'total_tests': len(self.test_results),
            'passed_tests': sum(1 for r in self.test_results if r.passed),
            'failed_tests': sum(1 for r in self.test_results if not r.passed),
            'tests': []
        }
        
        for result in self.test_results:
            test_data = {
                'test_type': result.test_type.value,
                'params': result.test_params,
                'passed': result.passed,
                'expected': {
                    'distance': result.expected_distance,
                    'angle': result.expected_angle,
                    'position': result.expected_end_position
                },
                'actual': {
                    'distance': result.actual_distance,
                    'angle': result.actual_angle,
                    'position': result.actual_end_position
                },
                'errors': {
                    'distance': result.distance_error,
                    'angle': result.angle_error,
                    'position': result.position_error
                },
                'error_messages': result.error_messages,
                'duration': result.end_time - result.start_time
            }
            report['tests'].append(test_data)
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        
        self.get_logger().info(f'测试报告已保存: {filename}')
        self._print_summary()
    
    def _print_summary(self):
        """打印测试摘要"""
        total = len(self.test_results)
        passed = sum(1 for r in self.test_results if r.passed)
        failed = total - passed
        
        self.get_logger().info(f'\n{"="*70}')
        self.get_logger().info('测试摘要')
        self.get_logger().info(f'{"="*70}')
        self.get_logger().info(f'总测试数: {total}')
        self.get_logger().info(f'通过: {passed} ({passed/total*100 if total > 0 else 0:.1f}%)')
        self.get_logger().info(f'失败: {failed} ({failed/total*100 if total > 0 else 0:.1f}%)')
        
        if failed > 0:
            self.get_logger().info('\n失败的测试:')
            for i, result in enumerate(self.test_results):
                if not result.passed:
                    self.get_logger().warn(f'  {i+1}. {result.test_type.value}')
                    for msg in result.error_messages:
                        self.get_logger().warn(f'     - {msg}')
        
        self.get_logger().info(f'{"="*70}\n')


def main(args=None):
    rclpy.init(args=args)
    tester = RobotMotionTester()
    
    try:
        print("\n" + "="*70)
        print("ROS2机器人运动测试系统 - 简化版")
        print("解决了rate阻塞问题")
        print("="*70)
        print("\n请选择测试项目:")
        print("1. 直线行走测试")
        print("2. 原地旋转测试")
        print("3. 转弯测试")
        print("4. 正方形路径测试")
        print("5. 圆形路径测试")
        print("6. 完整测试套件")
        print("0. 退出")
        
        choice = input("\n请输入选项 (0-6): ").strip()
        
        if choice == '1':
            distance = float(input("输入前进距离 (m, 默认1.0): ") or "1.0")
            velocity = float(input("输入速度 (m/s, 默认0.3): ") or "0.3")
            tester.test_straight_line(distance, velocity)
            
        elif choice == '2':
            angle = float(input("输入旋转角度 (度, 默认90): ") or "90")
            angular_vel = float(input("输入角速度 (rad/s, 默认0.5): ") or "0.5")
            tester.test_in_place_rotation(angle, angular_vel)
            
        elif choice == '3':
            radius = float(input("输入转弯半径 (m, 默认1.0): ") or "1.0")
            angle = float(input("输入转弯角度 (度, 默认90): ") or "90")
            velocity = float(input("输入速度 (m/s, 默认0.3): ") or "0.3")
            tester.test_curved_path(radius, angle, velocity)
            
        elif choice == '4':
            side = float(input("输入正方形边长 (m, 默认1.0): ") or "1.0")
            velocity = float(input("输入速度 (m/s, 默认0.3): ") or "0.3")
            tester.test_square_path(side, velocity)
            
        elif choice == '5':
            radius = float(input("输入圆形半径 (m, 默认0.5): ") or "0.5")
            velocity = float(input("输入速度 (m/s, 默认0.3): ") or "0.3")
            tester.test_circle_path(radius, velocity)
            
        elif choice == '6':
            print("\n开始完整测试套件...")
            print("这将执行多个测试，请确保机器人周围有足够空间！")
            input("按Enter键继续...")
            
            tester.test_straight_line(1.0, 0.3)
            time.sleep(2)
            tester.test_straight_line(0.5, 0.2)
            time.sleep(2)
            
            tester.test_in_place_rotation(90, 0.5)
            time.sleep(2)
            tester.test_in_place_rotation(180, 0.5)
            time.sleep(2)
            tester.test_in_place_rotation(-90, 0.5)
            time.sleep(2)
            
            tester.test_curved_path(1.0, 90, 0.3)
            time.sleep(2)
            tester.test_curved_path(0.5, 180, 0.2)
            time.sleep(2)
            
            tester.test_square_path(1.0, 0.3)
            
        elif choice == '0':
            print("退出测试")
        else:
            print("无效选项")
        
        if choice != '0':
            tester.generate_report('test_report.json')
        
    except KeyboardInterrupt:
        print("\n测试被中断")
    except Exception as e:
        tester.get_logger().error(f'测试出错: {str(e)}')
        import traceback
        traceback.print_exc()
    finally:
        # 最终确保机器人停止
        print("\n确保机器人完全停止...")
        tester.stop_robot()
        time.sleep(1)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
