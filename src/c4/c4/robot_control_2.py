#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
# Apache License 2.0

import rclpy
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import math

class AMCLMonitor(Node):
    def __init__(self):
        super().__init__('amcl_monitor')
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot9/amcl_pose',
            self.pose_callback,
            10)
        self.amcl_ready = False

    def pose_callback(self, msg):
        cov = msg.pose.covariance
        if cov[0] < 0.5 and cov[7] < 0.5 and cov[35] < 0.1:
            self.amcl_ready = True
            self.get_logger().info('✅ AMCL 안정화 완료')

def wait_for_amcl_ready(node, timeout=10):
    start_time = time.time()
    while not node.amcl_ready:
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start_time > timeout:
            node.get_logger().warn('⚠️ AMCL 안정화 시간 초과, 강제 진행')
            break

class ObstacleAwareNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_aware_nav')
        
        # 초기화 파라미터
        self.stop_distance = 1.5      # 신호등 정지 거리 (m)
        self.signal_cooldown = 1.0    # 신호등 상태 변경 쿨타임 (초)
        self.car_timeout = 2.0        # 차량 감지 타임아웃
        
        # 네비게이션 설정
        self.navigator = TurtleBot4Navigator()
        self.subscription = self.create_subscription(
            String,
            '/detected_object/distance',
            self.object_callback,
            10)
        
        # 목적지 설정
        self.goal_pose = [
            self.navigator.getPoseStamped([-2.1, 2.62], -2.5896),
            self.navigator.getPoseStamped([-0.3, 2.6], 0.32024)
        ]
        
        # 상태 변수
        self.current_goal_index = 0
        self.navigation_active = False
        self.current_signal = None       # 현재 신호등 상태 (None/red/green)
        self.last_signal_time = 0        # 마지막 신호 변경 시간
        self.waiting_for_red = False     # 빨간 신호 대기 중 여부
        self.car_detected = False        # 차량 감지 상태
        self.car_target_pose = None      # 차량 회피 목표 위치
        self.last_seen_car_time = 0      # 마지막 차량 감지 시간

        # 타이머 설정 (1초 주기)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        """주기적인 상태 확인 및 네비게이션 관리"""
        if not self.navigation_active:
            return

        # 목적지 도착 확인
        if self.navigator.isTaskComplete():
            self.handle_goal_completion()

        # 차량 감지 타임아웃 처리
        if self.car_detected and (time.time() - self.last_seen_car_time > self.car_timeout):
            self.handle_car_timeout()

    def handle_goal_completion(self):
        """목적지 도착 시 처리"""
        self.get_logger().info(f'🎯 목적지 {self.current_goal_index + 1} 도착 완료')
        self.current_goal_index += 1

        if self.current_goal_index < len(self.goal_pose):
            self.navigator.startToPose(self.goal_pose[self.current_goal_index])
        else:
            self.get_logger().info('🎉 모든 목적지 도착 완료')
            self.navigation_active = False

    def handle_car_timeout(self):
        """차량 감지 타임아웃 처리"""
        self.get_logger().info('🚗 차량이 시야에서 사라졌습니다. 현재 위치에서 대기합니다.')
        self.navigator.cancelTask()
        self.car_detected = False
        self.car_target_pose = None

    def object_callback(self, msg):
        """객체 감지 콜백 (신호등/차량 처리)"""
        now = time.time()
        
        # 쿨타임 중인 경우 추가 처리 생략
        if now - self.last_signal_time < self.signal_cooldown:
            return

        # 객체 정보 파싱 (거리 정보 포함)
        red_detected, green_detected, car_detected, min_red_distance, min_green_distance = self.parse_object_data(msg.data)

        # 차량 감지 우선 처리 (쿨타임 적용 없음)
        if car_detected:
            self.handle_car_detection(msg.data)
            return

        # 가장 가까운 신호등 거리 결정
        active_signal = None
        min_distance = float('inf')
        
        if red_detected and min_red_distance < min_distance:
            active_signal = 'red'
            min_distance = min_red_distance
        if green_detected and min_green_distance < min_distance:
            active_signal = 'green'
            min_distance = min_green_distance

        # 신호등이 1.5m 이내인 경우 정지
        if active_signal and min_distance <= self.stop_distance:
            self.handle_signal_at_stop_distance(active_signal, now)
        elif active_signal:
            self.get_logger().info(f'🚦 신호등까지 {min_distance:.2f}m 남음, 접근 중...')
                        # class, map_x, map_y, map_z, distance
                        # green, 1.34, -0.34, 1.02, 1.23
                        # car, 1.23, -0.45, 0.98, 0.98
                        # red, 2.34, -1.10, 0.95, 0.95
                        # dummy, 0.67,  0.12, 1.05, 1.05
    def parse_object_data(self, data):
        """객체 감지 데이터 파싱 (거리 정보 포함)"""
        lines = data.strip().split('\n')
        red_detected = green_detected = car_detected = False
        min_red_distance = float('inf')
        min_green_distance = float('inf')

        for line in lines:
            parts = line.split(',')
            if len(parts) == 5:
                obj_class = parts[0].strip()
                try:
                    distance = float(parts[4].strip())
                    if obj_class == 'red':
                        red_detected = True
                        if distance < min_red_distance:
                            min_red_distance = distance
                    elif obj_class == 'green':
                        green_detected = True
                        if distance < min_green_distance:
                            min_green_distance = distance
                    elif obj_class == 'car':
                        car_detected = True
                except ValueError:
                    continue

        return red_detected, green_detected, car_detected, min_red_distance, min_green_distance

    def handle_car_detection(self, data):
        """차량 감지 처리"""
        self.car_detected = True
        self.last_seen_car_time = time.time()

        # 차량 위치 계산
        for line in data.strip().split('\n'):
            parts = line.split(',')
            if len(parts) == 5 and parts[0].strip() == 'car':
                car_x = float(parts[1].strip())
                car_y = float(parts[2].strip())
                self.calculate_car_target_pose(car_x, car_y)
                break

        # 차량 회피 이동 시작
        if self.car_target_pose:
            self.navigator.cancelTask()
            self.navigator.startToPose(self.car_target_pose)
            self.navigation_active = True

    def calculate_car_target_pose(self, car_x, car_y):
        """차량으로부터 1m 떨어진 안전 위치 계산"""
        current_pose = self.navigator.getCurrentPose()
        robot_x = current_pose.pose.position.x
        robot_y = current_pose.pose.position.y

        dx = car_x - robot_x
        dy = car_y - robot_y
        distance = math.hypot(dx, dy)

        if distance > 0.5:
            unit_dx = dx / distance
            unit_dy = dy / distance
            target_x = car_x - unit_dx * 1.0
            target_y = car_y - unit_dy * 1.0
            self.car_target_pose = self.navigator.getPoseStamped([target_x, target_y], 0.0)

    def handle_signal_at_stop_distance(self, signal_type, timestamp):
        """신호등이 정지 거리(1.5m) 내에 있을 때 처리"""
        self.last_signal_time = timestamp

        # 이미 정지 상태인 경우
        if self.current_signal == 'stop':
            # 빨간 신호 대기 중에 초록 신호가 온 경우
            if self.waiting_for_red and signal_type == 'green':
                self.get_logger().info('🟢 초록 신호등 확인, 주행 재개!')
                self.resume_navigation()
                self.waiting_for_red = False
            return

        # 신호등이 1.5m 이내 도착 (처음 정지)
        self.get_logger().info(f'🛑 신호등 1.5m 도착, 정지 (신호: {signal_type})')
        self.navigator.cancelTask()
        self.current_signal = 'stop'

        # 빨간 신호면 대기, 초록 신호면 빨간 신호 올 때까지 대기
        if signal_type == 'red':
            self.waiting_for_red = False
            self.get_logger().info('🔴 빨간 신호등 대기 중...')
        else:
            self.waiting_for_red = True
            self.get_logger().info('🟢 초록 신호등 - 빨간 신호될 때까지 대기...')

    def resume_navigation(self):
        """주행 재개"""
        self.current_signal = None
        if self.current_goal_index < len(self.goal_pose):
            self.navigator.startToPose(self.goal_pose[self.current_goal_index])
            self.get_logger().info('➡️ 목적지로 주행 재개')

def main():
    rclpy.init()
    #monitor = AMCLMonitor()
    node = ObstacleAwareNavigator()
    navigator = node.navigator
    
    navigator.waitUntilNav2Active()
    #wait_for_amcl_ready(monitor)

    # 초기 목적지 설정
    node.get_logger().info('🎯 초기 목적지 전송')
    navigator.startToPose(node.goal_pose[0])
    node.navigation_active = True

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()