#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Apache License 2.0 하에 라이선스됨.
# 이 파일은 해당 라이선스 조건 하에만 사용이 허가됩니다.
# 자세한 내용은 다음 링크 참조:
#     http://www.apache.org/licenses/LICENSE-2.0
#
# 이 소프트웨어는 명시적이거나 묵시적인 보증 없이 "있는 그대로" 제공됩니다.

# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy  # ROS 2 Python 클라이언트 라이브러리

# TurtleBot4의 네비게이션 관련 클래스 불러오기
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
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]
        
        # 안정화 조건 설정 (필요시 조정)
        if cov_x < 0.5 and cov_y < 0.5 and cov_yaw < 0.1:
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
        self.navigator = TurtleBot4Navigator()
        self.subscription = self.create_subscription(
            String,
            '/detected_object/distance',
            self.object_callback,
            10)
                        # class, map_x, map_y, map_z, distance
                        # green, 1.34, -0.34, 1.02, 1.23
                        # car, 1.23, -0.45, 0.98, 0.98
                        # red, 2.34, -1.10, 0.95, 0.95
                        # dummy, 0.67,  0.12, 1.05, 1.05
        self.goal_pose = []
        self.goal_pose.append(self.navigator.getPoseStamped([-2.3, 2.52], TurtleBot4Directions.SOUTH))
        self.goal_pose.append(self.navigator.getPoseStamped([-0.3, 2.6], 1.2))
        self.navigation_active = False
        self.current_goal_index = 0


        self.current_signal = None  # 현재 신호등 상태 (None, 'red', 'green')
        self.stopped_signal = None
        
        self.last_resume_time = time.time()
        self.resume_cooldown = 1.0  # 최소 1초 간격
        # 1초마다 상태 확인용 타이머 생성
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.car_detected = False
        self.car_target_pose = None
        self.last_seen_car_time = None
        self.car_timeout = 2.0   # 차량 사라졌다고 판단할 시간 (초)
    def timer_callback(self):
        if not self.navigation_active:
            return
        # 현재 이동 상태 확인
        if self.navigator.isTaskComplete():
            self.get_logger().info(f'🎯 목적지 {self.current_goal_index + 1} 도착 완료')

            self.current_goal_index += 1

            if self.current_goal_index < len(self.goal_pose):
                self.get_logger().info(f'🎯 다음 목적지 {self.current_goal_index + 1} 전송')
                self.navigator.startToPose(self.goal_pose[self.current_goal_index])
            else:
                self.get_logger().info('🎉 모든 목적지 도착 완료')
                self.navigation_active = False

        # 차량 감지 timeout 로직
        if self.car_detected and self.last_seen_car_time is not None:
            if time.time() - self.last_seen_car_time > self.car_timeout:
                self.get_logger().info('🚗 차량이 시야에서 사라졌습니다. 현재 위치에서 대기합니다.')
                self.navigator.cancelTask()
                self.car_detected = False
                self.car_target_pose = None
                return

    def object_callback(self, msg):
        lines = msg.data.strip().split('\n')

        red_distances = []
        green_distances = []
        dummy_distances = []

        for line in lines:
            parts = line.split(',')
            if len(parts) == 5:
                obj_class = parts[0].strip()
                try:
                    dist = float(parts[4].strip())
                    if obj_class == 'red':
                        red_distances.append(dist)
                    elif obj_class == 'green':
                        green_distances.append(dist)
                    elif obj_class == 'dummy':
                        dummy_distances.append(dist)
                    elif obj_class == 'car':
                        self.car_detected = True
                        self.last_seen_car_time = time.time()  # 차량을 본 시점 기록
                        car_x = float(parts[1].strip())
                        car_y = float(parts[2].strip())

                        # 현재 pose 받아오기
                        current_pose = self.navigator.getCurrentPose()
                        robot_x = current_pose.pose.position.x
                        robot_y = current_pose.pose.position.y

                        # 방향 벡터 계산
                        dx = car_x - robot_x
                        dy = car_y - robot_y
                        distance = math.hypot(dx, dy)

                        if distance > 0.5:
                            unit_dx = dx / distance
                            unit_dy = dy / distance

                            # car에서 1m 떨어진 위치 = car 좌표 - 방향 단위벡터 * 1m
                            target_x = car_x - unit_dx * 1.0
                            target_y = car_y - unit_dy * 1.0

                            # 이동 목표 생성
                            self.car_target_pose = self.navigator.getPoseStamped([target_x, target_y], 0.0)  # 방향은 일단 0으로 설정

                except ValueError:
                    pass

        min_red_distance = min(red_distances) if red_distances else None
        min_green_distance = min(green_distances) if green_distances else None
        min_dummy_distance = min(dummy_distances) if dummy_distances else None

        red_detected = len(red_distances) > 0
        green_detected = len(green_distances) > 0
        dummy_detected = len(dummy_distances) > 0

        # dummy distance info
        if dummy_detected:
            self.get_logger().info(f'dummy 감지됨, (현재 거리: {min_dummy_distance:.2f}m)')

        new_signal = None
        if red_detected:
            new_signal = 'red'
        elif green_detected:
            new_signal = 'green'

        now = time.time()

        # 신호등 보이면 1.5m까지 접근
        if new_signal in ['red', 'green']:
            min_dist = min_red_distance if new_signal == 'red' else min_green_distance
            if min_dist is not None and min_dist > 1.5:
                self.get_logger().info(f'{new_signal} 신호등 감지됨, 1.5m까지 접근 중 (현재 거리: {min_dist:.2f}m)')

            else:
                if self.current_signal != 'stop':
                    self.get_logger().info(f'{new_signal} 신호등 1.5m 도달, 정지 🛑')
                    self.navigator.cancelTask()
                    self.current_signal = 'stop'
                    self.stopped_signal = new_signal  # 정지 시 신호 저장
                    self.last_resume_time = now
            return  # 더 이상의 처리는 멈춘 후 상태변화 감시로 넘김

        # 상태 변화 감지
        if self.current_signal == 'stop':
            # 정지 상태일 때 신호 바뀌는지 감시
            if self.stopped_signal == 'red' and new_signal == 'green':
                self.get_logger().info('✅ red → green: 이동 재개')
                #self.navigator.startToPose(self.goal_pose[0])
                if self.current_goal_index < len(self.goal_pose):
                    self.navigator.startToPose(self.goal_pose[self.current_goal_index])
                self.current_signal = 'moving'
                self.last_resume_time = now
            elif self.stopped_signal == 'green' and new_signal == 'red':
                self.get_logger().info('🛑 green → red: 대기 유지')
            return

        # 신호등 사라짐 감지
        if new_signal is None and self.current_signal in ['moving', 'stop']:
            self.get_logger().info('⚠️ 신호등 사라짐, 이전 상태 유지')

        # car 목표가 계산되었으면 이동 실행
        if self.car_detected and self.car_target_pose is not None:
            self.get_logger().info(f'🚗 car 감지됨 → car에서 1m 떨어진 위치로 이동: ({target_x:.2f}, {target_y:.2f})')
            self.navigator.cancelTask()
            self.navigator.startToPose(self.car_target_pose)
            self.navigation_active = True
            self.current_signal = 'moving'
            return
        # car 목표가 사라지면 그자리에서 멈추고 main 종료
        elif not self.car_detected and self.car_target_pose is not None:
            self.get_logger().info('🚫 차량 사라짐 → 이동 중지 및 노드 종료')
            self.navigator.cancelTask()
            self.navigation_active = False

            # ROS 노드 종료 처리
            rclpy.shutdown()
        
        # 차량이 안 보이면 감지 플래그 초기화하지 않고 타이머에서 판단
        if 'car' not in [line.split(',')[0].strip() for line in lines]:
            self.get_logger().debug('이번 프레임에서 차량 감지 안됨')


def main():
    # ROS 2 노드 초기화
    rclpy.init()
    monitor = AMCLMonitor()
    node = ObstacleAwareNavigator()
    # TurtleBot4Navigator 객체 생성 (네비게이션 기능 제공)
    navigator = node.navigator
    # Nav2가 활성화될 때까지 대기 (planner, controller 등 준비될 때까지)
    navigator.waitUntilNav2Active()

    # ➕ AMCL 안정화 대기
    wait_for_amcl_ready(monitor)

    node.get_logger().info('🎯 초기 목적지 전송')
    navigator.startToPose(node.goal_pose[0])
    node.navigation_active = True


    rclpy.spin(node)
    # 작업 완료 후 ROS 종료
    rclpy.shutdown()


# 메인 함수 호출
if __name__ == '__main__':
    main()
