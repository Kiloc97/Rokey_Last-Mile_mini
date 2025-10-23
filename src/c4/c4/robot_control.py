#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator

import time

class PatrolBot(Node):
    def __init__(self):
        super().__init__('patrol_bot')

        # navigator: 목표점 이동 및 도킹 관리
        self.navigator = TurtleBot4Navigator()

        # car_label 토픽 구독
        self.subscription = self.create_subscription(
            String,
            '/car_label',
            self.car_label_callback,
            10
        )

        # 회전을 위한 cmd_vel 퍼블리셔
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 수신 여부 플래그
        self.detected_car = False

    def car_label_callback(self, msg):
        self.get_logger().info(f'[car_label] 수신됨: {msg.data}')
        self.detected_car = True
        self.navigator.cancelTask()  # 이동 중단

    def rotate_360(self):
        self.get_logger().info('360도 회전 시작')

        twist = Twist()
        twist.angular.z = 0.5  # 양의 각속도 → 반시계 회전

        # 360도 회전을 위해 약 12초 회전 (0.5 rad/s × 12 ≈ 6 rad ≈ 360도)
        start_time = time.time()
        while time.time() - start_time < 12.0:
            self.cmd_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 정지
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('회전 완료')


def main():
    rclpy.init()
    node = PatrolBot()

    nav = node.navigator

    # 도킹 해제
    if nav.getDockedStatus():
        nav.undock()

    # 초기 pose는 RViz에서 수동 설정한다고 가정
    nav.waitUntilNav2Active()

    # 목적지 설정
    goal_pose = nav.getPoseStamped([-2.336, 2.362], 0.055)

    # 목적지 이동 시작
    nav.startToPose(goal_pose)

    # 이벤트 루프
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        # car_label 감지되면 회전 및 이동 재개
        if node.detected_car:
            node.get_logger().info('car_label 감지됨 → 정지 및 회전 시작')
            node.rotate_360()

            node.get_logger().info('회전 완료 → 목적지 재출발')
            nav.startToPose(goal_pose)
            node.detected_car = False

    node.destroy_node()
    rclpy.shutdown()
