# ROS2 및 관련 라이브러리 임포트
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs  # TF 좌표계 변환을 위한 메시지 타입 변환 지원
from custom_msgs.msg import DetectedObject, DetectedObjectArray  # 사용자 정의 메시지
        # subscribe 메시지 예시
        # /detected_objects
        # header:
        #     stamp:
        #     frame_id:
        # objects:
        #     - class_name: "car"
        #     x_center: 320
        #     y_center: 240
        #     distance: 2.45
        #     - class_name: "red"
        #     x_center: 220
        #     y_center: 200
        #     distance: 1.35 #meter

# Depth 이미지를 클릭하여 로봇을 목표 지점으로 이동시키는 노드 클래스
class YoloAndDepthToMap(Node):
    def __init__(self):
        super().__init__('yolo_and_depth_to_map_node')

        # OpenCV <-> ROS 이미지 변환을 위한 브리지 생성
        self.bridge = CvBridge()
        self.K = None  # 카메라 내부 파라미터 행렬 (Intrinsic Matrix)


        # 내비게이션 제어 객체 생성 (TurtleBot4Navigator)
        self.navigator = TurtleBot4Navigator()

        # 초기 위치 세팅: 도킹 상태 확인 → 도킹 → Pose 설정 → 언도킹
        # if not self.navigator.getDockedStatus():
        #     self.get_logger().info('Docking before initializing pose')
        #     self.navigator.dock()

        # 초기 위치를 맵 기준 [0.0, 0.0], 북쪽 방향으로 설정
        # initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        # self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        # self.navigator.undock()

        # TF 좌표계 변환을 위한 Buffer 및 Listener 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logged_intrinsics = False  # Intrinsics 로그 중복 방지

        # DetectedObject 수신 구독 설정
        self.create_subscription(DetectedObject, '/detected_objects', self.object_callback, 10)

        # TF 트리 안정화 대기 (5초 후 변환 시작)
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        # TF Tree 준비 완료 → 이미지 디스플레이 타이머 시작
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.start_timer.cancel()

    def send_goal(self, x, y, z):
        # map 좌표계로 목표 Pose 생성 및 로봇 이동 명령
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # 방향은 생략 (기본값)

        self.get_logger().info(f"Sending goal to ({x:.2f}, {y:.2f}) in map frame")
        self.navigator.startToPose(goal_pose)

        # 이동 피드백 모니터링
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)
            feedback = self.navigator.getFeedback()
            if feedback and feedback.distance_remaining is not None:
                self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")

        # 이동 완료 결과 출력
        result = self.navigator.getResult()
        self.get_logger().info(f"Navigation result: {result}")

# 메인 함수: ROS2 초기화 및 노드 실행
def main():
    rclpy.init()
    node = YoloAndDepthToMap()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()

# 진입점
if __name__ == '__main__':
    main()
