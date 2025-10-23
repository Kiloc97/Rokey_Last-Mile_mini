import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import cv2
import time


class DepthToNavGoal(Node):
    def __init__(self):
        super().__init__('depth_to_nav_goal_node')

        # OpenCV 이미지-ROS 메시지 변환 도구
        self.bridge = CvBridge()
        self.K = None  # 카메라 내참 행렬
        self.latest_map_point = None  # 변환된 좌표 저장용
        self.capture_enabled = True  # depth 캡처 활성화 여부

        # Depth 이미지 및 카메라 정보 토픽
        self.depth_topic = '/robot9/oakd/rgb/preview/depth'
        self.info_topic = '/robot9/oakd/rgb/preview/camera_info'

        # TF 변환 처리를 위한 버퍼 및 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 정보 및 depth 이미지 수신 콜백 설정
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)

        # Nav2 행동 목표를 보내기 위한 액션 클라이언트 생성
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 키 입력 감지를 위한 타이머 설정 (0.1초 간격)
        self.create_timer(0.1, self.check_key)

        self.logged_intrinsics = False  # 내참 행렬 로그 플래그

        # 제어용 더미 윈도우 생성
        cv2.namedWindow("Control")
        cv2.imshow("Control", np.zeros((100, 300), dtype=np.uint8))

        self.last_feedback_log_time = 0  # 피드백 로그 출력 시간 기록


    def camera_info_callback(self, msg):
        # 카메라 내부 파라미터 저장
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        # 내참 행렬 미수신 또는 캡처 비활성 상태이면 중단
        if self.K is None or not self.capture_enabled:
            return

        try:
            # depth 이미지 ROS 메시지를 OpenCV 이미지로 변환
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        # 중심 좌표 픽셀 값 기준으로 거리 계산
        cx = self.K[0, 2]
        cy = self.K[1, 2]
        fx = self.K[0, 0]
        fy = self.K[1, 1]

        u = int(cx)
        v = int(cy)
        z = float(depth_image[v, u])
        if z == 0.0:
            self.get_logger().warn('Invalid depth at center pixel')
            return

        # 픽셀 기준 거리(z)와 내참값으로 x, y 좌표 계산
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        camera_frame = msg.header.frame_id

        # 카메라 기준 PointStamped 메시지 생성
        pt = PointStamped()
        pt.header.frame_id = camera_frame
        pt.header.stamp = msg.header.stamp
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        try:
            # 최신 시간으로 갱신한 포인트 생성
            pt_latest = PointStamped()
            pt_latest.header.frame_id = camera_frame
            pt_latest.header.stamp = rclpy.time.Time().to_msg()
            pt_latest.point = pt.point

            # 카메라 기준 좌표를 'map' 프레임으로 변환
            pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
            self.latest_map_point = pt_map
            self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}")

    def check_key(self):
        # 키 입력을 감지하고 'g' 키를 누르면 goal 전송
        cv2.imshow("Control", np.zeros((100, 300), dtype=np.uint8))  # refresh dummy window
        key = cv2.waitKey(1) & 0xFF
        if key == ord('g'):
            self.get_logger().info("Key 'g' pressed: stopping capture and sending goal.")
            self.capture_enabled = False
            self.send_goal()

    def send_goal(self):
        # latest_map_point가 없다면 전송 중단
        if self.latest_map_point is None:
            self.get_logger().warn("No map coordinate available to send as goal.")
            return

        # PoseStamped 메시지 생성 (map 프레임 기준)
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_map_point.point.x
        pose.pose.position.y = self.latest_map_point.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # face forward

        # NavigateToPose 목표 메시지 구성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(f"Sending goal to map position: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # goal 수락 여부 확인
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by Nav2.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        # 목표 수행 결과 출력 및 노드 종료
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        
        # Stop spinning and shutdown ROS
        self.get_logger().info("Shutting down after goal.")
        rclpy.shutdown()
        cv2.destroyAllWindows()

    def feedback_callback(self, feedback_msg):
        # 남은 거리 피드백 주기적으로 출력 (1초 간격)
        feedback = feedback_msg.feedback
        current_time = time.time()
        if current_time - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
            self.last_feedback_log_time = current_time

def main():
    # ROS2 노드 초기화 및 실행
    rclpy.init()
    node = DepthToNavGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # Don't shut down here, it's handled in callback
    node.destroy_node()


if __name__ == '__main__':
    main()
