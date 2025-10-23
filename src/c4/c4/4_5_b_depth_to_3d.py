# rclpy 및 ROS2 관련 메시지, 패키지 임포트
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs

# 깊이 이미지에서 3D 좌표를 계산하고, 이를 map 좌표계로 변환하는 노드 정의
class DepthToMap(Node):
    def __init__(self):
        super().__init__('depth_to_map_node')

        self.bridge = CvBridge()  # 이미지 변환용 CV Bridge
        self.K = None             # 카메라 내부 파라미터

        ns = self.get_namespace()  # 노드 네임스페이스 획득
        self.depth_topic = f'{ns}/oakd/stereo/image_raw'
        self.rgb_topic = f'{ns}/oakd/rgb/image_raw/compressed'
        self.info_topic = f'{ns}/oakd/rgb/camera_info'

        # 내부 상태 초기화
        self.depth_image = None
        self.rgb_image = None
        self.clicked_point = None
        self.depth_header = None
        self.shutdown_requested = False

        # 내비게이터 객체 생성
        self.navigator = TurtleBot4Navigator()
        
        # 로봇이 도킹되지 않은 상태라면 도킹 수행 후 초기 pose 설정
        if not self.navigator.getDockedStatus():
            self.get_logger().info('Docking before initializing pose')
            self.navigator.dock()

        # 초기 위치 지정 및 Navigation2 활성화
        initial_pose = self.navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()
        self.navigator.undock()

        # TF2 버퍼 및 리스너 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logged_intrinsics = False  # 카메라 내부 파라미터 로그 출력 여부

        # 토픽 구독 설정
        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 1)
        self.create_subscription(CompressedImage, self.rgb_topic, self.rgb_callback, 1)

        # 5초 후 TF 변환 시작 (TF 트리 안정화를 위해)
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    # TF 안정화 후 실제 이미지 처리 시작
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        self.timer = self.create_timer(1.0, self.display_images)  # 1초마다 이미지 갱신
        cv2.namedWindow('RGB (left) | Depth (right)')
        cv2.setMouseCallback('RGB (left) | Depth (right)', self.mouse_callback)

        self.start_timer.cancel()

    # 카메라 내부 파라미터 수신 시 콜백
    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    # 깊이 이미지 수신 시 콜백
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth CV bridge conversion failed: {e}")

    # RGB 이미지 수신 시 콜백
    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Compressed RGB decode failed: {e}")

    # 마우스 클릭 시 콜백 - 클릭한 위치 저장
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.clicked_point = (x, y)
            self.get_logger().info(f"Clicked RGB pixel: ({x}, {y})")

    # 이미지 표시 및 클릭 좌표 → 3D 변환 → TF 변환
    def display_images(self):
        if self.rgb_image is not None and self.depth_image is not None:
            try:
                rgb_display = self.rgb_image.copy()
                depth_display = self.depth_image.copy()

                # 깊이 이미지를 시각적으로 표시하기 위한 처리
                depth_normalized = cv2.normalize(depth_display, None, 0, 255, cv2.NORM_MINMAX)
                depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_JET)

                # 클릭된 좌표가 있다면
                if self.clicked_point and self.K is not None and self.depth_header:
                    x, y = self.clicked_point
                    if x < rgb_display.shape[1] and y < rgb_display.shape[0]:
                        z = float(depth_display[y, x]) / 1000.0  # mm → m
                        if z > 0:
                            # 픽셀 좌표 → 카메라 기준 3D 좌표 변환
                            fx, fy = self.K[0, 0], self.K[1, 1]
                            cx, cy = self.K[0, 2], self.K[1, 2]

                            X = (x - cx) * z / fx
                            Y = (y - cy) * z / fy

                            # PointStamped 메시지 생성
                            pt = PointStamped()
                            pt.header.frame_id = self.depth_header.frame_id
                            pt.header.stamp = self.depth_header.stamp
                            pt.point.x = X
                            pt.point.y = Y
                            pt.point.z = z

                            # map 프레임으로 변환
                            try:
                                pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                                self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
                            except Exception as e:
                                self.get_logger().warn(f"TF transform to map failed: {e}")

                        # 디버깅 및 UI 표시용 텍스트 그리기
                        text = f"{z:.2f} m" if z > 0 else "Invalid"
                        cv2.putText(rgb_display, '+', (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        cv2.circle(rgb_display, (x, y), 4, (0, 255, 0), -1)
                        cv2.putText(depth_colored, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        cv2.circle(depth_colored, (x, y), 4, (255, 255, 255), -1)

                # 좌우 이미지 병합 및 표시
                combined = np.hstack((rgb_display, depth_colored))
                cv2.imshow('RGB (left) | Depth (right)', combined)
                key = cv2.waitKey(1)

                # 사용자가 'q' 누르면 종료 요청
                if key == ord('q'):
                    self.get_logger().info("Shutdown requested by user.")
                    self.navigator.dock()
                    self.shutdown_requested = True
            except Exception as e:
                self.get_logger().warn(f"Image display error: {e}")


# 메인 함수
def main():
    rclpy.init()
    node = DepthToMap()
    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
