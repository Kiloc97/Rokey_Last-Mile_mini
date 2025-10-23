import rclpy  # ROS2 Python 클라이언트 라이브러리
from rclpy.node import Node  # 노드 생성을 위한 기본 클래스
from sensor_msgs.msg import Image, CameraInfo  # 이미지 및 카메라 정보 메시지 타입
from geometry_msgs.msg import PointStamped  # 3D 좌표 표현을 위한 메시지 타입
from cv_bridge import CvBridge  # ROS 이미지 메시지를 OpenCV 이미지로 변환하는 도구
import numpy as np  # 수치 계산을 위한 라이브러리
import tf2_ros  # TF2 프레임 변환 라이브러리
import tf2_geometry_msgs  # TF2에서 geometry_msgs 타입 변환 지원

class DepthToMap(Node):  # ROS2 노드를 상속하여 DepthToMap 클래스 정의
    def __init__(self):
        super().__init__('depth_to_map_node')  # 노드 이름 지정

        self.bridge = CvBridge()  # CvBridge 객체 생성 (이미지 변환용)
        self.K = None  # 카메라 내부 파라미터 (intrinsic matrix)
        self.depth_topic = '/robot9/oakd/rgb/preview/depth'  # 깊이 이미지 토픽
        self.info_topic = '/robot9/oakd/rgb/preview/camera_info'  # 카메라 정보 토픽

        self.tf_buffer = tf2_ros.Buffer()  # TF 변환 버퍼 생성
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)  # TF 리스너 시작

        self.create_subscription(CameraInfo, self.info_topic, self.camera_info_callback, 10)  # 카메라 정보 구독 설정
        self.create_subscription(Image, self.depth_topic, self.depth_callback, 10)  # 깊이 이미지 구독 설정

        self.logged_intrinsics = False  # 내부 파라미터 로그 출력 여부

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)  # 카메라 내부 행렬을 3x3 형태로 변환
        if not self.logged_intrinsics:  # 처음 한 번만 로그 출력
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        if self.K is None:  # 카메라 내부 파라미터 수신 전이면 대기
            self.get_logger().warn('Waiting for camera intrinsics...')
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')  # 깊이 이미지를 OpenCV 형식으로 변환
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")  # 변환 실패 시 로그
            return

        # height, width = depth_image.shape  # 이미지 크기 (사용 안함)
        cx = self.K[0, 2]  # 카메라 중심 x좌표
        cy = self.K[1, 2]  # 카메라 중심 y좌표
        fx = self.K[0, 0]  # 초점 거리 fx
        fy = self.K[1, 1]  # 초점 거리 fy

        u = int(cx)  # 중심 픽셀 x좌표
        v = int(cy)  # 중심 픽셀 y좌표

        z = float(depth_image[v, u])  # 중심 픽셀의 깊이값
        if z == 0.0:  # 깊이 값이 없으면 경고 후 리턴
            self.get_logger().warn('Invalid depth at center pixel')
            return

        # Compute 3D point in camera frame
        x = (u - cx) * z / fx  # 픽셀좌표 → 카메라좌표계 x
        y = (v - cy) * z / fy  # 픽셀좌표 → 카메라좌표계 y

        # Auto-detect the camera frame from the depth image header
        camera_frame = msg.header.frame_id  # 프레임 이름 자동 추출
        self.get_logger().info(f"camera_frame_id ({camera_frame})")  # 카메라 프레임 로그 출력
        self.get_logger().info(f"camera_frame: ({x:.2f}, {y:.2f}, {z:.2f})")  # 카메라 좌표계 상의 좌표 출력

        # Prepare PointStamped in camera frame
        pt = PointStamped()  # 좌표를 담을 메시지 생성
        pt.header.frame_id = camera_frame  # 원본 좌표계 설정
        pt.header.stamp = msg.header.stamp  # 정확한 시간 설정
        pt.point.x = x
        pt.point.y = y
        pt.point.z = z

        # Transform to base_link (use exact timestamp)
        try:
            pt_base = self.tf_buffer.transform(pt, 'base_link', timeout=rclpy.duration.Duration(seconds=0.5))  # base_link로 좌표 변환
            self.get_logger().info(f"base_link:    ({pt_base.point.x:.2f}, {pt_base.point.y:.2f}, {pt_base.point.z:.2f})")  # base_link 상의 좌표 출력
        except Exception as e:
            self.get_logger().warn(f"TF to base_link failed: {e}")  # 변환 실패 시 경고

        # Transform to map (avoid extrapolation by using latest available time)
        try:
            # Update header with latest time to avoid extrapolation error
            pt_latest = PointStamped()  # 최신 시간으로 새 좌표 메시지 생성
            pt_latest.header.frame_id = camera_frame
            pt_latest.header.stamp = rclpy.time.Time().to_msg()  # 현재 시간 사용
            pt_latest.point = pt.point  # 기존 3D 좌표 복사

            pt_map = self.tf_buffer.transform(pt_latest, 'map', timeout=rclpy.duration.Duration(seconds=0.5))  # map 프레임으로 변환
            self.get_logger().info(f"map:          ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")  # map 좌표 출력
        except Exception as e:
            self.get_logger().warn(f"TF to map failed: {e}")  # 변환 실패 시 경고

def main():
    rclpy.init()  # rclpy 초기화
    node = DepthToMap()  # 노드 객체 생성
    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass  # Ctrl+C로 중지 시 예외 처리
    node.destroy_node()  # 노드 종료
    rclpy.shutdown()  # rclpy 종료

if __name__ == '__main__':
    main()  # main 함수 실행

# This script converts depth images to 3D point clouds and transforms them to the map frame.
# It uses the camera intrinsics and depth information to compute the 3D coordinates.
# The script also handles TF transformations to convert the points from the camera frame to the base_link and map frames.
