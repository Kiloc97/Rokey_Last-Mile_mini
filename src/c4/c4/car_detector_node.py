import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from ultralytics import YOLO
import cv2

class CarDetectorNode(Node):
    def __init__(self):
        super().__init__('car_detector_node')
        self.bridge = CvBridge()

        # RGB 카메라 파라미터
        self.rgb_K = None
        self.rgb_camera_frame = None

        # Stereo(Depth) 카메라 파라미터
        self.stereo_K = None
        self.stereo_camera_frame = None

        self.depth_image = None    # 최신 depth 이미지
        self.rgb_image = None      # 최신 RGB 이미지(디코딩 완료)

        # YOLO 모델 로드
        self.model = YOLO("/home/khc/turtlebot4_ws/src/paddy_package/my_best_model_PADDY_3.pt")

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # map 좌표 퍼블리셔
        self.point_pub = self.create_publisher(PointStamped, '/car_goal_point', 10)

        # 카메라 정보, 압축 RGB, 압축 Depth 이미지 구독
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/preview/camera_info', self.rgb_camera_info_callback, 10)
        self.create_subscription(CameraInfo, '/robot9/oakd/stereo/camera_info', self.stereo_camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', self.compressed_rgb_callback, 10)
        self.create_subscription(CompressedImage, '/robot9/oakd/stereo/image_raw/compressedDepth', self.compressed_depth_callback, 10)

        # 0.5초마다 객체 탐지 후 좌표 publish
        self.create_timer(0.5, self.detect_and_publish)

    def rgb_camera_info_callback(self, msg):
        """RGB 카메라의 CameraInfo를 저장."""
        self.rgb_K = np.array(msg.k).reshape(3, 3)
        self.rgb_camera_frame = msg.header.frame_id
        self.get_logger().info(f"[RGB] CameraInfo updated: frame={self.rgb_camera_frame}")

    def stereo_camera_info_callback(self, msg):
        """Stereo(Depth) 카메라의 CameraInfo를 저장."""
        self.stereo_K = np.array(msg.k).reshape(3, 3)
        self.stereo_camera_frame = msg.header.frame_id
        self.get_logger().info(f"[Stereo] CameraInfo updated: frame={self.stereo_camera_frame}")

    def compressed_rgb_callback(self, msg):
        """압축(RGB) 이미지 수신 콜백 (JPEG 등)."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV로 디코딩

    def compressed_depth_callback(self, msg):
        """압축된 Depth 이미지(JPEG 등) 수신 콜백."""
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)  # 16bit/32bit depth

    def detect_and_publish(self):
        """YOLO로 사람 감지, map좌표로 변환, 퍼블리시."""
        # 아래에서 사용할 depth용 카메라 파라미터 선택
        # (예: depth 기준이 stereo라면 self.stereo_K, 아니면 self.rgb_K 사용)
        K = self.stereo_K if self.stereo_K is not None else self.rgb_K
        camera_frame = self.stereo_camera_frame if self.stereo_K is not None else self.rgb_camera_frame

        # 모든 데이터 준비됐을 때만 실행
        if K is None or camera_frame is None or self.rgb_image is None or self.depth_image is None:
            return

        # YOLO 추론 (사람 감지)
        results = self.model(self.rgb_image, verbose=False)[0]
        for det in results.boxes:
            cls = int(det.cls[0])
            label = self.model.names[cls]
            x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())
            if label.lower() == "car":
                u = int((x1 + x2) // 2)
                v = int((y1 + y2) // 2)
                z = float(self.depth_image[v, u])
                if z == 0.0:
                    self.get_logger().warn("Depth value is 0 at detected car's center.")
                    continue
                fx, fy = K[0, 0], K[1, 1]
                cx, cy = K[0, 2], K[1, 2]
                x = (u - cx) * z / fx
                y = (v - cy) * z / fy

                pt = PointStamped()
                pt.header.frame_id = camera_frame
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x, pt.point.y, pt.point.z = x, y, z

                try:
                    pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    pt_map.header.stamp = self.get_clock().now().to_msg()
                    self.point_pub.publish(pt_map)
                    self.get_logger().info(f"Published car at map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")
                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")
                break  # 첫 사람만 처리

def main():
    rclpy.init()
    node = CarDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
