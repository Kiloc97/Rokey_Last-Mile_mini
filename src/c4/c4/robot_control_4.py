import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import numpy as np
import cv2
import tf2_ros
from message_filters import Subscriber, ApproximateTimeSynchronizer

class YOLODepthPublisher(Node):
    def __init__(self):
        super().__init__('yolo_depth_publisher')

        # YOLO 모델 로드
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')
        self.bridge = CvBridge()
        self.K = None

        # TF Buffer/Listener 생성
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 파라미터 구독
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

        # RGB/Depth 이미지 동기화 구독
        self.rgb_sub = Subscriber(self, CompressedImage, '/robot9/oakd/rgb/image_raw/compressed')
        self.depth_sub = Subscriber(self, Image, '/robot9/oakd/stereo/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        # 3D 좌표 퍼블리셔 (PointStamped)
        self.distance_pub = self.create_publisher(PointStamped, '/detected_object/distance', 10)

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def synced_callback(self, rgb_msg, depth_msg):
        # RGB 디코딩
        try:
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_header = depth_msg.header
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        if self.K is None:
            self.get_logger().warn("Camera intrinsic not received yet.")
            return

        # YOLO 추론
        results = self.model.predict(img)
        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                # 탐지 신뢰도 필터 (예: 0.5 이상만)
                if conf < 0.5:
                    continue

                # 객체 중심 픽셀
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]:
                    z = float(depth_image[cy, cx]) / 1000.0  # mm→m
                    if z == 0:
                        self.get_logger().warn("Depth is zero at object center, skipping")
                        continue

                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]
                    X = (cx - cx_cam) * z / fx
                    Y = (cy - cy_cam) * z / fy

                    # PointStamped 메시지 생성
                    pt = PointStamped()
                    pt.header.frame_id = depth_header.frame_id
                    pt.header.stamp = rgb_msg.header.stamp  # 이미지의 타임스탬프 사용
                    pt.point.x = X
                    pt.point.y = Y
                    pt.point.z = z

                    try:
                        pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                        self.get_logger().info(f"Publishing object @ ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
                        self.distance_pub.publish(pt_map)
                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {e}")

        # (디버깅용, 결과 영상 보기)
        # cv2.imshow("YOLO+Depth", img)
        # cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLODepthPublisher()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    # cv2.destroyAllWindows()  # 만약 영상 디버깅 했을 때만 필요
    rclpy.shutdown()

if __name__ == '__main__':
    main()
