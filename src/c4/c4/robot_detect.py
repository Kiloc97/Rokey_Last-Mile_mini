import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from ultralytics import YOLO
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs

class YOLODepthDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance_node')
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')

        # 초기화
        self.K = None
        self.depth_image = None
        self.depth_header = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 구독자 설정
        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot9/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

        # 퍼블리셔
        self.distance_pub = self.create_publisher(String, '/detected_object/distance', 10)
        self.get_logger().info("노드 초기화 완료. 데이터 대기 중...")

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth 변환 오류: {e}")

    def rgb_callback(self, msg):
        try:
            # 이미지 디코딩
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB 디코딩 오류: {e}")
            return

        # 데이터 가용성 확인
        if self.depth_image is None or self.K is None:
            self.get_logger().warn("카메라 데이터 미준비", throttle_duration_sec=1)
            return

        # 객체 탐지
        results = self.model.predict(img)

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                # 박스 정보 추출
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                # 중심점 계산
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                # 유효한 깊이 값 확인
                if not (0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]):
                    continue

                z = float(self.depth_image[cy, cx]) / 1000.0
                if z <= 0:
                    continue

                # 3D 좌표 변환
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]
                X = (cx - cx_cam) * z / fx
                Y = (cy - cy_cam) * z / fy

                # TF 변환 준비
                pt = PointStamped()
                pt.header.frame_id = self.depth_header.frame_id
                pt.header.stamp = self.get_clock().now().to_msg()  # 현재 시간 사용
                pt.point.x = X
                pt.point.y = Y
                pt.point.z = z

                # TF 변환 시도
                transform_success = False
                try:
                    pt_map = self.tf_buffer.transform(pt, 'map', 
                            timeout=rclpy.duration.Duration(seconds=0.5))
                    transform_success = True
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}", throttle_duration_sec=1)

                # 데이터 퍼블리시
                label1 = f"{self.model.names[cls]} {conf:.2f} Dist: {z:.2f}m"
                label2 = "Map coordinates: N/A"

                if transform_success:
                    dist_str = f"{self.model.names[cls]}, {pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}, {z:.2f}"
                    self.distance_pub.publish(String(data=dist_str))
                    label2 = f"X:{pt_map.point.x:.2f}, Y:{pt_map.point.y:.2f}, Z:{pt_map.point.z:.2f}"

                # 시각화
                cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                cv2.putText(img, label1, (x1, y1-20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.putText(img, label2, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                cv2.circle(img, (cx, cy), 5, (255,0,0), -1)

        cv2.imshow("YOLO + Depth", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLODepthDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()