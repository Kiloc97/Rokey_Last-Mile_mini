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
import time
class YOLODepthDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance_node')

        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')  # 모델 경로 수정

        self.K = None  # 카메라 내부 파라미터

        self.depth_image = None
        self.depth_header = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 구독자
        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot9/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)
    
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(10.0, self.start_transform)
        # 거리 알림 퍼블리셔 예시
        self.distance_pub = self.create_publisher(String, '/detected_object/distance', 10)
    
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        self.start_timer.cancel()
    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth image conversion error: {e}")

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB image decode error: {e}")
            return

        if self.depth_image is None or self.K is None:
            self.get_logger().warn("Depth image or camera info not ready yet.")
            return

        # YOLO 객체 탐지
        results = self.model.predict(img)

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 깊이 값 추출 (mm → m)
                if 0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]:
                    z = float(self.depth_image[cy, cx]) / 1000.0
                    if z == 0:
                        self.get_logger().warn("Depth is zero at object center, skipping")
                        continue

                    # 픽셀 → 카메라 좌표계 3D 변환
                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]

                    X = (cx - cx_cam) * z / fx
                    Y = (cy - cy_cam) * z / fy

                    pt = PointStamped()
                    pt.header.frame_id = self.depth_header.frame_id
                    pt.header.stamp = rclpy.time.Time().to_msg()
                    pt.point.x = X
                    pt.point.y = Y
                    pt.point.z = z

                    # map 좌표계로 변환 시도
                    try:
                        pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                        dist_str = f"{cls}, {pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}, {z:.2f}"
                                    # class, map_x, map_y, map_z, z
                        self.get_logger().info(dist_str)

                        # 거리 퍼블리시
                        msg = String()
                        msg.data = dist_str
                        self.distance_pub.publish(msg)
                        # topic/msg    /detected_object/distance
                        # class, map_x, map_y, map_z, distance
                        # green, 1.34, -0.34, 1.02, 1.23
                        # car, 1.23, -0.45, 0.98, 0.98
                        # red, 2.34, -1.10, 0.95, 0.95
                        # dummy, 0.67,  0.12, 1.05, 1.05
                        
                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {e}")

                    # 이미지에 표시
                    label1 = f"{self.model.names[cls]} {conf:.2f} Dist: {z:.2f}m"
                    label2 = f"X:{pt_map.point.x:.2f}, Y:{pt_map.point.y:.2f}, Z:{pt_map.point.z:.2f}"

                    cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(img, label1, (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.putText(img, label2, (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.circle(img, (cx, cy), 5, (255,0,0), -1)


        cv2.imshow("YOLO + Depth Distance", img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = YOLODepthDistanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
