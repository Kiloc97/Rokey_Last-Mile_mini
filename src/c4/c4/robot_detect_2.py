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
        
        # TF 버퍼 캐시 시간 확장 (30초)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')
        self.K = None
        self.depth_image = None
        self.depth_header = None

        # 구독자 설정
        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', 
                               self.rgb_callback, 10)
        self.create_subscription(Image, '/robot9/oakd/stereo/image_raw', 
                               self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', 
                               self.camera_info_callback, 10)

        self.distance_pub = self.create_publisher(String, '/detected_object/distance', 10)
        self.get_logger().info("노드 초기화 완료. AMCL 준비 대기 중...")

        # AMCL 준비 확인 타이머
        self.amcl_ready = False
        self.check_timer = self.create_timer(1.0, self.check_amcl_ready)

    def check_amcl_ready(self):
        try:
            # map → base_link 변환 가능 여부 확인
            if self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                self.get_logger().info("AMCL 준비 완료. 변환 시작 가능")
                self.amcl_ready = True
                self.check_timer.cancel()
        except Exception as e:
            self.get_logger().warn(f"AMCL 확인 오류: {e}")

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Depth 변환 오류: {e}")

    def transform_point(self, pt, target_frame, max_attempts=3):
        for attempt in range(max_attempts):
            try:
                # 깊이 이미지의 타임스탬프 사용
                transform_time = self.depth_header.stamp
                return self.tf_buffer.transform(pt, target_frame, 
                                             timeout=rclpy.duration.Duration(seconds=0.5))
            except tf2_ros.ExtrapolationException as e:
                if attempt == max_attempts - 1:
                    raise
                wait_time = 0.1 * (attempt + 1)
                time.sleep(wait_time)
            except Exception as e:
                raise
        return None

    def rgb_callback(self, msg):
        if not self.amcl_ready:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB 디코딩 오류: {e}")
            return

        if self.depth_image is None or self.K is None:
            self.get_logger().warn("카메라 데이터 미준비", throttle_duration_sec=1)
            return

        results = self.model.predict(img)

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                if not (0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]):
                    continue

                z = float(self.depth_image[cy, cx]) / 1000.0
                if z <= 0:
                    continue

                fx, fy = self.K[0, 0], self.K[1, 1]
                cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]
                X = (cx - cx_cam) * z / fx
                Y = (cy - cy_cam) * z / fy

                pt = PointStamped()
                pt.header.frame_id = self.depth_header.frame_id
                pt.header.stamp = self.depth_header.stamp  # 깊이 이미지 타임스탬프 사용
                pt.point.x = X
                pt.point.y = Y
                pt.point.z = z

                transform_success = False
                try:
                    pt_map = self.transform_point(pt, 'map')
                    transform_success = True
                except Exception as e:
                    self.get_logger().warn(f"TF 변환 실패: {e}", throttle_duration_sec=1)

                label1 = f"{self.model.names[cls]} {conf:.2f} Dist: {z:.2f}m"
                label2 = "Map coordinates: N/A"

                if transform_success:
                    dist_str = f"{self.model.names[cls]}, {pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}, {z:.2f}"
                    self.distance_pub.publish(String(data=dist_str))
                    label2 = f"X:{pt_map.point.x:.2f}, Y:{pt_map.point.y:.2f}, Z:{pt_map.point.z:.2f}"

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