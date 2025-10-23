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
from message_filters import Subscriber, ApproximateTimeSynchronizer
import time

class YOLODepthDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance_node')

        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')

        self.K = None  # 카메라 내부 파라미터

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 카메라 정보 구독
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

        # 이미지 동기화
        self.rgb_sub = Subscriber(self, CompressedImage, '/robot9/oakd/rgb/image_raw/compressed')
        self.depth_sub = Subscriber(self, Image, '/robot9/oakd/stereo/image_raw')
        self.ts = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        #time.sleep(5)
        self.start_timer = self.create_timer(5.0, self.start_transform)
        # 거리 알림 퍼블리셔 예시
        self.distance_pub = self.create_publisher(String, '/detected_object/distance', 10)
    
    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")

        self.start_timer.cancel()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)

    def synced_callback(self, rgb_msg, depth_msg):
        try:
            # RGB 디코딩
            np_arr = np.frombuffer(rgb_msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Depth 이미지
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            depth_header = depth_msg.header

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")
            return

        if self.K is None:
            self.get_logger().warn("Camera intrinsic not received yet.")
            return

        results = self.model.predict(img)

        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                cx = float((x1 + x2) / 2)
                cy = float((y1 + y2) / 2)

                if 0 <= cx < depth_image.shape[1] and 0 <= cy < depth_image.shape[0]:
                    z = float(depth_image[cy, cx]) / 1000.0
                    if z == 0:
                        self.get_logger().warn("Depth is zero at object center, skipping")
                        continue

                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]
                    X = (cx - cx_cam) * z / fx
                    Y = (cy - cy_cam) * z / fy

                    pt = PointStamped()
                    pt.header.frame_id = depth_header.frame_id
                    pt.header.stamp = rclpy.time.Time().to_msg() #depth_header.stamp
                    pt.point.x = X
                    pt.point.y = Y
                    pt.point.z = z

                    try:
                        pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                        dist_str = f"Object class {cls}, distance to map frame: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}) m"
                        self.get_logger().info(dist_str)

                        msg = String()
                        msg.data = dist_str
                        self.distance_pub.publish(msg)

                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {e}")

                    label = f"{self.model.names[cls]} {conf:.2f} Dist: {z:.2f}m,X: {X:.2f}m,Y: {Y:.2f}m"
                    cv2.rectangle(img, (x1, y1), (x2, y2), (0,255,0), 2)
                    cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
                    cv2.circle(img, (cx, cy), 5, (255,0,0), -1)

        cv2.imshow("YOLO + Depth Distance", img)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YOLODepthDistanceNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
