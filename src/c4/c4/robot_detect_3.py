import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
#from builtin_interfaces.msg import Duration
from rclpy.duration import Duration
class YOLODepthDistanceNode(Node):
    def __init__(self):
        super().__init__('yolo_depth_distance_node')

        self.bridge = CvBridge()
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')

        self.K = None
        self.depth_image = None
        self.depth_header = None
        self.start_transform_enabled = False  # ★ 변환 시작 플래그

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot9/oakd/stereo/image_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/camera_info', self.camera_info_callback, 10)

        self.distance_pub = self.create_publisher(String, '/detected_object/distance', 10)
        self.point_pub = self.create_publisher(PointStamped, '/detected_object/point', 10)

        # ★ 5초 후 변환 시작 타이머 설정
        self.get_logger().info("TF Tree 안정화 시작. 5초 후 변환 시작합니다.")
        self.start_timer = self.create_timer(5.0, self.start_transform)

    def start_transform(self):
        self.get_logger().info("TF Tree 안정화 완료. 변환 시작합니다.")
        self.start_transform_enabled = True
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
        if not self.start_transform_enabled:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            rgb_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"RGB image decode error: {e}")
            return

        if self.depth_image is None or self.K is None:
            self.get_logger().warn("Depth image or camera info not ready yet.")
            return

        results = self.model.predict(rgb_img)

        for r in results:
            if r.boxes is None or len(r.boxes) == 0:
                continue

            for box in r.boxes:
                xyxy = box.xyxy.cpu().numpy().astype(int)[0]
                x1, y1, x2, y2 = xyxy
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                if 0 <= cx < self.depth_image.shape[1] and 0 <= cy < self.depth_image.shape[0]:
                    z = float(self.depth_image[cy, cx]) / 1000.0

                    if not np.isfinite(z) or z <= 0:
                        self.get_logger().warn("Invalid depth at object center, skipping")
                        continue

                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx_cam, cy_cam = self.K[0, 2], self.K[1, 2]

                    X = (cx - cx_cam) * z / fx
                    Y = (cy - cy_cam) * z / fy

                    pt = PointStamped()
                    pt.header.frame_id = self.depth_header.frame_id
                    pt.header.stamp = self.depth_header.stamp
                    pt.point.x = X
                    pt.point.y = Y
                    pt.point.z = z

                    try:
                        pt_map = self.tf_buffer.transform(
                            pt, 'map',
                            timeout=Duration(seconds=0.5)
                        )

                        self.point_pub.publish(pt_map)

                        dist_str = f"Object class {cls}, distance to map frame: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f}) m"
                        self.get_logger().info(dist_str)

                        msg = String()
                        msg.data = dist_str
                        self.distance_pub.publish(msg)

                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {e}")

                    class_name = self.model.names[cls] if cls < len(self.model.names) else "Unknown"
                    label = f"{class_name} {conf:.2f} Dist: {z:.2f}m"
                    cv2.rectangle(rgb_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(rgb_img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.circle(rgb_img, (cx, cy), 5, (255, 0, 0), -1)

        # Depth 컬러맵 생성
        depth_vis = np.clip(self.depth_image / 10000.0, 0, 1)
        depth_vis = (depth_vis * 255).astype(np.uint8)
        depth_vis_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 크기 맞추기
        if depth_vis_color.shape != rgb_img.shape:
            depth_vis_color = cv2.resize(depth_vis_color, (rgb_img.shape[1], rgb_img.shape[0]))

        combined = cv2.hconcat([rgb_img, depth_vis_color])
        cv2.imshow("RGB + Depth", combined)
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
