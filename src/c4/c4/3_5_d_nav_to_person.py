import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from geometry_msgs.msg import PointStamped, PoseStamped
from cv_bridge import CvBridge
import numpy as np
import tf2_ros
from ultralytics import YOLO
import threading
import time
import cv2
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_point


class YoloPersonNavGoal(Node):
    def __init__(self):
        super().__init__('nav_to_person')

        self.bridge = CvBridge()
        self.K = None
        self.depth_image = None
        self.rgb_image = None
        self.camera_frame = None
        self.latest_map_point = None

        self.goal_handle = None
        self.shutdown_requested = False

        self.logged_intrinsics = False
        self.current_distance = None
        self.close_enough_distance = 0.5
        self.block_goal_updates = False
        self.close_distance_hit_count = 0

        # YOLO 모델 로드
        self.model = YOLO("/home/rokey/rokey3_C4_ws/best_3.pt")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # ROS 구독자
        self.create_subscription(CameraInfo, '/robot9/oakd/rgb/preview/camera_info', self.camera_info_callback, 10)
        self.create_subscription(CompressedImage, '/robot9/oakd/rgb/image_raw/compressed', self.rgb_callback, 10)
        self.create_subscription(Image, '/robot9/oakd/stereo/image_raw', self.depth_callback, 10)

        self.display_frame = None

        # 별도 스레드에서 탐지 실행
        self.thread = threading.Thread(target=self.run_detection, daemon=True)
        self.thread.start()

    def camera_info_callback(self, msg):
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, "
                                   f"cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def rgb_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.camera_frame = msg.header.frame_id
            #self.get_logger().info("RGB image received")
        except Exception as e:
            self.get_logger().error(f"Compressed RGB conversion failed: {e}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            #self.get_logger().info("depth received")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def run_detection(self):
        while rclpy.ok() and not self.shutdown_requested:
            if self.K is None or self.rgb_image is None or self.depth_image is None:
                time.sleep(0.1)
                continue

            frame = self.rgb_image.copy()
            results = self.model(frame, verbose=False)[0]

            for det in results.boxes:
                cls = int(det.cls[0])
                label = self.model.names[cls]
                conf = float(det.conf[0])
                x1, y1, x2, y2 = map(int, det.xyxy[0].tolist())

                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, f"{label} {conf:.2f}", (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

                if label.lower() == "car":
                    u = (x1 + x2) // 2
                    v = (y1 + y2) // 2
                    z = float(self.depth_image[v, u])

                    if z == 0.0:
                        self.get_logger().warn("Depth value is 0 at detected person's center.")
                        continue

                    fx, fy = self.K[0, 0], self.K[1, 1]
                    cx, cy = self.K[0, 2], self.K[1, 2]
                    x = (u - cx) * z / fx
                    y = (v - cy) * z / fy

                    pt = PointStamped()
                    pt.header.frame_id = self.camera_frame
                    pt.header.stamp = self.get_clock().now().to_msg()
                    pt.point.x, pt.point.y, pt.point.z = x, y, z

                    try:
                        transform = self.tf_buffer.lookup_transform('map', pt.header.frame_id, rclpy.time.Time())
                        pt_map = do_transform_point(pt, transform)
                        #pt_map = self.tf_buffer.transform(pt, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                        self.latest_map_point = pt_map

                        if self.block_goal_updates:
                            self.get_logger().info(f"Within ({self.close_enough_distance}) meter — skipping goal updates.")
                            break

                        self.get_logger().info(f"Detected person at map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f})")

                        if self.goal_handle:
                            self.get_logger().info("Canceling previous goal...")
                            self.goal_handle.cancel_goal_async()

                        self.send_goal()
                    except Exception as e:
                        self.get_logger().warn(f"TF transform to map failed: {e}")
                    break

            self.display_frame = frame
            #self.get_logger().info('Frame updated for display')

            time.sleep(0.01)

    def send_goal(self):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_map_point.point.x
        pose.pose.position.y = self.latest_map_point.point.y
        pose.pose.orientation.w = 1.0

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.get_logger().info(f"Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        if self.current_distance is not None and self.current_distance < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
            self.block_goal_updates = True
            self.get_logger().info("Confirmed: within close distance — blocking further goal updates.")

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.goal_handle = None

    def show_image(self):
        if self.display_frame is not None:
            #self.get_logger().info('show_image method')
            cv2.imshow("YOLO Detection", self.display_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC key
                self.shutdown_requested = True



def main():
    rclpy.init()
    cv2.startWindowThread()
    node = YoloPersonNavGoal()

    try:
        while rclpy.ok() and not node.shutdown_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.show_image()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_requested = True
        node.thread.join(timeout=1.0)
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
