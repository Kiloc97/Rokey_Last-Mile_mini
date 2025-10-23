import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.subscription = self.create_subscription(
            Image,
            '/robot9/oakd/stereo/image_raw',
            self.depth_callback,
            qos_profile)

    def depth_callback(self, msg):
        self.get_logger().info("depth callback")

        try:
            # dtype 추론
            dtype = np.uint16 if msg.encoding == '16UC1' else np.uint8

            # 이미지 크기와 dtype으로 재구성
            depth_image = np.frombuffer(msg.data, dtype=dtype).reshape((msg.height, msg.width))

            height, width = depth_image.shape
            u, v = width // 2, height // 2
            distance_raw = depth_image[v, u]

            if depth_image.dtype == np.uint16:
                distance = distance_raw / 1000.0  # mm → m
            else:
                distance = float(distance_raw)

            self.get_logger().info(
                f"Image size: {width}x{height}, "
                f"Distance at center ({u},{v}) = {distance:.2f} meters"
            )
        except Exception as e:
            self.get_logger().error(f"Depth 처리 중 오류: {str(e)}")

def main():
    rclpy.init()
    node = DepthChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
