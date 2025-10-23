import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os

class ImageCaptureNode(Node):
    def __init__(self, save_directory, file_prefix):
        super().__init__('image_capture_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot9/oakd/rgb/image_raw/compressed',
            self.listener_callback,
            10)
        self.frame = None
        self.save_directory = save_directory
        self.file_prefix = f"{file_prefix}_"
        self.image_count = 0
        os.makedirs(self.save_directory, exist_ok=True)

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn("Failed to decode image.")
            return
        self.frame = frame

def main():
    save_directory = input("Enter directory name to save images: ")
    file_prefix = input("Enter a file prefix to use: ")

    rclpy.init()
    node = ImageCaptureNode(save_directory, file_prefix)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.frame is not None:
                cv2.imshow("Live Feed", node.frame)
                key = cv2.waitKey(1) & 0xFF

                if key == ord('c'):
                    file_name = os.path.join(
                        node.save_directory,
                        f"{node.file_prefix}img_{node.image_count}.jpg"
                    )
                    cv2.imwrite(file_name, node.frame)
                    print(f"Image saved: {file_name}")
                    node.image_count += 1

                elif key == ord('q'):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
