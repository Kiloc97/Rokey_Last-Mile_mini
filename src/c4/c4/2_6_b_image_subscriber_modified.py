import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import os

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot9/oakd/rgb/image_raw/compressed',
            self.image_callback,
            10
        )
        self.img_count = 0
        self.latest_frame = None
        self.get_logger().info('CameraSubscriber node initialized.')

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            self.get_logger().warn('Failed to decode image.')
            return

        self.latest_frame = frame
        cv2.imshow('Received Image - Press "c" to save, "q" to quit', frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rclpy.shutdown()
        elif key == ord('c'):
            self.save_image()

    def save_image(self):
        if self.latest_frame is not None:
            filename = f'image_{self.img_count:04d}.jpg'
            cv2.imwrite(filename, self.latest_frame)
            self.get_logger().info(f'Saved image to {filename}')
            self.img_count += 1

def main(args=None):
    rclpy.init(args=args)
    camera_subscriber = CameraSubscriber()

    try:
        rclpy.spin(camera_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
