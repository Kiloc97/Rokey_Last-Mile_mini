import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import os

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera_image', 10)
        timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(timer_period, self.publish_image)

        self.cap = cv2.VideoCapture('/dev/video0')  # Change if needed
        self.bridge = CvBridge()
        self.image_count = 0
        self.save_directory = "ros2_img_capture"
        os.makedirs(self.save_directory, exist_ok=True)

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open camera')
            raise RuntimeError('Camera not accessible')

        self.get_logger().info('CameraPublisher node started. Press "c" to capture image, "q" to quit.')

    def publish_image(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
            self.get_logger().info('Published image')

            # OpenCV 창 표시 및 키 입력 처리
            cv2.imshow("ROS2 Camera Feed", frame)
            key = cv2.waitKey(1)

            if key == ord('c'):
                file_name = f"{self.save_directory}/ros2_img_{self.image_count}.jpg"
                cv2.imwrite(file_name, frame)
                self.get_logger().info(f"Image saved: {file_name}")
                self.image_count += 1

            elif key == ord('q'):
                self.get_logger().info("Quit key pressed. Shutting down node.")
                rclpy.shutdown()

        else:
            self.get_logger().warn('Failed to capture image')

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()

    try:
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        camera_publisher.get_logger().info("KeyboardInterrupt detected. Shutting down.")
    finally:
        camera_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
