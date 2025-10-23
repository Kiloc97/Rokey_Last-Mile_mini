import time
import math
import os
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage  # 변경된 부분
from cv_bridge import CvBridge
from ultralytics import YOLO
from pathlib import Path
import cv2
import numpy as np  # Compressed 이미지 디코딩용

class YOLOTracker(Node):
    def __init__(self, model):
        super().__init__('yolo_tracker')
        self.model = model
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            CompressedImage,
            '/robot9/oakd/rgb/preview/image_raw/compressed',  # 변경된 부분
            self.listener_callback,
            10)

        self.classNames = model.names if hasattr(model, 'names') else ['Object']
        self.should_shutdown = False

    def listener_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().error(f"Compressed image decoding failed: {e}")
            return

        results = self.model.track(source=img, stream=True, persist=True)

        for r in results:
            if not hasattr(r, 'boxes') or r.boxes is None:
                continue
            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cls = int(box.cls[0]) if box.cls is not None else 0
                conf = float(box.conf[0]) if box.conf is not None else 0.0
                track_id = int(box.id[0]) if box.id is not None else -1

                label = f"{self.classNames[cls]} ID:{track_id} {conf:.2f}"
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                cv2.putText(img, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow("YOLOv8 Tracking", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("Shutdown requested by 'q'.")
            self.should_shutdown = True

def main():
    model_path = '/home/rokey/rokey3_C4_ws/best_3.pt'

    if not os.path.exists(model_path):
        print(f"File not found: {model_path}")
        sys.exit(1)

    model = YOLO(model_path)  # supports tracking with .track()

    rclpy.init()
    node = YOLOTracker(model)

    try:
        while rclpy.ok() and not node.should_shutdown:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print("Interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        print("Shutdown complete.")

if __name__ == '__main__':
    main()
