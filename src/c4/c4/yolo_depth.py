import rclpy  # ROS 2 Python 클라이언트 라이브러리
from rclpy.node import Node  # ROS 노드 클래스

from sensor_msgs.msg import Image,CompressedImage,CameraInfo  # RGB, Depth 이미지 메시지 타입
from std_msgs.msg import Header    # 메시지의 시간/프레임 정보
from cv_bridge import CvBridge     # ROS <-> OpenCV 이미지 변환
import cv2                         # OpenCV 라이브러리
import numpy as np                # 배열 처리용

from custom_msgs.msg import DetectedObject, DetectedObjectArray  # 사용자 정의 메시지
from ultralytics import YOLO  # YOLOv8 모델 사용

from geometry_msgs.msg import PointStamped, PoseStamped
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions
import tf2_ros
import tf2_geometry_msgs  # TF 좌표계 변환을 위한 메시지 타입 변환 지원


class YoloObjectDetector(Node):
    def __init__(self):
        super().__init__('yolo_object_detector')  # 노드 이름 초기화
        self.bridge = CvBridge()  # OpenCV <-> ROS 이미지 변환기 생성

        # 사전 학습된 YOLO 모델 로드 (YOLOv8 포맷의 .pt 파일)
        self.model = YOLO('/home/rokey/rokey3_C4_ws/best_3.pt')
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/robot9/oakd/rgb/camera_info', 
            self.camera_info_callback, 
            1
            )
        
        # RGB 카메라 이미지 구독 (압축된 이미지 사용)
        self.rgb_sub = self.create_subscription(
            CompressedImage,
            '/robot9/oakd/rgb/image_raw/compressed',
            self.rgb_callback,
            1
        )

        # 깊이 카메라 이미지 구독
        self.depth_sub = self.create_subscription(
            Image,
            '/robot9/oakd/stereo/image_raw',
            self.depth_callback,
            1
        )

        # 객체 정보 퍼블리셔 설정
        self.publisher = self.create_publisher(
            DetectedObjectArray,
            '/detected_objects',
            10
        )

        # 이미지 저장용 변수 초기화
        self.rgb_image = None
        self.depth_image = None
        self.depth_header = None
        self.K = None

        # TF 좌표계 변환을 위한 Buffer 및 Listener 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.logged_intrinsics = False  # Intrinsics 로그 중복 방지

        # 주기적으로 process_image 함수 실행 (0.1초 간격)
        self.create_timer(0.1, self.process_image)

    def camera_info_callback(self, msg):
        # 카메라 내부 파라미터(K 행렬) 추출 및 저장
        self.K = np.array(msg.k).reshape(3, 3)
        if not self.logged_intrinsics:
            self.get_logger().info(f"Camera intrinsics received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")
            self.logged_intrinsics = True

    def depth_callback(self, msg):
        """깊이 이미지 수신 콜백 함수"""
        try:
            # depth 이미지: 16비트 정수 형식으로 변환
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_header = msg.header  # 타임스탬프 유지
            self.get_logger().info("Depth image received.")
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def rgb_callback(self, msg):
        """RGB 이미지 수신 콜백 함수"""
        try:
            # 압축된 이미지 데이터를 디코딩하여 OpenCV 이미지로 변환

            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.get_logger().info("RBG image received.")
            if self.rgb_image is None:
                self.get_logger().error("Failed to decode compressed RGB image.")
            else:
                self.get_logger().info(f"RGB image shape: {self.rgb_image.shape}, dtype: {self.rgb_image.dtype}, mean pixel: {np.mean(self.rgb_image):.2f}")

        except Exception as e:
            self.get_logger().error(f"RGB decode failed: {e}")


        """RGB + Depth 이미지 기반으로 객체 탐지 및 퍼블리시"""
    def process_image(self):
        if self.rgb_image is None or self.depth_image is None:
            self.get_logger().warn("RGB, Depth not ready. Skipping frame.")
            return

        if self.K is None or self.depth_header is None:
            self.get_logger().warn("CameraInfo not ready. Skipping frame.")
            return  # 둘 다 준비되지 않으면 처리하지 않음
        # RGB 이미지 복사본 생성 (원본 보호용)
        #rgb_display = self.rgb_image.copy()
        # 깊이 이미지 복사본 생성
        depth_display = self.depth_image.copy()

        # YOLOv8 객체 탐지 수행
        input_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB)
        results = self.model.predict(input_image)

        # 사용자 정의 메시지 배열 생성
        object_array = DetectedObjectArray()
        object_array.header = self.depth_header  # 프레임 정보 유지

        # 원본 이미지 복사 (디버깅용 표시)
        rgb_display = self.rgb_image.copy()

        # 정렬 기준 정의
        desired_order = ["car", "red", "green", "dummy"]

        # YOLO 결과 순회 → 리스트에 먼저 담기
        detected_list = []

        for result in results:
            for box in result.boxes:
                class_id = int(box.cls) if not isinstance(box.cls, (list, np.ndarray)) else int(box.cls[0])
                class_name = self.model.names[class_id]
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                x_center = int((x1 + x2) / 2)
                y_center = int((y1 + y2) / 2)

                try:
                    h, w = self.depth_image.shape
                    if 0 <= x_center < w and 0 <= y_center < h:
                        distance = float(self.depth_image[y_center, x_center]) / 1000.0
                    else:
                        distance = None
                except:
                    distance = None

                
     
                obj = DetectedObject()
                obj.class_name = class_name
                obj.x_center = x_center
                obj.y_center = y_center
                obj.distance = distance
                if self.K is None or self.depth_header is None:
                    self.get_logger().warn("Camera intrinsics or depth header not ready")
                detected_list.append((obj, (x1, y1, x2, y2)))  # 시각화용 박스도 저장

######################

                x = x_center
                y = y_center
                z = distance  # 이미 m 단위라고 가정

                # X, Y 계산 (핀홀 카메라 모델)
                fx, fy = self.K[0, 0], self.K[1, 1]
                cx, cy = self.K[0, 2], self.K[1, 2]
                X = (x - cx) * z / fx
                Y = (y - cy) * z / fy

                obj.header.frame_id = self.depth_header.frame_id
                #pt.header.stamp = rclpy.time.Time().to_msg()
                obj.header.stamp = self.depth_header.stamp

                obj.point.x = X
                obj.point.y = Y
                obj.point.z = z
                # TF 변환 시도: 카메라 좌표 → 지도(map) 좌표계 변환
                try:
                    pt_map = self.tf_buffer.transform(obj, 'map', timeout=rclpy.duration.Duration(seconds=0.5))
                    obj.map_x = pt_map.point.x
                    obj.map_y = pt_map.point.y
                    obj.map_z = pt_map.point.z
                    obj.frame_id = pt_map.header.frame_id
                    obj.stamp = pt_map.header.stamp
                    self.get_logger().info(f"map: ({pt_map.point.x:.2f}, {pt_map.point.y:.2f}, {pt_map.point.z:.2f})")
                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {e}")
                    obj.map_x = obj.map_y = obj.map_z = float('nan')  # NaN 값으로 지정
                    obj.frame_id = obj.header.frame_id
                    obj.stamp = obj.header.stamp


        # 클래스 이름 기준 정렬
        sorted_objects = sorted(
            detected_list,
            key=lambda item: desired_order.index(item[0].class_name) if item[0].class_name in desired_order else 99
        )

        # 정렬된 순서대로 메시지와 시각화 처리
        for obj, (x1, y1, x2, y2) in sorted_objects:
            object_array.objects.append(obj)
            label = f"{obj.class_name} ({obj.distance:.2f}m)"
            cv2.rectangle(rgb_display, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(rgb_display, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        # 객체 탐지 결과 퍼블리시
        self.publisher.publish(object_array)
        
        # 퍼블리시 메시지 예시
        # /detected_objects
        # header:
        #     stamp:
        #     frame_id:
        # objects:
        #     - class_name: "car"
        #     x_center: 320
        #     y_center: 240
        #     distance: 2.45
        #     - class_name: "red"
        #     x_center: 220
        #     y_center: 200
        #     distance: 1.35

        # 결과 화면 출력
        if len(depth_display.shape) == 2:
            depth_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)

        combined = np.hstack((rgb_display, depth_display))
        cv2.imshow('RGB (left) | Depth (right)', combined)  # 창에 이미지 출력
        cv2.waitKey(10)


def main(args=None):
    rclpy.init(args=args)        # ROS 노드 초기화
    node = YoloObjectDetector()  # 노드 실행
    rclpy.spin(node)             # 콜백 무한 대기
    node.destroy_node()          # 종료 시 노드 제거
    rclpy.shutdown()             # ROS 종료
    cv2.destroyAllWindows()      # OpenCV 창 닫기
