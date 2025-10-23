import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateToPose
import time

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator

class PersonFollowerNavNode(Node):
    def __init__(self):
        super().__init__('person_follower_nav_node')

        # Nav2 action client 생성 (navigate_to_pose 사용)
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 이동 좌표 토픽 구독 (탐지 노드가 보낸 목표 좌표 수신)
        self.create_subscription(PointStamped, '/person_goal_point', self.goal_callback, 10)

        self.latest_goal = None           # 최근 받은 목표 좌표
        self.goal_handle = None           # 현재 이동중인 goal handle
        self.block_goal_updates = False   # 목표 갱신 차단 (로봇이 목표점에 가까우면 True)
        self.close_enough_distance = 2.0  # 몇 m 이내면 도착으로 간주
        self.current_distance = None      # 현재 목표까지 남은 거리
        self.close_distance_hit_count = 0 # 연속 근접 횟수
        self.last_feedback_log_time = 0   # 피드백 로그 출력 타이밍 관리

        # TurtleBot4Navigator 객체 (도킹/언도킹, pose 제어)
        self.navigator = TurtleBot4Navigator()

        # === [중요] 도킹/언도킹 위치를 초기 pose로 Nav2에 등록 ===
        self.set_initial_pose_to_dock_position()

        # Nav2 활성화까지 대기 (localization, 경로계획 등 준비)
        self.navigator.waitUntilNav2Active()

    def set_initial_pose_to_dock_position(self):
        """
        도킹 또는 언도킹 위치를 Nav2의 초기 pose로 설정
        """
        # 1. 도킹 상태가 아니면 도킹 먼저 시도
        if not self.navigator.getDockedStatus():
            self.get_logger().info('[InitPose] Docking before setting initial pose...')
            self.navigator.dock()
            # 실제 도킹 동작이 끝날 때까지 잠시 대기
            time.sleep(2.0)

        # 2. 현재 위치/자세를 읽어와서 초기 pose로 변환
        current_pose = self.navigator.getPose()
        # getPoseStamped([x, y], 방향/라디안)으로 PoseStamped 생성
        pose_stamped = self.navigator.getPoseStamped(
            [current_pose.position.x, current_pose.position.y],
            TurtleBot4Directions.NORTH  # 도킹 방향 (필요하면 라디안값 사용)
        )

        # 3. Nav2에 초기 pose 세팅 (localization)
        self.navigator.setInitialPose(pose_stamped)
        self.get_logger().info(f'[InitPose] Initial pose set to dock position: ({current_pose.position.x:.2f}, {current_pose.position.y:.2f})')

        # 4. 언도킹 (이제 이동 준비 완료)
        self.navigator.undock()
        self.get_logger().info('[InitPose] Undocked and ready to navigate!')

    def goal_callback(self, msg):
        """
        /person_goal_point로부터 좌표를 수신하면 Nav2 goal로 전송
        """
        if self.block_goal_updates:
            self.get_logger().info("Goal updates blocked (robot is close enough to goal).")
            return

        self.latest_goal = msg
        self.send_goal()

    def send_goal(self):
        """
        저장된 latest_goal 좌표를 NAV2 goal로 전송
        """
        if self.latest_goal is None:
            return

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.latest_goal.point.x
        pose.pose.position.y = self.latest_goal.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 단순히 정면 방향

        self.get_logger().info(f"Sending goal: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
        self.action_client.wait_for_server()
        # goal 메시지 생성 및 NAV2에 비동기 전송
        self._send_goal_future = self.action_client.send_goal_async(
            goal := NavigateToPose.Goal(pose=pose),
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """
        NAV2에서 목표 이동 중 피드백(남은 거리 등) 수신 콜백
        """
        feedback = feedback_msg.feedback
        self.current_distance = feedback.distance_remaining

        # 연속 3회 이상 가까우면(2m 이내) 추가 목표 차단
        if self.current_distance is not None and self.current_distance < self.close_enough_distance:
            self.close_distance_hit_count += 1
        else:
            self.close_distance_hit_count = 0

        if self.close_distance_hit_count >= 3 and not self.block_goal_updates:
            self.block_goal_updates = True
            self.get_logger().info("Confirmed: within 2 meters — blocking further goal updates.")

        # 1초에 한 번 남은 거리 로그 출력
        now = time.time()
        if now - self.last_feedback_log_time > 1.0:
            self.get_logger().info(f"Distance remaining: {self.current_distance:.2f} m")
            self.last_feedback_log_time = now

    def goal_response_callback(self, future):
        """
        NAV2로 goal 보낸 후 응답 처리
        """
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().warn("Goal was rejected.")
            return
        self.get_logger().info("Goal accepted.")
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """
        이동 목표 완료 콜백
        """
        result = future.result().result
        self.get_logger().info(f"Goal finished with result code: {future.result().status}")
        self.goal_handle = None

def main():
    rclpy.init()
    node = PersonFollowerNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
