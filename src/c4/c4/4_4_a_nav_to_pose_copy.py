#!/usr/bin/env python3

import rclpy
import threading
import time

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    rclpy.init()

    navigator = TurtleBot4Navigator()

    if not navigator.getDockedStatus():
        navigator.info('초기 위치 설정 전에 도킹 수행 중...')
        navigator.dock()

    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    navigator.waitUntilNav2Active()

    # 목적지 설정
    goal_pose = navigator.getPoseStamped([-2.336, 2.52], 0.055)

    # 도킹 해제
    navigator.undock()

    # ===== 일시정지용 쓰레드 시작 =====
    def cancel_after_delay():
        time.sleep(5)  # 5초 후 탐색 취소
        navigator.info("⚠️ 5초 경과: 탐색 취소 요청 보냄")
        navigator.cancelTask()

    threading.Thread(target=cancel_after_delay).start()
    # ==================================

    # 1차 탐색 (중간에 취소될 예정)
    navigator.info("🏁 목적지 1으로 이동 시작 (1차)")
    navigator.startToPose(goal_pose)

    # 취소 후 잠시 대기
    time.sleep(2)

    # 2차 재탐색
    navigator.info("🔁 목적지 1으로 이동 재시도 (2차)")
    navigator.startToPose(goal_pose)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
