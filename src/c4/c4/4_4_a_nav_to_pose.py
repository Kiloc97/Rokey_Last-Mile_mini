#!/usr/bin/env python3

# Copyright 2022 Clearpath Robotics, Inc.
#
# Apache License 2.0 하에 라이선스됨.
# 이 파일은 해당 라이선스 조건 하에만 사용이 허가됩니다.
# 자세한 내용은 다음 링크 참조:
#     http://www.apache.org/licenses/LICENSE-2.0
#
# 이 소프트웨어는 명시적이거나 묵시적인 보증 없이 "있는 그대로" 제공됩니다.

# @author Roni Kreinin (rkreinin@clearpathrobotics.com)

import rclpy  # ROS 2 Python 클라이언트 라이브러리

# TurtleBot4의 네비게이션 관련 클래스 불러오기
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main():
    # ROS 2 노드 초기화
    rclpy.init()

    # TurtleBot4Navigator 객체 생성 (네비게이션 기능 제공)
    navigator = TurtleBot4Navigator()

    # 현재 도킹 상태 확인. 도킹되어 있지 않으면 도킹부터 수행
    # if not navigator.getDockedStatus():
    #     navigator.info('초기 위치 설정 전에 도킹 수행 중...')
    #     navigator.dock()

    # 초기 위치 설정 (맵 좌표: [0.0, 0.0], 방향: 북쪽)
    # initial_pose = navigator.getPoseStamped([0.0, 0.0], TurtleBot4Directions.NORTH)
    # navigator.setInitialPose(initial_pose)

    # Nav2가 활성화될 때까지 대기 (planner, controller 등 준비될 때까지)
    # navigator.waitUntilNav2Active()

    # 목표 위치 설정
    # 아래는 특정 좌표와 방향(라디안 단위 각도)으로 목표 위치를 설정한 예시입니다.
    # goal_pose = navigator.getPoseStamped([-13.0, 9.0], TurtleBot4Directions.EAST)
    goal_pose = navigator.getPoseStamped([-2.336, 2.362], 0.055)  # yaw = 0.055 라디안

    # 참고용: 실제 pose 정보 예시
    # 위치(Position): (-1.55069, 0.0668084, 0)
    # 방향(Orientation 쿼터니언): (0, 0, -0.962154, 0.272507)
    # -> 이 방향은 약 -2.5896 라디안임

    # 도킹 해제 (언도킹)
    # navigator.undock()

    # 목표 지점까지 단일 이동 수행
    navigator.startToPose(goal_pose)

    # 작업 완료 후 ROS 종료
    rclpy.shutdown()


# 메인 함수 호출
if __name__ == '__main__':
    main()
