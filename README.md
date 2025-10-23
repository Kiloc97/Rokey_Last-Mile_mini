# 🤖 Rokey Last-Mile Mini - TurtleBot4 ROS2 프로젝트

TurtleBot4 로봇을 활용한 **컴퓨터 비전**, **자율 주행**, **객체 탐지** 기능을 구현한 ROS2 기반 프로젝트입니다.

## 📖 프로젝트 개요

이 프로젝트는 TurtleBot4 로봇에서 다음과 같은 기능들을 구현합니다:

- **🎥 실시간 영상 처리**: RGB/Depth 카메라 데이터 수집 및 처리
- **🔍 YOLO 객체 탐지**: YOLOv8을 활용한 실시간 객체 인식
- **📏 3D 좌표 변환**: Depth 정보를 활용한 3차원 좌표 계산
- **🗺️ 자율 주행**: Nav2를 활용한 자율 네비게이션
- **👤 사람 추적**: 객체 탐지와 네비게이션을 결합한 사람 추적
- **🚗 차량 탐지**: 차량 객체 탐지 및 추적
- **📦 배송 시뮬레이션**: 웨이포인트 기반 배송 루트 실행

## 🏗️ 프로젝트 구조

```
Rokey_Last-Mile_mini/
├── src/
│   ├── c4/                    # 메인 ROS2 패키지
│   │   ├── c4/                # Python 모듈
│   │   │   ├── 2_x_*.py      # 카메라 & 영상 처리
│   │   │   ├── 3_x_*.py      # 객체 탐지 & 3D 좌표
│   │   │   ├── 4_x_*.py      # 네비게이션 & 주행
│   │   │   ├── car_*.py      # 차량 탐지 관련
│   │   │   ├── robot_*.py    # 로봇 제어 관련
│   │   │   └── yolo_*.py     # YOLO 모델 관련
│   │   ├── package.xml       # ROS2 패키지 설정
│   │   └── setup.py          # Python 패키지 설정
│   ├── custom_msgs/          # 커스텀 ROS2 메시지
│   └── my_package/           # 추가 패키지
├── .gitignore               # Git 무시 파일 목록
└── README.md               # 프로젝트 문서
```

## 🚀 주요 기능

### 1. 카메라 & 영상 처리 (2_x 시리즈)

- **이미지 캡처**: 웹캠 및 로봇 카메라에서 이미지 수집
- **ROS2 Publisher/Subscriber**: 이미지 데이터 실시간 전송
- **데이터 관리**: 이미지 저장 및 라벨링 데이터 처리

### 2. 객체 탐지 & 3D 변환 (3_x 시리즈)

- **YOLOv8 객체 탐지**: 실시간 객체 인식 및 바운딩 박스 생성
- **Depth 기반 3D 좌표 계산**: 카메라 좌표계를 맵 좌표계로 변환
- **사람 추적**: 탐지된 사람 객체를 추적하여 네비게이션 목표 설정

### 3. 자율 주행 & 네비게이션 (4_x 시리즈)

- **경로 계획**: 목표 지점까지의 최적 경로 생성
- **웨이포인트 주행**: 다중 목표점을 순차적으로 방문
- **장애물 회피**: Nav2를 활용한 동적 장애물 회피
- **배송 시뮬레이션**: 실제 배송 시나리오 구현

### 4. 특수 기능

- **차량 탐지 시스템**: 주차된 차량이나 이동 차량 탐지
- **보안 알림**: 특정 영역 침입 시 알림 시스템
- **음성 알림**: 상황별 음성 피드백

## 🛠️ 설치 및 설정

### 필수 요구사항

- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.8+
- **TurtleBot4**: 하드웨어 및 소프트웨어 패키지

### 종속성 설치

```bash
# ROS2 Humble 설치 (Ubuntu 22.04)
sudo apt update
sudo apt install ros-humble-desktop-full

# TurtleBot4 패키지 설치
sudo apt install ros-humble-turtlebot4-desktop

# Python 패키지 설치
pip install ultralytics opencv-python numpy
```

### 프로젝트 빌드

```bash
# 워크스페이스 생성 및 이동
mkdir -p ~/turtlebot4_ws/src
cd ~/turtlebot4_ws/src

# 프로젝트 클론
git clone https://github.com/Kiloc97/Rokey_Last-Mile_mini.git

# 빌드
cd ~/turtlebot4_ws
colcon build --packages-select c4

# 환경 설정
source install/setup.bash
```

## 🎮 사용법

### 1. 기본 실행

```bash
# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/turtlebot4_ws/install/setup.bash

# 로봇 기본 시스템 시작
ros2 launch turtlebot4_navigation localization.launch.py
ros2 launch turtlebot4_navigation nav2.launch.py
```

### 2. 객체 탐지 실행

```bash
# YOLO 객체 탐지 시작
ros2 run c4 robot_detect

# 실시간 카메라 피드
ros2 run c4 pub_image
ros2 run c4 show_image
```

### 3. 자율 주행 실행

```bash
# 기본 네비게이션
ros2 run c4 nav_to_pose

# 사람 추적 모드
ros2 run c4 nav_to_person

# 배송 시뮬레이션
ros2 run c4 mail_delivery
```

### 4. 차량 탐지 시스템

```bash
# 차량 탐지 노드 실행
ros2 run c4 car_detector_node

# 차량 추적 및 네비게이션
ros2 run c4 car_follower_nav_node
```

## 📊 주요 노드 및 토픽

### 주요 노드

- `robot_detect`: YOLO 기반 객체 탐지
- `robot_control`: 로봇 제어 및 상태 관리
- `car_detector_node`: 차량 탐지 전용 노드
- `depth_to_3d`: Depth 정보를 3D 좌표로 변환

### 주요 토픽

- `/camera/image_raw`: 카메라 이미지 스트림
- `/camera/depth/image_raw`: Depth 이미지 스트림
- `/car_label`: 탐지된 차량 정보
- `/goal_pose`: 네비게이션 목표 좌표
- `/cmd_vel`: 로봇 속도 제어

## 🔧 설정 및 커스터마이징

### YOLO 모델 설정

```python
# robot_detect.py에서 모델 경로 변경
self.model = YOLO("path/to/your/model.pt")
```

### 네비게이션 파라미터 조정

```python
# robot_control.py에서 목표 지점 설정
self.goal_pose = [
    navigator.getPoseStamped([x, y], TurtleBot4Directions.NORTH)
]
```