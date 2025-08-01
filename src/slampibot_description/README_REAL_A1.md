# 실제 환경에서 RPLidar A1 사용하기

이 문서는 실제 환경에서 RPLidar A1 라이다를 사용하기 위한 설정과 실행 방법을 설명합니다.

## 파일 구조

- `urdf/spb_urdf_a1_lidar.xacro`: A1 라이다를 위한 xacro 파일
- `urdf/spb_urdf_real_a1.urdf`: 실제 환경용 간단한 로봇 URDF (base_link + A1 라이다)
- `launch/spb_real_a1_launch.py`: RViz와 함께 실행하는 launch 파일
- `launch/spb_real_a1_simple_launch.py`: RViz 없이 실행하는 간단한 launch 파일

## 하드웨어 요구사항

- RPLidar A1 센서
- USB 연결 (기본 포트: `/dev/ttyUSB0`)
- Linux 시스템

## 설치 및 설정

### 1. USB 권한 설정

라이다에 접근할 수 있도록 USB 권한을 설정합니다:

```bash
# udev 규칙 생성
sudo cp src/slampibot_gazebo/src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 2. USB 포트 확인

라이다가 연결된 USB 포트를 확인합니다:

```bash
ls -l /dev/ttyUSB*
```

## 실행 방법

### 방법 1: RViz와 함께 실행 (시각화 포함)

```bash
ros2 launch slampibot_description spb_real_a1_launch.py
```

### 방법 2: 간단한 실행 (RViz 없음)

```bash
ros2 launch slampibot_description spb_real_a1_simple_launch.py
```

### 방법 3: 커스텀 포트로 실행

```bash
ros2 launch slampibot_description spb_real_a1_simple_launch.py serial_port:=/dev/ttyUSB1
```

## 파라미터 설정

launch 파일에서 다음 파라미터들을 조정할 수 있습니다:

- `serial_port`: USB 포트 (기본값: `/dev/ttyUSB0`)
- `serial_baudrate`: 통신 속도 (기본값: `115200`)
- `frame_id`: 라이다 데이터의 프레임 ID (기본값: `laser`)

## 토픽 확인

실행 후 다음 토픽들이 생성됩니다:

```bash
# 라이다 스캔 데이터 확인
ros2 topic echo /scan

# TF 트리 확인
ros2 run tf2_tools view_frames

# 라이다 정보 확인
ros2 topic echo /sllidar_node/device_info
```

## 문제 해결

### 1. 권한 오류
```bash
sudo chmod 666 /dev/ttyUSB0
```

### 2. 포트를 찾을 수 없는 경우
```bash
# USB 장치 목록 확인
lsusb
# 시리얼 포트 확인
dmesg | grep tty
```

### 3. 라이다가 응답하지 않는 경우
- USB 케이블 연결 상태 확인
- 다른 USB 포트 시도
- 라이다 전원 상태 확인

## URDF 구조

생성된 URDF는 다음과 같은 구조를 가집니다:

```
base_link
└── lidar_a1_joint
    └── lidar_a1_link
        └── laser_joint
            └── laser (센서 데이터 프레임)
```

## 라이다 사양

- **모델**: RPLidar A1
- **직경**: 115mm
- **높이**: 40mm
- **무게**: 190g
- **스캔 범위**: 360도
- **스캔 주파수**: 5.5Hz
- **거리 범위**: 0.15m ~ 12m
- **해상도**: 0.5도 