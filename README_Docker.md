# ROS2 Slampibot Docker 실행 가이드

## 라즈베리파이5에서 Docker를 사용하여 ROS2 Slampibot 실행하기

### 사전 요구사항

1. **라즈베리파이5에 Docker 설치**
```bash
# Docker 설치
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# 현재 사용자를 docker 그룹에 추가
sudo usermod -a -G docker $USER

# Docker Compose 설치
sudo apt-get install docker-compose-plugin
```

2. **USB 장치 권한 설정**
```bash
# 사용자를 dialout 그룹에 추가
sudo usermod -a -G dialout $USER

# udev 규칙 설정
sudo cp src/sllidar_ros2/scripts/rplidar.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 사용법

#### 1. 이미지 빌드 및 실행
```bash
# Docker 이미지 빌드
docker-compose build

# 서비스 실행 (launch 파일 자동 실행)
docker-compose up

# 백그라운드 실행
docker-compose up -d
```

#### 2. 개발 모드로 실행
```bash
# 개발용 컨테이너 실행 (launch 파일 실행하지 않음)
docker-compose --profile dev up

# 컨테이너에 접속
docker-compose --profile dev exec ros2_slampibot_dev bash
```

#### 3. 수동으로 launch 파일 실행
```bash
# 컨테이너에 접속
docker-compose exec ros2_slampibot bash

# 컨테이너 내부에서 launch 파일 실행
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch slampibot_description spb_real_a1_launch.py
```

### 서비스 중지
```bash
# 서비스 중지
docker-compose down

# 이미지까지 삭제
docker-compose down --rmi all
```

### 문제 해결

#### 1. USB 장치 접근 문제
```bash
# USB 장치 권한 확인
ls -la /dev/ttyUSB*

# 권한이 666이 아니면 수동으로 변경
sudo chmod 666 /dev/ttyUSB0
```

#### 2. GUI 표시 문제 (RViz)
```bash
# X11 포워딩 허용
xhost +local:docker

# 또는 특정 사용자만 허용
xhost +local:$USER
```

#### 3. 네트워크 문제
```bash
# ROS2 네트워크 설정 확인
export ROS_DOMAIN_ID=0
```

### 파일 구조
```
ros2spb_ws/
├── Dockerfile              # Docker 이미지 정의
├── docker-compose.yaml     # Docker Compose 설정
├── .dockerignore          # Docker 빌드 시 제외 파일
├── README_Docker.md       # 이 파일
└── src/                   # ROS2 패키지 소스 코드
    ├── slampibot_description/
    └── sllidar_ros2/
```

### 주의사항

1. **권한**: 컨테이너는 `privileged` 모드로 실행되어 USB 장치에 접근할 수 있습니다.
2. **네트워크**: `host` 네트워크 모드를 사용하여 ROS2 통신이 원활하게 이루어집니다.
3. **볼륨**: 소스 코드와 설치 파일들이 볼륨으로 마운트되어 개발 중에도 변경사항이 반영됩니다.
4. **GUI**: X11 포워딩을 통해 RViz가 호스트 시스템에서 표시됩니다. 