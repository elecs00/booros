# 기본 ROS 이미지 사용
FROM ros:humble-ros-base

# 1. 컨테이너 내부 오디오 및 필수 패키지 설치
# root 권한으로 실행됩니다.
RUN apt-get update && apt-get install -y \
    alsa-utils \
    libasound2-dev \
    libportaudio2 \
    espeak-ng \
    nano \
    --no-install-recommends && rm -rf /var/lib/apt/lists/*

# 2. ROS 노드 파일 복사
# your_package_name/src/audio_player.py 파일이 있다고 가정합니다.
# root 홈 디렉토리로 복사
COPY src/ /root/src

# 3. 컨테이너 시작 시 환경 변수 및 ROS 설정 자동화
# .bashrc 파일에 ROS 설정을 추가합니다.
RUN echo "export ROS_DOMAIN_ID=50" >> /root/.bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# 4. 작업 디렉토리 설정
WORKDIR /root

# CMD ["/bin/bash"]
