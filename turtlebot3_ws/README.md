# turtlebot3_ws

이 폴더에는 소스를 넣지 않는다. 아래 ROBOTIS 공식 저장소를 받아 쓴다.
(이전에는 하위모듈 연결만 커밋돼 있었으나 `.gitmodules` 가 없어 GitHub 에서 열리지 않았으므로 제거하고, 개발 당시 쓰던 커밋을 여기에 기록한다.)

| 패키지 | 원본 저장소 | 당시 커밋 |
|---|---|---|
| DynamixelSDK | https://github.com/ROBOTIS-GIT/DynamixelSDK | `b5dd448848fabea636bea559653b1da0e671385a` |
| turtlebot3 | https://github.com/ROBOTIS-GIT/turtlebot3 | `6b7a53d57a9901c06114710d8c710bb2529c938d` |
| turtlebot3_msgs | https://github.com/ROBOTIS-GIT/turtlebot3_msgs | `cdee60f7618b9ed2f85f65259c0f96e3f02b1c55` |

## 받는 방법

```bash
mkdir -p turtlebot3_ws/src && cd turtlebot3_ws/src
git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
# 당시와 같은 버전으로 맞추려면 각 폴더에서 위 표의 커밋으로 checkout
cd .. && colcon build --symlink-install
source install/setup.bash
```

4륜 구동 개조에 쓴 모터 드라이버 수정본은 저장소 최상위의 `turtlebot3_motor_driver.cpp` 이다.
