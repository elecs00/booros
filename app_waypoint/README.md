
## 🔧 4륜 구동으로 변경하기

TurtleBot3를 4륜 구동으로 변경하려면 OpenCR 보드의 모터 드라이버 파일을 수정해야 합니다.

### 수정할 파일 위치
```
/home/hkit/.arduino15/packages/OpenCR/hardware/OpenCR/1.5.3/libraries/turtlebot3_ros2/src/turtlebot3/turtlebot3_motor_driver.cpp
```

### 변경 사항
1. **모터 ID 추가**: 기존 2개 모터(ID 1, 2)에 추가로 2개 모터(ID 3, 4) 설정
2. **뒤바퀴 설정**: ID 3, 4번 모터에 대한 초기화 및 제어 코드 추가
3. **동기화 제어**: 4개 모터 모두 동시 제어하도록 수정

### 주요 수정 내용
- `DXL_MOTOR_ID_LEFT_REAR = 3` (왼쪽 뒷바퀴)
- `DXL_MOTOR_ID_RIGHT_REAR = 4` (오른쪽 뒷바퀴)
- 뒤바퀴 모터 초기화 및 토크 제어
- 4륜 동시 속도 제어 (`write_velocity` 함수 수정)

### 주의사항
- OpenCR 보드의 Dynamixel 포트에 4개 모터가 모두 연결되어 있어야 함
- 모터 ID가 1, 2, 3, 4로 올바르게 설정되어 있어야 함
- 전원 공급이 4개 모터 모두 충분한지 확인 필요

