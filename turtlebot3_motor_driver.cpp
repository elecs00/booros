/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#include "../../include/turtlebot3/turtlebot3_motor_driver.h"
#include <Dynamixel2Arduino.h>   // 이미 헤더 체인에 있어도 무해
// --- OpenCR DXL setup ---
#ifndef OPENCR_DXL_DIR_PIN
  #define OPENCR_DXL_DIR_PIN 84   // OpenCR 보드에서 DXL 방향핀 // Arduino pin number of DYNAMIXEL direction pin on OpenCR.
#endif

// ---- Raw R/W helpers (XM/XL430 공용, Little-Endian) ----
Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN);
// extern Dynamixel2Arduino dxl;
// ▼▼ 여기 아래에 헬퍼 함수들 배치
// ---- Raw R/W helpers (Little-Endian) ----
// write(id, addr, const uint8_t* data, uint16_t length)
// read (id, addr, uint16_t addr_length, uint8_t* data, uint16_t length)

static inline bool wr8 (uint8_t id, uint16_t addr, uint8_t v){
  return dxl.write(id, addr, &v, (uint16_t)1);
}
static inline bool wr16(uint8_t id, uint16_t addr, uint16_t v){
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)((v >> 8) & 0xFF) };
  return dxl.write(id, addr, b, (uint16_t)2);
}
static inline bool wr32(uint8_t id, uint16_t addr, int32_t v){
  uint8_t b[4] = {
    (uint8_t)( v        & 0xFF),
    (uint8_t)((v >> 8)  & 0xFF),
    (uint8_t)((v >>16)  & 0xFF),
    (uint8_t)((v >>24)  & 0xFF)
  };
  return dxl.write(id, addr, b, (uint16_t)4);
}

static inline uint8_t rd8(uint8_t id, uint16_t addr){
  uint8_t v = 0;
  dxl.read(id, addr, (uint16_t)2, &v, (uint16_t)1);
  return v;
}
static inline uint16_t rd16(uint8_t id, uint16_t addr){
  uint8_t b[2] = {0,0};
  dxl.read(id, addr, (uint16_t)2, b, (uint16_t)2);
  return (uint16_t)(b[0] | (b[1] << 8));
}
static inline int32_t rd32(uint8_t id, uint16_t addr){
  uint8_t b[4] = {0,0,0,0};
  dxl.read(id, addr, (uint16_t)2, b, (uint16_t)4);
  return (int32_t)((uint32_t)b[0] | ((uint32_t)b[1] << 8) | ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24));
}
// 실패 로그 유틸 8.27
static inline bool WR8 (uint8_t id, uint16_t addr, uint8_t v, const char* tag){
  bool ok = wr8(id, addr, v);
  if(!ok) Serial.printf("[ERR] wr8  %-10s id=%u addr=%u val=%u FAIL\n", tag, id, addr, v);
  return ok;
}
static inline bool WR16(uint8_t id, uint16_t addr, uint16_t v, const char* tag){
  bool ok = wr16(id, addr, v);
  if(!ok) Serial.printf("[ERR] wr16 %-10s id=%u addr=%u val=%u FAIL\n", tag, id, addr, v);
  return ok;
}
static inline bool WR32(uint8_t id, uint16_t addr, int32_t v, const char* tag){
  bool ok = wr32(id, addr, v);
  if(!ok) Serial.printf("[ERR] wr32 %-10s id=%u addr=%u val=%ld FAIL\n", tag, id, addr, (long)v);
  return ok;
}


// X-Series Control Table (XM430 기준)
enum {
  ADDR_DRIVE_MODE              = 10,   // 1B
  ADDR_OPERATING_MODE          = 11,   // 1B
  ADDR_TORQUE_ENABLE           = 64,   // 1B
  ADDR_PWM_LIMIT               = 36,   // 2B
  ADDR_VELOCITY_LIMIT          = 44,   // 4B
  ADDR_VELOCITY_I_GAIN         = 76,   // 2B
  ADDR_VELOCITY_P_GAIN         = 78,   // 2B
  ADDR_BUS_WATCHDOG            = 98,   // 1B
  ADDR_GOAL_PWM                = 100,  // 2B
  ADDR_GOAL_VELOCITY           = 104,  // 4B
  ADDR_PRESENT_PWM             = 124,  // 2B
  ADDR_PRESENT_CURRENT         = 126,  // 2B
  ADDR_PRESENT_VELOCITY        = 128,  // 4B
  ADDR_PRESENT_POSITION        = 132,  // 4B
  ADDR_PRESENT_INPUT_VOLTAGE   = 144,  // 2B (0.1V 단위)
};
// ---- Helper: status dump (파일 전역 영역에 위치해야 함) ----
static void dump_status(uint8_t id){
  using namespace ControlTableItem;
  int op  = dxl.readControlTableItem(OPERATING_MODE, id);
  int tq  = dxl.readControlTableItem(TORQUE_ENABLE,  id);
  int hw  = dxl.readControlTableItem(HARDWARE_ERROR_STATUS, id);
  int vin = dxl.readControlTableItem(PRESENT_INPUT_VOLTAGE, id);
  int32_t pv = rd32(id, ADDR_PRESENT_VELOCITY);
  uint16_t pp= rd16(id, ADDR_PRESENT_PWM);
  Serial.printf("[ID%u] OP=%d TQ=%d HW=%d Vin=%.1fV PV=%ld PWM=%u\n",
                id, op, tq, hw, vin/10.0f, (long)pv, pp);
}
using namespace DYNAMIXEL;  // 컨트롤테이블 식별자 전역 노출

// Limit values (XM430-W210-T and XM430-W350-T)
// MAX RPM is 77 when DXL is powered 12.0V
// 77 / 0.229 (RPM) = 336.24454...
const uint16_t LIMIT_X_MAX_VELOCITY = 337; 
// V = r * w = r     *        (RPM             * 0.10472)
//           = 0.033 * (0.229 * Goal_Velocity) * 0.10472
// Goal_Velocity = V * 1263.632956882
const float VELOCITY_CONSTANT_VALUE = 1263.632956882; 

/* DYNAMIXEL Information for controlling motors and  */
const uint8_t DXL_MOTOR_ID_LEFT = 1; // ID of left motor
const uint8_t DXL_MOTOR_ID_RIGHT = 2; // ID of right motor
// ▼▼ 추가: 뒤 바퀴 ID #add
const uint8_t DXL_MOTOR_ID_LEFT_REAR  = 3; // Left-Rear
const uint8_t DXL_MOTOR_ID_RIGHT_REAR = 4; // Right-Rear
const float DXL_PORT_PROTOCOL_VERSION = 2.0; // Dynamixel protocol version 2.0
const uint32_t DXL_PORT_BAUDRATE = 1000000; // baurd rate of Dynamixel

ParamForSyncReadInst_t sync_read_param;
ParamForSyncWriteInst_t sync_write_param;
RecvInfoFromStatusInst_t read_result;
// Dynamixel2Arduino dxl(Serial3, OPENCR_DXL_DIR_PIN); //여기에 중복 선언되어있는지 몰랐음;



Turtlebot3MotorDriver::Turtlebot3MotorDriver()
: left_wheel_id_(DXL_MOTOR_ID_LEFT),
  right_wheel_id_(DXL_MOTOR_ID_RIGHT),
  torque_(false)
{
}

Turtlebot3MotorDriver::~Turtlebot3MotorDriver()
{
  close();
  digitalWrite(BDPIN_DXL_PWR_EN, LOW);
}

bool Turtlebot3MotorDriver::init(void)
{
  if (!Serial) { Serial.begin(115200); delay(50); } // ★ 가장 먼저

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  digitalWrite(BDPIN_DXL_PWR_EN, HIGH);
  drv_dxl_init();

  dxl.begin(DXL_PORT_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PORT_PROTOCOL_VERSION);

  // ---- Sync Write/Read는 "앞바퀴 2개"만 ----
  // (TB3 기본 구조체가 xel[2]까지만 안전하므로 xel[2], xel[3] 접근 금지)
  sync_write_param.id_count   = 2;
  sync_write_param.xel[0].id  = DXL_MOTOR_ID_LEFT;       // 1
  sync_write_param.xel[1].id  = DXL_MOTOR_ID_RIGHT;      // 2

  sync_read_param.addr        = ADDR_PRESENT_POSITION;
  sync_read_param.length      = 4;
  sync_read_param.id_count    = 2;
  sync_read_param.xel[LEFT].id  = left_wheel_id_;
  sync_read_param.xel[RIGHT].id = right_wheel_id_;

  // ---- 뒤 바퀴(3,4) 설정 : raw write (WR* 래퍼 사용 권장) ----
  WR8 (3, ADDR_TORQUE_ENABLE, 0, "TQ_OFF_3");
  WR8 (4, ADDR_TORQUE_ENABLE, 0, "TQ_OFF_4");
  WR8 (3, ADDR_OPERATING_MODE, 1, "OPMODE3");   // Velocity
  WR8 (4, ADDR_OPERATING_MODE, 1, "OPMODE4");
  WR16(3, ADDR_PWM_LIMIT, 885, "PWM_LIM3");
  WR16(4, ADDR_PWM_LIMIT, 885, "PWM_LIM4");
  WR32(3, ADDR_VELOCITY_LIMIT, 1023, "VEL_LIM3");
  WR32(4, ADDR_VELOCITY_LIMIT, 1023, "VEL_LIM4");
  WR16(3, ADDR_VELOCITY_I_GAIN, 100, "VI3");
  WR16(4, ADDR_VELOCITY_I_GAIN, 100, "VI4");
  WR16(3, ADDR_VELOCITY_P_GAIN, 400, "VP3");
  WR16(4, ADDR_VELOCITY_P_GAIN, 400, "VP4");
  // wr8 (3, ADDR_BUS_WATCHDOG, 100);   // ~1s
  // wr8 (4, ADDR_BUS_WATCHDOG, 100);
  // // 필요시 우측 반전:
  // // wr8(2, ADDR_DRIVE_MODE, 1); wr8(4, ADDR_DRIVE_MODE, 1);
  // 테스트 중엔 watchdog 0 권장(동작 확인 후 다시 원하는 값으로)
  WR8 (3, ADDR_BUS_WATCHDOG, 0, "WDG3");
  WR8 (4, ADDR_BUS_WATCHDOG, 0, "WDG4");
  WR8 (3, ADDR_TORQUE_ENABLE, 1, "TQ_ON_3");
  WR8 (4, ADDR_TORQUE_ENABLE, 1, "TQ_ON_4");
  // wr8 (3, ADDR_TORQUE_ENABLE, 1);
  // wr8 (4, ADDR_TORQUE_ENABLE, 1);

  // ---- 전체 토크 ON (앞/뒤) ----
  set_torque(true);

  // ---- 상태 로그 (정확한 길이로 읽기) ----

  int op3  = rd8 (3, ADDR_OPERATING_MODE);
  int tq3  = rd8 (3, ADDR_TORQUE_ENABLE);
  int dv3  = rd8 (3, ADDR_DRIVE_MODE);
  int bw3  = rd8 (3, ADDR_BUS_WATCHDOG);
  int op4  = rd8 (4, ADDR_OPERATING_MODE);
  int tq4  = rd8 (4, ADDR_TORQUE_ENABLE);
  int dv4  = rd8 (4, ADDR_DRIVE_MODE);
  int bw4  = rd8 (4, ADDR_BUS_WATCHDOG);

  int32_t  vl3 = rd32(3, ADDR_VELOCITY_LIMIT);
  int32_t  vl4 = rd32(4, ADDR_VELOCITY_LIMIT);
  uint16_t pl3 = rd16(3, ADDR_PWM_LIMIT);
  uint16_t pl4 = rd16(4, ADDR_PWM_LIMIT);
  uint16_t vin3= rd16(3, ADDR_PRESENT_INPUT_VOLTAGE);
  uint16_t vin4= rd16(4, ADDR_PRESENT_INPUT_VOLTAGE);

  Serial.printf("[ID3] OP=%d TQ=%d DRIVE=%d WDG=%d VEL_LIM=%ld PWM_LIM=%u Vin=%.1fV\n",
                op3,tq3,dv3,bw3,(long)vl3,pl3, vin3/10.0f);
  Serial.printf("[ID4] OP=%d TQ=%d DRIVE=%d WDG=%d VEL_LIM=%ld PWM_LIM=%u Vin=%.1fV\n",
                op4,tq4,dv4,bw4,(long)vl4,pl4, vin4/10.0f);

  // ---- 간단 구동 테스트 (Velocity) ----
  WR32(3, ADDR_GOAL_VELOCITY, 200, "TEST_GV3");
  WR32(4, ADDR_GOAL_VELOCITY, 200, "TEST_GV4");
  delay(300);
  int32_t pv3 = rd32(3, ADDR_PRESENT_VELOCITY);
  int32_t pv4 = rd32(4, ADDR_PRESENT_VELOCITY);
  uint16_t pp3= rd16(3, ADDR_PRESENT_PWM);
  uint16_t pp4= rd16(4, ADDR_PRESENT_PWM);
  Serial.printf("[VEL] PV %ld/%ld  PWM %u/%u\n", (long)pv3,(long)pv4, pp3,pp4);
  WR32(3, ADDR_GOAL_VELOCITY, 0, "TEST_GV3_0");
  WR32(4, ADDR_GOAL_VELOCITY, 0, "TEST_GV4_0");

  dump_status(3);
  dump_status(4);

  return true;
}


Dynamixel2Arduino& Turtlebot3MotorDriver::getDxl()
{
  return dxl;
}

bool Turtlebot3MotorDriver::is_connected()
{
  return (dxl.ping(DXL_MOTOR_ID_LEFT) == true && dxl.ping(DXL_MOTOR_ID_RIGHT) == true);
}

bool Turtlebot3MotorDriver::set_torque(bool onoff)
{
  // TORQUE_ENABLE = 64
  uint8_t v = onoff ? 1 : 0;
  wr8(1, ADDR_TORQUE_ENABLE, v);
  wr8(2, ADDR_TORQUE_ENABLE, v);
  wr8(3, ADDR_TORQUE_ENABLE, v);
  wr8(4, ADDR_TORQUE_ENABLE, v);
  torque_ = onoff;
  return true;
}

bool Turtlebot3MotorDriver::get_torque()
{
  int32_t lt = dxl.readControlTableItem(TORQUE_ENABLE, left_wheel_id_);
  int32_t rt = dxl.readControlTableItem(TORQUE_ENABLE, right_wheel_id_);
  torque_ = (lt != 0 && rt != 0);
  return torque_;
}

void Turtlebot3MotorDriver::close(void)
{
  // Disable Dynamixel Torque
  set_torque(false);
}

bool Turtlebot3MotorDriver::read_present_position(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 132;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_velocity(int32_t &left_value, int32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 128;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_present_current(int16_t &left_value, int16_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 126;
  sync_read_param.length = 2;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}

bool Turtlebot3MotorDriver::read_profile_acceleration(uint32_t &left_value, uint32_t &right_value)
{
  bool ret = false;

  sync_read_param.addr = 108;
  sync_read_param.length = 4;

  if(dxl.syncRead(sync_read_param, read_result)){
    memcpy(&left_value, read_result.xel[LEFT].data, read_result.xel[LEFT].length);
    memcpy(&right_value, read_result.xel[RIGHT].data, read_result.xel[RIGHT].length);
    ret = true;
  }

  return ret;
}


bool Turtlebot3MotorDriver::write_velocity(int32_t left_value, int32_t right_value)
{
  // GOAL_VELOCITY = 104
  // 오른쪽(ID2,4)이 하드웨어적으로 DRIVE_MODE=1(Reverse)라면 보통 같은 부호로 써도 전/후가 같은 방향으로 돕니다.
  // 만약 우측 앞/뒤가 서로 반대로 돈다면 아래 한 줄을 활성화해서 부호를 맞춰주세요.
  // right_value = -right_value;

  // GOAL_VELOCITY = 104 (4 bytes, signed)
  bool ok = true;
  ok &= WR32(1, ADDR_GOAL_VELOCITY, left_value,  "GV1");
  ok &= WR32(2, ADDR_GOAL_VELOCITY, right_value, "GV2");
  ok &= WR32(3, ADDR_GOAL_VELOCITY, left_value,  "GV3");
  ok &= WR32(4, ADDR_GOAL_VELOCITY, right_value, "GV4");
  return ok;
}

bool Turtlebot3MotorDriver::write_profile_acceleration(uint32_t left_value, uint32_t right_value)
{
  bool ok = true;
  // 앞바퀴
  ok &= WR32(1, 108, (int32_t)left_value,  "PROF_ACC_1");
  ok &= WR32(2, 108, (int32_t)right_value, "PROF_ACC_2");
  // 뒤바퀴
  ok &= WR32(3, 108, (int32_t)left_value,  "PROF_ACC_3");
  ok &= WR32(4, 108, (int32_t)right_value, "PROF_ACC_4");
  return ok;
}

bool Turtlebot3MotorDriver::control_motors(const float wheel_separation, float linear_value, float angular_value)
{
  bool dxl_comm_result = false;
  
  float wheel_velocity[MortorLocation::MOTOR_NUM_MAX];
  float lin_vel = linear_value;
  float ang_vel = angular_value;

  wheel_velocity[LEFT]   = lin_vel - (ang_vel * wheel_separation / 2);
  wheel_velocity[RIGHT]  = lin_vel + (ang_vel * wheel_separation / 2);

  wheel_velocity[LEFT]  = constrain(wheel_velocity[LEFT]  * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);
  wheel_velocity[RIGHT] = constrain(wheel_velocity[RIGHT] * VELOCITY_CONSTANT_VALUE, -LIMIT_X_MAX_VELOCITY, LIMIT_X_MAX_VELOCITY);

  dxl_comm_result = write_velocity((int32_t)wheel_velocity[LEFT], (int32_t)wheel_velocity[RIGHT]);
  if (dxl_comm_result == false)
    return false;

  return true;
}

