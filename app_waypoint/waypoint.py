#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from std_msgs.msg import String
from sensor_msgs.msg import Range
import time
import os
import requests
import configparser
from action_msgs.msg import GoalStatus

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.get_logger().info('WaypointNavigator 노드 초기화 중')

        # --- 웨이포인트 로드 ---
        self.locations = self.load_waypoints_from_ini('waypoints.ini')
        if not self.locations:
            self.get_logger().error('웨이포인트 로딩 실패! 프로그램을 종료합니다.')
            return

        # --- ROS2 통신 설정 ---
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._client.wait_for_server()
        self.get_logger().info('navigate_to_pose 액션 서버에 연결되었습니다')

        # 발행자
        self.tts_publisher = self.create_publisher(String, '/tts_text', 10)
        self.stt_control_pub = self.create_publisher(String, '/stt_control', 10)
        self.nod_control_pub = self.create_publisher(String, '/nod_control', 10)

        # 구독자
        self.distance_sub = self.create_subscription(Range, '/distance', self.distance_callback, 10)
        self.stt_sub = self.create_subscription(String, '/stt_result', self.stt_callback, 10)
        self.nod_sub = self.create_subscription(String, '/action/nod', self.nod_callback, 10)
        self.greeting_sub = self.create_subscription(String, '/action/greeting', self.greeting_callback, 10)

        # --- 상태 변수 ---
        self.saved_goal_coords = None
        self.current_goal_handle = None
        self.is_paused_by_obstacle = False
        self.current_goal_name = ""
        self.retry_count = 0
        self.max_retries = 3
        self.pending_explanation = None
        self.qa_mode = False
        self.tts_playing = False
        self._send_goal_future = None
        self._get_result_future = None

        # --- API 키 ---
        self.perplexity_api_key = os.getenv('PERPLEXITY_API_KEY', '')

        time.sleep(1)
        self.publish_nod_control("disable")
        self.get_logger().info('✅ WaypointNavigator 노드 초기화 완료')

        # 시작 시 STT 활성화
        self.publish_stt_control("start")

    # -----------------------------
    # 구독 콜백
    # -----------------------------
    def greeting_callback(self, msg: String):
        if msg.data == "hello":
            self.publish_tts_text("안녕하세요. 오늘 관람을 함께할 boo입니다. 어떤 작품에 관심이 있으신가요?")

    def stt_callback(self, msg: String):
        recognized_text = msg.data
        self.get_logger().info(f'STT 결과: "{recognized_text}"')

        # 🔹 TTS 중, 이동 중에는 무시
        if self.tts_playing or self.current_goal_handle is not None or (self._send_goal_future and not self._send_goal_future.done()):
            self.get_logger().info("현재 상태로 STT 무시")
            return

        # QA 모드
        if self.qa_mode:
            if any(word in recognized_text for word in ["없어요", "아니요", "괜찮아요", "아니"]):
                self.publish_tts_text("알겠습니다. 다음 작품으로 이동하시겠습니까?")
                self.qa_mode = False
                self.pending_explanation = None
            else:
                answer = self.get_location_description_from_llm(recognized_text)
                self.publish_tts_text(answer)
                time.sleep(max(len(answer) * 0.1, 2))
                self.publish_tts_text("추가로 궁금한 점 있으신가요?")
            return

        # 이동 명령
        for location_name, coords in self.locations.items():
            if location_name in recognized_text:
                self.publish_tts_text(f"{location_name}으로 이동하겠습니다.")
                self.current_goal_name = location_name
                self.saved_goal_coords = coords
                self.send_goal_async(coords)
                return

        self.get_logger().warn("이동할 장소를 인식 못함")
        self.publish_tts_text("이동할 장소를 인식하지 못했습니다. 다시 말씀해주세요.")

    def nod_callback(self, msg: String):
        if msg.data == "start_explanation" and self.pending_explanation:
            self.publish_tts_text(self.pending_explanation)
            time.sleep(max(len(self.pending_explanation) * 0.1, 2))
            self.pending_explanation = None
            self.qa_mode = True
            self.publish_tts_text("추가로 궁금한 점 있으신가요?")
            self.publish_nod_control("disable")
        elif msg.data == "start_farewell":
            self.publish_tts_text("마무리 멘트 완료.")

    # -----------------------------
    # 장애물 감지 콜백
    # -----------------------------
    def distance_callback(self, msg: Range):
        distance = msg.range

        if distance < 0.3:
            if self.current_goal_handle is not None and not self.is_paused_by_obstacle:
                self.is_paused_by_obstacle = True
                # 목표 취소 후 TTS 호출
                cancel_future = self.current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self.after_obstacle_stop(distance))
        else:
            if self.is_paused_by_obstacle:
                self.is_paused_by_obstacle = False
                self.get_logger().info("장애물 해제됨. 저장된 목표로 경로를 재개합니다.")
                self.publish_tts_text("경로를 다시 시작합니다.")
                if self.current_goal_name in self.locations:
                    self.current_goal_handle = None
                    self.send_goal_async(self.locations[self.current_goal_name])
                else:
                    self.get_logger().warn("재전송할 저장된 목표가 없습니다.")

    def after_obstacle_stop(self, distance):
        self.get_logger().warn(f"20cm 이내 장애물 감지 (거리: {distance:.3f}m)! 경로를 일시 중지합니다.")
        self.publish_tts_text("전방에 장애물이 있어 잠시 멈춥니다.")

    def cancel_done_callback(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('목표 취소 성공')
        else:
            self.get_logger().warn('취소할 활성 목표 없음')

    # -----------------------------
    # TTS / STT / Nod 제어
    # -----------------------------
    def publish_tts_text(self, text):
        self.publish_stt_control("stop")
        self.tts_playing = True

        msg = String()
        msg.data = text
        self.tts_publisher.publish(msg)
        self.get_logger().info(f"TTS 발행: {text}")

        # 임시 처리
        time.sleep(max(len(text) * 0.15, 4))
        self.tts_playing = False

        if self.current_goal_handle is None:
            self.publish_stt_control("start")

    def publish_stt_control(self, command):
        self.stt_control_pub.publish(String(data=command))
        self.get_logger().info(f"STT 제어: {command}")

    def publish_nod_control(self, command):
        self.nod_control_pub.publish(String(data=command))
        self.get_logger().info(f"Nod 제어: {command}")

    # -----------------------------
    # 웨이포인트 로딩
    # -----------------------------
    def load_waypoints_from_ini(self, ini_path):
        config = configparser.ConfigParser()
        if not os.path.exists(ini_path):
            self.get_logger().error(f"'{ini_path}' 파일을 찾을 수 없습니다!")
            return {}
        config.read(ini_path, encoding='utf-8')
        locations = {}
        for section in config.sections():
            try:
                x = config.getfloat(section, 'x')
                y = config.getfloat(section, 'y')
                z = config.getfloat(section, 'z')
                w = config.getfloat(section, 'w')
                locations[section] = {'x': x, 'y': y, 'z': z, 'w': w}
            except (configparser.NoOptionError, ValueError) as e:
                self.get_logger().error(f"'{section}' 섹션 파싱 오류: {e}")
                continue
        self.get_logger().info(f"성공적으로 {len(locations)}개의 웨이포인트 로드: {list(locations.keys())}")
        return locations

    # -----------------------------
    # LLM 설명
    # -----------------------------
    def get_location_description_from_llm(self, location_name):
        if not self.perplexity_api_key:
            self.get_logger().warn('Perplexity API 키가 없어 설명 생성 불가.')
            return f"{location_name}에 도착했습니다."
        try:
            if location_name == "홈":
                prompt = "마크다운, 별표, 숫자 등 포맷은 쓰지 말고 관람을 마친 관람객에게 자연스럽고 반갑게 인사해주세요."
            else:
                prompt = f"미술작품인 {location_name}에 대해 한국어로 자연스럽게 설명해 주세요. 마크다운, 별표, 숫자 등 포맷은 쓰지 말고 사람처럼 이야기하듯 2~3문장으로."

            headers = {"Authorization": f"Bearer {self.perplexity_api_key}", "Content-Type": "application/json"}
            data = {"model": "sonar-pro", "messages": [{"role": "user", "content": prompt}], "max_tokens": 150}
            response = requests.post("https://api.perplexity.ai/chat/completions", headers=headers, json=data, timeout=10)
            
            if response.status_code == 200:
                result = response.json()
                return result['choices'][0]['message']['content']
            else:
                self.get_logger().error(f"LLM API 에러: {response.status_code} - {response.text}")
                return f"{location_name}에 도착했습니다. 잠시 후 다시 시도해주세요."
        except Exception as e:
            self.get_logger().error(f'LLM 설명 생성 오류: {e}')
            return f"{location_name}에 도착했습니다."

    # -----------------------------
    # 목표 전송
    # -----------------------------
    def send_goal_async(self, coords):
        if self._send_goal_future and not self._send_goal_future.done(): 
            return
        
        self.publish_stt_control("stop")
        self.is_paused_by_obstacle = False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = coords['x']
        goal_msg.pose.pose.position.y = coords['y']
        goal_msg.pose.pose.orientation.z = coords['z']
        goal_msg.pose.pose.orientation.w = coords['w']
        
        self._send_goal_future = self._client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('목표 거부됨')
            self.publish_tts_text("이동 목표가 거부되었습니다. 다시 시도해주세요.")
            self.publish_stt_control("start")
            self.current_goal_handle = None
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info('목표 수락됨')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'"{self.current_goal_name}" 목표 달성 성공!')
            self.retry_count = 0
            
            try:
                if self.current_goal_name == "홈":
                    farewell_text = self.get_location_description_from_llm(self.current_goal_name)
                    self.publish_tts_text(farewell_text)
                    self.get_logger().info('✅ "홈" 도착 완료')
                else:
                    self.pending_explanation = self.get_location_description_from_llm(self.current_goal_name)
                    self.publish_tts_text("설명이 필요하면 고개를 끄덕여 주세요.")
                    self.publish_nod_control("enable")
                    self.get_logger().info('✅ 목표 완료! 끄덕임 대기 중')
              
            except Exception as e:
                self.get_logger().error(f'장소 설명 오류: {e}')
                self.pending_explanation = f"{self.current_goal_name}에 도착했습니다."
                self.publish_tts_text("설명이 필요하면 고개를 끄덕여 주세요.")
                self.publish_nod_control("enable")

        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info(f'"{self.current_goal_name}" 목표가 장애물로 취소됨')
        
        else:
            self.get_logger().warn(f'"{self.current_goal_name}" 목표 실패: {status}')
            self.retry_count += 1
            if self.retry_count <= self.max_retries:
                self.publish_tts_text("이동에 실패했습니다. 다시 시도하겠습니다.")
                time.sleep(1.0)
                coords = self.locations[self.current_goal_name]
                self.current_goal_handle = None
                self.send_goal_async(coords)
                return
            else:
                self.get_logger().error(f'"{self.current_goal_name}" 재시도 초과')
                self.publish_tts_text(f"{self.current_goal_name} 이동을 포기합니다.")
                self.retry_count = 0
                self.current_goal_name = ""
                self.pending_explanation = None
        
        self.current_goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self.publish_stt_control("start")

# -----------------------------
# Main
# -----------------------------
def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    if not navigator.locations:
        return
    try:
        rclpy.spin(navigator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        navigator.get_logger().fatal(f"스핀 중 예외 발생: {e}")
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

