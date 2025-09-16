#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess

class AudioPlayer(Node):
    def __init__(self):
        super().__init__('audio_player')
        self.get_logger().info('AudioPlayer 노드 초기화 중. 스피커 연결을 확인하세요.')
        
        # 🌟 eSpeak NG는 클라이언트 초기화가 필요 없음
        
        self.tts_sub = self.create_subscription(String, '/tts_text', self.tts_callback, 10)
        self.get_logger().info('/tts_text 토픽 수신 준비 완료.')

    def synthesize_text_to_speech(self, text):
        try:
            # espeak-ng 명령어 실행
            # -s: 속도, -v: 언어
            command = ["espeak-ng", "-s", "150", "-v", "ko", text]
            subprocess.run(command, check=True)
            self.get_logger().info(f'"{text}" 음성 재생 완료.')
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"TTS 오류 발생: {e}")

    def tts_callback(self, msg):
        self.get_logger().info(f'텍스트 수신: "{msg.data}"')
        self.synthesize_text_to_speech(msg.data)


def main(args=None):
    try:
        rclpy.init(args=args)
        audio_player = AudioPlayer()
        rclpy.spin(audio_player)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"예외 발생: {e}")
    finally:
        if 'audio_player' in locals() and audio_player.get_name():
            audio_player.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
