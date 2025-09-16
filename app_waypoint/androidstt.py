#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class STTResultPublisher(Node):
    def __init__(self):
        super().__init__('stt_result_publisher')
        self.publisher = self.create_publisher(String, '/stt_result', 10)
        self.get_logger().info("STTResultPublisher 노드 시작, /stt_result 토픽 광고 중")

    # 외부에서 받는 텍스트 메시지를 퍼블리시하는 메서드 예시
    def publish_text(self, text: str):
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        self.get_logger().info(f"퍼블리시된 텍스트: {text}")

def main(args=None):
    rclpy.init(args=args)
    node = STTResultPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
