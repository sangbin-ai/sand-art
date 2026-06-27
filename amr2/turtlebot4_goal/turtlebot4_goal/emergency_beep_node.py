#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


class EmergencyBeepNode(Node):
    def __init__(self):
        super().__init__('emergency_beep_node')

        self.audio_pub = self.create_publisher(
            AudioNoteVector,
            '/robot9/cmd_audio',
            10
        )

        # 나중에 주행 노드에서 이 토픽으로 True/False 보내면 됨
        self.beep_sub = self.create_subscription(
            Bool,
            '/robot9/emergency_beep_enable',
            self.beep_enable_callback,
            10
        )

        self.beep_enabled = False
        self.beep_timer = self.create_timer(1.2, self.beep_timer_callback)
        self.beep_timer.cancel()

    def beep_enable_callback(self, msg):

        if msg.data:
            if not self.beep_enabled:
                self.beep_enabled = True
                self.beep_timer.reset()
                self.get_logger().warn('응급 삐뽀삐뽀 시작')

        else:
            self.get_logger().info('응급 삐뽀삐뽀 정지')

            self.beep_enabled = False
            self.beep_timer.cancel()

    def beep_timer_callback(self):
        if not self.beep_enabled:
            return

        if self.audio_pub.get_subscription_count() == 0:
            self.get_logger().warn('cmd_audio subscriber 기다리는 중...')
            return

        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),
        ]

        self.audio_pub.publish(msg)
        self.get_logger().info('삐뽀삐뽀 송출 중...')


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBeepNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()