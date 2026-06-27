#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration


class SucessfulBeepNode(Node):
    def __init__(self):
        super().__init__('sucessful_beep_node')

        self.audio_pub = self.create_publisher(
            AudioNoteVector,
            '/robot9/cmd_audio',
            10
        )

        self.beep_sub = self.create_subscription(
            Bool,
            '/robot9/sucessful_beep_enable',
            self.beep_callback,
            10
        )

    def beep_callback(self, msg):
        if not msg.data:
            return

        if self.audio_pub.get_subscription_count() == 0:
            self.get_logger().warn('cmd_audio subscriber 없음. 소리 전송 실패')
            return

        beep_msg = AudioNoteVector()
        beep_msg.append = False
        beep_msg.notes = [
            AudioNote(
                frequency=1000,
                max_runtime=Duration(sec=1, nanosec=0)
            )
        ]

        self.get_logger().info('처리 완료 소리 전송 중...')
        self.audio_pub.publish(beep_msg)
        self.get_logger().info('처리 완료 소리 전송 완료')


def main(args=None):
    rclpy.init(args=args)
    node = SucessfulBeepNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()