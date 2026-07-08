#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lifecycle_manage_node.py

/bond 토픽으로 들어오는 sandart_msgs/msg/Bond heartbeat를 감시하는 노드.

Bond.msg 예상 형식:
    string id
    string instance_id
    bool active
    float64 heartbeat_timeout
    float64 heartbeat_period

동작:
    - Bond.id 별로 마지막 수신 시간을 저장
    - active=True 상태에서 heartbeat_timeout 이상 메시지가 안 오면 DEAD 경고
    - active=False 메시지가 오면 정상 비활성/종료로 판단
    - 같은 id에서 instance_id가 바뀌면 노드 재시작으로 판단
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sandart_msgs.msg import Bond


class LifecycleManageNode(Node):
    def __init__(self):
        super().__init__("lifecycle_manage_node")

        self.declare_parameter("bond_topic", "/bond")
        # self.declare_parameter("default_timeout_sec", 3.0)
        self.declare_parameter("check_period_sec", 0.1)

        self.bond_topic = self.get_parameter("bond_topic").value

        self.check_period_sec = float(self.get_parameter("check_period_sec").value)

        self.bonds = {}

        qos = QoSProfile(
            history = HistoryPolicy.KEEP_LAST,
            depth = 20,
            reliability = ReliabilityPolicy.RELIABLE,
        )

        self.sub = self.create_subscription(
            Bond,
            self.bond_topic,
            self.bond_callback,
            qos,
        )

        self.timer = self.create_timer(
            self.check_period_sec,
            self.check_dead_bonds,
        )
        self.get_logger().info(f"Listening on {self.bond_topic}")

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9
    def bond_callback(self, msg: Bond):
        if not msg.id:
            return
        now = self.now_sec()
        timeout = msg.heartbeat_timeout if msg.heartbeat_timeout > 0.0 else 3.0
        self.bonds[msg.id] = {
            "last_seen": now,
            "timeout": timeout,
            "active": bool(msg.active),
            "instance_id": msg.instance_id,
        }
        if msg.active:
            self.get_logger().info(
                f"heartbeat: id={msg.id}, instance_id={msg.instance_id}"
            )
        else:
            self.get_logger().info(
                f"inactive: id={msg.id}, instance_id={msg.instance_id}"
            )
    def check_dead_bonds(self):
        now = self.now_sec()
        for bond_id, info in self.bonds.items():
            if not info["active"]:
                continue
            elapsed = now - info["last_seen"]
            if elapsed > info["timeout"]:
                self.get_logger().warn(
                    f"DEAD: id={bond_id}, instance_id={info['instance_id']}, "
                    f"no heartbeat for {elapsed:.2f}s"
                )
def main(args=None):
    rclpy.init(args=args)
    node = LifecycleManageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
    main()