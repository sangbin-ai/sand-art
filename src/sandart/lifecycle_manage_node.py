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

        self.bond_topic = "/bond"
        self.check_period_sec = 0.1

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
        
        prev_info = self.bonds.get(bond_id)

        if prev_info is None:
            state = STATE_ACTIVE if is_active else STATE_INACTIVE

            self.bonds[bond_id] = {
                "last_seen": now,
                "timeout": timeout,
                "active": is_active,
                "instance_id": instance_id,
                "state": state,
            }

            self.get_logger().info(
                f"[{state}] id={bond_id}, instance_id={instance_id}"
            )
            return
        
        if not is_active:
            if prev_state != STATE_INACTIVE:
                self.get_logger().info(
                    f"[{prev_state} -> {STATE_INACTIVE}] "
                    f"id={bond_id}, instance_id={instance_id}"
                )

            prev_info["state"] = STATE_INACTIVE
            return

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