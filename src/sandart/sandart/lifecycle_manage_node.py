#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
lifecycle_manage_node.py

/bond 토픽으로 들어오는 sandart_msgs/msg/Bond heartbeat를 감시하는 노드.

Bond.msg 예상 형식:
    string id
    string instance_id
    bool working
    bool active
    float64 heartbeat_timeout
    float64 heartbeat_period

동작:
    - Bond.id 별로 마지막 수신 시간을 저장
    - active=True 상태에서 heartbeat_timeout 이상 메시지가 안 오면 DEAD 경고
    - active=False 메시지가 오면 정상 비활성/종료로 판단
    - active=True , working=True 상태이면 작업중
    - active=True , working=False 상태이면 대기중 
    - 같은 id에서 instance_id가 바뀌면 노드 재시작으로 판단
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sandart_msgs.msg import Bond

STATE_INACTIVE = "INACTIVE"
STATE_IDLE = "IDLE"
STATE_WORKING = "WORKING"
STATE_DEAD = "DEAD"

CONFIG = {
    "bond_topic": "/bond",
    "check_period_sec": 0.1,
    "default_timeout_sec": 6.0,
}

class LifecycleManageNode(Node):
    def __init__(self):
        super().__init__("lifecycle_manage_node")

        self.bond_topic = CONFIG["bond_topic"]
        self.check_period_sec = CONFIG["check_period_sec"]
        self.default_timeout_sec = CONFIG["default_timeout_sec"]

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
    
    def get_state(self, active: bool, working: bool) -> str:
        if not active:
            return STATE_INACTIVE
        if working:
            return STATE_WORKING
        return STATE_IDLE
        
    def bond_callback(self, msg: Bond):
        if not msg.id:
            return
        now = self.now_sec()
        
        bond_id = msg.id
        instance_id = msg.instance_id
        is_active = msg.active
        is_working = msg.working

        # 메시지에 timeout 값이 있으면 그걸 쓰고, 없거나 0 이하이면 기본값 3.0초를 씁니다.
        timeout = msg.heartbeat_timeout if msg.heartbeat_timeout > 0.0 else 3.0
        
        new_state = self.get_state(is_active, is_working)
        prev_info = self.bonds.get(bond_id)


        # 처음 보는 Bond id
        if prev_info is None:
            self.bonds[bond_id] = {
                "last_seen": now,
                "timeout": timeout,
                "active": is_active,
                "working": is_working,
                "instance_id": instance_id,
                "state": new_state,
            }

            self.get_logger().info(
                f"[{new_state}] id={bond_id}, instance_id={instance_id}"
            )
            return
        
        prev_state = prev_info["state"]
        prev_instance_id = prev_info["instance_id"]

        # 같은 id인데 instance_id가 바뀌면 재시작으로 판단
        if prev_instance_id != instance_id:
            self.get_logger().info(
                f"[RESTART] id={bond_id}, "
                f"old_instance_id={prev_instance_id}, "
                f"new_instance_id={instance_id}"
            )

        # DEAD 상태였다가 다시 heartbeat가 들어오면 복구로 판단
        if prev_state == STATE_DEAD and new_state != STATE_DEAD:
            self.get_logger().info(
                f"[RECOVERED -> {new_state}] id={bond_id}, "
                f"instance_id={instance_id}"
            )
        
         # 일반 상태 변화 로그
        elif prev_state != new_state:
            self.get_logger().info(
                f"[{prev_state} -> {new_state}] id={bond_id}, "
                f"instance_id={instance_id}"
            )

        # 상태 갱신
        self.bonds[bond_id] = {
            "last_seen": now,
            "timeout": timeout,
            "active": is_active,
            "working": is_working,
            "instance_id": instance_id,
            "state": new_state,
        }
        
    def check_dead_bonds(self):
        now = self.now_sec()
        
        for bond_id, info in list(self.bonds.items()):
            # 정상 inactive 상태는 DEAD로 보지 않음
            if not info["active"]:
                continue

            # 이미 DEAD로 판단한 것은 반복 경고하지 않음
            if info["state"] == STATE_DEAD:
                continue
            
            elapsed = now - info["last_seen"]
            
            if elapsed > info["timeout"]:
                info["state"] = STATE_DEAD

                self.get_logger().warn(
                    f"[DEAD] id={bond_id}, "
                    f"instance_id={info['instance_id']}, "
                    f"no heartbeat for {elapsed:.2f}s "
                    f"(timeout={info['timeout']:.2f}s)"
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