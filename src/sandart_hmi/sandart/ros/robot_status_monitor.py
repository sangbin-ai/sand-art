"""
=========================================================
Sand Art Robot HMI
ros/robot_status_monitor.py

ROS2 RobotStatus Topic -> PySide6 Signal Bridge
=========================================================

구독 토픽:
  /sandart/robot_status

필요 메시지:
  sandart_msgs/msg/RobotStatus.msg
"""

import threading

from PySide6.QtCore import QObject, Signal

try:
    import rclpy
    from sandart_msgs.msg import RobotStatus
except Exception:
    rclpy = None
    RobotStatus = None


class RobotStatusMonitor(QObject):
    """RobotStatus topic을 받아 Qt Signal로 MainWindow에 전달."""

    status_received = Signal(dict)
    log_received = Signal(str)

    def __init__(self):
        super().__init__()
        self.node = None
        self.thread = None
        self.running = False

    def start(self):
        """백그라운드 스레드에서 ROS spin 시작."""

        if rclpy is None or RobotStatus is None:
            self.log_received.emit("[ROS STATUS] rclpy 또는 sandart_msgs import 실패 - status monitor 비활성")
            return

        if self.running:
            return

        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            if not rclpy.ok():
                rclpy.init(args=None)

            self.node = rclpy.create_node("sandart_hmi_robot_status_monitor")
            self.node.create_subscription(
                RobotStatus,
                "/sandart/robot_status",
                self._status_callback,
                10,
            )
            self.log_received.emit("[ROS STATUS] monitor started")
            rclpy.spin(self.node)

        except Exception as e:
            self.log_received.emit(f"[ROS STATUS] monitor failed: {e}")

    def _status_callback(self, msg):
        data = {
            "connected": bool(msg.connected),
            "servo": bool(msg.servo_on),
            "force": bool(msg.force_on),
            "compliance": bool(msg.compliance_on),
            "drawing": bool(msg.drawing),
            "state": str(msg.state),
            "force_value": float(msg.force),
            "speed_value": float(msg.speed),
            "message": str(msg.message),
        }
        self.status_received.emit(data)

    def shutdown(self):
        self.running = False
        try:
            if self.node is not None:
                self.node.destroy_node()
        except Exception:
            pass
