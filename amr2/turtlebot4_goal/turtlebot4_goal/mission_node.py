#!/usr/bin/env python3
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Bool
from sensor_msgs.msg import BatteryState, CompressedImage
from geometry_msgs.msg import PoseStamped

from turtlebot4_goal import YoloDetections
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)

from turtlebot4_goal.config import (
    BATTERY_LOW,
    NAV_TIMEOUT,
    WP_DWELL_SEC,
    STATUS_PUBLISH_INTERVAL_S,
    PRE_DOCK_POSE,
    INITIAL_POSE,
    EMERGENCY_CLASSES,
    INSPECTION_CLASSES,
    OBSTACLE_CLASSES,
    START_SIGNAL_TOPIC,
    WAYPOINT_TOPIC,
    BATTERY_TOPIC,
    YOLO_DETECTION_TOPIC,
    YOLO_DISPLAY_TOPIC,
    MISSION_STATUS_TOPIC,
    MISSION_EVENT_TOPIC,
    MISSION_DETECTION_TOPIC,
    MISSION_PHOTO_TOPIC,
    EMERGENCY_BEEP_TOPIC,
    SUCESSFUL_BEEP_TOPIC,
)


class MissionNode(Node):
    def __init__(self):
        super().__init__("amr2_mission_node")
        ns = self.get_namespace()

        self._lock = threading.Lock()
        self._stop_event = threading.Event()

        self._start_flag = False
        self._return_requested = False

        self._current_goal = None
        self._current_target = None

        self._pending_goal = None
        self._pending_target = None
        self._preempt_requested = False

        self._battery_percent = None
        self._latest_jpeg = None

        self._class_counts = {}
        self._detections = []

        self._robot_status = "CHARGING"

        self.nav = TurtleBot4Navigator()

        if not self.nav.getDockedStatus():
            self.nav.dock()

        initial_xy, initial_yaw = INITIAL_POSE
        self.nav.setInitialPose(
        self._make_pose(initial_xy[0], initial_xy[1], initial_yaw))
        self.nav.waitUntilNav2Active()

        pre_dock_xy, pre_dock_yaw = PRE_DOCK_POSE
        self.pre_dock_pose = self._make_pose(
            pre_dock_xy[0],
            pre_dock_xy[1],
            pre_dock_yaw)

        self.status_pub = self.create_publisher(
            String, self._ns_topic(ns, MISSION_STATUS_TOPIC), 10
        )
        self.event_pub = self.create_publisher(
            String, self._ns_topic(ns, MISSION_EVENT_TOPIC), 10
        )
        self.det_pub = self.create_publisher(
            String, self._ns_topic(ns, MISSION_DETECTION_TOPIC), 10
        )
        self.photo_pub = self.create_publisher(
            CompressedImage, self._ns_topic(ns, MISSION_PHOTO_TOPIC), 10
        )

        self.emergency_beep_pub = self.create_publisher(
            Bool, EMERGENCY_BEEP_TOPIC, 10
        )
        self.sucessful_beep_pub = self.create_publisher(
            Bool, SUCESSFUL_BEEP_TOPIC, 10
        )

        self.create_subscription(String, START_SIGNAL_TOPIC, self._start_cb, 10)
        self.create_subscription(String, WAYPOINT_TOPIC, self._waypoint_cb, 10)

        self.create_subscription(
            BatteryState,
            BATTERY_TOPIC,
            self._battery_cb,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            YoloDetections,
            self._ns_topic(ns, YOLO_DETECTION_TOPIC),
            self._det_cb,
            10,
        )

        self.create_subscription(
            CompressedImage,
            self._ns_topic(ns, YOLO_DISPLAY_TOPIC),
            self._display_cb,
            10,
        )

        self.create_timer(STATUS_PUBLISH_INTERVAL_S, self._publish_status)

    @staticmethod
    def _ns_topic(ns, topic):
        if topic.startswith("/"):
            return topic
        if ns == "/":
            return f"/{topic}"
        return f"{ns}/{topic}"

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        return pose

    def _publish_bool(self, pub, value):
        msg = Bool()
        msg.data = bool(value)
        pub.publish(msg)

    def _emergency_beep_on(self):
        self._publish_bool(self.emergency_beep_pub, True)

    def _emergency_beep_off(self):
        self._publish_bool(self.emergency_beep_pub, False)

    def _sucessful_beep(self):
        self._publish_bool(self.suceesful_beep_pub, True)

    def _start_cb(self, msg):
        with self._lock:
            bp = self._battery_percent

        if bp is not None and bp < BATTERY_LOW:
            self._publish_event(f"START_REJECTED,BATTERY_LOW,{bp * 100:.1f}")
            return

        with self._lock:
            self._start_flag = True

        self._publish_event("START_ACCEPTED")

    def _waypoint_cb(self, msg):
        need_cancel = False
        old_target = None

        try:
            data = json.loads(msg.data)
            target = data["target"]

            if target in ["RETURN", "DONE"]:
                with self._lock:
                    self._return_requested = True
                self.nav.cancelTask()
                self._publish_event(f"{target}_REQUESTED")
                return

            pose = self._make_pose(
                float(data["x"]),
                float(data["y"]),
                float(data["angle"]),
            )

        except Exception as e:
            self.get_logger().error(f"Invalid waypoint msg: {msg.data} / {e}")
            return

        with self._lock:
            self._pending_goal = pose
            self._pending_target = target

            if self._current_goal is not None:
                self._preempt_requested = True
                old_target = self._current_target
                need_cancel = True

        if need_cancel:
            self.nav.cancelTask()
            self._publish_event(f"GOAL_REPLACED,{old_target}->{target}")
        else:
            self._publish_event(f"GOAL_RECEIVED,{target}")

    def _battery_cb(self, msg):
        with self._lock:
            self._battery_percent = msg.percentage

    def _display_cb(self, msg):
        with self._lock:
            self._latest_jpeg = bytes(msg.data)

    def _det_cb(self, msg):
        with self._lock:
            self._detections = []
            self._class_counts = {cc.name: cc.count for cc in msg.class_counts}

            for d in msg.emergencies:
                self._detections.append({"class": d.label, "x": d.x, "y": d.y})

            for d in msg.obstacles:
                self._detections.append({"class": d.label, "x": d.x, "y": d.y})

            for fo in msg.floor_obstacles:
                self._detections.append({
                    "class": fo.label,
                    "x": fo.x,
                    "y": fo.y,
                    "w": fo.width,
                    "h": fo.height,
                })

    def _publish_status(self):
        with self._lock:
            status = self._robot_status

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    def _set_status(self, status):
        with self._lock:
            self._robot_status = status
        self.get_logger().info(f"STATUS: {status}")

    def _publish_event(self, text):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def _publish_photo(self):
        with self._lock:
            jpeg = self._latest_jpeg

        if jpeg is None:
            self._publish_event("PHOTO_SKIPPED,NO_IMAGE")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = jpeg
        self.photo_pub.publish(msg)

    def _publish_detection(self, target):
        with self._lock:
            counts = dict(self._class_counts)
            detections = list(self._detections)

        parts = [target]
        for cls, cnt in counts.items():
            parts.append(f"{cls}:{cnt}")

        msg = String()
        msg.data = ",".join(parts)
        self.det_pub.publish(msg)

        for d in detections:
            cls = d["class"]

            if cls in EMERGENCY_CLASSES:
                self._publish_event(
                    f"DETECTED_EMERGENCY,{cls},{d['x']:.2f},{d['y']:.2f}"
                )

            elif cls == "dirty_box":
                self._publish_event(
                    f"DETECTED_DIRTY_BOX,{cls},{d['x']:.2f},{d['y']:.2f}"
                )

            elif cls == "normal_box":
                self._publish_event(
                    f"DETECTED_NORMAL_BOX,{cls},{d['x']:.2f},{d['y']:.2f}"
                )

            elif cls in OBSTACLE_CLASSES:
                self._publish_event(
                    f"DETECTED_OBSTACLE,{cls},{d['x']:.2f},{d['y']:.2f}"
                )

            elif cls == "normal_person":
                self._publish_event(
                    f"DETECTED_NORMAL_PERSON,{cls},{d['x']:.2f},{d['y']:.2f}"
                )

    def _is_battery_low(self):
        with self._lock:
            bp = self._battery_percent

        if bp is None:
            return False

        return bp < BATTERY_LOW

    def _pop_pending_goal(self):
        with self._lock:
            if self._pending_goal is None:
                return None, None

            pose = self._pending_goal
            target = self._pending_target

            self._pending_goal = None
            self._pending_target = None

            self._current_goal = pose
            self._current_target = target
            self._preempt_requested = False

            return pose, target

    def _clear_current_goal(self):
        with self._lock:
            self._current_goal = None
            self._current_target = None
            self._preempt_requested = False

    def _wait_nav(self, timeout=NAV_TIMEOUT):
        start = time.time()

        while not self.nav.isTaskComplete():
            if self._stop_event.is_set():
                self.nav.cancelTask()
                return "STOP"

            if self._is_battery_low():
                self.nav.cancelTask()
                self._publish_event("BATTERY_LOW")
                return "BATTERY_LOW"

            with self._lock:
                if self._return_requested:
                    self.nav.cancelTask()
                    return "RETURN_REQUESTED"

                if self._preempt_requested:
                    return "PREEMPTED"

            if time.time() - start > timeout:
                self.nav.cancelTask()
                return "TIMEOUT"

            time.sleep(0.05)

        return "OK"

    def _after_sucessful_process(self, target):
        if target in EMERGENCY_CLASSES:
            self._set_status("DETECTING_EMERGENCY")
            
            time.sleep(2.0)
            self._sucessful_beep()
            time.sleep(WP_DWELL_SEC)
            self._publish_photo()
            self._publish_detection(target)
            self._publish_event(f"MISSION_DONE,{target}")
            return

        if target in INSPECTION_CLASSES:
            self._set_status("DETECTING_INSPECTION")
            self._sucessful_beep()
            time.sleep(WP_DWELL_SEC)

            # 나중에 렉 접근 로직 붙일 자리
            # self._approach_rack_by_yolo_or_depth(target)

            self._publish_photo()
            self._publish_detection(target)
            self._publish_event(f"MISSION_DONE,{target}")
            return

        if target in OBSTACLE_CLASSES:
            self._set_status("DETECTING_OBSTACLE")
            self._sucessful_beep()
            time.sleep(WP_DWELL_SEC)
            self._publish_photo()
            self._publish_detection(target)
            self._publish_event(f"MISSION_DONE,{target}")
            return

        self._set_status("DETECTING_UNKNOWN")
        self._sucessful_beep()
        time.sleep(WP_DWELL_SEC)
        self._publish_photo()
        self._publish_detection(target)
        self._publish_event(f"MISSION_DONE,{target}")

    def _execute_goal(self, pose, target):
        emergency_moving = target in EMERGENCY_CLASSES

        if emergency_moving:
            self._set_status("MOVING_EMERGENCY")
            self._emergency_beep_on()
        else:
            self._set_status("MOVING_WAYPOINT")

        self.nav.goToPose(pose)
        result = self._wait_nav()

        if emergency_moving:
            self._emergency_beep_off()

        if result == "OK":
            self._after_sucessful_process(target)
            self._clear_current_goal()
            return "OK"

        if result == "PREEMPTED":
            self._publish_event(f"GOAL_PREEMPTED,{target}")
            self._clear_current_goal()
            return "PREEMPTED"

        if result == "BATTERY_LOW":
            self._clear_current_goal()
            return "BATTERY_LOW"

        if result == "RETURN_REQUESTED":
            self._clear_current_goal()
            return "RETURN_REQUESTED"

        if result == "TIMEOUT":
            self._publish_event(f"GOAL_TIMEOUT,{target}")
            self._clear_current_goal()
            return "TIMEOUT"

        self._clear_current_goal()
        return result

    def _return_and_dock(self):
        self._emergency_beep_off()
        self._set_status("RETURNING")

        self.nav.goToPose(self.pre_dock_pose)

        start = time.time()
        while not self.nav.isTaskComplete():
            if time.time() - start > NAV_TIMEOUT:
                self.nav.cancelTask()
                self._publish_event("PRE_DOCK_TIMEOUT")
                break
            time.sleep(0.05)

        self._set_status("DOCKING")
        self.nav.dock()
        self._set_status("CHARGING")

    def run_mission(self):
        while not self._stop_event.is_set():
            with self._lock:
                if self._start_flag:
                    break
            time.sleep(0.1)

        if self._stop_event.is_set():
            return

        if self._is_battery_low():
            self._publish_event("START_REJECTED,BATTERY_LOW")
            return

        self.nav.undock()
        self._set_status("UNDOCKED")

        while not self._stop_event.is_set():
            if self._is_battery_low():
                self._publish_event("BATTERY_LOW")
                break

            with self._lock:
                if self._return_requested:
                    break

            pose, target = self._pop_pending_goal()

            if pose is None:
                self._set_status("WAITING_WAYPOINT")
                time.sleep(0.1)
                continue

            result = self._execute_goal(pose, target)

            if result in ["BATTERY_LOW", "RETURN_REQUESTED", "STOP"]:
                break

        self._return_and_dock()

    def destroy_node(self):
        self._stop_event.set()
        self._emergency_beep_off()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionNode()

    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_mission()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop_event.set()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()