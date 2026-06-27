#!/usr/bin/env python3
"""
파일명: config.py
역할:
  AMR2 mission_node, yolo_detector_node, beep_node에서 공통으로 사용할 설정값 모음.
  HTTP 서버 통신은 사용하지 않고 ROS2 topic 기반 통신만 사용한다.
"""

# ══════════════════════════════════════
# 기본 네임스페이스
# ══════════════════════════════════════

DEFAULT_NAMESPACE = "/robot9"


# ══════════════════════════════════════
# YOLO 설정
# ══════════════════════════════════════

MODEL_PATH = "/home/rokey/rokey_ws/src/ros_final_pjt_refact/yolov8n.pt"

YOLO_CONF = 0.5

# 현재 YOLO class
# 0: fallen_person
# 1: obstacle_car
# 2: obstacle_dummy
# 3: normal_box
# 4: dirty_box
# 5: normal_person

YOLO_CLASS_NAMES = {
    0: "fallen_person",
    1: "obstacle_car",
    2: "obstacle_dummy",
    3: "normal_box",
    4: "dirty_box",
    5: "normal_person",
}

EMERGENCY_CLASSES = [
    "fallen_person",
]

OBSTACLE_CLASSES = [
    "obstacle_car",
    "obstacle_dummy",
]

INSPECTION_CLASSES = [
    "normal_box",
    "dirty_box",
]

NOTICE_CLASSES = [
    "normal_person",
]


# ══════════════════════════════════════
# Mission 설정
# ══════════════════════════════════════

BATTERY_LOW = 0.20

NAV_TIMEOUT = 120.0

WP_DWELL_SEC = 3.0

STATUS_PUBLISH_INTERVAL_S = 2.0


# ══════════════════════════════════════
# 도킹 전 경유 좌표
# ══════════════════════════════════════

PRE_DOCK_POSE = ([-6.9546, -0.9646], -17.5)

# ══════════════════════════════════════
# ROS Topic 이름
# ══════════════════════════════════════

START_SIGNAL_TOPIC = "/amr2/start_signal"

WAYPOINT_TOPIC = "/amr2/waypoint"

MISSION_STATUS_TOPIC = "mission/status"

MISSION_EVENT_TOPIC = "mission/events"

MISSION_DETECTION_TOPIC = "mission/detection"

MISSION_PHOTO_TOPIC = "mission/photo"

YOLO_DETECTION_TOPIC = "yolo/detections"

YOLO_DISPLAY_TOPIC = "yolo/display"

BATTERY_TOPIC = "battery_state"


# ══════════════════════════════════════
# Beep 제어 토픽
# ══════════════════════════════════════

EMERGENCY_BEEP_TOPIC = "/robot9/emergency_beep_enable"

SUCESSFUL_BEEP_TOPIC = "/robot9/sucessful_beep_enable"