#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sandart_movesx_node.py  (v3 로직 + 서비스 통신)

두산 M0609 + ROS2 + DSR_ROBOT2 샌드아트 실행 엔진

핵심 목표:
  - 좌표 경로를 부드럽게 따라 그림
  - Rz 진행방향 추종 기능 완전 제거
  - Rx, Ry, Rz는 고정 자세 사용
  - Stroke 단위로 끊어서 그림
  - Stroke 사이에는 SAFE_Z로 상승 후 다음 Stroke 시작점으로 이동

[v3 크리티컬 패스 - 유지]
  force로 하강하며 접촉 감지(MAX_WAIT까지 대기), 접촉 순간
  get_current_posx()로 실측 z를 읽어 movesx 전체 waypoint z에 반영.


유지 기능:
  1) 중복 좌표 제거
  2) Centripetal Catmull-Rom Spline
  3) Uniform Sampling
  4) Lead In / Lead Out
  5) movesx chunk 실행
  6) SAFE_Z 접근/복귀
  7) Compliance / Force Control (접촉 실측 z 반영)
"""

import math
import os
import rclpy
import DR_init
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor                     # [SRV] 서비스 노드용

import uuid
from sandart_msgs.msg import Bond
from sandart_msgs.msg import DrawingProgress
from sandart_msgs.srv import PathPlanList       # [SRV] path 수신 서비스 타입
from sandart_msgs.srv import StartDrawing
from std_srvs.srv import Trigger
# ================================================================
# [1] Robot Config
# ================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ================================================================
# [2] Input / Run Config
# ================================================================
RUN_ROBOT = True

# ================================================================
# [3] Pose Config  (v3 값 유지)
# ================================================================
SAFE_Z = 200.0
DRAW_Z = 83.95

# Force를 켜기 전 바로 DRAW_Z로 박지 않고, DRAW_Z보다 살짝 위에서 켠 뒤 천천히 내려감
FORCE_APPROACH_OFFSET_Z = 10.0

# 실측 접촉 z가 DRAW_Z에서 이 값 이상 벗어나면 경고 출력 (센서 오감지 의심용)
CONTACT_Z_WARN_DEV = 15.0

# Rz 추종 제거. 아래 자세로 고정해서 그림.
BASE_RX = 83.34
BASE_RY = -177.62
BASE_RZ = 85.14

# ================================================================
# [4] Motion Config
# ================================================================
VEL_DEFAULT = 110
ACC_DEFAULT = 110

APPROACH_VEL = 20
APPROACH_ACC = 20

SAMPLE_DIST_MM = 1.0
MOVESX_CHUNK_SIZE = 20

MIN_WAYPOINT_COUNT = 4
MIN_STROKE_POINT_COUNT = 2
DUPLICATE_DIST_MM = 0.2

CR_ALPHA = 0.5
SPLINE_STEPS_PER_SEG = 20

USE_LEAD_IN_OUT = True
LEAD_DIST = 3.0

# ================================================================
# [4-1] Pen Change Config
# ================================================================

PEN_CHANGE_VEL = 40
PEN_CHANGE_ACC = 40
PEN_GRIP_WAIT = 0.5

# 그리퍼 Digital Output 번호
# 실제 현재 사용하는 그리퍼 출력 번호와 다르면 이 값만 변경
GRIP_CLOSE_DO = 1
GRIP_OPEN_DO = 2

# 각 펜꽂이 좌표
#
# approach:
#   펜꽂이 바로 위의 안전 위치
#
# grip:
#   실제로 펜을 잡거나 내려놓는 위치
#
# [x, y, z, rx, ry, rz]
PEN_HOLDER_POSES = {
    1: {
        "approach":  [652.17,-85.69 , 87.77, 99.17, -175.71, 139.46],
        "grip":     [652.17,-85.69 , 37.77, 99.17, -175.71, 139.46],
    },
    2: {
        "approach": [629.45 , 37.80 , 83.21 , 177.92 , 179.16 , 177.34],
        "grip":     [629.45 , 37.80 , 33.21 , 177.92 , 179.16 , 177.34],
    },
    3: {
        "approach": [668.65, 40.47 , 86.56 , 41.09 , -175.07 , 112.54],
        "grip":     [668.65, 40.47 , 36.56 , 41.09 , -175.07 , 112.54],
    },
}

# ================================================================
# [5] Force / Compliance Config  (v3 값 유지)
# ================================================================
USE_FORCE_CONTROL = True
DRAW_FORCE = 1.5
DRAW_SPEED = 60

# 순응제어 강성값 [x, y, z, rx, ry, rz]
# Z값이 작을수록 위아래로 더 부드럽게 순응함.
COMPLIANCE_STX = [3000, 3000, 100, 200, 200, 200]

# 목표 힘 [Fx, Fy, Fz, Mx, My, Mz]
DESIRED_FORCE_FD = [0, 0, -0.5, 0, 0, 0]
DESIRED_FORCE_DIR = [0, 0, 1, 0, 0, 0]
FORCE_SETTLE_WAIT = 0.3

# ================================================================
# [6] Basic Math Utils
# ================================================================
def dist2d(a, b):
    """2D 두 점 사이 거리 계산."""
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    return math.sqrt(dx * dx + dy * dy)


def path_length(points):
    """2D 경로 전체 길이 계산."""
    if len(points) < 2:
        return 0.0

    total = 0.0
    for i in range(1, len(points)):
        total += dist2d(points[i - 1], points[i])

    return total


def remove_near_duplicate_points(points, min_dist=DUPLICATE_DIST_MM):
    """너무 가까운 중복 좌표 제거."""
    if len(points) < 2:
        return points

    cleaned = [points[0]]

    for p in points[1:]:
        if dist2d(cleaned[-1], p) >= min_dist:
            cleaned.append(p)

    return cleaned


# ================================================================
# [8] SRV / Custom Msg 변환
# ================================================================
def points_from_sand_stroke(stroke_msg):
    """SandStroke.msg -> [[x, y], ...]"""
    points = []

    for p in stroke_msg.points:
        points.append([float(p.x), float(p.y)])

    points = remove_near_duplicate_points(points)
    return points


def path_from_sand_strokes(stroke_msgs):
    """SandStroke[] -> [ [[x,y],...], [[x,y],...], ... ]"""
    path = []

    for stroke_msg in stroke_msgs:
        points = points_from_sand_stroke(stroke_msg)
        if len(points) >= MIN_STROKE_POINT_COUNT:
            path.append(points)

    return path

# ================================================================
# [9] Centripetal Catmull-Rom Spline
# ================================================================
def tj(ti, pi, pj, alpha=CR_ALPHA):
    """Centripetal Catmull-Rom parameter 계산."""
    d = dist2d(pi, pj)

    if d < 0.000001:
        d = 0.000001

    return ti + math.pow(d, alpha)


def lerp_point(p0, p1, t):
    """2D 선형 보간."""
    return [
        p0[0] + (p1[0] - p0[0]) * t,
        p0[1] + (p1[1] - p0[1]) * t,
    ]


def safe_ratio(t, ta, tb):
    """0으로 나누는 것 방지용 비율 계산."""
    denom = tb - ta

    if abs(denom) < 0.000001:
        return 0.0

    return (t - ta) / denom


def centripetal_catmull_rom_segment(p0, p1, p2, p3, steps):
    """p1 -> p2 구간을 Centripetal Catmull-Rom으로 보간."""
    t0 = 0.0
    t1 = tj(t0, p0, p1)
    t2 = tj(t1, p1, p2)
    t3 = tj(t2, p2, p3)

    result = []

    for i in range(steps):
        ratio = i / float(steps)
        t = t1 + (t2 - t1) * ratio

        a1 = lerp_point(p0, p1, safe_ratio(t, t0, t1))
        a2 = lerp_point(p1, p2, safe_ratio(t, t1, t2))
        a3 = lerp_point(p2, p3, safe_ratio(t, t2, t3))

        b1 = lerp_point(a1, a2, safe_ratio(t, t0, t2))
        b2 = lerp_point(a2, a3, safe_ratio(t, t1, t3))

        c = lerp_point(b1, b2, safe_ratio(t, t1, t2))
        result.append(c)

    return result


def centripetal_catmull_rom_spline(points):
    """듬성듬성한 waypoint를 부드러운 spline 점들로 변환."""
    if len(points) < 4:
        return points

    result = []

    # 양 끝을 복제해서 첫/끝 구간도 안정적으로 보간
    extended = [points[0]] + points + [points[-1]]

    for i in range(1, len(extended) - 2):
        p0 = extended[i - 1]
        p1 = extended[i]
        p2 = extended[i + 1]
        p3 = extended[i + 2]

        seg_points = centripetal_catmull_rom_segment(
            p0, p1, p2, p3, SPLINE_STEPS_PER_SEG)

        result.extend(seg_points)

    result.append(points[-1])
    result = remove_near_duplicate_points(result, min_dist=0.05)

    return result

# ================================================================
# [10] Uniform Sampling
# ================================================================
def uniform_resample(points, step_mm=SAMPLE_DIST_MM):
    """Spline 결과를 일정 간격으로 다시 샘플링."""
    if len(points) < 2:
        return points

    sampled = [points[0]]

    prev = points[0]
    i = 1
    accumulated = 0.0

    while i < len(points):
        curr = points[i]
        seg_len = dist2d(prev, curr)

        if seg_len < 0.0001:
            prev = curr
            i += 1
            continue

        if accumulated + seg_len >= step_mm:
            remain = step_mm - accumulated
            ratio = remain / seg_len

            nx = prev[0] + (curr[0] - prev[0]) * ratio
            ny = prev[1] + (curr[1] - prev[1]) * ratio

            new_point = [nx, ny]
            sampled.append(new_point)

            prev = new_point
            accumulated = 0.0

        else:
            accumulated += seg_len
            prev = curr
            i += 1

    if dist2d(sampled[-1], points[-1]) > 0.5:
        sampled.append(points[-1])

    return sampled

# ================================================================
# [11] Lead In / Lead Out
# ================================================================
def calc_lead_point(p0, p1, distance):
    """p0 -> p1 방향으로 distance 만큼 이동한 점 생성."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    length = math.sqrt(dx * dx + dy * dy)

    if length < 0.001:
        return p0.copy()

    ux = dx / length
    uy = dy / length

    return [
        p0[0] + ux * distance,
        p0[1] + uy * distance,
    ]


def insert_lead_in_out_points(points):
    """좌표 앞뒤에 Lead In / Out 점 추가."""
    if not USE_LEAD_IN_OUT:
        return points

    if len(points) < 3:
        return points

    start = points[0]
    next_pt = points[1]
    prev = points[-2]
    end = points[-1]

    lead_in = calc_lead_point(start, next_pt, -LEAD_DIST)
    lead_out = calc_lead_point(prev, end, LEAD_DIST)

    return [lead_in] + points + [lead_out]

# ================================================================
# [12] Path / Motion Builder
# ================================================================
def build_smooth_points_from_waypoints(waypoints):
    """waypoint -> spline -> uniform sampling -> lead in/out."""
    waypoints = remove_near_duplicate_points(waypoints)

    if len(waypoints) < MIN_STROKE_POINT_COUNT:
        return []

    spline_points = centripetal_catmull_rom_spline(waypoints)
    sampled_points = uniform_resample(spline_points, SAMPLE_DIST_MM)
    final_points = insert_lead_in_out_points(sampled_points)

    return final_points


def build_motion_from_points(points):
    """
    sampled point -> motion [x, y, z, rx, ry, rz].

    z는 일단 DRAW_Z로 채워두고, 실행 시점에 실측 접촉 z로 교체됨.
    """
    if len(points) < 2:
        return []

    motion = []

    for p in points:
        motion.append([
            p[0],
            p[1],
            DRAW_Z,
            BASE_RX,
            BASE_RY,
            BASE_RZ,
        ])

    return motion


def build_motion_from_stroke(stroke_points):
    """stroke 좌표 1개를 smooth point와 motion으로 변환."""
    smooth_points = build_smooth_points_from_waypoints(stroke_points)
    motion = build_motion_from_points(smooth_points)
    return motion

# ================================================================
# [13] DSR Pos / Chunk Utils
# ================================================================
def motion_to_posx(m):
    """motion [x,y,z,rx,ry,rz] -> DSR posx."""
    return posx(m[0], m[1], m[2], m[3], m[4], m[5])


def motion_to_pos_list(motion, z_override=None):
    """motion 전체를 movesx용 posx 리스트로 변환.

    z_override가 주어지면 모든 waypoint의 z를 그 값으로 교체.
    (force 접촉으로 실측한 z를 주행에 반영하기 위함)
    """
    if z_override is None:
        return [motion_to_posx(m) for m in motion]
    return [posx(m[0], m[1], z_override, m[3], m[4], m[5]) for m in motion]


def split_pos_list_to_chunks(pos_list, chunk_size=MOVESX_CHUNK_SIZE):
    """pos_list를 movesx용 chunk로 분할. chunk 사이 마지막 점 1개를 겹침."""
    chunks = []

    if len(pos_list) < 2:
        return chunks

    start = 0

    while start < len(pos_list):
        end = min(start + chunk_size, len(pos_list))
        chunk = pos_list[start:end]

        if len(chunk) >= 4:
            chunks.append(chunk)

        if end >= len(pos_list):
            break

        start = end - 1

    return chunks

# ================================================================
# [14] Force / Compliance Wrapper
# ================================================================
def enable_force_control():
    """Compliance와 Force를 켠다."""
    if not USE_FORCE_CONTROL:
        print("[FORCE] disabled by config")
        return

    print("[FORCE] enable compliance")

    try:
        task_compliance_ctrl(stx=COMPLIANCE_STX)
    except TypeError:
        task_compliance_ctrl(COMPLIANCE_STX)

    wait(0.2)

    fd = [0, 0, -DRAW_FORCE, 0, 0, 0]

    print(f"[FORCE] set desired force: fd={fd}, dir={DESIRED_FORCE_DIR}")

    try:
        set_desired_force(fd=fd, dir=DESIRED_FORCE_DIR)
    except TypeError:
        set_desired_force(DESIRED_FORCE_FD, DESIRED_FORCE_DIR)

    wait(FORCE_SETTLE_WAIT)


def disable_force_control():
    """Force와 Compliance를 안전하게 해제한다."""
    if not USE_FORCE_CONTROL:
        return

    try:
        print("[FORCE] release force")
        release_force()
    except Exception as e:
        print(f"[WARN] release_force failed: {e}")

    wait(0.1)

    try:
        print("[FORCE] release compliance")
        release_compliance_ctrl()
    except Exception as e:
        print(f"[WARN] release_compliance_ctrl failed: {e}")

    wait(0.1)

# ================================================================
# [14-1] Pen Change
# ================================================================
def gripper_close():
    """펜을 잡는다."""
    set_digital_output(GRIP_OPEN_DO, 0)
    wait(0.1)

    set_digital_output(GRIP_CLOSE_DO, 1)
    wait(PEN_GRIP_WAIT)


def gripper_open():
    """펜을 놓는다."""
    set_digital_output(GRIP_CLOSE_DO, 0)
    wait(0.1)

    set_digital_output(GRIP_OPEN_DO, 1)
    wait(PEN_GRIP_WAIT)


def move_pen_pose(pose):
    """펜 교체 위치로 직선 이동한다."""
    movel(
        posx(*pose),
        vel=PEN_CHANGE_VEL,
        acc=PEN_CHANGE_ACC,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
    )


def pick_pen(path_idx, node=None):
    """
    path_idx에 해당하는 펜꽂이에서 펜을 집는다.

    순서:
      1. 펜꽂이 위 approach 위치
      2. 그리퍼 열기
      3. 펜 위치까지 하강
      4. 그리퍼 닫기
      5. approach 위치로 상승
    """
    holder = PEN_HOLDER_POSES[path_idx]

    print(f"[PEN] PATH {path_idx} 펜 집기 시작")

    move_pen_pose(holder["approach"])
    wait_motion_near_finish(node) 
    wait(1.0)

    gripper_open()

    move_pen_pose(holder["grip"])
    wait_motion_near_finish(node)
    
    gripper_close()

    move_pen_pose(holder["approach"])
    wait_motion_near_finish(node)

    print(f"[PEN] PATH {path_idx} 펜 집기 완료")


def return_pen(path_idx, node=None):
    """
    사용한 펜을 원래 펜꽂이에 돌려놓는다.

    순서:
      1. 펜꽂이 위 approach 위치
      2. 펜 위치까지 하강
      3. 그리퍼 열기
      4. approach 위치로 상승
    """
    holder = PEN_HOLDER_POSES[path_idx]

    print(f"[PEN] PATH {path_idx} 펜 반환 시작")

    move_pen_pose(holder["approach"])
    wait_motion_near_finish(node)
    wait(1.0)

    move_pen_pose(holder["grip"])
    wait_motion_near_finish(node)

    gripper_open()

    move_pen_pose(holder["approach"])
    wait_motion_near_finish(node)

    print(f"[PEN] PATH {path_idx} 펜 반환 완료")

# ================================================================
# [15] Robot Engine - movesx  (v3 크리티컬 패스 유지)
# ================================================================
STATUS_IDLE = 0

def wait_motion_near_finish(node=None):
    """현재 motion을 확인하며 Pause 상태에서는 다음 진행을 대기한다."""

    pause_logged = False

    while True:

        if node is not None and node.pause_requested:

            if not pause_logged:
                print("[PAUSE] Waiting before next motion...")
                pause_logged = True

            wait(0.05)
            continue

        if pause_logged:
            print("[RESUME] Motion sequence resumed")
            pause_logged = False

        status = check_motion()

        if status == STATUS_IDLE:
            break

        wait(0.01)
def call_movesx_chunk(chunk, vel=DRAW_SPEED, acc=DRAW_SPEED):
    """movesx 호출 래퍼."""
    movesx(
        chunk,
        vel=vel,
        acc=acc,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
    )


def execute_motion_with_movesx(
    motion,
    label="stroke",
    progress_state=None,
    progress_callback=None,
    node=None,
):
    """
    motion 하나를 실제 로봇으로 실행. (v3 로직 그대로)

    순서:
      1) 시작점 SAFE_Z 이동
      2) DRAW_Z + FORCE_APPROACH_OFFSET_Z 위치까지 접근
      3) Compliance / Force ON
      4) Force로 하강하며 접촉 감지 + 실측 접촉 z 저장
      5) 실측 접촉 z 기준으로 movesx chunk 실행
      6) Force / Compliance OFF
      7) 끝점 SAFE_Z 상승
    """
    if len(motion) < 4:
        print(f"[SKIP] {label}: motion too short ({len(motion)})")
        return

    force_enabled = False

    print("\n================ ROBOT EXECUTE ================")
    print(f"target        = {label}")
    print(f"motion points = {len(motion)}")

    try:
        # 1) 시작점 SAFE_Z 이동
        start = motion[0].copy()
        start_safe = start.copy()
        start_safe[2] = SAFE_Z

        print("[ROBOT] move to start safe z")
        movel(
            motion_to_posx(start_safe),
            vel=VEL_DEFAULT,
            acc=ACC_DEFAULT,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        wait_motion_near_finish(node)

        # 2) DRAW_Z보다 살짝 위로 접근
        start_pre_force = start.copy()
        start_pre_force[2] = DRAW_Z + FORCE_APPROACH_OFFSET_Z

        print("[ROBOT] approach pre-force z")
        movel(
            motion_to_posx(start_pre_force),
            vel=APPROACH_VEL,
            acc=APPROACH_ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        wait_motion_near_finish(node)

        # 3) Compliance / Force ON
        enable_force_control()
        force_enabled = True

        # Force ON 상태에서 DRAW_Z까지 천천히 내려간다.
        print("[ROBOT] force approach draw z")

        movel(
            motion_to_posx(start),
            vel=APPROACH_VEL,
            acc=APPROACH_ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        # 4) Force 켠 상태에서 접촉 감지 + 실측 z 저장
        print("[FORCE] searching contact by force only...")

        MAX_WAIT = 200.0
        CHECK_DT = 0.05
        elapsed = 0.0
        contact_z = None  # 접촉 순간 실측 z를 담을 변수

        while elapsed < MAX_WAIT:
            if check_force_condition(DR_AXIS_Z, min=DRAW_FORCE, ref=DR_BASE) == 0:
                print("[FORCE] Contact detected")

                # 접촉을 확인만 하지 않고, 그 순간의 실제 z를 읽어서 저장
                # get_current_posx는 (posx, sol) 튜플 반환 -> posx[2]가 z
                cur_posx, _sol = get_current_posx(ref=DR_BASE)
                if cur_posx is not None:
                    contact_z = float(cur_posx[2])
                    dev = contact_z - DRAW_Z
                    print(f"[FORCE] contact z = {contact_z:.3f} "
                          f"(DRAW_Z={DRAW_Z}, 편차 {dev:+.3f} mm)")
                    if abs(dev) > CONTACT_Z_WARN_DEV:
                        print(f"[WARN] 접촉 z 편차가 {CONTACT_Z_WARN_DEV}mm 초과. "
                              f"센서 오감지 또는 표면 높이 확인 필요")
                else:
                    print("[WARN] get_current_posx 실패, DRAW_Z로 대체 진행")
                break

            wait(CHECK_DT)
            elapsed += CHECK_DT
        else:
            print("[ERROR] Contact not found")
            disable_force_control()
            return

        # 5) movesx 실행 - 실측 접촉 z로 전체 waypoint z 교체
        draw_z_actual = contact_z if contact_z is not None else DRAW_Z
        print(f"[ROBOT] drawing at z = {draw_z_actual:.3f}")
        pos_list = motion_to_pos_list(motion, z_override=draw_z_actual)
        chunks = split_pos_list_to_chunks(pos_list)

        print(f"chunks = {len(chunks)}")
        print(f"chunk size = {MOVESX_CHUNK_SIZE}")

        for idx, chunk in enumerate(chunks):

            # STOP 요청
            if node is not None and node.stop_requested:
                print("[STOP] Drawing stopped")
                return

            # PAUSE 요청
            while node is not None and node.pause_requested:
                wait(0.05)

            call_movesx_chunk(
                chunk,
                vel=DRAW_SPEED,
                acc=DRAW_SPEED,
            )

            wait_motion_near_finish(node)

            if progress_state is not None:
                progress_state["current"] += len(chunk)

                total_points = max(1, progress_state["total"])
                percent = int(
                    progress_state["current"] * 100 / total_points
                )
                percent = max(0, min(99, percent))

                if percent > progress_state["last_percent"]:
                    progress_state["last_percent"] = percent

                    if progress_callback is not None:
                        progress_callback(
                            percent,
                            progress_state["current"],
                            total_points,
                            label,
                        )

        # 6) Force / Compliance OFF
        disable_force_control()
        force_enabled = False

        # 7) 끝점 SAFE_Z 상승
        last = motion[-1].copy()
        last[2] = SAFE_Z

        print("[ROBOT] lift to safe z")
        movel(
            motion_to_posx(last),
            vel=APPROACH_VEL,
            acc=APPROACH_ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )

        print("[ROBOT] execute done")
        print("===============================================\n")

    except Exception:
        if force_enabled:
            disable_force_control()
        raise

# ================================================================
# [16] Stroke / Path / Response Execute
# ================================================================
def execute_stroke(
    stroke_points,
    path_idx=1,
    stroke_idx=1,
    stroke_total=1,
    progress_state=None,
    progress_callback=None,
    node=None,
):
    """stroke 1개를 독립적으로 그림. 끝나면 반드시 SAFE_Z로 상승."""
    if len(stroke_points) < MIN_STROKE_POINT_COUNT:
        print(f"[SKIP] PATH {path_idx} STROKE {stroke_idx}: points too short")
        return

    label = f"PATH {path_idx} / STROKE {stroke_idx}/{stroke_total}"

    print("\n------------------------------------------------")
    print(f"[{label}]")
    print(f"raw points    = {len(stroke_points)}")
    print(f"raw length    = {path_length(stroke_points):.2f} mm")

    motion = build_motion_from_stroke(stroke_points)

    if len(motion) < 4:
        print(f"[SKIP] {label}: motion too short after smoothing")
        return

    print(f"motion points = {len(motion)}")

    execute_motion_with_movesx(
        motion,
        label=label,
        progress_state=progress_state,
        progress_callback=progress_callback,
        node=node,
    )

def execute_path(
    path_strokes,
    path_idx=1,
    progress_state=None,
    progress_callback=None,
    node=None,
):
    """path 1개 안의 stroke들을 순서대로 실행."""
    if path_strokes is None or len(path_strokes) == 0:
        print(f"[SKIP] PATH {path_idx}: empty")
        return

    print("\n================================================")
    print(f"PATH {path_idx} START")
    print(f"strokes = {len(path_strokes)}")
    print("================================================")

    total = len(path_strokes)

    for i, stroke_points in enumerate(path_strokes):
        execute_stroke(
            stroke_points,
            path_idx=path_idx,
            stroke_idx=i + 1,
            stroke_total=total,
            progress_state=progress_state,
            progress_callback=progress_callback,
            node=node,
        )

    print("\n================================================")
    print(f"PATH {path_idx} DONE")
    print("================================================\n")
def count_total_motion_points(path1, path2, path3):
    total = 0

    for path in [path1, path2, path3]:
        if path is None:
            continue

        for stroke_points in path:
            motion = build_motion_from_stroke(stroke_points)
            total += len(motion)

    return total
def execute_response_paths(
    path1,
    path2,
    path3,
    progress_callback=None,
    node=None,
):
    """path1 -> path2 -> path3 순서로 실행."""

    total_points = count_total_motion_points(path1, path2, path3)

    progress_state = {
        "current": 0,
        "total": max(1, total_points),
        "last_percent": -1,
    }

    if progress_callback is not None:
        progress_callback(
            0,
            0,
            progress_state["total"],
            "START",
        )

    # ============================================================
    # PATH 1
    # ============================================================
    if path1:
        pick_pen(1, node=node)

        execute_path(
            path1,
            path_idx=1,
            progress_state=progress_state,
            progress_callback=progress_callback,
            node=node,
        )

        return_pen(1, node=node)

    # ============================================================
    # PATH 2
    # ============================================================
    if path2:
        pick_pen(2, node=node)

        execute_path(
            path2,
            path_idx=2,
            progress_state=progress_state,
            progress_callback=progress_callback,
            node=node,
        )

        return_pen(2, node=node)

    # ============================================================
    # PATH 3
    # ============================================================
    if path3:
        pick_pen(3, node=node)

        execute_path(
            path3,
            path_idx=3,
            progress_state=progress_state,
            progress_callback=progress_callback,
            node=node,
        )

        # 마지막 펜도 원래 펜꽂이에 반환
        return_pen(3, node=node)

    if progress_callback is not None:
        progress_callback(
            100,
            progress_state["total"],
            progress_state["total"],
            "DONE",
        )
# ================================================================
# [17] Service Node  # [SRV] 서비스 수신 노드
# ================================================================
class SandEngineNode(Node):
    """path_plan_node가 보낸 PathPlanList 요청을 받아 로봇 실행."""

    def __init__(self):
        super().__init__("sandart_movesx_node", namespace=ROBOT_ID)

        # Drawing 콜백과 Pause/Resume 콜백을 서로 다른 그룹으로 분리합니다.
        # start_drawing이 로봇 동작을 수행하는 동안에도 제어 서비스를 받을 수 있습니다.
        self.drawing_callback_group = MutuallyExclusiveCallbackGroup()
        self.control_callback_group = MutuallyExclusiveCallbackGroup()

        self.stop_requested = False
        self.pause_requested = False
        self.srv = self.create_service(
            PathPlanList,
            "path_plan_list",
            self.handle_path_plan_list,
            callback_group=self.drawing_callback_group,
        )
        self.get_logger().info("path_plan_list 서비스 대기 중...")

        self.start_srv = self.create_service(
            StartDrawing,
            "start_drawing",
            self.handle_start_drawing,
            callback_group=self.drawing_callback_group,
        )
        self.pause_srv = self.create_service(
            Trigger,
            "pause_drawing",
            self.handle_pause,
            callback_group=self.control_callback_group,
        )

        self.resume_srv = self.create_service(
            Trigger,
            "resume_drawing",
            self.handle_resume,
            callback_group=self.control_callback_group,
        )
        self.get_logger().info("start_drawing 서비스 대기 중...")

        self.saved_path1 = []
        self.saved_path2 = []
        self.saved_path3 = []

        # HMI Parameter 기본값
        # 실제 speed / force / tool 값은 handle_start_drawing()에서
        # StartDrawing.srv 요청을 받을 때 갱신한다.
        self.draw_tool = "MEDIUM"

        self.bond_id = "sandart_movesx_node_to_lifecycle"
        self.bond_instance_id = str(uuid.uuid4())
        self.heartbeat_timeout = 6.0
        self.heartbeat_period = 0.1
        self.is_working = False
        self.is_active = True

        self.bond_pub = self.create_publisher(Bond, "/bond", 10)
        self.progress_pub = self.create_publisher(
            DrawingProgress,
            "/drawing_progress",
            10,
        )
        self.bond_timer = self.create_timer(0.1, self.publish_bond)
    def handle_pause(self, request, response):
        """현재 chunk가 끝난 뒤 다음 chunk 실행을 대기시킨다."""

        if not self.is_working:
            response.success = False
            response.message = "Drawing is not running"
            self.get_logger().warn("[DRAW] Pause rejected: drawing is not running")
            return response

        if self.pause_requested:
            response.success = True
            response.message = "Already paused"
            self.get_logger().info("[DRAW] Already paused")
            return response

        self.pause_requested = True

        self.get_logger().info(
            "[DRAW] Pause requested - waiting after current chunk"
        )

        response.success = True
        response.message = "Pause requested"

        return response


    def handle_resume(self, request, response):
        """Pause 대기를 해제하고 다음 chunk부터 계속 실행한다."""

        if not self.is_working:
            response.success = False
            response.message = "Drawing is not running"
            self.get_logger().warn("[DRAW] Resume rejected: drawing is not running")
            return response

        if not self.pause_requested:
            response.success = True
            response.message = "Already running"
            self.get_logger().info("[DRAW] Already running")
            return response

        self.pause_requested = False

        self.get_logger().info("[DRAW] Resume requested")

        response.success = True
        response.message = "Resumed"

        return response
    def publish_bond(self):
        msg = Bond()
        msg.id = self.bond_id
        msg.instance_id = self.bond_instance_id
        msg.working = self.is_working
        msg.active = self.is_active
        msg.heartbeat_timeout = self.heartbeat_timeout
        msg.heartbeat_period = self.heartbeat_period
        self.bond_pub.publish(msg)
    
    def publish_progress(
        self,
        percent,
        current,
        total,
        status="DRAWING",
    ):
        msg = DrawingProgress()

        msg.percent = int(max(0, min(100, percent)))
        msg.current_point = int(current)
        msg.total_point = int(total)
        msg.status = str(status)

        self.progress_pub.publish(msg)

    def handle_path_plan_list(self, request, response):
        self.is_working = True
        self.publish_bond()

        try:
            path1 = path_from_sand_strokes(request.path1)
            path2 = path_from_sand_strokes(request.path2)
            path3 = path_from_sand_strokes(request.path3)
        except Exception as e:
            self.is_working = False
            self.publish_bond()
            response.accepted = False
            response.message = f"경로 변환 실패: {e}"
            self.get_logger().error(response.message)
            return response

        total = len(path1) + len(path2) + len(path3)

        if total == 0:
            self.is_working = False
            self.publish_bond()
            response.accepted = False
            response.message = "받은 path가 비어 있습니다."
            self.get_logger().warn(response.message)
            return response

        self.saved_path1 = path1
        self.saved_path2 = path2
        self.saved_path3 = path3

        self.is_working = False
        self.publish_bond()

        response.accepted = True
        response.message = (
            f"Path Ready: path1={len(path1)}, "
            f"path2={len(path2)}, path3={len(path3)}"
        )

        self.get_logger().info(
            f"[READY] {response.message}. Waiting start_drawing..."
        )

        return response
    
    def handle_start_drawing(self, request, response):
        total = (
            len(self.saved_path1)
            + len(self.saved_path2)
            + len(self.saved_path3)
        )

        if total == 0:
            response.success = False
            response.message = "No path loaded. Process image first."
            self.get_logger().warn(response.message)
            return response

        if self.is_working:
            response.success = False
            response.message = "Drawing already running."
            self.get_logger().warn(response.message)
            return response

        self.is_working = True
        self.publish_bond()
        # HMI Parameter 저장
        global DRAW_SPEED
        global DRAW_FORCE
        global VEL_DEFAULT
        global ACC_DEFAULT
        global SAMPLE_DIST_MM

        DRAW_SPEED = float(request.draw_speed)

        VEL_DEFAULT = float(request.robot_speed)
        ACC_DEFAULT = float(request.robot_speed)

        # DSR 내부 기본 속도/가속도도 같이 갱신
        set_velx(VEL_DEFAULT, VEL_DEFAULT)
        set_accx(ACC_DEFAULT, ACC_DEFAULT)

        DRAW_FORCE = float(request.force)

        SAMPLE_DIST_MM = float(request.sampling)

        self.draw_tool = request.tool
        self.get_logger().info(
            f"[PARAM] draw_speed={DRAW_SPEED}, "
            f"robot_speed={VEL_DEFAULT}, "
            f"force={DRAW_FORCE}, "
            f"sampling={SAMPLE_DIST_MM}, "
            f"tool={self.draw_tool}"
        )
        self.get_logger().info("[DRAW] start_drawing accepted")
        self.publish_progress(
            0,
            0,
            total,
            "START",
        )
        # 새 Drawing 시작 시 Pause 상태를 반드시 초기화합니다.
        self.stop_requested = False
        self.pause_requested = False

        try:
            if RUN_ROBOT:
                execute_response_paths(
                    self.saved_path1,
                    self.saved_path2,
                    self.saved_path3,
                    progress_callback=self.publish_progress,
                    node=self,
                )
            else:
                self.get_logger().info(
                    "[DRY RUN] RUN_ROBOT=False, 로봇 미동작"
                )

            response.success = True
            response.message = "Drawing Finished"

        except Exception as e:
            response.success = False
            response.message = f"Drawing failed: {e}"
            self.get_logger().error(response.message)

        finally:
            self.pause_requested = False
            self.is_working = False
            self.publish_bond()

        return response
# ================================================================
# [19] Service Main  # [SRV] 기본 실행 진입점 (서비스 모드)
# ================================================================
def main(args=None):
    global posx, movel, movesx, wait
    global set_velx, set_accx, check_motion
    global set_digital_output
    global task_compliance_ctrl, set_desired_force
    global release_force, release_compliance_ctrl
    global check_force_condition, get_current_posx      # [SRV] v3 로직에 필요
    global DR_BASE, DR_MV_MOD_ABS, DR_AXIS_Z            # [SRV] v3 로직에 필요


    rclpy.init(args=args)

    # [SRV] 서비스 처리용 노드와 DSR_ROBOT2 내부 spin용 노드를 분리
    #       (서비스 콜백 안에서 DSR 함수가 내부 spin을 돌 때
    #        같은 노드를 중첩 spin하면 데드락 나는 문제 회피)
    service_node = SandEngineNode()
    dsr_node = rclpy.create_node("sandart_dsr_internal", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node   # movel/movesx 내부 대기는 이 노드만 사용

    try:
        from DSR_ROBOT2 import (
            posx, movel, movesx, wait, check_motion,
            set_velx, set_accx,
            set_digital_output,
            task_compliance_ctrl, set_desired_force,
            release_force, release_compliance_ctrl,
            check_force_condition, get_current_posx,
            DR_AXIS_Z,
            DR_BASE, DR_MV_MOD_ABS,
        )
    except ImportError as e:
        service_node.get_logger().error(f"DSR_ROBOT2 import 실패: {e}")
        rclpy.shutdown()
        return

    try:
        set_velx(VEL_DEFAULT, VEL_DEFAULT)
        set_accx(ACC_DEFAULT, ACC_DEFAULT)
    except Exception as e:
        service_node.get_logger().warn(f"set_velx/set_accx 실패(무시하고 진행): {e}")

    # 명시적인 MultiThreadedExecutor를 사용합니다.
    # start_drawing 콜백이 실행 중이어도 별도 callback group의
    # pause_drawing / resume_drawing 콜백을 동시에 처리할 수 있습니다.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(service_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        service_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()