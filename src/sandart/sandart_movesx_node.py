#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sandart_movesx_node.py

두산 M0609 + ROS2 + DSR_ROBOT2 샌드아트 실행 엔진

핵심 목표:
  - 좌표 경로를 부드럽게 따라 그림
  - Rz 진행방향 추종 기능 완전 제거
  - Rx, Ry, Rz는 고정 자세 사용
  - Stroke 단위로 끊어서 그림
  - Stroke 사이에는 SAFE_Z로 상승 후 다음 Stroke 시작점으로 이동
  - TXT 테스트와 추후 SRV/Custom Msg 구조를 동일하게 맞춤

유지 기능:
  1) 중복 좌표 제거
  2) Centripetal Catmull-Rom Spline
  3) Uniform Sampling
  4) Lead In / Lead Out
  5) movesx chunk 실행
  6) SAFE_Z 접근/복귀
  7) Compliance / Force Control

TXT 입력 형식:
  - x y
  - x,y
  - x,y,z  # z는 무시
  - 빈 줄은 stroke 구분자로 사용

예:
  100 100
  101 101
  102 102

  200 200
  201 201
  202 202

나중에 SRV 연결 시:
  - response.path1, response.path2, response.path3 각각은 SandStroke[]
  - 각 SandStroke.points를 [[x,y], ...] 형태로 변환해서 execute_path()에 넣으면 됨
"""

import math
import os
import rclpy
import DR_init

from rclpy.node import Node
from sandart_msgs.srv import PathPlanList

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
TXT_PATH = "/home/rokey/Downloads/path_points.txt"
RUN_ROBOT = True

# TXT에서는 하나의 path 안에 여러 stroke가 있다고 보고 실행
# 나중에 SRV에서는 path1/path2/path3를 각각 execute_path()로 넘기면 됨

# ================================================================
# [3] Pose Config
# ================================================================
SAFE_Z = 365.0
DRAW_Z = 90.03

# Force를 켜기 전 바로 DRAW_Z로 박지 않고, DRAW_Z보다 살짝 위에서 켠 뒤 천천히 내려감
FORCE_APPROACH_OFFSET_Z = 0.0

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
MOVESX_CHUNK_SIZE = 80

MIN_WAYPOINT_COUNT = 4
MIN_STROKE_POINT_COUNT = 2
DUPLICATE_DIST_MM = 0.2

CR_ALPHA = 0.5
SPLINE_STEPS_PER_SEG = 20

USE_LEAD_IN_OUT = True
LEAD_DIST = 3.0

# ================================================================
# [5] Force / Compliance Config
# ================================================================
USE_FORCE_CONTROL = True

# 순응제어 강성값 [x, y, z, rx, ry, rz]
# Z값이 작을수록 위아래로 더 부드럽게 순응함.
COMPLIANCE_STX = [3000, 3000, 1, 200, 200, 200]

# 목표 힘 [Fx, Fy, Fz, Mx, My, Mz]
DESIRED_FORCE_FD = [0, 0, -1, 0, 0, 0]
DESIRED_FORCE_DIR = [0, 0, 1, 0, 0, 0]
FORCE_SETTLE_WAIT = 0.3

# ================================================================
# [6] Basic Math Utils
# ================================================================
class SandEngineNode(Node):
    def __init__(self):
        super().__init__("sandart_movesx_node", namespace=ROBOT_ID)
        self.srv = self.create_service(
            PathPlanList, "path_plan_list", self.handle_path_plan_list)
        self.get_logger().info("path_plan_list 서비스 대기 중...")

    def handle_path_plan_list(self, request, response):
        try:
            path1 = path_from_sand_strokes(request.path1)
            path2 = path_from_sand_strokes(request.path2)
            path3 = path_from_sand_strokes(request.path3)
        except Exception as e:
            response.accepted = False
            response.message = f"경로 변환 실패: {e}"
            self.get_logger().error(response.message)
            return response

        total = len(path1) + len(path2) + len(path3)
        if total == 0:
            response.accepted = False
            response.message = "받은 path가 비어 있습니다."
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info(
            f"path 수신: path1={len(path1)}, path2={len(path2)}, path3={len(path3)} strokes")

        try:
            if RUN_ROBOT:
                execute_response_paths(path1, path2, path3)
            else:
                self.get_logger().info("[DRY RUN] RUN_ROBOT=False, 로봇 미동작")
        except Exception as e:
            response.accepted = False
            response.message = f"실행 중 오류: {e}"
            self.get_logger().error(response.message)
            return response

        response.accepted = True
        response.message = f"path1={len(path1)}, path2={len(path2)}, path3={len(path3)} 실행 완료"
        self.get_logger().info(f"[OK] {response.message}")
        return response

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
def load_paths_from_txt(txt_path):
    """
    TXT를 읽어서 SRV와 같은 내부 구조로 변환한다.

    반환 형태:
        path1, path2, path3

    각 path 형태:
        [stroke1, stroke2, ...]

    각 stroke 형태:
        [[x, y], [x, y], ...]

    지원 TXT 예시 1: GUI dump 형식
        === path1 RED ===
        stroke1 (점 92개, z=94.0, vel=40.0)
          waypoint1  322.0, 13.5
          waypoint2  328.63, 11.0

        === path2 YELLOW ===
        stroke1
          waypoint1  366.0, -68.0

    지원 TXT 예시 2: 단순 좌표 형식
        322.0 13.5
        328.63 11.0

        366.0 -68.0
        360.95 -74.55

    단순 좌표 형식에서는 빈 줄을 stroke 구분자로 사용하고,
    전체는 path1로 들어간다.
    """
    import re

    if not os.path.exists(txt_path):
        raise RuntimeError(f"TXT 파일 없음 : {txt_path}")

    path1 = []
    path2 = []
    path3 = []

    # path 헤더가 없는 단순 TXT도 바로 path1에 들어가도록 기본값을 path1로 둔다.
    current_path = path1
    current_path_idx = 1
    current_stroke = []

    def save_stroke():
        """현재 모으고 있는 stroke를 현재 path에 저장한다."""
        nonlocal current_stroke, current_path

        if current_path is None:
            current_stroke = []
            return

        if len(current_stroke) < MIN_STROKE_POINT_COUNT:
            current_stroke = []
            return

        cleaned = remove_near_duplicate_points(current_stroke)

        if len(cleaned) >= MIN_STROKE_POINT_COUNT:
            current_path.append(cleaned)

        current_stroke = []

    def select_path_from_header(line):
        """=== path1 RED === 같은 헤더를 보고 현재 path를 선택한다."""
        lower = line.lower()

        if "path1" in lower:
            return path1, 1
        if "path2" in lower:
            return path2, 2
        if "path3" in lower:
            return path3, 3

        return None, None

    def parse_xy_from_line(line):
        """
        한 줄에서 x, y만 뽑는다.

        지원:
          waypoint1  322.0, 13.5
          waypoint 322.0 13.5
          waypoint1 : 322.0 13.5
          322.0 13.5
          322.0, 13.5, 94.0

        z, vel 등 뒤에 값이 더 있어도 x,y만 사용한다.
        """
        text = line.strip()

        # waypoint1 / waypoint 1 / waypoint1: 같은 접두어 제거
        if text.lower().startswith("waypoint"):
            text = re.sub(r"^waypoint\s*\d*\s*[:\-]?\s*", "", text, flags=re.IGNORECASE)

        # 콤마를 공백처럼 취급
        text = text.replace(",", " ")

        # 숫자만 추출. 부호/소수/지수표기 지원.
        nums = re.findall(r"[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?", text)

        if len(nums) < 2:
            return None

        try:
            x = float(nums[0])
            y = float(nums[1])
            return [x, y]
        except ValueError:
            return None

    with open(txt_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()

            # 빈 줄은 단순 TXT에서 stroke 구분자로 사용한다.
            if line == "":
                save_stroke()
                continue

            # 주석 무시
            if line.startswith("#"):
                continue

            # PATH 헤더 처리
            if line.startswith("==="):
                save_stroke()

                selected_path, selected_idx = select_path_from_header(line)

                if selected_path is not None:
                    current_path = selected_path
                    current_path_idx = selected_idx
                    print(f"[TXT] select PATH {current_path_idx}: {line}")
                else:
                    print(f"[TXT] unknown path header ignored: {line}")

                continue

            # stroke 시작 처리
            # 예: stroke1, stroke2 (점 8개, z=94.0, vel=40.0)
            if line.lower().startswith("stroke"):
                save_stroke()
                current_stroke = []
                continue

            # waypoint 또는 단순 좌표 처리
            xy = parse_xy_from_line(line)

            if xy is not None:
                current_stroke.append(xy)

    # 파일 마지막 stroke 저장
    save_stroke()

    paths = [path1, path2, path3]

    print("\n================ TXT RESULT =================")
    for path_idx, path in enumerate(paths, start=1):
        total_points = sum(len(stroke) for stroke in path)
        total_len = sum(path_length(stroke) for stroke in path)
        print(f"PATH {path_idx}: strokes={len(path)}, points={total_points}, length={total_len:.2f} mm")

        for stroke_idx, stroke in enumerate(path, start=1):
            print(
                f"  stroke {stroke_idx}: "
                f"points={len(stroke)}, length={path_length(stroke):.2f} mm"
            )
    print("=============================================\n")

    if len(path1) + len(path2) + len(path3) == 0:
        raise RuntimeError("TXT에서 유효한 path/stroke 좌표를 찾지 못함")

    return path1, path2, path3

# ================================================================
# [8] SRV / Custom Msg 변환용 준비 함수
# ================================================================
def points_from_sand_stroke(stroke_msg):
    """
    SandStroke.msg를 [[x, y], ...]로 변환하는 함수.

    나중에 SRV 연결할 때 사용:
      points = points_from_sand_stroke(stroke)
      execute_stroke(points, ...)
    """
    points = []

    for p in stroke_msg.points:
        points.append([float(p.x), float(p.y)])

    points = remove_near_duplicate_points(points)
    return points


def path_from_sand_strokes(stroke_msgs):
    """
    SandStroke[]를 Python stroke list로 변환.
    반환 형태: [ [[x,y], [x,y]], [[x,y], [x,y]], ... ]
    """
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
            p0,
            p1,
            p2,
            p3,
            SPLINE_STEPS_PER_SEG,
        )

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
    """
    p0 -> p1 방향으로 distance 만큼 이동한 점 생성.
    distance < 0 : 시작점 앞쪽
    distance > 0 : 끝점 뒤쪽
    """
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
    sampled point를 최종 motion으로 변환.

    motion 구조:
        [x, y, z, rx, ry, rz]

    Rz는 진행 방향을 따라가지 않고 BASE_RZ로 고정.
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


def motion_to_pos_list(motion):
    """motion 전체를 movesx에 넣을 posx 리스트로 변환."""
    return [motion_to_posx(m) for m in motion]


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

    print(f"[FORCE] set desired force: fd={DESIRED_FORCE_FD}, dir={DESIRED_FORCE_DIR}")

    try:
        set_desired_force(fd=DESIRED_FORCE_FD, dir=DESIRED_FORCE_DIR)
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
# [15] Robot Engine - movesx
# ================================================================
STATUS_IDLE = 0


def wait_motion_near_finish():
    """현재 motion이 끝날 때까지 대기."""
    while True:
        status = check_motion()

        if status == STATUS_IDLE:
            break

        wait(0.01)


def call_movesx_chunk(chunk, vel=VEL_DEFAULT, acc=ACC_DEFAULT):
    """movesx 호출 래퍼."""
    movesx(
        chunk,
        vel=vel,
        acc=acc,
        ref=DR_BASE,
        mod=DR_MV_MOD_ABS,
    )


def execute_motion_with_movesx(motion, label="stroke"):
    """
    motion 하나를 실제 로봇으로 실행.

    순서:
      1) 시작점 SAFE_Z 이동
      2) DRAW_Z + FORCE_APPROACH_OFFSET_Z 위치까지 접근
      3) Compliance / Force ON
      4) DRAW_Z까지 천천히 하강
      5) movesx chunk 실행
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
        result = movel(
            motion_to_posx(start_safe),
            vel=VEL_DEFAULT,
            acc=ACC_DEFAULT,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        print(f"[DEBUG] movel result = {result}")
        wait(0.3)
        
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
        wait(0.2)

        # 3) Compliance / Force ON
        enable_force_control()
        force_enabled = True

        # 4) Force 켠 상태에서 DRAW_Z까지 천천히 내려가기
        print("[ROBOT] force approach draw z")
        movel(
            motion_to_posx(start),
            vel=APPROACH_VEL,
            acc=APPROACH_ACC,
            ref=DR_BASE,
            mod=DR_MV_MOD_ABS,
        )
        wait(0.2)

        # 5) movesx 실행
        pos_list = motion_to_pos_list(motion)
        chunks = split_pos_list_to_chunks(pos_list)

        print(f"chunks = {len(chunks)}")
        print(f"chunk size = {MOVESX_CHUNK_SIZE}")

        for idx, chunk in enumerate(chunks):
            print(f"[MOVESX {idx + 1}/{len(chunks)}] points={len(chunk)}")
            call_movesx_chunk(chunk, vel=VEL_DEFAULT, acc=ACC_DEFAULT)
            wait_motion_near_finish()

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
def execute_stroke(stroke_points, path_idx=1, stroke_idx=1, stroke_total=1):
    """
    stroke 1개를 독립적으로 그림.

    중요:
      stroke가 끝나면 반드시 SAFE_Z로 올라감.
      다음 stroke와 직선으로 이어 그리지 않음.
    """
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
    execute_motion_with_movesx(motion, label=label)


def execute_path(path_strokes, path_idx=1):
    """
    path 1개 안의 stroke들을 순서대로 실행.

    path_strokes 형태:
      [ [[x,y], [x,y], ...], [[x,y], [x,y], ...], ... ]
    """
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
        )

    print("\n================================================")
    print(f"PATH {path_idx} DONE")
    print("================================================\n")


def execute_response_paths(path1, path2, path3):
    """
    SRV 응답 구조와 동일하게 path1/path2/path3를 순서대로 실행.

    현재 TXT 테스트에서는 path1에 TXT stroke들을 넣고,
    path2/path3는 빈 리스트로 호출하면 됨.
    """
    execute_path(path1, path_idx=1)
    execute_path(path2, path_idx=2)
    execute_path(path3, path_idx=3)

# ================================================================
# [17] Robot Main
# ================================================================
def robot_test_main(args=None):
    """
    ROS2/DSR 환경에서 실행되는 main.
    RUN_ROBOT=False면 TXT stroke 로딩과 motion 생성 확인만 수행.
    RUN_ROBOT=True면 실제 movesx 실행.
    """
    global posx, movel, movesx, wait
    global set_velx, set_accx, check_motion
    global task_compliance_ctrl, set_desired_force
    global release_force, release_compliance_ctrl
    global DR_BASE, DR_MV_MOD_ABS

    rclpy.init(args=args)
    node = rclpy.create_node("sandart_movesx_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            posx,
            movel,
            movesx,
            wait,
            check_motion,
            set_velx,
            set_accx,
            task_compliance_ctrl,
            set_desired_force,
            release_force,
            release_compliance_ctrl,
            DR_BASE,
            DR_MV_MOD_ABS,
        )
    except ImportError as e:
        node.get_logger().error(f"DSR_ROBOT2 import 실패: {e}")
        rclpy.shutdown()
        return

    try:
        set_velx(VEL_DEFAULT, VEL_DEFAULT)
        set_accx(ACC_DEFAULT, ACC_DEFAULT)

        print("[INIT] Load strokes from TXT")
        path1, path2, path3 = load_paths_from_txt(TXT_PATH)

        if not RUN_ROBOT:
            print("\n[DRY RUN] RUN_ROBOT=False")
            print("[DRY RUN] Robot will not move.")

            for path_idx, path in enumerate([path1, path2, path3], start=1):
                print(f"\n[DRY RUN] PATH {path_idx}: strokes={len(path)}")
                for stroke_idx, stroke in enumerate(path, start=1):
                    motion = build_motion_from_stroke(stroke)
                    print(
                        f"  stroke {stroke_idx}: raw={len(stroke)}, "
                        f"motion={len(motion)}, length={path_length(stroke):.2f} mm"
                    )
                    for i, m in enumerate(motion[:3]):
                        print(
                            f"    M{i}: x={m[0]:.3f}, y={m[1]:.3f}, z={m[2]:.1f}, "
                            f"rx={m[3]:.2f}, ry={m[4]:.2f}, rz={m[5]:.2f}"
                        )
            return

        print("\n[READY] RUN_ROBOT=True")
        print("[INFO] 2초 후 로봇 실행")
        wait(2.0)

        # TXT 테스트는 path1에 전체 stroke를 넣고 실행
        # 나중에 SRV 연결 시 response.path1/path2/path3를 변환해서 여기에 넣으면 됨
        execute_response_paths(path1, path2, path3)

    except KeyboardInterrupt:
        print("[STOP] KeyboardInterrupt")
        disable_force_control()

    except Exception as e:
        print(f"[ERROR] {e}")
        disable_force_control()

    finally:
        rclpy.shutdown()


def main(args=None):
    global posx, movel, movesx, wait
    global set_velx, set_accx, check_motion
    global task_compliance_ctrl, set_desired_force
    global release_force, release_compliance_ctrl
    global DR_BASE, DR_MV_MOD_ABS

    rclpy.init(args=args)

    # 서비스 처리용 노드와 DSR_ROBOT2 내부 spin용 노드를 분리
    service_node = SandEngineNode()
    dsr_node = rclpy.create_node("sandart_dsr_internal", namespace=ROBOT_ID)
    DR_init.__dsr__node = dsr_node   # movel/movesx 내부 대기는 이 노드만 사용

    try:
        from DSR_ROBOT2 import (
            posx, movel, movesx, wait, check_motion,
            set_velx, set_accx, task_compliance_ctrl, set_desired_force,
            release_force, release_compliance_ctrl, DR_BASE, DR_MV_MOD_ABS,
        )
    except ImportError as e:
        service_node.get_logger().error(f"DSR_ROBOT2 import 실패: {e}")
        rclpy.shutdown()
        return

    try:
        rclpy.spin(service_node)   # 서비스 콜백은 여기서만 처리
    except KeyboardInterrupt:
        pass
    finally:
        service_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
