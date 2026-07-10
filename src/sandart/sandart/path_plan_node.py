#!/usr/bin/env python3
"""
sandart_path_planner.py

서비스 /triple_layerd_images (요청: TripleLayerdImages.srv)
  요청 : level0_outer_thickest, level1_middle, level2_inner_thinnest
         (팀원 쪽에서 이미 0~255 정규화 + 흑백화 + 스켈레톤화까지 끝낸 이미지)
  응답 : accepted(bool), message(string),
         path1(SandStroke[]) - level0 외곽선     -> RED
         path2(SandStroke[]) - level1 중간 디테일 -> YELLOW
         path3(SandStroke[]) - level2 내부 디테일 -> BLUE

  좌표는 서비스 응답에 담아 보냄 (토픽 없음).
  아핀 변환으로 로봇 베이스 기준 mm 좌표로 변환.

로봇 실행 순서: path1 전체 stroke -> path2 전체 stroke -> path3 전체 stroke.
각 stroke 사이 이동은 pen-up 이동.
"""

import cv2
import numpy as np
import networkx as nx

import rclpy
from rclpy.node import Node

import uuid
from sandart_msgs.msg import SandStroke, SandPoint, Bond
from sandart_msgs.srv import TripleLayeredImages, PathPlanList


CONFIG = {
    "merge_distance_mm": 8.0,
    "max_merge_distance_mm": 15.0,
    "max_total_strokes": 35,

    # --- 아핀 변환용 (강아지 테스트에서 검증된 값) ---
    "board_origin_xy": (270.17, -141.78),  # 종이 기준 원점 (로봇 mm)
    "board_width_mm": 220.0,
    "board_height_mm": 220.0,
}

def image_to_mask(img):
    """이미 스켈레톤화+정규화된 이미지 -> 이진 마스크 (0 초과 = 선)."""
    return (img > 0).astype(np.uint8) * 255

# 1. 스켈레톤 이미지 전체에서 stroke path 리스트를 추출한다.
def trace_all_paths(skel, min_stroke_px=5, resample_step=None):
    
    G = build_pixel_graph(skel)
    if G.number_of_nodes() == 0:
        return []
    all_strokes = []
    components = sorted(                        # component 1 = {A, B, C}        # 3개
        nx.connected_components(G),             # component 2 = {D, E, F, G}     # 4개
        key=len,                                # component 3 = {H, I}           # 2개
        reverse=True                            # key=len 길이에 따라 reverse=  큰 거 앞으로
    )
    for comp in components:
        if len(comp) < min_stroke_px:           
            continue                            # min_stroke_px 보다 작은 건 버림 stroke로 만들지 않음
        subG = G.subgraph(comp).copy()              
        strokes = trace_paths(
            subG,
            min_stroke_px=min_stroke_px,
            resample_step=resample_step
        )
        all_strokes.extend(strokes)
    all_strokes.sort(key=len, reverse=True)
    return all_strokes

# 2. 이미지 픽셀을 그래프로 변환
def build_pixel_graph(skel):
    """
    . . . . .       흰색 픽셀 # 하나가 node가 됨
    . # # # .       픽셀 하나          ->    node
    . . . # .       붙어있는 픽셀관계   ->    edge
    . . . . .       

    A --- B --- C    G 안에 들어있는 정보
                |    nodes: A, B, C, D
                D    edges: A-B, B-C, C-D      
    
    """
    on = skel > 0
    ys, xs = np.nonzero(on)
    pixels = set(zip(ys.tolist(), xs.tolist()))
    G = nx.Graph()
    for (r, c) in pixels:
        for dr, dc in _ORTHO:
            q = (r + dr, c + dc)
            if q in pixels:
                G.add_edge((r, c), q)
        for dr, dc, o1, o2 in _DIAG:
            q = (r + dr, c + dc)
            if q not in pixels:
                continue
            n1 = (r + o1[0], c + o1[1])
            n2 = (r + o2[0], c + o2[1])
            if n1 in pixels and n2 in pixels:
                continue
            G.add_edge((r, c), q)
    return G
 
#3. 하나의 connected component graph에서 stroke path들을 추출한다.
def trace_paths(G, min_stroke_px=5, resample_step=None):
    """
    1. 오일러 패스 가능하면 한 stroke 반환
    2. 오일러 불가능하면 degree == 1 endpoint에서 먼저 stroke 생성
    3. 남은 edge는 degree >= 3 junction 기준으로 stroke 생성
    4. 그래도 남은 edge는 잔여 edge 기준으로 처리
    """
    euler_path = trace_euler_path(
        G,
        resample_step=resample_step
    )

    if euler_path is not None:
        if len(euler_path) >= min_stroke_px:
            return [euler_path]
        return []
    visited_edges = set()
    strokes = []
    endpoints = [n for n in G.nodes() if G.degree(n) == 1]
    junctions = [n for n in G.nodes() if G.degree(n) >= 3]
    # (1). 끝점에서 먼저 stroke 생성
    for start in endpoints:
        for nb in G.neighbors(start):
            if edge_key(start, nb) in visited_edges:
                continue
            node_path = walk_from_edge(
                G,
                start,
                nb,
                visited_edges
            )
            if len(node_path) >= min_stroke_px:
                strokes.append(
                    make_path(node_path, resample_step=resample_step)
                )

    # (2). 남은 edge를 분기점 기준으로 stroke 생성
    for start in junctions:
        for nb in G.neighbors(start):
            if edge_key(start, nb) in visited_edges:
                continue
            node_path = walk_from_edge(
                G,
                start,
                nb,
                visited_edges
            )
            if len(node_path) >= min_stroke_px:
                strokes.append(
                    make_path(node_path, resample_step=resample_step)
                )
    # (3). 그래도 남은 edge 처리
    for u, v in G.edges():
        if edge_key(u, v) in visited_edges:
            continue
        node_path = walk_from_edge(
            G,
            u,
            v,
            visited_edges
        )
        if len(node_path) >= min_stroke_px:
            strokes.append(
                make_path(node_path, resample_step=resample_step)
            )
    return strokes

def pair_odd_nodes_nearest(odd_nodes):
    """
    홀수 차수 노드를 가까운 것끼리 묶는다.

    이 연결은 실제로 그리는 선이 아니라,
    오일러 경로를 계산하기 위한 virtual edge다.
    """
    remaining = set(odd_nodes)
    pairs = []

    while len(remaining) >= 2:
        a = remaining.pop()

        b = min(
            remaining,
            key=lambda node: (
                (node[0] - a[0]) ** 2
                + (node[1] - a[1]) ** 2
            )
        )

        remaining.remove(b)
        pairs.append((a, b))

    return pairs


def trace_minimum_trails(G):
    """
    하나의 connected component를 가능한 적은 stroke로 다시 구성한다.

    특징:
    - 기존 edge를 삭제하지 않음
    - 존재하지 않는 선을 실제 path에 추가하지 않음
    - 분기점마다 무조건 자르지 않음
    - virtual edge에서는 stroke를 분리함
    """

    if G.number_of_edges() == 0:
        return []

    # 일반 Graph를 MultiGraph로 복사
    MG = nx.MultiGraph()
    MG.add_nodes_from(G.nodes())

    for u, v in G.edges():
        MG.add_edge(
            u,
            v,
            virtual=False
        )

    # degree가 홀수인 노드
    odd_nodes = [
        node
        for node in G.nodes()
        if G.degree(node) % 2 == 1
    ]

    # 홀수 노드를 virtual edge로 묶어서
    # 전체 그래프를 오일러 회로가 가능한 상태로 만든다.
    for a, b in pair_odd_nodes_nearest(odd_nodes):
        MG.add_edge(
            a,
            b,
            virtual=True
        )

    source = next(iter(MG.nodes()))

    edge_sequence = list(
        nx.eulerian_circuit(
            MG,
            source=source,
            keys=True
        )
    )

    if not edge_sequence:
        return []

    # virtual edge가 있다면 그 직후부터 시작하도록 회전한다.
    # 이렇게 해야 처음과 마지막 stroke가 잘못 쪼개지지 않는다.
    virtual_index = None

    for i, (u, v, key) in enumerate(edge_sequence):
        if MG[u][v][key].get("virtual", False):
            virtual_index = i
            break

    if virtual_index is not None:
        edge_sequence = (
            edge_sequence[virtual_index + 1:]
            + edge_sequence[:virtual_index + 1]
        )

    trails = []
    current_nodes = []

    for u, v, key in edge_sequence:
        is_virtual = MG[u][v][key].get(
            "virtual",
            False
        )

        # virtual edge는 실제로 그리지 않고
        # 여기서 stroke를 끊는다.
        if is_virtual:
            if len(current_nodes) >= 2:
                trails.append(current_nodes)

            current_nodes = []
            continue

        if not current_nodes:
            current_nodes = [u, v]

        else:
            # 정상적인 Euler sequence라면
            # current_nodes[-1]과 u는 동일하다.
            if current_nodes[-1] != u:
                current_nodes.append(u)

            current_nodes.append(v)

    if len(current_nodes) >= 2:
        trails.append(current_nodes)

    return trails


def trace_all_paths_minimum(
    skel,
    min_stroke_px=5,
    resample_step=None
):
    """
    스켈레톤 전체를 최소 stroke 방식으로 새로 추적한다.
    기존 trace_all_paths() 결과는 사용하지 않는다.
    """

    G = build_pixel_graph(skel)

    if G.number_of_nodes() == 0:
        return []

    all_strokes = []

    components = sorted(
        nx.connected_components(G),
        key=len,
        reverse=True
    )

    for component in components:
        if len(component) < min_stroke_px:
            continue

        subG = G.subgraph(component).copy()

        node_trails = trace_minimum_trails(subG)

        for nodes in node_trails:
            if len(nodes) < min_stroke_px:
                continue

            path = make_path(
                nodes,
                resample_step=resample_step
            )

            if len(path) >= 2:
                all_strokes.append(path)

    all_strokes.sort(
        key=len,
        reverse=True
    )

    return all_strokes

#3-1. 무방향 edge 방문 체크용 key
def edge_key(a, b):
   return tuple(sorted((a, b)))

# 3-2. 노드를 입력하면 path로 바꾸고 resample로 점의 수를 줄이는 작업 
def make_path(nodes, resample_step=None):
    path = np.array(
        [(c, r) for (r, c) in nodes],
        dtype=np.float64
    )

    if resample_step is not None:
        path = resample(path, resample_step)

    return path

# 3-3. 오일러 패스 가능할 때만 한 stroke 생성.
def trace_euler_path(G, resample_step=None):

    odd_nodes = [n for n in G.nodes() if G.degree(n) % 2 == 1] #degree 노드에 연결된 엣지 수/ 엣지가 홀수인 것만 담겟다

    if len(odd_nodes) not in (0, 2):        # 홀수점이 0,2 아니면 불가능 None 반환
        return None

    if not nx.has_eulerian_path(G):         # 한붓그리기가 불가능 하면 None 반환
        return None

    if len(odd_nodes) == 2:            
        source = odd_nodes[0]               # 홀수점이 2개면 둘중 한개에서 시작
    else:
        source = next(iter(G.nodes()))      # 홀수점이 0개면 아무데서나 시작

    edge_seq = list(nx.eulerian_path(G, source=source))     # source 노드에서 시작해 모든 edge를 한 번씩 지나는 오일러 경로를 구한다.
                                                            # [(A, B), (B, C), (C, D)] 엣지 순서가 들어간다
    node_seq = [edge_seq[0][0]] + [v for _, v in edge_seq]  # [A → B → C → D] 엣지 순을 노드순으로 바꿈 
                                                            # 오일러 패스가 가능하니까 엣지끼리 무조건 이어져 있어 (u, v) 중에 v 만 다 가져오고 처음거만 [edge_seq[0][0]] 이걸로 붙암

    return make_path(node_seq, resample_step=resample_step)


def walk_from_edge(G, start, next_node, visited_edges):
    """
    start에서 next_node 방향으로 출발해서,
    degree != 2 노드를 만날 때까지 하나의 stroke를 만든다.
    """
    path = [start, next_node]
    visited_edges.add(edge_key(start, next_node))

    prev = start
    cur = next_node

    while True:
        if G.degree(cur) != 2:
            break

        candidates = []

        for nb in G.neighbors(cur):
            if nb == prev:
                continue

            if edge_key(cur, nb) not in visited_edges:
                candidates.append(nb)

        if not candidates:
            break

        nxt = candidates[0]

        visited_edges.add(edge_key(cur, nxt))
        path.append(nxt)
        prev = cur
        cur = nxt
    return path

# 수정본
def resample(stroke_xy, step, min_points=8):
    if len(stroke_xy) < 2:
        return stroke_xy
    seg = np.linalg.norm(np.diff(stroke_xy, axis=0), axis=1)
    dist = np.concatenate([[0.0], np.cumsum(seg)])
    total = dist[-1]
    if total < 1e-6:
        return stroke_xy[:1]
    n = max(int(total // step) + 1, min_points)   # <- 2 대신 min_points로 바닥을 깔아줌
    targets = np.linspace(0.0, total, n)
    out = np.empty((n, 2))
    out[:, 0] = np.interp(targets, dist, stroke_xy[:, 0])
    out[:, 1] = np.interp(targets, dist, stroke_xy[:, 1])
    return out

_ORTHO = [(-1, 0), (1, 0), (0, -1), (0, 1)]
_DIAG = [(-1, -1, (-1, 0), (0, -1)),
         (-1,  1, (-1, 0), (0,  1)),
         ( 1, -1, ( 1, 0), (0, -1)),
         ( 1,  1, ( 1, 0), (0,  1))]

def pixel_to_robot(stroke_px, img_shape, cfg):
    """픽셀 좌표 -> 로봇 베이스 기준 mm 좌표 (아핀 변환)."""
    H, W = img_shape
    ox, oy = cfg["board_origin_xy"]
    sx = cfg["board_width_mm"] / W
    sy = cfg["board_height_mm"] / H
    out = np.empty_like(stroke_px)
    out[:, 0] = ox + stroke_px[:, 0] * sx
    out[:, 1] = oy + (H - stroke_px[:, 1]) * sy
    return out

def distance(p1, p2):
    return np.linalg.norm(p1 - p2)


def merge_two_strokes(a, b, mode):
    """
    a, b: np.array([[x, y], ...])

    mode:
    - "a_end_b_start": a 끝 ↔ b 시작
    - "a_end_b_end"  : a 끝 ↔ b 끝
    - "a_start_b_end": a 시작 ↔ b 끝
    - "a_start_b_start": a 시작 ↔ b 시작
    """
    if mode == "a_end_b_start":
        return np.vstack([a, b[1:]])

    if mode == "a_end_b_end":
        b_rev = b[::-1]
        return np.vstack([a, b_rev[1:]])

    if mode == "a_start_b_end":
        return np.vstack([b, a[1:]])

    if mode == "a_start_b_start":
        b_rev = b[::-1]
        return np.vstack([b_rev, a[1:]])

    return a


def merge_close_strokes(paths, merge_dist_px):
    """
    stroke의 start/end가 서로 가까우면 하나로 합친다.

    paths: [np.array([[x, y], ...]), ...]
    merge_dist_px: 몇 픽셀 이하이면 같은 stroke로 합칠지 기준
    """
    paths = list(paths)
    merged = True

    while merged:
        merged = False

        for i in range(len(paths)):
            if merged:
                break

            a = paths[i]
            a_start = a[0]
            a_end = a[-1]

            for j in range(i + 1, len(paths)):
                b = paths[j]
                b_start = b[0]
                b_end = b[-1]

                candidates = [
                    (distance(a_end, b_start), "a_end_b_start"),
                    (distance(a_end, b_end), "a_end_b_end"),
                    (distance(a_start, b_end), "a_start_b_end"),
                    (distance(a_start, b_start), "a_start_b_start"),
                ]

                best_dist, best_mode = min(candidates, key=lambda x: x[0])

                if best_dist <= merge_dist_px:
                    new_stroke = merge_two_strokes(a, b, best_mode)

                    paths[i] = new_stroke
                    paths.pop(j)

                    merged = True
                    break

    paths.sort(key=len, reverse=True)

    return paths


def build_strokes_for_level(
    img,
    cfg,
    path_name,
    strength,
    min_stroke_px,
    resample_mm,
    use_minimum_trace
):
    """
    하나의 이미지 레벨에서 경로를 한 번 생성한다.

    반복 재생성과 merge 거리 증가는
    handle_edge_layers()에서만 담당한다.
    """

    skel = image_to_mask(img)

    px_per_mm = (
        img.shape[1]
        / cfg["board_width_mm"]
    )

    resample_step = (
        resample_mm
        * px_per_mm
    )

    merge_mm = cfg["merge_distance_mm"]
    merge_px = merge_mm * px_per_mm

    if use_minimum_trace:
        paths_px = trace_all_paths_minimum(
            skel,
            min_stroke_px=min_stroke_px,
            resample_step=resample_step
        )
    else:
        paths_px = trace_all_paths(
            skel,
            min_stroke_px=min_stroke_px,
            resample_step=resample_step
        )

    paths_px = merge_close_strokes(
        paths_px,
        merge_px
    )

    strokes_out = []

    for path_px in paths_px:
        path_robot = pixel_to_robot(
            path_px,
            img.shape,
            cfg
        )

        stroke = SandStroke()
        stroke.strength = strength

        for x, y in path_robot:
            point = SandPoint()
            point.x = float(round(x, 2))
            point.y = float(round(y, 2))
            stroke.points.append(point)

        strokes_out.append(stroke)

    return strokes_out

def image_msg_to_cv2(msg) -> np.ndarray:
    """sensor_msgs/Image -> OpenCV 그레이스케일 배열 (cv_bridge 없이 직접 변환)."""
    H, W = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding == "mono8":
        img = buf.reshape(H, W)
    elif msg.encoding in ("bgr8", "rgb8"):
        img = buf.reshape(H, W, 3)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    else:
        raise ValueError(f"지원하지 않는 encoding: {msg.encoding}")
    return img


class SandartPathPlannerNode(Node):
    def __init__(self):
        super().__init__("path_plan_node")

        self.srv = self.create_service(
            TripleLayeredImages, "triple_layered_images", self.handle_edge_layers)
        
        self.path_client = self.create_client(PathPlanList, "/dsr01/path_plan_list")

        self.bond_id = "path_planner_to_lifecycle"
        self.bond_instance_id = str(uuid.uuid4())
        self.heartbeat_timeout = 6.0
        self.heartbeat_period = 0.1
        self.is_working = False
        self.is_active = True

        self.bond_pub = self.create_publisher(Bond, "/bond", 10)
        self.bond_timer = self.create_timer(0.1, self.publish_bond)

        self.get_logger().info("SandartPathPlannerNode 시작, /triple_layered_images 서비스 대기 중...")

    def publish_bond(self):
        msg = Bond()
        msg.id = self.bond_id
        msg.instance_id = self.bond_instance_id
        msg.working = self.is_working
        msg.active = self.is_active
        msg.heartbeat_timeout = self.heartbeat_timeout
        msg.heartbeat_period = self.heartbeat_period
        self.bond_pub.publish(msg)
        
    def handle_edge_layers(self, request, response):
        levels = [
            ("path1", request.level0_outer_thickest, 3, 8, 15),
            ("path2", request.level1_middle,         2, 5, 8),
            ("path3", request.level2_inner_thinnest, 1, 3, 8),
        ]

        result_paths = {}

        original_merge = CONFIG["merge_distance_mm"]

        try:
            use_minimum_trace = False

            while True:
                result_paths.clear()

                for (
                    field,
                    img_msg,
                    strength,
                    min_stroke_px,
                    resample_mm
                ) in levels:

                    img = image_msg_to_cv2(img_msg)

                    result_paths[field] = build_strokes_for_level(
                        img=img,
                        cfg=CONFIG,
                        path_name=field,
                        strength=strength,
                        min_stroke_px=min_stroke_px,
                        resample_mm=resample_mm,
                        use_minimum_trace=use_minimum_trace
                    )

                path1_count = len(result_paths["path1"])
                path2_count = len(result_paths["path2"])
                path3_count = len(result_paths["path3"])

                total_strokes = (
                    path1_count
                    + path2_count
                    + path3_count
                )

                # 전체 35개 이하이면 최종 채택
                if total_strokes <= CONFIG["max_total_strokes"]:
                    break

                # 기존 방식에서 실패했다면
                # merge를 늘리기 전에 최소 경로 방식으로 다시 생성
                if not use_minimum_trace:
                    use_minimum_trace = True
                    continue

                # 최소 경로에서도 실패하면 merge 1mm 증가
                next_merge = (
                    CONFIG["merge_distance_mm"]
                    + 1.0
                )

                if next_merge > CONFIG["max_merge_distance_mm"]:
                    raise ValueError(
                        f"전체 스트로크를 "
                        f"{CONFIG['max_total_strokes']}개 이하로 "
                        f"만들지 못했습니다. "
                        f"현재 결과={total_strokes}개, "
                        f"최대 병합 거리="
                        f"{CONFIG['max_merge_distance_mm']:.1f}mm"
                    )

                CONFIG["merge_distance_mm"] = next_merge

        except Exception as e:
            response.accepted = False
            response.message = f"경로 생성 중 오류: {e}"
            self.get_logger().error(response.message)
            return response

        finally:
            CONFIG["merge_distance_mm"] = original_merge

        total_strokes = sum(
            len(strokes)
            for strokes in result_paths.values()
        )

        if total_strokes == 0:
            response.accepted = False
            response.message = "생성된 스트로크가 없습니다."
            self.get_logger().warning(response.message)
            return response

        self.send_path_to_robot(result_paths)

        total_points = sum(
            len(stroke.points)
            for strokes in result_paths.values()
            for stroke in strokes
        )

        response.accepted = True
        response.message = (
            f"path1={len(result_paths['path1'])}, "
            f"path2={len(result_paths['path2'])}, "
            f"path3={len(result_paths['path3'])}, "
            f"전체={total_strokes} 스트로크, "
            f"웨이포인트={total_points}개"
        )

        self.get_logger().info(
            f"[경로 생성 완료] "
            f"{response.message} "
        )
        return response

    def send_path_to_robot(self, result_paths):
        """계산한 path를 PathPlanList 서비스로 주행노드에 전송."""
        if not self.path_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("path_plan_list 서비스 없음 (주행노드 미실행)")
            return

        req = PathPlanList.Request()
        req.path1 = result_paths["path1"]
        req.path2 = result_paths["path2"]
        req.path3 = result_paths["path3"]

        future = self.path_client.call_async(req)
        future.add_done_callback(self.path_response_callback)

    def path_response_callback(self, future):
        try:
            res = future.result()
            if res.accepted:
                self.get_logger().info(f"주행노드 path 접수: {res.message}")
            else:
                self.get_logger().warn(f"주행노드 path 거부: {res.message}")
        except Exception as e:
            self.get_logger().error(f"path 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SandartPathPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()