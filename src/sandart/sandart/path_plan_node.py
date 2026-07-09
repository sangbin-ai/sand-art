#!/usr/bin/env python3
"""
sandart_path_planner.py

서비스 /triple_layered_images (요청: TripleLayeredImages.srv)
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
from datetime import datetime

import uuid
from sandart_msgs.msg import SandStroke, SandPoint, Bond
from sandart_msgs.srv import TripleLayeredImages, PathPlanList


CONFIG = {
    "min_stroke_px": 5,      # 이보다 작은 조각(노이즈)은 버림
    "resample_mm": 8.0,      # 리샘플 간격 (mm)
    "max_strokes": 20,       # 레벨당 스트로크 개수 상한

    # --- 아핀 변환용 (강아지 테스트에서 검증된 값) ---
    "board_origin_xy": (250.0, -150.0),  # 종이 기준 원점 (로봇 mm)
    "board_width_mm": 200.0,
    "board_height_mm": 200.0,
}


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


def _build_pixel_graph(skel):
    """이미 스켈레톤화된(1픽셀 폭) 이미지 -> networkx 그래프."""
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


def trace_as_single_path(skel, resample_step=None):
    G = _build_pixel_graph(skel)
    if G.number_of_nodes() == 0 or not nx.is_connected(G):
        return None

    odd = [n for n in G.nodes() if G.degree(n) % 2 == 1]
    MG = nx.MultiGraph()
    MG.add_edges_from(G.edges())

    deg1 = [n for n in G.nodes() if G.degree(n) == 1]
    if len(deg1) >= 2:
        keep_start, keep_end = deg1[0], deg1[1]
    elif len(odd) >= 2:
        keep_start, keep_end = odd[0], odd[1]
    else:
        keep_start = keep_end = None

    if len(odd) > 2 and keep_start is not None:
        remaining = [n for n in odd if n not in (keep_start, keep_end)]
        remaining = sorted(remaining, key=lambda n: n)
        while len(remaining) >= 2:
            a = remaining.pop()
            best, best_d = None, None
            for b in remaining:
                try:
                    d = nx.shortest_path_length(G, a, b)
                except nx.NetworkXNoPath:
                    continue
                if best_d is None or d < best_d:
                    best, best_d = b, d
            if best is None:
                break
            remaining.remove(best)
            sp = nx.shortest_path(G, a, best)
            for u, v in zip(sp[:-1], sp[1:]):
                MG.add_edge(u, v)

    if not nx.is_connected(MG) or not nx.has_eulerian_path(MG):
        return None

    seq = list(nx.eulerian_path(MG, source=keep_start))
    full = [seq[0][0]] + [v for (_, v) in seq]
    raw_path = np.array([(c, r) for (r, c) in full], dtype=np.float64)

    if resample_step is not None:
        return resample(raw_path, resample_step)
    return raw_path


def trace_all_paths(skel, min_stroke_px, resample_step=None):
    """조각(연결 성분)별 원큐 경로 리스트. 큰 조각(점 많은 순)부터 정렬해서 반환."""
    G = _build_pixel_graph(skel)
    if G.number_of_nodes() == 0:
        return []

    paths = []
    comps = sorted(nx.connected_components(G), key=len, reverse=True)
    for comp in comps:
        if len(comp) < min_stroke_px:
            continue
        sub_mask = np.zeros_like(skel)
        for (r, c) in comp:
            sub_mask[r, c] = 255
        result = trace_as_single_path(sub_mask, resample_step=resample_step)
        if result is not None:
            paths.append(result)
    paths.sort(key=len, reverse=True)
    return paths


def image_to_mask(img):
    """이미 스켈레톤화+정규화된 이미지 -> 이진 마스크 (0 초과 = 선)."""
    return (img > 0).astype(np.uint8) * 255


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


def enforce_max_strokes(paths, max_strokes):
    """스트로크가 max_strokes를 넘으면 점 적은(=덜 중요한) 것부터 잘라낸다."""
    if len(paths) <= max_strokes:
        return paths
    return paths[:max_strokes]


def build_strokes_for_level(img, cfg, strength):
    """이미지 한 장(한 레벨) -> SandStroke 리스트.

    strength 는 참고값(로봇 압력 등). 색/번호는 path 순서로 정해지므로 여기선 저장만.
    """
    mask = image_to_mask(img)
    px_per_mm = img.shape[1] / cfg["board_width_mm"]
    resample_step = cfg["resample_mm"] * px_per_mm
    paths_px = trace_all_paths(mask, cfg["min_stroke_px"], resample_step=resample_step)
    paths_px = enforce_max_strokes(paths_px, cfg["max_strokes"])

    strokes_out = []
    for s_px in paths_px:
        s_rb = pixel_to_robot(s_px, img.shape, cfg)
        stroke = SandStroke()
        stroke.strength = strength
        for x, y in s_rb:
            pt = SandPoint()
            pt.x = float(round(x, 2))
            pt.y = float(round(y, 2))
            stroke.points.append(pt)
        strokes_out.append(stroke)
    return strokes_out


def save_path_txt(named_paths, path="strokes.txt"):
    """named_paths: [(헤더라벨, SandStroke 리스트), ...]

    waypoint 번호는 path별 누적:
      - path가 바뀌면 1부터 다시 시작
      - 같은 path 안에서는 stroke가 여러 개여도 번호를 이어서 증가
    """
    with open(path, "w") as f:
        for label, strokes in named_paths:
            f.write(f"=== {label} ===\n")
            wp = 0  # path별 누적 waypoint 카운터 (path 시작마다 0으로 리셋)
            for s_idx, stroke in enumerate(strokes, start=1):
                f.write(f"stroke{s_idx} (점 {len(stroke.points)}개)\n")

                for pt in stroke.points:
                    wp += 1
                    f.write(f"  waypoint{wp}  {pt.x}, {pt.y}\n")
                f.write("\n")


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
        # path 고정 매핑: path1=level0=RED, path2=level1=YELLOW, path3=level2=BLUE
        levels = [
            ("path1", request.level0_outer_thickest, 3),
            ("path2", request.level1_middle, 2),
            ("path3", request.level2_inner_thinnest, 1),
        ]

        result_paths = {}   # response 대신 로컬에 담기 (응답엔 path 필드 없음)

        self.is_working = True
        self.publish_bond()

        try:
            for field, img_msg, strength in levels:
                img = image_msg_to_cv2(img_msg)
                strokes = build_strokes_for_level(img, CONFIG, strength)
                result_paths[field] = strokes
        except Exception as e:
            response.accepted = False
            response.message = f"경로 생성 중 오류: {e}"
            self.get_logger().error(response.message)
            return response

        total_strokes = sum(len(v) for v in result_paths.values())
        if total_strokes == 0:
            response.accepted = False
            response.message = "생성된 스트로크가 없습니다."
            self.get_logger().warn(response.message)
            return response

        # 디버깅용 txt 저장
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        txt_path = f"strokes_{timestamp}.txt"
        named_paths = [
            ("path1 RED", result_paths["path1"]),
            ("path2 YELLOW", result_paths["path2"]),
            ("path3 BLUE", result_paths["path3"]),
        ]
        save_path_txt(named_paths, txt_path)

        # path를 주행노드로 전송 (PathPlanList 클라이언트)
        self.send_path_to_robot(result_paths)

        # skeleton한테는 접수 확인만 응답
        total_pts = sum(len(s.points) for v in result_paths.values() for s in v)
        response.accepted = True
        response.message = (f"path1={len(result_paths['path1'])}, "
                            f"path2={len(result_paths['path2'])}, "
                            f"path3={len(result_paths['path3'])} 스트로크, "
                            f"웨이포인트 {total_pts}개")
        self.get_logger().info(f"[OK] {response.message} ({txt_path} 저장됨)")

        self.is_working = False
        self.publish_bond()
        
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