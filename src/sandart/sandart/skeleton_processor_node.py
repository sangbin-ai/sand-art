#!/usr/bin/env python3
from collections import deque, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Set, Tuple

import cv2
import numpy as np
from skimage.morphology import skeletonize

Point = Tuple[int, int]
Segment = List[Point]


@dataclass
class ProcessResult:
    level_images: List[np.ndarray]
    color_visualization: np.ndarray
    segment_index_visualization: np.ndarray
    debug: Dict[str, int]


class SkeletonTreeProcessor:
    def __init__(self):
        self.resize_size = 400
        self.blur_ksize = 5
        self.sobel_ksize = 3
        self.skeleton_dilate_iterations = 2
        self.outer_match_radius = 3
        self.min_segment_length = 8
        self.num_levels = 3
        self.colors_bgr = [
            (0, 0, 255),
            (0, 255, 255),
            (255, 0, 0),
        ]

    def process_image_file(self, image_path: str) -> ProcessResult:
        path = Path(image_path).expanduser()
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            raise FileNotFoundError(f"ì´ë¯¸ì§€ë¥¼ ì½ì„ ìˆ˜ ì—†ìŒ: {image_path}")
        return self.process_gray(gray)

    def process_gray(self, gray: np.ndarray) -> ProcessResult:
        gray = cv2.resize(
            gray,
            (self.resize_size, self.resize_size),
            interpolation=cv2.INTER_AREA,
        )

        blur = cv2.GaussianBlur(gray, (self.blur_ksize, self.blur_ksize), 0)

        sobel_x = cv2.Sobel(blur, cv2.CV_64F, 1, 0, ksize=self.sobel_ksize)
        sobel_y = cv2.Sobel(blur, cv2.CV_64F, 0, 1, ksize=self.sobel_ksize)
        edge = cv2.magnitude(sobel_x, sobel_y)
        edge_norm = cv2.normalize(edge, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)

        nonzero = edge_norm[edge_norm > 0]
        if len(nonzero) == 0:
            raise ValueError("edge_normì— 0ì´ ì•„ë‹Œ í”½ì…€ì´ ì—†ìŠµë‹ˆë‹¤.")

        median_value = float(np.median(nonzero))
        _, binary = cv2.threshold(edge_norm, median_value, 255, cv2.THRESH_BINARY)

        skeleton_first = skeletonize(binary > 0).astype(np.uint8) * 255

        kernel = np.ones((3, 3), np.uint8)
        skeleton_dilated = cv2.dilate(
            skeleton_first,
            kernel,
            iterations=self.skeleton_dilate_iterations,
        )
        skeleton = skeletonize(skeleton_dilated > 0).astype(np.uint8) * 255

        if np.count_nonzero(skeleton) == 0:
            raise ValueError("ìŠ¤ì¼ˆë ˆí†¤ ì ì´ ì—†ìŠµë‹ˆë‹¤.")

        segments, point_to_segments = self.split_skeleton_into_segments(skeleton)
        if not segments:
            raise ValueError("ë¶„ë¦¬ëœ segmentê°€ ì—†ìŠµë‹ˆë‹¤.")

        graph = self.build_segment_graph(segments, point_to_segments)
        outer_mask = self.get_outer_contour_mask(binary, radius=self.outer_match_radius)
        outer_segments = self.find_outer_segments(segments, outer_mask)
        levels, _ = self.assign_tree_levels_balanced_inner(segments, graph, outer_segments)

        level_images, color_vis, _ = self.make_level_images(skeleton, segments, levels)
        segment_index_vis = self.draw_segment_index(skeleton, segments, levels)

        debug = {
            "median_threshold": int(round(median_value)),
            "segment_count": len(segments),
            "outer_segment_count": len(outer_segments),
            "level0_length_sum": int(sum(len(segments[i]) for i in range(len(segments)) if levels[i] == 0)),
            "level1_length_sum": int(sum(len(segments[i]) for i in range(len(segments)) if levels[i] == 1)),
            "level2_length_sum": int(sum(len(segments[i]) for i in range(len(segments)) if levels[i] == 2)),
        }
        return ProcessResult(level_images, color_vis, segment_index_vis, debug)

    def get_neighbors(self, x: int, y: int, mask: np.ndarray) -> List[Point]:
        h, w = mask.shape
        out = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = x + dx
                ny = y + dy
                if 0 <= nx < w and 0 <= ny < h and mask[ny, nx]:
                    out.append((nx, ny))
        return out

    def split_skeleton_into_segments(self, skeleton: np.ndarray):
        mask = skeleton > 0
        ys, xs = np.where(mask)
        points = set(zip(xs, ys))

        degree = {p: len(self.get_neighbors(p[0], p[1], mask)) for p in points}
        key_points = {p for p, d in degree.items() if d != 2}

        visited_edges = set()
        segments: List[Segment] = []

        def edge_key(a: Point, b: Point):
            return tuple(sorted((a, b)))

        for start in key_points:
            for nb in self.get_neighbors(start[0], start[1], mask):
                e = edge_key(start, nb)
                if e in visited_edges:
                    continue
                seg = [start]
                prev = start
                cur = nb
                visited_edges.add(e)

                while True:
                    seg.append(cur)
                    if cur in key_points and cur != start:
                        break
                    nbs = self.get_neighbors(cur[0], cur[1], mask)
                    nbs = [p for p in nbs if p != prev]
                    if len(nbs) != 1:
                        break
                    nxt = nbs[0]
                    e = edge_key(cur, nxt)
                    if e in visited_edges:
                        break
                    visited_edges.add(e)
                    prev = cur
                    cur = nxt

                if len(seg) >= 2:
                    segments.append(seg)

        used_points = set()
        for seg in segments:
            used_points.update(seg)

        remaining = list(points - used_points)
        while remaining:
            start = remaining[0]
            q = deque([start])
            seen = {start}
            comp = []
            while q:
                p = q.popleft()
                comp.append(p)
                for nb in self.get_neighbors(p[0], p[1], mask):
                    if nb not in seen and nb not in used_points:
                        seen.add(nb)
                        q.append(nb)
            if len(comp) >= 2:
                segments.append(comp)
            used_points.update(comp)
            remaining = list(points - used_points)

        point_to_segments = defaultdict(set)
        for i, seg in enumerate(segments):
            for p in seg:
                point_to_segments[p].add(i)
        return segments, point_to_segments

    def build_segment_graph(self, segments: Sequence[Segment], point_to_segments):
        graph = [set() for _ in segments]
        for _, seg_ids in point_to_segments.items():
            if len(seg_ids) < 2:
                continue
            ids = list(seg_ids)
            for i in range(len(ids)):
                for j in range(i + 1, len(ids)):
                    a = ids[i]
                    b = ids[j]
                    graph[a].add(b)
                    graph[b].add(a)
        return graph

    def get_outer_contour_mask(self, binary: np.ndarray, radius: int = 3) -> np.ndarray:
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            raise ValueError("ë°”ê¹¥ ìœ¤ê³½ì„ ì„ ì°¾ì§€ ëª»í–ˆìŠµë‹ˆë‹¤.")
        outer = max(contours, key=cv2.contourArea)
        mask = np.zeros_like(binary)
        cv2.drawContours(mask, [outer], -1, 255, thickness=1)
        if radius > 0:
            kernel_size = radius * 2 + 1
            kernel = np.ones((kernel_size, kernel_size), np.uint8)
            mask = cv2.dilate(mask, kernel, iterations=1)
        return mask

    def find_outer_segments(self, segments: Sequence[Segment], outer_mask: np.ndarray) -> Set[int]:
        outer_segments = set()
        for i, seg in enumerate(segments):
            if not seg:
                continue
            hit = sum(1 for x, y in seg if outer_mask[y, x] > 0)
            ratio = hit / max(len(seg), 1)
            if ratio >= 0.35 or hit >= max(5, int(len(seg) * 0.25)):
                outer_segments.add(i)
        return outer_segments

    def compute_tree_distance_from_outer(self, segments, graph, outer_segments):
        if not outer_segments:
            longest = int(np.argmax([len(s) for s in segments]))
            outer_segments = {longest}

        dist = np.full(len(segments), -1, dtype=np.int32)
        q = deque()
        for sidx in outer_segments:
            dist[sidx] = 0
            q.append(sidx)

        while q:
            cur = q.popleft()
            for nb in graph[cur]:
                if dist[nb] != -1:
                    continue
                dist[nb] = dist[cur] + 1
                q.append(nb)

        outer_points = []
        for idx in outer_segments:
            outer_points.extend(segments[idx])
        outer_points = np.array(outer_points, dtype=np.float32)

        for idx in np.where(dist == -1)[0]:
            pts = np.array(segments[idx], dtype=np.float32)
            if len(pts) == 0 or len(outer_points) == 0:
                dist[idx] = 2
                continue
            pts_sample = pts[::max(1, len(pts) // 30)]
            outer_sample = outer_points[::max(1, len(outer_points) // 200)]
            d2 = ((pts_sample[:, None, :] - outer_sample[None, :, :]) ** 2).sum(axis=2)
            min_d = float(np.sqrt(d2.min()))
            dist[idx] = 1 if min_d <= 6 else 2
        return dist, outer_segments

    def assign_tree_levels_balanced_inner(self, segments, graph, outer_segments):
        dist, outer_segments = self.compute_tree_distance_from_outer(segments, graph, outer_segments)
        levels = np.full(len(segments), 2, dtype=np.int32)
        for idx in outer_segments:
            levels[idx] = 0

        inner_indices = [idx for idx in range(len(segments)) if idx not in outer_segments]
        if len(inner_indices) == 0:
            return levels, dist
        if len(inner_indices) == 1:
            levels[inner_indices[0]] = 1
            return levels, dist

        inner_indices.sort(key=lambda idx: (dist[idx], -len(segments[idx]), idx))
        lengths = np.array([len(segments[idx]) for idx in inner_indices], dtype=np.int32)
        total = int(lengths.sum())

        best_cut = 1
        best_score = None
        prefix = 0
        for cut in range(1, len(inner_indices)):
            prefix += int(lengths[cut - 1])
            score = abs(prefix - (total - prefix))
            if best_score is None or score < best_score:
                best_score = score
                best_cut = cut

        for idx in inner_indices[:best_cut]:
            levels[idx] = 1
        for idx in inner_indices[best_cut:]:
            levels[idx] = 2

        for idx, seg in enumerate(segments):
            if idx in outer_segments or len(seg) >= self.min_segment_length:
                continue
            nbs = [n for n in graph[idx] if n not in outer_segments]
            if not nbs:
                continue
            levels[idx] = int(np.clip(round(np.median([levels[n] for n in nbs])), 1, 2))
        return levels, dist

    def make_level_images(self, skeleton, segments, levels):
        level_map = np.full(skeleton.shape, -1, dtype=np.int32)
        for i, seg in enumerate(segments):
            lv = int(levels[i])
            for x, y in seg:
                level_map[y, x] = lv

        imgs = []
        for lv in range(self.num_levels):
            img = np.zeros_like(skeleton)
            img[level_map == lv] = 255
            imgs.append(img)

        color = np.zeros((skeleton.shape[0], skeleton.shape[1], 3), dtype=np.uint8)
        for lv in range(self.num_levels):
            color[level_map == lv] = self.colors_bgr[lv]
        return imgs, color, level_map

    def draw_segment_index(self, skeleton, segments, levels):
        vis = cv2.cvtColor(skeleton, cv2.COLOR_GRAY2BGR)
        for idx, seg in enumerate(segments):
            if not seg:
                continue
            mx = int(np.mean([p[0] for p in seg]))
            my = int(np.mean([p[1] for p in seg]))
            lv = int(levels[idx])
            cv2.putText(
                vis,
                f"{idx}:{lv}",
                (mx, my),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.32,
                self.colors_bgr[lv],
                1,
                cv2.LINE_AA,
            )
        return vis


import threading

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

import uuid
from sandart_msgs.msg import SandStroke, SandPoint, Bond
from sandart_msgs.srv import ProcessImage, TripleLayerdImages

class SkeletonProcessorNode(Node):
    def __init__(self):
        super().__init__('skeleton_processor_node')
        self.processor = SkeletonTreeProcessor()
        self.bridge = CvBridge()

        self.service = self.create_service(ProcessImage, 'process_image', self.process_image_callback)
        self.popup_client = self.create_client(TripleLayerdImages, 'triple_layerd_images')

        self.bond_id = "skeleton_processor_node_to_lifecycle"
        self.bond_instance_id = str(uuid.uuid4())
        self.heartbeat_timeout = 3.0
        self.heartbeat_period = 0.1

        self.bond_pub = self.create_publisher(Bond, "/bond", 10)
        self.bond_timer = self.create_timer(0.1, self.publish_bond)

        self.get_logger().info('Skeleton processor node started.')
        self.get_logger().info('Service server: /process_image')
        self.get_logger().info('Service client: /triple_layerd_images')

    def publish_bond(self):
        msg = Bond()
        msg.id = self.bond_id
        msg.instance_id = self.bond_instance_id
        msg.active = True
        msg.heartbeat_timeout = self.heartbeat_timeout
        msg.heartbeat_period = self.heartbeat_period
        self.bond_pub.publish(msg)

    def process_image_callback(self, request, response):
        image_path = request.image_path
        self.get_logger().info(f'Accepted image request: {image_path}')
        threading.Thread(target=self.process_and_send, args=(image_path,), daemon=True).start()

        response.accepted = True
        response.message = 'accepted: ì´ë¯¸ì§€ ì²˜ë¦¬ ì‹œìž‘'
        return response

    def process_and_send(self, image_path: str):
        try:
            result = self.processor.process_image_file(image_path)
            cv2.imwrite("/home/sb/ws_cobot_pjt/ws_dsr/level0.png", result.level_images[0])
            cv2.imwrite("/home/sb/ws_cobot_pjt/ws_dsr/level1.png", result.level_images[1])
            cv2.imwrite("/home/sb/ws_cobot_pjt/ws_dsr/level2.png", result.level_images[2])
            cv2.imwrite("/home/sb/ws_cobot_pjt/ws_dsr/full_color_skeleton.png", result.color_visualization)
            self.get_logger().info(f'Processing done. debug={result.debug}')

            if not self.popup_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('/triple_layerd_images service not available')
                return

            req = TripleLayerdImages.Request()
            req.level0_outer_thickest = self.bridge.cv2_to_imgmsg(result.level_images[0], encoding='mono8')
            req.level1_middle = self.bridge.cv2_to_imgmsg(result.level_images[1], encoding='mono8')
            req.level2_inner_thinnest = self.bridge.cv2_to_imgmsg(result.level_images[2], encoding='mono8')
            req.full_color_skeleton = self.bridge.cv2_to_imgmsg(result.color_visualization, encoding='bgr8')

            future = self.popup_client.call_async(req)
            future.add_done_callback(self.popup_response_callback)
        except Exception as exc:
            self.get_logger().error(f'Processing failed: {exc}')

    def popup_response_callback(self, future):
        try:
            response = future.result()
            if response.accepted:
                self.get_logger().info(f'Popup node accepted images: {response.message}')
            else:
                self.get_logger().warn(f'Popup node rejected images: {response.message}')
        except Exception as exc:
            self.get_logger().error(f'Popup service call failed: {exc}')


def main():
    rclpy.init()
    node = SkeletonProcessorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()