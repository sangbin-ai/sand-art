#!/usr/bin/env python3
"""
파일명: floor_obstacle_node.py
패키지: ros_final_pjt_refact
역할  : TurtleBot4의 LiDAR가 감지하지 못하는 낮은 바닥 장애물(가위, 칼, 병 등)을
        YOLO 탐지 결과에서 추출하여 Nav2 global costmap에 가상 장애물로 등록한다.

        LiDAR는 일정 높이 이상만 스캔하기 때문에 바닥에 놓인 작은 물체를 감지하지 못한다.
        이 노드는 YOLO로 탐지된 바닥 장애물의 map 좌표와 물리 크기를 기반으로
        PointCloud2 포인트 그리드를 생성하여 /virtual_obstacles 토픽에 발행한다.
        Nav2가 이를 occupied로 마킹하면 경로 계획에서 해당 영역을 회피한다.

        새로운 바닥 장애물이 발견되면 YOLO 어노테이션 이미지 한 장을 {ns}/mission/photo로
        발행한다 — server_node가 HTTP POST /api/photo로 서버에 전송한다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
구독 토픽 (Subscribe)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  토픽명                        타입                                       QoS   설명
  {ns}/yolo/detections          ros_final_pjt_msgs/msg/YoloDetections      10    YOLO 탐지 결과
                                                                                 (floor_obstacles 배열만 사용)
  {ns}/yolo/display             sensor_msgs/msg/CompressedImage            10    YOLO 어노테이션 JPEG
                                                                                 (새 장애물 발견 시 사진 발행용 캐시)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
발행 토픽 (Publish)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  토픽명                  타입                               QoS   설명
  /virtual_obstacles      sensor_msgs/msg/PointCloud2        10    바닥 장애물 포인트 그리드 (map 프레임)
                                                                   Nav2 costmap의 virtual_obstacle_layer가 구독
                                                                   FLOOR_PUBLISH_INTERVAL_S(1초) 주기로 재발행
  {ns}/mission/events     std_msgs/msg/String                10    새 바닥 장애물 최초 발견 시 이벤트 문자열
                                                                   형식: FLOOR_OBSTACLE,label,x,y,width,height
                                                                   → server_node가 HTTP POST /api/event
  {ns}/mission/photo      sensor_msgs/msg/CompressedImage    10    새 바닥 장애물 최초 발견 시 사진 한 장
                                                                   → server_node가 HTTP POST /api/photo

  {ns} = config.py의 DEFAULT_NAMESPACE (기본값: /robot9)

Nav2 costmap 필수 설정:
  global_costmap에 virtual_obstacle_layer 추가:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    observation_sources: virtual
    virtual:
      topic: /virtual_obstacles
      data_type: PointCloud2
      clearing: false        ← LiDAR가 못 봐도 장애물이 사라지지 않도록
      marking: true
      raycasting_range: 0.0  ← 다른 센서의 ray clearing으로 삭제 방지
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
"""
import struct

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2, PointField, CompressedImage
from std_msgs.msg import String
from ros_final_pjt_msgs.msg import YoloDetections

from ros_final_pjt_refact.config import (
    FLOOR_POINT_SPACING_M, FLOOR_SIZE_MARGIN_M,
    FLOOR_DEDUP_DIST_M, FLOOR_PUBLISH_INTERVAL_S)


class FloorObstacleNode(Node):
    """
    바닥 장애물 → Nav2 costmap 등록 노드.

    탐지된 바닥 장애물을 누적 저장하고, 1초 주기로 전체 장애물을
    PointCloud2 포인트 그리드로 변환하여 /virtual_obstacles에 발행한다.
    같은 위치의 장애물은 FLOOR_DEDUP_DIST_M(10cm) 기준으로 중복 제거한다.
    """

    def __init__(self):
        super().__init__('floor_obstacle_node')
        ns = self.get_namespace()   # 런타임 네임스페이스 (/robot9 등)

        # 누적된 바닥 장애물 목록 — (cx, cy, width_m, height_m, label) 튜플의 리스트
        # 프로그램 실행 중 계속 추가되며, 삭제되지 않는다 (costmap에 영구 마킹)
        self._obstacles = []
        # 최신 YOLO 어노테이션 JPEG — 새 장애물 발견 시 사진 발행에 사용
        self._latest_jpeg = None

        # ── 구독 토픽 ──────────────────────────────────────────────────────────
        # {ns}/yolo/detections : YoloDetections에서 floor_obstacles 배열만 추출
        self.create_subscription(
            YoloDetections, f'{ns}/yolo/detections', self._det_cb, 10)

        # {ns}/yolo/display : 새 장애물 발견 시 사진 발행용 최신 프레임 캐시
        self.create_subscription(
            CompressedImage, f'{ns}/yolo/display', self._display_cb, 10)

        # ── 발행 토픽 ──────────────────────────────────────────────────────────
        # /virtual_obstacles : Nav2 costmap이 구독하는 가상 장애물 포인트 클라우드
        self.pc_pub = self.create_publisher(PointCloud2, '/virtual_obstacles', 10)

        # {ns}/mission/events : 새 바닥 장애물 발견 이벤트 문자열
        #   server_node가 구독하여 HTTP POST /api/event로 전송
        #   형식: FLOOR_OBSTACLE,scissors,2.10,3.45,0.11,0.05
        self.event_pub = self.create_publisher(String, f'{ns}/mission/events', 10)

        # {ns}/mission/photo : 새 바닥 장애물 최초 발견 시 사진 한 장
        self.photo_pub = self.create_publisher(
            CompressedImage, f'{ns}/mission/photo', 10)

        # 1초 주기 타이머 — 누적된 모든 장애물을 PointCloud2로 재발행
        self.create_timer(FLOOR_PUBLISH_INTERVAL_S, self._publish)

    # ── 구독 콜백 ─────────────────────────────────────────────────────────────

    def _display_cb(self, msg):
        """
        {ns}/yolo/display 콜백.
        최신 YOLO 어노테이션 JPEG를 _latest_jpeg에 캐시한다.
        _publish_photo()가 이 데이터를 {ns}/mission/photo로 발행한다.
        """
        self._latest_jpeg = bytes(msg.data)

    def _det_cb(self, msg):
        """
        {ns}/yolo/detections 콜백.
        YoloDetections 메시지의 floor_obstacles 배열을 처리한다.

        각 바닥 장애물에 대해:
          - 기존 누적 목록과 거리 비교 (FLOOR_DEDUP_DIST_M = 10cm)
          - 10cm 이내에 이미 등록된 장애물이 있으면 중복으로 무시
          - 새로운 위치이면 누적 목록에 추가 + 사진 발행
        """
        for fo in msg.floor_obstacles:
            cx, cy = fo.x, fo.y
            w, h = fo.width, fo.height
            label = fo.label
            # 기존 장애물과의 거리 비교 — 유클리드 거리 제곱으로 비교 (sqrt 생략)
            is_dup = any(
                (cx - ox) ** 2 + (cy - oy) ** 2 < FLOOR_DEDUP_DIST_M ** 2
                for ox, oy, _, _, _ in self._obstacles)
            if not is_dup:
                self._obstacles.append((cx, cy, w, h, label))
                self.get_logger().info(
                    f'Floor obstacle [{label}]: ({cx:.2f}, {cy:.2f}) '
                    f'size: {w:.2f}x{h:.2f}m')
                # 새 장애물 최초 발견 시 이벤트 전송 + 사진 발행
                self._publish_event(label, cx, cy, w, h)
                self._publish_photo()

    # ── 발행 헬퍼 ─────────────────────────────────────────────────────────────

    def _publish_event(self, label, cx, cy, w, h):
        """
        {ns}/mission/events 토픽에 바닥 장애물 이벤트 문자열을 발행한다.
        server_node가 구독하여 HTTP POST /api/event로 서버에 전송한다.

        발행 형식: FLOOR_OBSTACLE,{label},{x:.2f},{y:.2f},{width:.2f},{height:.2f}
        예시: FLOOR_OBSTACLE,scissors,2.10,3.45,0.11,0.05

        호출 시점: 새로운 바닥 장애물이 최초로 발견될 때 (_det_cb)
        """
        msg = String()
        msg.data = f'FLOOR_OBSTACLE,{label},{cx:.2f},{cy:.2f},{w:.2f},{h:.2f}'
        self.event_pub.publish(msg)

    def _publish_photo(self):
        """
        캐시된 최신 YOLO 어노테이션 JPEG를 {ns}/mission/photo 토픽으로 발행한다.
        server_node가 이를 구독하여 HTTP POST /api/photo로 서버에 전송한다.

        호출 시점: 새로운 바닥 장애물이 최초로 발견될 때 (_det_cb)
        """
        if self._latest_jpeg is None:
            return   # 아직 카메라 프레임이 수신되지 않은 경우
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        out.data = self._latest_jpeg
        self.photo_pub.publish(out)

    # ── 타이머 콜백 ───────────────────────────────────────────────────────────

    def _publish(self):
        """
        FLOOR_PUBLISH_INTERVAL_S(1초) 주기 타이머 콜백.
        누적된 모든 바닥 장애물을 PointCloud2 포인트 그리드로 변환하여
        /virtual_obstacles에 발행한다.
        Nav2 costmap의 virtual_obstacle_layer가 이를 구독하여 occupied로 마킹한다.
        """
        if not self._obstacles:
            return   # 장애물이 없으면 발행 불필요
        all_points = []
        for cx, cy, w, h, _label in self._obstacles:
            all_points.extend(self._generate_points(cx, cy, w, h))
        self.pc_pub.publish(self._make_pc2(all_points))

    # ── 포인트 생성 ───────────────────────────────────────────────────────────

    @staticmethod
    def _generate_points(cx, cy, width_m, height_m):
        """
        장애물 중심 (cx, cy)과 물리 크기 (width_m, height_m)를 기반으로
        FLOOR_POINT_SPACING_M(5cm) 간격의 포인트 그리드를 생성한다.
        FLOOR_SIZE_MARGIN_M(5cm) 마진을 양쪽에 추가하여 여유를 둔다.

        예) 가위 (가로 0.12m, 세로 0.04m):
            마진 포함 영역: 0.22m × 0.14m
            포인트 수: 5 × 3 = 15개 (5cm 간격)

        Args:
            cx, cy    : 장애물 중심 map 좌표 (미터)
            width_m   : 장애물 가로 크기 (미터)
            height_m  : 장애물 세로 크기 (미터)

        Returns:
            list of (x, y, z) — z는 항상 0.0 (지면)
        """
        half_w = width_m / 2.0 + FLOOR_SIZE_MARGIN_M
        half_h = height_m / 2.0 + FLOOR_SIZE_MARGIN_M
        points = []
        x = cx - half_w
        while x <= cx + half_w:
            y = cy - half_h
            while y <= cy + half_h:
                points.append((x, y, 0.0))
                y += FLOOR_POINT_SPACING_M
            x += FLOOR_POINT_SPACING_M
        if not points:
            points.append((cx, cy, 0.0))   # 크기가 0인 경우 최소 1개
        return points

    def _make_pc2(self, points):
        """
        (x, y, z) 튜플 리스트를 sensor_msgs/msg/PointCloud2 메시지로 변환한다.

        포인트 구조:
          - frame: map (절대 좌표)
          - 필드: x(FLOAT32, 4B) + y(FLOAT32, 4B) + z(FLOAT32, 4B) = 12B/포인트
          - z = 0.0 (지면 높이)

        Args:
            points: [(x, y, z), ...] 리스트

        Returns:
            PointCloud2 메시지
        """
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'   # Nav2 costmap과 같은 프레임 사용
        msg.height = 1                 # 비정형(unorganized) 포인트 클라우드
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12              # 포인트 하나의 바이트 수 (4+4+4)
        msg.row_step = 12 * len(points)  # 전체 데이터 바이트 수
        msg.is_dense = True              # NaN/Inf 포인트 없음
        msg.data = b''.join(
            struct.pack('fff', x, y, z) for x, y, z in points)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FloorObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
