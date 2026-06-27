#!/usr/bin/env python3
"""
================================================================================
YOLO 객체 탐지 GUI 노드 (YOLOHumanReadableGUINode)
================================================================================

목적:
카메라에서 들어오는 RGB 이미지를 YOLO로 분석해서
객체(박스) 탐지 및 거리 측정

기능:
1. YOLO로 객체 탐지
2. 깊이 카메라로 거리 측정
3. 실시간 GUI로 시각화
4. 탐지된 객체의 중심좌표와 거리를 토픽으로 발행

작동 흐름:
RGB 카메라 → YOLO 분석 → 객체 탐지 + 거리 측정 → GUI 표시 + 토픽 발행

================================================================================
"""

import rclpy
from rclpy.node import Node

# ==================== ROS2 메시지 타입 ====================
from sensor_msgs.msg import CompressedImage, Image  # 카메라 이미지
from geometry_msgs.msg import Point                 # 좌표(x, y, z)

# ==================== 이미지 처리 라이브러리 ====================
import cv2                          # OpenCV (이미지 처리, GUI)
import numpy as np                  
from ultralytics import YOLO        


class YOLOHumanReadableGUINode(Node):
    """
    YOLO 객체 탐지 + 거리 측정 + GUI 표시 노드
    
    Parameters:
        robot_id (int): 로봇 ID (기본값: 6)
    """
    
    def __init__(self, robot_id=6, use_gpu=True):
        # ========== 1단계: 노드 초기화 ==========
        super().__init__(f'robot{robot_id}_yolo_node')

        self.robot_id = robot_id
        self.ns = f'/robot{robot_id}'

        # ========== 2단계: YOLO 모델 로드 ==========
        self.model = YOLO('/home/sb/turtlebot4_goal/src/turtlebot4_goal/turtlebot4_goal/yolov8n.pt')
        
        try:
            if use_gpu:
                import torch
                if torch.cuda.is_available():
                    self.model.to('cuda')
                    self.get_logger().info('GPU 모드 활성화')
                else:
                    self.get_logger().warn('GPU 없음. CPU 모드 실행')
                    self.model.to('cpu')
            else:
                self.model.to('cpu')
                self.get_logger().info('CPU 모드 활성화')
        except Exception as e:
            self.get_logger().error(f'GPU 설정 오류: {e}. CPU로 진행')
            self.model.to('cpu')

        # ========== 3단계: 상태 변수 초기화 ==========
        # 최근에 받은 깊이 이미지를 저장
        # (RGB와 depth는 주기가 다를 수 있어서 최신값 보관 필요)
        self.latest_depth = None

        # ========== 4단계: RGB 이미지 구독 ==========
        # 압축된 RGB 이미지 (대역폭 절약)
        self.rgb_sub = self.create_subscription(
            CompressedImage,
            f'{self.ns}/oakd/rgb/image_raw/compressed',  # OAK-D 카메라 RGB
            self.rgb_callback,
            10
        )

        # ========== 5단계: 깊이 이미지 구독 ==========
        # 깊이 정보 (mm 단위의 거리값)
        self.depth_sub = self.create_subscription(
            Image,
            f'{self.ns}/oakd/stereo/image_raw',  # OAK-D 카메라 Depth
            self.depth_callback,
            10
        )

        # ========== 6단계: 탐지된 객체 중심좌표 발행 ==========
        # Navigation이나 다른 노드가 이 좌표를 받아서 로봇 제어
        self.center_pub = self.create_publisher(
            Point,
            f'{self.ns}/detected_object_center',
            10
        )

        # ========== 초기화 완료 ==========
        self.get_logger().info(f'YOLO GUI Node 시작: {self.ns}')
        self.get_logger().info(f'RGB   : {self.ns}/oakd/rgb/image_raw/compressed')
        self.get_logger().info(f'Depth : {self.ns}/oakd/stereo/image_raw')


    # ================================================================================
    # 깊이 이미지 처리
    # ================================================================================

    def depth_callback(self, msg: Image):
        """
        깊이 이미지 수신 콜백
        
        목적:
          - 깊이 이미지를 numpy 배열로 변환
          - 최신 깊이 데이터 저장 (나중에 거리 계산할 때 사용)
        
        깊이 인코딩 종류:
          - '16UC1': 16비트 정수 (mm 단위, 0~65535)
          - 'mono16': 16UC1과 같음
          - '32FC1': 32비트 실수 (m 단위)
        """
        
        try:
            # ========== 16비트 정수 깊이 (mm 단위) ==========
            if msg.encoding in ['16UC1', 'mono16']:
                # 바이트 데이터를 16비트 정수 배열로 변환
                # reshape: (높이, 너비) 형태로 2D 배열로 변환
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(
                    msg.height,
                    msg.width
                )
                self.latest_depth = depth

            # ========== 32비트 실수 깊이 (m 단위) ==========
            elif msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape(
                    msg.height,
                    msg.width
                )
                self.latest_depth = depth

            # ========== 지원하지 않는 형식 ==========
            else:
                self.get_logger().warn(f'지원하지 않는 depth encoding: {msg.encoding}')

        except Exception as e:
            self.get_logger().warn(f'Depth 변환 실패: {e}')


    # ================================================================================
    # 거리 계산
    # ================================================================================

    def get_distance_at_center(self, cx, cy):
        """
        객체 중심점에서의 거리 계산
        
        목적:
          RGB 이미지의 중심좌표 (cx, cy)에 대응하는
          깊이 이미지에서의 거리값 추출
        
        좌표 변환이 필요한 이유:
          - RGB 해상도와 Depth 해상도가 다를 수 있음
          - RGB 640x480, Depth 400x300 같은 경우
          - 같은 비율로 스케일 조정 필요
        
        Parameters:
            cx, cy: RGB 이미지에서의 중심좌표
            frame_shape: RGB 이미지의 (높이, 너비, 채널)
        
        Returns:
            거리 (미터 단위), 또는 None (거리 측정 실패)
        """
        
        if self.latest_depth is None:
            return None
        dx = cx
        dy = cy
        depth_h, depth_w = self.latest_depth.shape[:2]
        # ========== 해상도 가져오기 ==========
        #frame_h, frame_w = frame_shape[:2]          # RGB 해상도
        #depth_h, depth_w = self.latest_depth.shape[:2]  # Depth 해상도

        # ========== RGB 좌표 → Depth 좌표 변환 ==========
        # RGB와 Depth 해상도 비율을 맞춰서 좌표 변환
        #     RGB의 (320, 240) → Depth의 (200, 150)
        #dx = int(cx * depth_w / frame_w)
        #dy = int(cy * depth_h / frame_h)

        # ========== 경계 검사 ==========
        # 좌표가 깊이 이미지 범위 밖이면 None 반환
        if dx < 0 or dy < 0 or dx >= depth_w or dy >= depth_h:
            return None

        # ========== 5x5 영역에서 유효한 깊이값 추출 ==========
        # 왜 5x5?
        #   - 한 픽셀의 깊이값은 노이즈가 많을 수 있음
        #   - 주변 5x5 픽셀의 중간값(median)을 사용하면 더 안정적
        x_min = max(dx - 2, 0)
        x_max = min(dx + 3, depth_w)
        y_min = max(dy - 2, 0)
        y_max = min(dy + 3, depth_h)

        # 5x5 영역 추출
        patch = self.latest_depth[y_min:y_max, x_min:x_max]

        # ========== 유효한 깊이값만 필터링 ==========
        # np.isfinite(): NaN, Inf가 아닌 값만 선택
        valid = patch[np.isfinite(patch)]
        # 0 이하의 값(오류값) 제거
        valid = valid[valid > 0]

        # ========== 유효한 값이 없으면 None ==========
        if valid.size == 0:
            return None

        # ========== 중간값(median) 계산 ==========
        # 평균(mean) 대신 중간값을 사용
        # 이유: 노이즈에 더 강함 (이상치에 영향 적음)
        depth_value = float(np.median(valid))

        # ========== 깊이값을 미터 단위로 변환 ==========
        # 16UC1: mm 단위 → 미터로 변환 (1000으로 나누기)
        # 32FC1: 이미 미터 단위
        if self.latest_depth.dtype == np.uint16:
            distance_m = depth_value / 1000.0
        else:
            distance_m = depth_value

        # ========== 거리 범위 검사 ==========
        # 0m 이하 또는 10m 이상: 오류값으로 판단
        if distance_m <= 0.0 or distance_m > 10.0:
            return None

        return distance_m


    # ================================================================================
    # 🎨 GUI 그리기 (정보 패널)
    # ================================================================================

    def draw_info_panel(self, frame, x1, y1, x2, y2, label, conf, distance_m, cx, cy):
        """
        탐지된 객체 주변에 정보 패널 그리기
        
        🎨 표시 항목:
          1. 바운딩 박스 (초록색 테두리)
          2. 중심점 (빨간 점)
          3. 정보 패널 (반투명 검은 배경)
             - 객체 이름
             - 거리
             - 신뢰도
             - 중심좌표
        
        Parameters:
            frame: OpenCV 이미지
            x1, y1, x2, y2: 바운딩 박스 좌표 (좌상단, 우하단)
            label: 객체 이름 ('BOX' 등)
            conf: 신뢰도 (0~1)
            distance_m: 거리 (미터)
            cx, cy: 중심좌표
        """
        
        # ========== 1단계: 바운딩 박스 그리기 ==========
        # cv2.rectangle(이미지, 좌상단, 우하단, 색상BGR, 두께)
        # 초록색: (30, 220, 30), 두께: 2픽셀
        cv2.rectangle(frame, (x1, y1), (x2, y2), (30, 220, 30), 2)

        # ========== 2단계: 중심점 표시 ==========
        # 빨간 원 (중심점): (0, 0, 255)
        cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
        # 흰 테두리: (255, 255, 255)
        cv2.circle(frame, (cx, cy), 10, (255, 255, 255), 2)

        # ========== 3단계: 표시할 텍스트 준비 ==========
        distance_text = 'Distance: N/A' if distance_m is None else f'Distance: {distance_m:.2f} m'
        title_text = f'{label.upper()}'
        conf_text = f'Confidence: {conf * 100:.0f}%'
        center_text = f'Center: ({cx}, {cy})'

        # 여러 줄 텍스트를 리스트로 관리
        lines = [
            title_text,
            distance_text,
            conf_text,
            center_text
        ]

        # ========== 4단계: 텍스트 크기 계산 ==========
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.55
        thickness = 2
        line_height = 24          # 각 줄의 높이
        padding = 8               # 패널 내부 여백

        # 가장 긴 텍스트의 너비를 패널 너비로 설정
        panel_w = 0
        for line in lines:
            text_size, _ = cv2.getTextSize(line, font, font_scale, thickness)
            panel_w = max(panel_w, text_size[0])

        panel_w += padding * 2
        panel_h = line_height * len(lines) + padding

        # ========== 5단계: 패널 위치 계산 ==========
        # 패널을 바운딩 박스 위에 배치
        panel_x1 = x1
        panel_y1 = max(y1 - panel_h - 8, 0)  # 위로 갈수록 상단 방지
        panel_x2 = min(panel_x1 + panel_w, frame.shape[1] - 1)  # 화면 오른쪽 넘지 않게
        panel_y2 = panel_y1 + panel_h

        # ========== 6단계: 반투명 검은 배경 그리기 ==========
        # 배경 투명도 조절하기 위해 overlay 사용
        overlay = frame.copy()
        cv2.rectangle(
            overlay,
            (panel_x1, panel_y1),
            (panel_x2, panel_y2),
            (0, 0, 0),  # 검은색
            -1  # 채우기
        )
        alpha = 0.65  # 투명도 (0.65 = 65% 불투명)
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)

        # ========== 7단계: 상단 상태 바 (강조색) ==========
        # 패널의 상단에 노란색 바 (시각적 강조)
        cv2.rectangle(
            frame,
            (panel_x1, panel_y1),
            (panel_x2, panel_y1 + 5),
            (0, 255, 255),  # 노란색
            -1
        )

        # ========== 8단계: 텍스트 출력 ==========
        text_x = panel_x1 + padding
        text_y = panel_y1 + padding + 16

        for idx, line in enumerate(lines):
            # 각 줄마다 다른 색상 적용
            if idx == 0:
                color = (0, 255, 255)      # 노란색 (제목)
            elif idx == 1:
                color = (255, 255, 255)    # 흰색 (거리)
            else:
                color = (200, 200, 200)    # 회색 (나머지)

            cv2.putText(
                frame,
                line,
                (text_x, text_y + idx * line_height),
                font,
                font_scale,
                color,
                thickness,
                cv2.LINE_AA  # 안티앨리어싱 (글자 부드럽게)
            )


    # ================================================================================
    # 📷 RGB 이미지 처리 (YOLO 실행)
    # ================================================================================

    def rgb_callback(self, msg: CompressedImage):
        """
        RGB 이미지 수신 및 YOLO 분석
        
        🔄 처리 순서:
          1. 압축된 이미지 → numpy 배열 변환
          2. YOLO로 객체 탐지
          3. 각 탐지 객체에 대해:
             - 거리 측정
             - 중심좌표 발행
             - GUI 그리기
          4. 결과 화면 표시
        """
        
        # ========== 1단계: 압축 이미지 디코딩 ==========
        # msg.data: JPEG 압축된 바이트 데이터
        # cv2.imdecode: 바이트 → OpenCV 이미지로 변환
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # ========== 이미지 변환 실패 확인 ==========
        if frame is None:
            self.get_logger().warn('RGB 이미지 변환 실패')
            return

        # ========== 2단계: YOLO로 객체 탐지 ==========
        # verbose=False: 상세 로그 안 보이기
        results = self.model(frame, verbose=False)

        # ========== 탐지된 객체 개수 ==========
        detected_count = 0

        # ========== 3단계: 탐지된 각 객체 처리 ==========
        for result in results:
            for box in result.boxes:
                # 신뢰도 추출 (0~1 범위)
                conf = float(box.conf[0])

                # ========== 신뢰도 필터링 ==========
                # 신뢰도 30% 미만은 무시 (오탐지 방지)
                if conf < 0.30:
                    continue

                # ========== 바운딩 박스 좌표 ==========
                # xyxy: (x1, y1, x2, y2) = (좌상단, 우하단)
                # .cpu().numpy(): GPU 텐서 → numpy 배열로 변환
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                # ========== 클래스 정보 ==========
                # cls_id: 클래스 인덱스 (YOLO 모델의 클래스 ID)
                cls_id = int(box.cls[0])
                # raw_name: YOLO 모델의 원본 클래스 이름
                raw_name = self.model.names[cls_id]

                # ========== 표시 이름 설정 ==========
                # 프로젝트에서는 모든 객체를 'BOX'로 통일
                # (YOLO 모델의 다양한 클래스를 프로젝트 용도에 맞게 단순화)
                display_name = 'BOX'

                # ========== 중심점 계산 ==========
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # ========== 거리 측정 ==========
                distance_m = self.get_distance_at_center(cx, cy)

                # ========== 4단계: 중심좌표 발행 ==========
                # Navigation 등 다른 노드에서 이 정보를 받아서 로봇 제어
                point_msg = Point()
                point_msg.x = float(cx)            # 화면상 x좌표
                point_msg.y = float(cy)            # 화면상 y좌표
                point_msg.z = -1.0 if distance_m is None else float(distance_m)  # 거리
                self.center_pub.publish(point_msg)

                # ========== 5단계: GUI 그리기 ==========
                self.draw_info_panel(
                    frame,
                    x1, y1, x2, y2,
                    display_name,
                    conf,
                    distance_m,
                    cx, cy
                )

                detected_count += 1

                # ========== 로그 출력 ==========
                distance_log = 'N/A' if distance_m is None else f'{distance_m:.2f}m'
                self.get_logger().info(
                    f'탐지: raw={raw_name}, display={display_name}, '
                    f'conf={conf:.2f}, center=({cx},{cy}), distance={distance_log}'
                )

        # ========== 6단계: 화면 상단 전체 상태 표시 ==========
        # 정보 패널이 아닌 화면 상단에 전체 현황 표시
        status_text = f'Robot{self.robot_id} YOLO View | Objects: {detected_count}'
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 34), (30, 30, 30), -1)
        cv2.putText(
            frame,
            status_text,
            (12, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA
        )

        # ========== 7단계: 결과 화면 표시 ==========
        # cv2.imshow: 윈도우에 이미지 표시
        # cv2.waitKey(1): 1ms 대기 (다음 프레임으로 넘어가기)
        cv2.imshow(f'Robot{self.robot_id} Detection GUI', frame)
        cv2.waitKey(1)


# ================================================================================
# 🚀 프로그램 시작점
# ================================================================================

def main(args=None):
    """프로그램 메인 함수"""
    
    rclpy.init(args=args)

    # ⚠️ 여기서 robot_id 변경 가능
    node = YOLOHumanReadableGUINode(robot_id=6)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ========== 정리 작업 ==========
        cv2.destroyAllWindows()  # OpenCV 윈도우 종료
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()