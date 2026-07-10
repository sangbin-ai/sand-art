import threading
import traceback

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image

from sandart_msgs.srv import ProcessImage


class ProcessImageClient(Node):
    """ROS2 ProcessImage service client.

    중요:
    - 이 클래스는 rclpy spin 스레드/worker 스레드에서 동작한다.
    - 여기서 Qt 위젯(SettingDialog.add_log 등)을 직접 호출하면 창 종료/크래시가 날 수 있다.
    - 그래서 worker 내부에서는 print만 하고, GUI 갱신은 finished_callback(Qt Signal)로만 넘긴다.
    """

    def __init__(self, log_callback=None):
        super().__init__("process_image_client")
        self.log = log_callback  # 직접 호출하지 않음. Qt 스레드 안전성 때문에 보관만 함.
        self.finished_callback = None
        self.client = self.create_client(ProcessImage, "/process_image")
        self._lock = threading.Lock()
        self._busy = False

    def cv2_to_ros_image(self, img):
        msg = Image()
        msg.height = int(img.shape[0])
        msg.width = int(img.shape[1])

        if img.ndim == 2:
            arr = np.ascontiguousarray(img.astype(np.uint8))
            msg.encoding = "mono8"
            msg.step = int(msg.width)
            msg.data = arr.tobytes()
            return msg

        if img.ndim == 3 and img.shape[2] == 3:
            arr = np.ascontiguousarray(img.astype(np.uint8))
            msg.encoding = "bgr8"
            msg.step = int(msg.width * 3)
            msg.data = arr.tobytes()
            return msg

        raise ValueError(f"Unsupported cv2 image shape: {img.shape}")

    def process(self, image_path):
        with self._lock:
            if self._busy:
                print("[PROCESS_CLIENT] busy - request ignored", flush=True)
                return
            self._busy = True

        print(f"[PROCESS_CLIENT] process start: {image_path}", flush=True)
        threading.Thread(
            target=self._process_worker,
            args=(image_path,),
            daemon=True,
        ).start()

    def _finish_busy(self):
        with self._lock:
            self._busy = False

    def _process_worker(self, image_path):
        try:
            if not self.client.wait_for_service(timeout_sec=3.0):
                print("[PROCESS_CLIENT] service /process_image not available", flush=True)
                self._finish_busy()
                return

            img = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
            if img is None:
                print(f"[PROCESS_CLIENT] image read failed: {image_path}", flush=True)
                self._finish_busy()
                return

            # 너무 큰 원본은 ROS 메시지 전송/처리 부담이 커진다. 안정화 우선으로 900px 제한.
            h, w = img.shape[:2]
            max_side = 900
            if max(h, w) > max_side:
                scale = max_side / float(max(h, w))
                img = cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)
                print(f"[PROCESS_CLIENT] resized input: {w}x{h} -> {img.shape[1]}x{img.shape[0]}", flush=True)

            req = ProcessImage.Request()
            req.input_img = self.cv2_to_ros_image(img)

            print(f"[PROCESS_CLIENT] request image: {image_path}", flush=True)
            future = self.client.call_async(req)
            future.add_done_callback(self.response_callback)
            print("[PROCESS_CLIENT] call_async done", flush=True)

        except Exception:
            print("[PROCESS_CLIENT] worker error", flush=True)
            traceback.print_exc()
            self._finish_busy()

    def response_callback(self, future):
        print("[PROCESS_CLIENT] response_callback", flush=True)
        try:
            response = future.result()
            print("[PROCESS_CLIENT] future.result done", flush=True)
            if self.finished_callback:
                self.finished_callback(response)
        except Exception:
            print("[PROCESS_CLIENT] response error", flush=True)
            traceback.print_exc()
        finally:
            self._finish_busy()


class ProcessImageClientThread:
    """
    ProcessImageClient와 HMI의 다른 ROS 노드를 하나의 전용 Executor에서 처리합니다.

    기존처럼 rclpy.spin()과 GUI QTimer의 rclpy.spin_once()를 동시에 사용하면
    같은 전역 Executor/Context가 충돌하여 서비스 응답 callback이 간헐적으로
    실행되지 않을 수 있습니다.
    """

    def __init__(self, log_callback=None):
        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = ProcessImageClient(log_callback)

        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)

        self.thread = threading.Thread(
            target=self.executor.spin,
            daemon=True,
        )
        self.thread.start()

    def add_node(self, node):
        self.executor.add_node(node)

    def remove_node(self, node):
        try:
            self.executor.remove_node(node)
        except Exception:
            pass

    def process(self, image_path):
        self.node.process(image_path)

    def shutdown(self):
        try:
            self.executor.remove_node(self.node)
        except Exception:
            pass

        try:
            self.node.destroy_node()
        except Exception:
            pass

        try:
            self.executor.shutdown(timeout_sec=1.0)
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
