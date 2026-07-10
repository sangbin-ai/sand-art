"""
=========================================================
Sand Art Robot HMI

ui/main_window.py

=========================================================
"""
import os
import signal
import traceback
from pathlib import Path

from PySide6.QtCore import Qt, QProcess, Signal, QTimer
import rclpy

from rclpy.node import Node

from sandart_msgs.msg import DrawingProgress
from std_msgs.msg import Bool
from PySide6.QtGui import QPixmap, QImage
import subprocess

from PySide6.QtWidgets import (
    QMainWindow,
    QStackedWidget,
    QMessageBox,
)

# =========================================================
# Pages
# =========================================================
# from ros.robot_control_client import RobotControlClient
from ros.process_image_client import ProcessImageClientThread
from pages.start_page import StartPage
from pages.image_page import ImagePage
from pages.gray_page import GrayPage
from pages.edge_page import EdgePage
from pages.sand_preview_page import SandPreviewPage
from pages.parameter_page import ParameterPage
from pages.drawing_page import DrawingPage
from pages.finish_page import FinishPage
from dialogs.setting_dialog import SettingDialog

class DrawingProgressSubscriber(Node):

    def __init__(self, callback):

        super().__init__("drawing_progress_subscriber")

        self.callback = callback

        self.create_subscription(
            DrawingProgress,
            "/drawing_progress",
            self.progress_callback,
            10,
        )

    def progress_callback(self, msg):

        self.callback(msg)


class SandClearDoneSubscriber(Node):

    def __init__(self, callback):
        super().__init__("sand_clear_done_subscriber")

        self.callback = callback

        self.create_subscription(
            Bool,
            "/dsr01/sand_clear_done",
            self.done_callback,
            10,
        )

    def done_callback(self, msg):
        self.callback(bool(msg.data))


class MainWindow(QMainWindow):

    process_image_result = Signal(object)
    sand_clear_done_result = Signal(bool)
    """
    메인 윈도우

    역할

    - 모든 페이지 관리
    - 화면 전환
    - 공통 데이터 관리
    """

    def __init__(self):
        super().__init__()

        # ==================================================
        # Window
        # ==================================================

        self.setWindowTitle(
            "Sand Art Robot HMI"
        )

        self.resize(1600, 900)

        self.setMinimumSize(
            1400,
            800,
        )

        # ==================================================
        # Stack Widget
        # ==================================================

        self.stack = QStackedWidget()

        self.setCentralWidget(
            self.stack
        )

        # ==================================================
        # Page 생성
        # ==================================================

        self.start_page = StartPage()

        self.image_page = ImagePage()

        self.gray_page = GrayPage()

        self.edge_page = EdgePage()

        self.sand_preview_page = SandPreviewPage()

        self.parameter_page = ParameterPage()

        self.drawing_page = DrawingPage()

        self.finish_page = FinishPage()

        # ==================================================
        # System Setting Dialog / Drawing Parameter
        # ==================================================

        self.draw_force = 0.8
        self.draw_speed = 60
        self.selected_tool = "MEDIUM"
        self.robot_speed = 110
        self.draw_sampling = 1.0

        self.setting_dialog = SettingDialog(self)
        self.setting_dialog.set_draw_parameter(
            tool=self.selected_tool,
            force=self.draw_force,
            speed=self.draw_speed,
        )

        # # Robot Control Client
        # self.robot_client = RobotControlClient(
        #     log_callback=self.setting_dialog.add_log
        # )
        # ==================================================
        # Process Image Client
        # ==================================================

        self.process_image_client = ProcessImageClientThread(
            log_callback=self.setting_dialog.add_log
        )
        self.process_image_result.connect(
            self.process_image_finished
        )
        self.sand_clear_done_result.connect(
            self.on_sand_clear_done
        )
        self.selected_image_path = ""
        self.processing = False
        self.process_image_client.node.finished_callback = (
            self.process_image_result.emit
        )
        # ==================================================
        # Robot Process 관리
        # ==================================================

        # ==================================================
        # Robot Process 관리
        # ==================================================

        # Robot Driver
        self.bringup_process = None

        # Sandart ROS2
        self.launch_process = None

        # STEP4까지 유지
        self.r2_process = None
        self.sand_clear_process = None

        self.robot_setup_path = "~/ws_cobot_pjt/ws_dsr/install/setup.bash"

        # Robot Driver
        self.robot_bringup_cmd = (
            "ros2 launch m0609_rg2_bringup bringup.launch.py "
            "mode:=real host:=192.168.1.100 model:=m0609"
        )

        # Sandart Launch
        self.sandart_launch_cmd = (
            "ros2 launch sandart sandart.launch.py"
        )

        # Sand Reset
        self.sand_clear_cmd = (
            'ros2 service call '
            '/dsr01/start_sand_clear '
            'std_srvs/srv/Trigger "{}"'
        )
                # Drawing 시작: sandart_movesx_node가 받은 path를 실제 실행한다.

        # ==================================================
        # Stack 등록
        # ==================================================

        self.stack.addWidget(
            self.start_page
        )

        self.stack.addWidget(
            self.image_page
        )

        self.stack.addWidget(
            self.gray_page
        )


        self.stack.addWidget(
            self.edge_page
        )

        self.stack.addWidget(
            self.sand_preview_page
        )


        self.stack.addWidget(
            self.parameter_page
        )

        self.stack.addWidget(
            self.drawing_page
        )

        self.stack.addWidget(
            self.finish_page
        )

        # ==================================================
        # Signal 연결
        # ==================================================

        self.connect_signal()

        self.connect_finish()

        self.connect_setting_buttons()

        # ==================================================
        # Style
        # ==================================================

        self.load_qss()

        # ==================================================
        # Start Page
        # ==================================================

        self.stack.setCurrentWidget(
            self.start_page
        )

        # ==================================================
        # Node Monitor Timer
        # ==================================================
        # ==========================================
        # Drawing Progress Subscriber
        # ==========================================

        try:
            rclpy.init(args=None)
        except RuntimeError:
            pass

        self.progress_node = DrawingProgressSubscriber(
            self.on_progress_received
        )

        # ProcessImageClient와 같은 전용 ROS Executor에서 처리합니다.
        # GUI 스레드의 rclpy.spin_once()를 제거하여 서비스 응답 누락을 방지합니다.
        self.process_image_client.add_node(
            self.progress_node
        )

        self.sand_clear_done_node = SandClearDoneSubscriber(
            self.sand_clear_done_result.emit
        )
        self.process_image_client.add_node(
            self.sand_clear_done_node
        )

        self.node_timer = QTimer(self)
        self.node_timer.timeout.connect(self.update_node_monitor)
        self.node_timer.start(2000)      # 2초마다 갱신

            # ======================================================
    # Signal 연결
    # ======================================================

    def connect_signal(self):
        """
        각 페이지의 이전 / 다음 버튼 연결
        """

        # ==================================================
        # Start Page
        # ==================================================

        self.start_page.image_select_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.image_page
            )
        )
        # ==================================================
        # Drawing Page
        # ==================================================


        self.drawing_page.stop_clicked.connect(
            self.on_setting_stop_requested
        )
        self.drawing_page.pause_clicked.connect(
            self.on_setting_pause_requested
        )
        self.start_page.sand_reset_clicked.connect(
            self.sand_reset
        )

        # ==================================================
        # Image Page
        # ==================================================

        self.image_page.prev_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.start_page
            )
        )
        self.image_page.image_selected.connect(
            self.on_image_selected
        )
        self.image_page.next_clicked.connect(
            self.goto_gray_page
        )

    

        # ==================================================
        # Gray Page
        # ==================================================

        self.gray_page.prev_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.image_page
            )
        )

        self.gray_page.next_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.edge_page
            )
        )

        # ==================================================
        # Edge Page
        # ==================================================

        self.edge_page.prev_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.gray_page
            )
        )

        self.edge_page.next_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.sand_preview_page
            )
        )

        # ==================================================
        # Sand Preview Page
        # ==================================================

        self.sand_preview_page.prev_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.edge_page
            )
        )

        self.sand_preview_page.next_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.parameter_page
            )
        )

        # ==================================================
        # Parameter Page
        # ==================================================

        self.parameter_page.prev_clicked.connect(
            lambda: self.stack.setCurrentWidget(
                self.sand_preview_page
            )
        )

        self.parameter_page.next_clicked.connect(
            self.start_drawing_from_hmi
        )

            # ======================================================
    # Drawing 완료
    # ======================================================

    def drawing_finished(self):
        """
        Drawing 완료 시 Finish 화면으로 이동

        추후 로봇 작업이 100% 끝나면
        이 함수만 호출하면 됩니다.
        """

        self.stack.setCurrentWidget(
            self.finish_page
        )

    # ======================================================
    # Finish Page
    # ======================================================

    def connect_finish(self):

        self.finish_page.home_clicked.connect(
            self.reset_all
        )

        self.finish_page.exit_clicked.connect(

            self.close

        )

    # ======================================================
    # Setting Dialog 연결
    # ======================================================
    def connect_setting_buttons(self):
        """모든 페이지 Header의 ⚙ QLabel을 하나의 SettingDialog에 연결."""

        pages = [
            self.start_page,
            self.image_page,
            self.gray_page,
            self.edge_page,
            self.sand_preview_page,
            self.parameter_page,
            self.drawing_page,
            self.finish_page,
        ]

        for page in pages:
            setting_widget = None

            if hasattr(page, "setting"):
                setting_widget = page.setting
            elif hasattr(page, "setting_label"):
                setting_widget = page.setting_label

            if setting_widget is None:
                continue

            setting_widget.setCursor(Qt.PointingHandCursor)
            setting_widget.mousePressEvent = self.open_setting_dialog

        # Robot Control
        self.setting_dialog.home_requested.connect(
            self.on_setting_home_requested
        )

        self.setting_dialog.emergency_requested.connect(
            self.on_setting_emergency_requested
        )

        self.setting_dialog.connect_requested.connect(
            self.start_robot_bringup
        )

        self.setting_dialog.disconnect_requested.connect(
            self.stop_robot_bringup
        )


    def open_setting_dialog(self, event=None):
        """⚙ 클릭 시 공통 SettingDialog 표시."""

        self.setting_dialog.set_draw_parameter(
            tool=self.selected_tool,
            force=self.draw_force,
            speed=self.draw_speed,
        )
        self.setting_dialog.show()
        self.setting_dialog.raise_()
        self.setting_dialog.activateWindow()

    def reset_all(self):
        """
        프로그램을 처음 실행한 상태처럼 초기화합니다.

        현재 1단계에서는
        - ImagePage 초기화
        - Drawing 진행률 초기화
        - 시작 화면 이동
        만 처리합니다.
        """

        self.selected_image_path = ""
        self.processing = False

        self.image_page.clear_preview()

        self.gray_page.clear_page()

        self.edge_page.clear_page()

        self.sand_preview_page.clear_page()

        self.drawing_page.set_progress(0)
        self.drawing_page.set_current_status("Ready")
        self.drawing_page.set_status("READY")

        self.stack.setCurrentWidget(
            self.start_page
        )
        self.setting_dialog.set_robot_status(
            connected=True,
            servo=True,
            force=False,
            compliance=False,
            drawing=False,
        )
        self.pause_state = False

        self.drawing_page.pause_button.setText(
            "⏸ Pause"
        )

        
    def on_setting_home_requested(self):
        """HOME 요청 처리."""

        self.setting_dialog.add_log(
            "[MAIN] HOME requested"
        )

        self.reset_all()
    def on_setting_pause_requested(self):
        """Drawing Pause / Resume 서비스 전환."""

        if not hasattr(self, "pause_state"):
            self.pause_state = False

        if not self.pause_state:
            cmd = (
                "ros2 service call "
                "/dsr01/pause_drawing "
                "std_srvs/srv/Trigger "
                "\"{}\""
            )

            self.pause_state = True

            self.setting_dialog.add_log(
                "[DRAW] Pause requested"
            )

            self.drawing_page.pause_button.setText(
                "▶ Resume"
            )

        else:
            cmd = (
                "ros2 service call "
                "/dsr01/resume_drawing "
                "std_srvs/srv/Trigger "
                "\"{}\""
            )

            self.pause_state = False

            self.setting_dialog.add_log(
                "[DRAW] Resume requested"
            )

            self.drawing_page.pause_button.setText(
                "⏸ Pause"
            )

        subprocess.Popen(
            [
                "bash",
                "-lc",
                f"source {self.robot_setup_path}; {cmd}",
            ]
        )
    def send_qstop(self):
        pass
    def on_setting_stop_requested(self):
        """STOP 버튼"""

        reply = QMessageBox.question(
            self,
            "Stop Drawing",
            "작업을 중단하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply != QMessageBox.Yes:
            return

        self.setting_dialog.add_log(
            "[MAIN] STOP requested"
        )

        # start_drawing service call 종료
        self.stop_robot_drawing()

        # sandart.launch 종료
        self.stop_sandart_launch()

        # sandart.launch 다시 실행
        self.start_sandart_launch()

        # 화면 초기화
        self.reset_all()
    def restart_after_stop(self):

        self.start_sandart_launch()

        self.reset_all()
    def on_setting_emergency_requested(self):
        """
        Emergency Stop

        1. Robot Emergency 명령
        2. Drawing 종료
        3. Bringup 종료(Ctrl+C)
        4. 상태 OFF
        """

        self.setting_dialog.add_log(
            "[EMERGENCY] Emergency Stop Requested"
        )

        # ROS Emergency Service
        if hasattr(self, "robot_client"):
            try:
                self.robot_client.send_command("EMERGENCY")
                self.setting_dialog.add_log(
                    "[EMERGENCY] Robot emergency command sent"
                )
            except Exception as e:
                self.setting_dialog.add_log(
                    f"[EMERGENCY] Service error : {e}"
                )

        # Drawing 종료
        self.stop_robot_drawing()

        # Sand Reset 종료
        self.stop_sand_clear()

        # Bringup 종료 (Ctrl+C 포함)
        self.stop_robot_bringup()

        self.setting_dialog.set_robot_status(
            connected=False,
            servo=False,
            force=False,
            compliance=False,
            drawing=False,
        )

        self.setting_dialog.set_robot_connection_state("OFF")

        self.drawing_page.set_status("EMERGENCY")

        self.drawing_page.set_current_status(
            "Emergency Stop"
        )

        self.setting_dialog.add_log(
            "[EMERGENCY] Robot stopped"
        )
    # ======================================================
    # Robot Process Control
    # ======================================================

    def _make_bash_command(self, command):
        """
        ROS2 명령을 bash 안에서 실행한다.

        핵심:
        - source setup.bash
        - exec 사용
        - QProcess가 bash를 실행하지만 실제 PID는 ros2 명령으로 교체됨
        """

        return f"source {self.robot_setup_path}; exec {command}"

    def _connect_process_log(self, process, name):
        """QProcess stdout/stderr를 SettingDialog 로그창으로 연결."""

        process.readyReadStandardOutput.connect(
            lambda: self._read_process_output(process, name)
        )
        process.readyReadStandardError.connect(
            lambda: self._read_process_error(process, name)
        )

    def _set_all_node_states(self, state):
        """Node Monitor 전체 ON/OFF 갱신."""

        self.setting_dialog.set_node_state("bringup", state)
        self.setting_dialog.set_node_state("controller", state)
        self.setting_dialog.set_node_state("planner", state)
        self.setting_dialog.set_node_state("skeleton", state)
        self.setting_dialog.set_node_state("movesx", state)
        self.setting_dialog.set_node_state("lifecycle", state)

    def update_node_monitor(self):
        """ros2 node list 기준으로 Node Monitor 상태를 갱신."""

        try:
            result = subprocess.run(
                [
                    "bash",
                    "-lc",
                    f"source {self.robot_setup_path} && ros2 node list",
                ],
                capture_output=True,
                text=True,
                timeout=2,
            )

            nodes = result.stdout.lower()


            self.setting_dialog.set_node_state(
                "controller",
                "/dsr01/dsr_controller2" in nodes,
            )

            self.setting_dialog.set_node_state(
                "planner",
                (
                    "/path_plan_node" in nodes
                    or "/dsr01/path_plan_node" in nodes
                ),
            )

            self.setting_dialog.set_node_state(
                "skeleton",
                (
                    "/skeleton_processor_node" in nodes
                    or "/dsr01/skeleton_processor_node" in nodes
                ),
            )

            self.setting_dialog.set_node_state(
                "movesx",
                (
                    "/dsr01/sandart_movesx_node" in nodes
                    or "/sandart_movesx_node" in nodes
                    or "/dsr01/sand_engine_final_v2" in nodes
                    or "/sand_engine_final_v2" in nodes
                ),
            )

            self.setting_dialog.set_node_state(
                "lifecycle",
                (
                    "/lifecycle_manage_node" in nodes
                    or "/dsr01/lifecycle_manage_node" in nodes
                ),
            )

        except Exception as e:
            self.setting_dialog.add_log(
                f"[NODE] Monitor Error : {e}"
            )
    def _update_node_state_from_log(self, line):
        """로그 내용을 기준으로 Node Monitor 상태를 갱신한다."""

        lower = line.lower()

        # Bringup 성공
        if "connected rt control stream" in lower:
            self.setting_dialog.set_node_state("bringup", True)

        # 연결 실패
        elif (
            "connecting failure" in lower
            or "connecting error" in lower
            or "connect timed out" in lower
            or "retry..." in lower
        ):
            self.setting_dialog.set_node_state("bringup", False)

        if "path_planner_to_lifecycle" in lower:
            self.setting_dialog.set_node_state("planner", True)

        if "skeleton_processor_node_to_lifecycle" in lower:
            self.setting_dialog.set_node_state("skeleton", True)

        if (
            "sandart_movesx_node_to_lifecycle" in lower
            or "movesx_node_to_lifecycle" in lower
            or "robot execute" in lower
            or "path 1 start" in lower
            or "move to start safe z" in lower
        ):
            self.setting_dialog.set_node_state("movesx", True)

        if "listening on /bond" in lower:
            self.setting_dialog.set_node_state("lifecycle", True)
    def _route_process_log(
        self,
        name,
        text,
        line,
        is_error=False,
    ):
        """프로세스 종류별 로그 분리"""

        lower = line.lower()

        # Error 탭
        if is_error:
            self.setting_dialog.add_error_log(text)

        # Bringup
        if name == "BRINGUP":
            self.setting_dialog.add_bringup_log(text)

        # Sandart
        elif name == "SANDART":
            self.setting_dialog.add_sandart_log(text)

        # Drawing
        elif name == "DRAW":
            self.setting_dialog.add_draw_log(text)


                # Sand Reset
        elif name == "SAND RESET":
            self.setting_dialog.add_draw_log(text)

        # Lifecycle
        if (
            "heartbeat" in lower
            or "lifecycle" in lower
            or "/bond" in lower
        ):
            self.setting_dialog.add_lifecycle_log(text)
    def _read_process_output(self, process, name):
        """프로세스 stdout 로그 수신."""

        data = bytes(
            process.readAllStandardOutput()
        ).decode(
            "utf-8",
            errors="ignore",
        )

        for line in data.splitlines():
            if not line.strip():
                continue

            text = f"[{name}] {line}"

            self._update_node_state_from_log(line)
            self._route_process_log(
                name,
                text,
                line,
                False,
            )

    def _read_process_error(self, process, name):
        """프로세스 stderr 로그 수신."""

        data = bytes(
            process.readAllStandardError()
        ).decode(
            "utf-8",
            errors="ignore",
        )

        for line in data.splitlines():
            if not line.strip():
                continue

            text = f"[{name} ERR] {line}"

            self._update_node_state_from_log(line)
            self._route_process_log(
                name,
                text,
                line,
                True,
            )

    def start_robot_bringup(self):
        """
        CONNECT

        순서
        1. Bringup 중복 실행 방지
        2. Bringup 실행
        3. Bringup Started -> sandart.launch 실행
        """

        if (
            self.bringup_process is not None
            and self.bringup_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.add_log(
                "[CONNECT] Bringup already running"
            )
            return

        if (
            self.launch_process is not None
            and self.launch_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.add_log(
                "[CONNECT] sandart.launch already running"
            )
            return

        self.setting_dialog.add_log(
            "[CONNECT] Robot Bringup Start"
        )

        self.setting_dialog.set_robot_connection_state(
            "CONNECTING"
        )

        self.setting_dialog.set_robot_status(
            connected=False,
            servo=False,
            force=False,
            compliance=False,
            drawing=False,
        )

        self._set_all_node_states(False)

        self.bringup_process = QProcess(self)

        self._connect_process_log(
            self.bringup_process,
            "BRINGUP",
        )

        self.bringup_process.started.connect(
            self.on_bringup_started
        )

        self.bringup_process.finished.connect(
            self.on_bringup_finished
        )

        self.bringup_process.errorOccurred.connect(
            self.on_bringup_error
        )

        self.bringup_process.start(
            "bash",
            [
                "-lc",
                self._make_bash_command(
                    self.robot_bringup_cmd
                ),
            ],
        )

    def on_bringup_started(self):
        """
        Bringup 실행 시작.

        Robot Driver가 시작되면 Sandart Launch를 실행한다.
        """

        self.setting_dialog.add_log(
            "[CONNECT] Robot Bringup Started"
        )

        self.setting_dialog.set_robot_status(
            connected=True,
            servo=True,
            drawing=False,
        )

        self.setting_dialog.set_robot_connection_state(
            "CONNECTING"
        )

        self.start_sandart_launch()

    def start_sandart_launch(self):
        """
        Bringup 시작 후 Sandart Launch 실행.
        """

        if (
            self.launch_process is not None
            and self.launch_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.add_log(
                "[CONNECT] sandart.launch already running"
            )
            return

        self.setting_dialog.add_log(
            "[CONNECT] Sandart Launch Start"
        )

        self.launch_process = QProcess(self)

        self._connect_process_log(
            self.launch_process,
            "SANDART",
        )

        self.launch_process.started.connect(
            lambda: self.setting_dialog.add_log(
                "[CONNECT] sandart.launch started"
            )
        )

        self.launch_process.finished.connect(
            lambda code, status: self.setting_dialog.add_log(
                f"[DISCONNECT] sandart.launch finished : code={code}"
            )
        )

        self.launch_process.errorOccurred.connect(
            lambda err: self.setting_dialog.add_log(
                f"[ERROR] sandart.launch : {err}"
            )
        )

        self.launch_process.start(
            "bash",
            [
                "-lc",
                self._make_bash_command(
                    self.sandart_launch_cmd
                ),
            ],
        )

        self.setting_dialog.set_robot_connection_state(
            "CONNECTED"
        )

    def stop_sandart_launch(self):
        """
        Sandart Launch 종료.
        """

        if self.launch_process is None:
            return

        if self.launch_process.state() == QProcess.NotRunning:
            self.launch_process = None
            return

        self.setting_dialog.add_log(
            "[DISCONNECT] Stop sandart.launch"
        )

        try:
            pid = int(self.launch_process.processId())

            if pid > 0:
                try:
                    os.kill(pid, signal.SIGINT)

                    self.setting_dialog.add_log(
                        f"[DISCONNECT] SIGINT -> sandart.launch ({pid})"
                    )

                except ProcessLookupError:
                    pass

            if not self.launch_process.waitForFinished(3000):
                self.launch_process.terminate()

                if not self.launch_process.waitForFinished(2000):
                    self.launch_process.kill()
                    self.launch_process.waitForFinished(1000)

        except Exception as e:
            self.setting_dialog.add_log(
                f"[DISCONNECT] sandart.launch error : {e}"
            )

        self.launch_process = None

    def on_bringup_finished(self, exit_code, exit_status):
        """bringup 종료."""

        self.setting_dialog.add_log(
            f"[DISCONNECT] bringup finished: code={exit_code}, status={exit_status}"
        )

        self.setting_dialog.set_robot_connection_state("OFF")

        self.setting_dialog.set_robot_status(
            connected=False,
            servo=False,
            force=False,
            compliance=False,
            drawing=False,
        )

        self._set_all_node_states(False)

        self.bringup_process = None

    def on_bringup_error(self, error):
        """bringup 실행 오류."""

        self.setting_dialog.add_log(
            f"[ERROR] bringup process error: {error}"
        )

        self.setting_dialog.set_robot_connection_state("ERROR")

    def stop_robot_bringup(self):
        """
        DISCONNECT

        순서
        1. sandart.launch 종료
        2. Drawing 프로세스 종료
        3. bringup 프로세스에 Ctrl+C(SIGINT) 전송
        4. 종료 실패 시 terminate / kill
        5. GUI 상태 OFF 갱신
        """

        self.stop_sandart_launch()

        self.stop_robot_drawing()

        self.stop_sand_clear()

        if self.bringup_process is None:
            self.setting_dialog.add_log(
                "[DISCONNECT] bringup is not running"
            )

            self.setting_dialog.set_robot_connection_state("OFF")

            self.setting_dialog.set_robot_status(
                connected=False,
                servo=False,
                force=False,
                compliance=False,
                drawing=False,
            )

            self._set_all_node_states(False)

            self.launch_process = None
            return

        if self.bringup_process.state() == QProcess.NotRunning:
            self.setting_dialog.add_log(
                "[DISCONNECT] bringup already stopped"
            )

            self.setting_dialog.set_robot_connection_state("OFF")

            self.setting_dialog.set_robot_status(
                connected=False,
                servo=False,
                force=False,
                compliance=False,
                drawing=False,
            )

            self._set_all_node_states(False)

            self.bringup_process = None
            self.launch_process = None
            return

        self.setting_dialog.add_log(
            "[DISCONNECT] Send Ctrl+C to bringup"
        )

        try:
            pid = int(self.bringup_process.processId())

            if pid > 0:
                try:
                    os.kill(pid, signal.SIGINT)

                    self.setting_dialog.add_log(
                        f"[DISCONNECT] SIGINT sent to pid={pid}"
                    )

                except ProcessLookupError:
                    self.setting_dialog.add_log(
                        "[DISCONNECT] process already gone"
                    )

            if self.bringup_process.waitForFinished(5000):
                self.setting_dialog.add_log(
                    "[DISCONNECT] bringup closed normally"
                )
            else:
                self.setting_dialog.add_log(
                    "[DISCONNECT] Ctrl+C timeout -> terminate"
                )

                self.bringup_process.terminate()

                if self.bringup_process.waitForFinished(3000):
                    self.setting_dialog.add_log(
                        "[DISCONNECT] bringup terminated"
                    )
                else:
                    self.setting_dialog.add_log(
                        "[DISCONNECT] terminate timeout -> kill"
                    )

                    self.bringup_process.kill()
                    self.bringup_process.waitForFinished(1000)

        except Exception as e:
            self.setting_dialog.add_log(
                f"[DISCONNECT] error: {e}"
            )

        self.setting_dialog.set_robot_connection_state("OFF")

        self.setting_dialog.set_robot_status(
            connected=False,
            servo=False,
            force=False,
            compliance=False,
            drawing=False,
        )

        self._set_all_node_states(False)

        self.bringup_process = None
        self.launch_process = None
    def start_drawing_from_hmi(self):
        self.pause_state = False
        self.drawing_page.pause_button.setText("⏸ Pause")
        self.setting_dialog.add_log(
            "[DRAW] Start Drawing"
        )

        param = self.parameter_page.get_parameter()

        self.draw_speed = float(param["draw_speed"])

        self.robot_speed = float(param["robot_speed"])

        self.draw_force = float(param["force"])

        self.draw_sampling = float(param["sampling"])

        self.setting_dialog.set_draw_parameter(
            tool=self.selected_tool,
            force=self.draw_force,
            speed=self.draw_speed,
            robot_speed=self.robot_speed,
            sampling=self.draw_sampling,
        )

        self.stack.setCurrentWidget(
            self.drawing_page
        )

        self.drawing_page.set_current_status(
            "Starting Robot..."
        )

        self.start_robot_drawing()
    def start_robot_drawing(self):
        if (
            self.r2_process is not None
            and self.r2_process.state() == QProcess.NotRunning
        ):
            self.r2_process = None
        if (
            self.r2_process is not None
            and self.r2_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.add_log(
                "[DRAW] Drawing command already running"
            )
            return

        self.setting_dialog.add_log(
            "[DRAW] Call /dsr01/start_drawing"
        )

        self.setting_dialog.set_robot_connection_state(
            "DRAWING"
        )

        self.setting_dialog.set_robot_status(
            connected=True,
            servo=True,
            force=True,
            compliance=True,
            drawing=True,
        )

        self.drawing_page.set_status("DRAWING")
        self.drawing_page.set_current_status("Waiting for robot start...")

        self.r2_process = QProcess(self)

        self._connect_process_log(
            self.r2_process,
            "DRAW",
        )

        self.r2_process.started.connect(
            self.on_r2_started
        )

        self.r2_process.finished.connect(
            self.on_r2_finished
        )

        self.r2_process.errorOccurred.connect(
            self.on_r2_error
        )
        # ==================================================
        # StartDrawing Service Command 생성
        # ==================================================

        self.robot_draw_cmd = (
            "ros2 service call "
            "/dsr01/start_drawing "
            "sandart_msgs/srv/StartDrawing "
            f"'{{"
            f"draw_speed: {self.draw_speed}, "
            f"robot_speed: {self.robot_speed}, "
            f"force: {self.draw_force}, "
            f"sampling: {self.draw_sampling}, "
            f"tool: \"{self.selected_tool}\""
            f"}}'"
        )
        self.r2_process.start(
            "bash",
            [
                "-lc",
                self._make_bash_command(
                    self.robot_draw_cmd
                ),
            ],
        )
    def on_r2_started(self):
        """r2 실행 시작."""

        self.setting_dialog.add_log("[DRAW] r2 process started")
        self.drawing_page.set_status("DRAWING")
        self.drawing_page.set_current_status("Drawing")

    def on_r2_finished(self, exit_code, exit_status):
        """r2 종료."""

        self.setting_dialog.add_log(
            f"[DRAW] r2 finished: code={exit_code}, status={exit_status}"
        )
        self.setting_dialog.set_robot_status(
            force=False,
            compliance=False,
            drawing=False,
        )
        if self.bringup_process is not None and self.bringup_process.state() != QProcess.NotRunning:
            self.setting_dialog.set_robot_connection_state("CONNECTED")
        else:
            self.setting_dialog.set_robot_connection_state("OFF")
        self.drawing_page.set_status("READY")
        self.drawing_page.set_current_status("Ready")
        self.r2_process = None

    def on_r2_error(self, error):
        """r2 실행 오류."""

        self.setting_dialog.add_log(f"[ERROR] r2 process error: {error}")
        self.drawing_page.set_status("ERROR")
    def stop_robot_drawing(self):
        """r2 그림 실행 프로세스를 종료한다."""

        if self.r2_process is None:
            self.setting_dialog.add_log("[DRAW STOP] r2 is not running")
            return

        if self.r2_process.state() == QProcess.NotRunning:
            self.setting_dialog.add_log("[DRAW STOP] r2 already stopped")
            self.r2_process = None
            return

        self.setting_dialog.add_log("[DRAW STOP] Send Ctrl+C to r2")

        try:
            pid = int(self.r2_process.processId())

            if pid > 0:
                try:
                    os.kill(pid, signal.SIGINT)
                    self.setting_dialog.add_log(
                        f"[DRAW STOP] SIGINT sent to pid={pid}"
                    )
                except ProcessLookupError:
                    self.setting_dialog.add_log(
                        "[DRAW STOP] process already gone"
                    )

            if self.r2_process.waitForFinished(3000):
                self.setting_dialog.add_log("[DRAW STOP] r2 closed normally")
            else:
                self.setting_dialog.add_log(
                    "[DRAW STOP] Ctrl+C timeout -> terminate"
                )

                self.r2_process.terminate()

                if not self.r2_process.waitForFinished(2000):
                    self.setting_dialog.add_log(
                        "[DRAW STOP] terminate timeout -> kill"
                    )
                    self.r2_process.kill()
                    self.r2_process.waitForFinished(1000)

        except Exception as e:
            self.setting_dialog.add_log(
                f"[DRAW STOP] error: {e}"
            )

        self.r2_process = None 
        self.pause_state = False

        self.drawing_page.pause_button.setText(
            "⏸ Pause"
        )
        self.drawing_page.set_status("READY")
        self.drawing_page.set_current_status("Ready")

        self.setting_dialog.set_robot_connection_state(
            "CONNECTED"
        )

        self.setting_dialog.set_robot_status(
            force=False,
            compliance=False,
            drawing=False,
        )

    def start_sand_clear(self):
        """StartPage의 샌드 초기화 버튼으로 sand_clear_node를 실행한다."""

        # --------------------------------------------------
        # 이미 샌드 초기화가 실행 중이면 중복 실행 방지
        # --------------------------------------------------
        if (
            self.sand_clear_process is not None
            and self.sand_clear_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.add_log(
                "[SAND RESET] Already running"
            )
            return

        # 이전 QProcess 객체가 종료된 상태로 남아 있으면 정리
        if (
            self.sand_clear_process is not None
            and self.sand_clear_process.state() == QProcess.NotRunning
        ):
            self.sand_clear_process = None

        # --------------------------------------------------
        # UI 상태 변경
        # --------------------------------------------------

        # 실행 중 버튼 연타 방지
        self.start_page.enable_reset_button(False)

        # 이미지 선택도 함께 막고 싶으면 활성화
        self.start_page.enable_image_button(False)

        # StartPage 상단 STATUS 표시
        self.start_page.set_status("RUNNING")

        # SettingDialog 연결 상태 표시
        self.setting_dialog.set_robot_connection_state(
            "SAND RESET"
        )

        # 로봇 상태 표시
        self.setting_dialog.set_robot_status(
            connected=True,
            servo=True,
            force=False,
            compliance=False,
            drawing=False,
        )

        self.setting_dialog.add_log(
            "[SAND RESET] sand_clear_node launch requested"
        )

        # --------------------------------------------------
        # 샌드 초기화 QProcess 생성
        # --------------------------------------------------
        self.sand_clear_process = QProcess(self)

        # stdout / stderr를 설정창 로그와 연결
        self._connect_process_log(
            self.sand_clear_process,
            "SAND RESET",
        )

        # 프로세스 실제 시작 이벤트
        self.sand_clear_process.started.connect(
            self.on_sand_clear_started
        )

        # 프로세스 종료 이벤트
        self.sand_clear_process.finished.connect(
            self.on_sand_clear_finished
        )

        # 실행 자체에 실패한 경우
        self.sand_clear_process.errorOccurred.connect(
            self.on_sand_clear_error
        )

        # --------------------------------------------------
        # 실제 명령 실행
        # source install/setup.bash
        # ros2 run sandart sand_clear_node
        # --------------------------------------------------
        self.sand_clear_process.start(
            "bash",
            [
                "-lc",
                self._make_bash_command(
                    self.sand_clear_cmd
                ),
            ],
        )

    def on_sand_clear_started(self):
        """sand_clear_node 프로세스가 실제로 시작된 경우."""

        self.setting_dialog.add_log(
            "[SAND RESET] sand_clear_node started"
        )

        self.start_page.set_status(
            "RUNNING"
        )
    def on_sand_clear_error(self, error):
        """sand_clear_node를 시작하지 못했거나 QProcess 오류가 발생한 경우."""

        self.setting_dialog.add_log(
            f"[SAND RESET] QProcess Error : {error}"
        )

        # 버튼 복구
        self.start_page.enable_reset_button(True)
        self.start_page.enable_image_button(True)

        # 화면 오류 표시
        self.start_page.set_status("ERROR")

        self.setting_dialog.set_robot_connection_state(
            "ERROR"
        )

        self.setting_dialog.set_robot_status(
            force=False,
            compliance=False,
            drawing=False,
        )

        self.sand_clear_process = None
    
    def on_sand_clear_finished(
        self,
        exit_code,
        exit_status,
    ):
        """서비스 호출 프로세스 종료 처리. 실제 완료는 done 토픽으로 판단합니다."""

        self.setting_dialog.add_log(
            f"[SAND RESET] Service call finished : "
            f"code={exit_code}, status={exit_status}"
        )

        self.sand_clear_process = None

        # 서비스 호출 자체가 실패한 경우에만 UI를 즉시 복구합니다.
        if exit_code != 0:
            self.start_page.enable_reset_button(True)
            self.start_page.enable_image_button(True)
            self.start_page.set_status("ERROR")

            self.setting_dialog.set_robot_connection_state(
                "ERROR"
            )
            self.setting_dialog.add_log(
                "[SAND RESET] Service call failed"
            )

    def on_sand_clear_done(self, success):
        """sand_clear_node가 실제 모션 완료 후 발행한 결과 처리."""

        self.start_page.enable_reset_button(True)
        self.start_page.enable_image_button(True)

        if success:
            self.start_page.set_status("READY")
            self.setting_dialog.add_log(
                "[SAND RESET] Sand reset completed"
            )
        else:
            self.start_page.set_status("ERROR")
            self.setting_dialog.add_log(
                "[SAND RESET] Sand reset failed"
            )

        if (
            self.bringup_process is not None
            and self.bringup_process.state() != QProcess.NotRunning
        ):
            self.setting_dialog.set_robot_connection_state(
                "CONNECTED"
            )
            self.setting_dialog.set_robot_status(
                connected=True,
                servo=True,
                force=False,
                compliance=False,
                drawing=False,
            )
        else:
            self.setting_dialog.set_robot_connection_state(
                "OFF"
            )
            self.setting_dialog.set_robot_status(
                connected=False,
                servo=False,
                force=False,
                compliance=False,
                drawing=False,
            )

    def stop_sand_clear(self):
        """실행 중인 sand_clear_node 프로세스를 종료한다."""

        if self.sand_clear_process is None:
            return

        if self.sand_clear_process.state() == QProcess.NotRunning:
            self.sand_clear_process = None

            self.start_page.enable_reset_button(True)
            self.start_page.enable_image_button(True)
            return

        self.setting_dialog.add_log(
            "[SAND RESET] Stop requested"
        )

        try:
            pid = int(
                self.sand_clear_process.processId()
            )

            if pid > 0:
                try:
                    # Ctrl+C와 같은 SIGINT 전달
                    os.kill(
                        pid,
                        signal.SIGINT,
                    )

                    self.setting_dialog.add_log(
                        f"[SAND RESET] SIGINT sent to pid={pid}"
                    )

                except ProcessLookupError:
                    self.setting_dialog.add_log(
                        "[SAND RESET] Process already stopped"
                    )

            # 정상 종료 대기
            if not self.sand_clear_process.waitForFinished(3000):
                self.setting_dialog.add_log(
                    "[SAND RESET] SIGINT timeout -> terminate"
                )

                self.sand_clear_process.terminate()

                if not self.sand_clear_process.waitForFinished(2000):
                    self.setting_dialog.add_log(
                        "[SAND RESET] terminate timeout -> kill"
                    )

                    self.sand_clear_process.kill()
                    self.sand_clear_process.waitForFinished(1000)

        except Exception as e:
            self.setting_dialog.add_log(
                f"[SAND RESET] Stop error : {e}"
            )

        self.sand_clear_process = None

        self.start_page.enable_reset_button(True)
        self.start_page.enable_image_button(True)
        self.start_page.set_status("READY")
    # ======================================================
    # QSS 적용
    # ======================================================

    def load_qss(self):
        """
        styles/dark_theme.qss 적용
        """

        qss_file = Path(
            "styles/dark_theme.qss"
        )

        if qss_file.exists():

            with open(
                qss_file,
                "r",
                encoding="utf-8",
            ) as file:

                self.setStyleSheet(
                    file.read()
                )

    # ======================================================
    # Sand Reset
    # ======================================================

    def sand_reset(self):

        reply = QMessageBox.question(
            self,
            "Sand Reset",
            "모래를 초기화하시겠습니까?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply != QMessageBox.Yes:
            return

        self.setting_dialog.add_log(
            "[SAND RESET] Start"
        )

        self.start_sand_clear()

    # ======================================================
    # Page 이동 함수
    # ======================================================

    def goto_start(self):

        self.stack.setCurrentWidget(
            self.start_page
        )

    def goto_image(self):

        self.stack.setCurrentWidget(
            self.image_page
        )
    def goto_gray_page(self):

        self.stack.setCurrentWidget(self.gray_page)

        if self.processing:
            return

        self.processing = True

        self.setting_dialog.add_log(
            "[PROCESS] Start Image Processing..."
        )

        QTimer.singleShot(
            50,
            lambda: self.process_image_client.process(
                self.selected_image_path
            )
        )
        
    def goto_gray(self):

        self.stack.setCurrentWidget(
            self.gray_page
        )



    def goto_edge(self):

        self.stack.setCurrentWidget(
            self.edge_page
        )

    def goto_preview(self):

        self.stack.setCurrentWidget(
            self.sand_preview_page
        )

    def goto_parameter(self):

        self.stack.setCurrentWidget(
            self.parameter_page
        )

    def goto_drawing(self):

        self.stack.setCurrentWidget(
            self.drawing_page
        )

    def goto_finish(self):

        self.stack.setCurrentWidget(
            self.finish_page
        )

            # ======================================================
    # 프로그램 종료
    # ======================================================
    def on_progress_received(self, msg):

        self.drawing_page.set_progress(
            msg.percent
        )

        self.drawing_page.set_current_status(
            msg.status
        )

        if msg.percent >= 100:

            self.drawing_finished()
    def closeEvent(self, event):
        """
        프로그램 종료

        추후

        - Robot Disconnect
        - Camera Release
        - Thread Stop

        등을 여기에 추가 예정
        """

        try:
            if self.r2_process is not None:
                self.stop_robot_drawing()
        except Exception:
            pass
        try:
            if self.sand_clear_process is not None:
                self.stop_sand_clear()
        except Exception:
            pass
        try:
            if self.bringup_process is not None or self.launch_process is not None:
                self.stop_robot_bringup()
        except Exception:
            pass

        try:
            if hasattr(self, "robot_client"):
                self.robot_client.shutdown()
        except Exception:
            pass

        try:
            if hasattr(self, "process_image_client"):
                self.process_image_client.shutdown()
        except Exception:
            pass
        try:
            if hasattr(self, "process_image_client") and hasattr(self, "progress_node"):
                self.process_image_client.remove_node(self.progress_node)
            self.progress_node.destroy_node()
        except Exception:
            pass

        try:
            if hasattr(self, "process_image_client") and hasattr(self, "sand_clear_done_node"):
                self.process_image_client.remove_node(self.sand_clear_done_node)
            self.sand_clear_done_node.destroy_node()
        except Exception:
            pass
        
        event.accept()

    # ======================================================
    # 이미지 전달
    # ======================================================

    def set_original_image(self, pixmap):
        """
        Image Page에서 선택한 이미지를
        이후 모든 페이지로 전달
        """

        self.gray_page.set_original_image(pixmap)

    # ======================================================
    # Gray 결과 전달
    # ======================================================
    def set_gray_image(self, pixmap):
        """
        Gray 결과를 Gray Page와 Edge Page에 전달
        """

        self.gray_page.set_gray_image(pixmap)

        self.edge_page.set_previous_image(pixmap)


    # ======================================================
    # Edge 결과 전달
    # ======================================================
    def set_edge_image(self, pixmap):
        """
        Edge 결과를 Edge Page와 Sand Preview Page에 전달
        """

        self.edge_page.set_edge_image(pixmap)

        self.sand_preview_page.set_previous_image(pixmap)
    # ======================================================
    # Sand Preview 전달
    # ======================================================

    def set_preview_image(self, pixmap):

        self.sand_preview_page.set_preview_image(pixmap)

    # ======================================================
    # Tool 전달
    # ======================================================

    def get_selected_tool(self):

        return self.tool_page.get_selected_tool()

    # ======================================================
    # Parameter 전달
    # ======================================================

    def get_parameter(self):

        return self.parameter_page.get_parameter()
    
    def on_image_selected(self, image_path):

        self.setting_dialog.add_log(
            f"[PROCESS] Selected : {image_path}"
        )

        self.selected_image_path = image_path

        pixmap = QPixmap(image_path)

        self.set_original_image(pixmap)
    def ros_image_to_qpixmap(self, msg):
        """sensor_msgs/Image를 HMI 표시용 QPixmap으로 안전하게 변환한다."""

        try:
            width = int(msg.width)
            height = int(msg.height)
            step = int(msg.step)
            encoding = str(msg.encoding).lower()
            data = bytes(msg.data)

            if width <= 0 or height <= 0 or step <= 0:
                self.setting_dialog.add_log(
                    f"[PROCESS] Invalid image size: {width}x{height}, step={step}"
                )
                return QPixmap()

            if encoding in ("mono8", "8uc1"):
                expected_min = step * height
                if len(data) < expected_min:
                    self.setting_dialog.add_log(
                        f"[PROCESS] Gray image data too small: len={len(data)}, expected={expected_min}"
                    )
                    return QPixmap()

                image = QImage(
                    data,
                    width,
                    height,
                    step,
                    QImage.Format_Grayscale8,
                ).copy()
                return QPixmap.fromImage(image)

            if encoding in ("rgb8", "bgr8"):
                expected_min = step * height
                if len(data) < expected_min:
                    self.setting_dialog.add_log(
                        f"[PROCESS] Color image data too small: len={len(data)}, expected={expected_min}"
                    )
                    return QPixmap()

                if encoding == "rgb8":
                    image = QImage(
                        data,
                        width,
                        height,
                        step,
                        QImage.Format_RGB888,
                    ).copy()
                    return QPixmap.fromImage(image)

                # bgr8은 Qt 버전마다 Format_BGR888 호환 문제가 있을 수 있어 RGB로 직접 변환한다.
                import numpy as _np
                import cv2 as _cv2

                arr = _np.frombuffer(data, dtype=_np.uint8).reshape((height, step))[:, : width * 3]
                bgr = arr.reshape((height, width, 3)).copy()
                rgb = _cv2.cvtColor(bgr, _cv2.COLOR_BGR2RGB)
                rgb_bytes = rgb.tobytes()
                image = QImage(
                    rgb_bytes,
                    width,
                    height,
                    width * 3,
                    QImage.Format_RGB888,
                ).copy()
                return QPixmap.fromImage(image)

            self.setting_dialog.add_log(
                f"[PROCESS] Unsupported encoding: {msg.encoding}"
            )
            return QPixmap()

        except Exception as e:
            self.setting_dialog.add_log(
                f"[PROCESS] QPixmap convert error: {e}"
            )
            traceback.print_exc()
            return QPixmap()

    def process_image_finished(self, response):
        """ProcessImage 응답을 받아 각 Preview에 반영한다.

        안정화 원칙:
        - Qt 위젯 갱신은 여기서만 한다.
        - QTimer.singleShot으로 다시 던지지 않는다.
        - 변환 실패 시 빈 pixmap을 넘기지 않고 로그만 남긴다.
        """

        print("========== PROCESS FINISHED ==========" , flush=True)

        try:
            self.setting_dialog.add_log(
                "[PROCESS] Response images received."
            )

            gray_pixmap = self.ros_image_to_qpixmap(response.gray_img)
            binary_pixmap = self.ros_image_to_qpixmap(response.binary_img)
            skeleton_pixmap = self.ros_image_to_qpixmap(response.skeleton_img)

            print("GRAY isNull:", gray_pixmap.isNull(), flush=True)
            print("EDGE isNull:", binary_pixmap.isNull(), flush=True)
            print("SAND isNull:", skeleton_pixmap.isNull(), flush=True)

            if not gray_pixmap.isNull():
                self.set_gray_image(gray_pixmap)
            else:
                self.setting_dialog.add_log("[PROCESS] Gray pixmap is empty.")

            if not binary_pixmap.isNull():
                self.set_edge_image(binary_pixmap)
            else:
                self.setting_dialog.add_log("[PROCESS] Edge pixmap is empty.")

            if not skeleton_pixmap.isNull():
                self.set_preview_image(skeleton_pixmap)
            else:
                self.setting_dialog.add_log("[PROCESS] Sand preview pixmap is empty.")

            self.setting_dialog.add_log(
                "[PROCESS] Response images loaded to HMI."
            )

        except Exception as e:
            self.setting_dialog.add_log(
                f"[PROCESS] process_image_finished error: {e}"
            )
            traceback.print_exc()

        finally:
            self.processing = False