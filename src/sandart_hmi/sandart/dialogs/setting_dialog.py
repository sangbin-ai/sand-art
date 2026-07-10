"""
=========================================================
Sand Art Robot HMI

dialogs/setting_dialog.py

System Setting / Robot Monitor Dialog
=========================================================
"""

from datetime import datetime

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QFrame,
    QPlainTextEdit,
    QTabWidget,
    QWidget,
    QMessageBox,
    QGridLayout,
)


class SettingDialog(QDialog):
    """
    모든 페이지의 ⚙ 버튼에서 공통으로 여는 시스템 설정창.

    역할:
      - 현재 Tool / Force / Speed 표시
      - Robot 상태 표시
      - Robot Log 표시
      - HOME / PAUSE / STOP / EMERGENCY STOP 요청 Signal 발생

    실제 로봇 제어는 MainWindow 또는 Robot Client에서 Signal을 받아 처리한다.
    """

    home_requested = Signal()
    emergency_requested = Signal()

    # Robot bringup / drawing process control
    connect_requested = Signal()
    disconnect_requested = Signal()
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("SYSTEM SETTING")
        self.setMinimumSize(720, 560)
        self.resize(820, 640)

        self.current_tool = "MEDIUM"
        self.current_force = 0.8
        self.current_speed = 60
        self.current_robot_speed = 110
        self.current_sampling = 1.0
        self.init_ui()
        self.set_draw_parameter(
            tool=self.current_tool,
            force=self.current_force,
            speed=self.current_speed,
        )
        self.set_robot_status(
            connected=False,
            servo=False,
            force=False,
            compliance=False,
            drawing=False,
        )
        self.set_robot_connection_state("OFF")
        self.set_node_state("bringup", False)
        self.set_node_state("controller", False)
        self.set_node_state("planner", False)
        self.set_node_state("skeleton", False)
        self.set_node_state("movesx", False)
        self.set_node_state("lifecycle", False)
        self.add_log("[INFO] Setting dialog ready")

    def init_ui(self):
        """Dialog 전체 UI 생성."""

        self.root_layout = QVBoxLayout()
        self.root_layout.setContentsMargins(18, 18, 18, 18)
        self.root_layout.setSpacing(14)
        self.setLayout(self.root_layout)

        self.title_label = QLabel("⚙ SYSTEM SETTING")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 24px; font-weight: 700;")
        self.root_layout.addWidget(self.title_label)

        self.tab_widget = QTabWidget()
        self.root_layout.addWidget(self.tab_widget, 1)

        self.general_tab = QWidget()
        self.robot_tab = QWidget()
        self.log_tab = QWidget()
        self.emergency_tab = QWidget()

        self.tab_widget.addTab(self.general_tab, "General")
        self.tab_widget.addTab(self.robot_tab, "Robot")
        self.tab_widget.addTab(self.log_tab, "Log")
        self.tab_widget.addTab(self.emergency_tab, "Emergency")

        self.init_general_tab()
        self.init_robot_tab()
        self.init_log_tab()
        self.init_emergency_tab()

        bottom_layout = QHBoxLayout()
        bottom_layout.addStretch()

        self.close_button = QPushButton("닫기")
        self.close_button.setFixedSize(120, 42)
        self.close_button.clicked.connect(self.close)
        bottom_layout.addWidget(self.close_button)

        self.root_layout.addLayout(bottom_layout)

    def make_section_title(self, text):
        """섹션 제목 QLabel 생성."""

        label = QLabel(text)
        label.setStyleSheet("font-size: 17px; font-weight: 700;")
        return label

    def make_status_label(self, text):
        """상태 표시 QLabel 생성."""

        label = QLabel(text)
        label.setStyleSheet("font-size: 15px;")
        return label

    def init_general_tab(self):
        """General 탭: Tool / Force / Speed 표시."""

        layout = QVBoxLayout()
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        self.general_tab.setLayout(layout)

        layout.addWidget(self.make_section_title("Drawing Parameter"))

        grid = QGridLayout()
        grid.setHorizontalSpacing(24)
        grid.setVerticalSpacing(12)
        grid.addWidget(QLabel("Current Tool"), 0, 0)
        self.tool_value_label = QLabel("-")
        grid.addWidget(self.tool_value_label, 0, 1)

        grid.addWidget(QLabel("Draw Force"), 1, 0)
        self.force_value_label = QLabel("-")
        grid.addWidget(self.force_value_label, 1, 1)

        grid.addWidget(QLabel("Drawing Speed"), 2, 0)
        self.speed_value_label = QLabel("-")
        grid.addWidget(self.speed_value_label, 2, 1)

        grid.addWidget(QLabel("Robot Speed"), 3, 0)
        self.robot_speed_value_label = QLabel("-")
        grid.addWidget(self.robot_speed_value_label, 3, 1)

        grid.addWidget(QLabel("Sampling"), 4, 0)
        self.sampling_value_label = QLabel("-")
        grid.addWidget(self.sampling_value_label, 4, 1)

        layout.addLayout(grid)
        layout.addSpacing(18)

        info = QLabel(
            "Tool 선택값이 여기 표시됩니다.\n"
            "Fine = 0.2N / 30mm/s\n"
            "Medium = 0.8N / 60mm/s\n"
            "Wide = 1.5N / 100mm/s"
        )
        info.setStyleSheet("font-size: 14px; color: #666;")
        layout.addWidget(info)
        layout.addStretch()

    def init_robot_tab(self):
        """Robot 탭: 로봇 연결/해제 및 상태 표시."""

        layout = QVBoxLayout()
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(14)
        self.robot_tab.setLayout(layout)

        layout.addWidget(self.make_section_title("Robot Connection"))

        self.robot_state_label = self.make_status_label("🔴 Robot : OFF")
        layout.addWidget(self.robot_state_label)

        button_layout = QHBoxLayout()

        self.connect_button = QPushButton("CONNECT")
        self.disconnect_button = QPushButton("DISCONNECT")

        self.connect_button.setMinimumHeight(42)
        self.disconnect_button.setMinimumHeight(42)

        button_layout.addWidget(self.connect_button)
        button_layout.addWidget(self.disconnect_button)

        layout.addLayout(button_layout)
        layout.addSpacing(12)

        layout.addWidget(self.make_section_title("Robot Status"))

        self.connected_label = self.make_status_label("🔴 Connected : OFF")
        self.servo_label = self.make_status_label("🔴 Servo : OFF")
        self.force_label = self.make_status_label("🔴 Force : OFF")
        self.compliance_label = self.make_status_label("🔴 Compliance : OFF")
        self.drawing_label = self.make_status_label("⚪ Drawing : IDLE")
        layout.addSpacing(18)

        layout.addWidget(
            self.make_section_title("Node Monitor")
        )

        self.node_bringup = self.make_status_label("⚪ Bringup")
        self.node_controller = self.make_status_label("⚪ Controller")
        self.node_planner = self.make_status_label("⚪ Planner")
        self.node_skeleton = self.make_status_label("⚪ Skeleton")
        self.node_movesx = self.make_status_label("⚪ MovesX")
        self.node_lifecycle = self.make_status_label("⚪ Lifecycle")

        layout.addWidget(self.node_bringup)
        layout.addWidget(self.node_controller)
        layout.addWidget(self.node_planner)
        layout.addWidget(self.node_skeleton)
        layout.addWidget(self.node_movesx)
        layout.addWidget(self.node_lifecycle)
    

        layout.addWidget(self.connected_label)
        layout.addWidget(self.servo_label)
        layout.addWidget(self.force_label)
        layout.addWidget(self.compliance_label)
        layout.addWidget(self.drawing_label)

        info = QLabel(
            "CONNECT 버튼은 아래 명령을 HMI에서 대신 실행합니다.\n"
            "source ~/ws_cobot_pjt/ws_dsr/install/setup.bash\n"
            "ros2 launch m0609_rg2_bringup bringup.launch.py mode:=real host:=192.168.1.100 model:=m0609"
        )
        info.setWordWrap(True)
        info.setStyleSheet("font-size: 13px; color: #666;")
        layout.addSpacing(8)
        layout.addWidget(info)
        layout.addStretch()

        self.connect_button.clicked.connect(self.on_connect_clicked)
        self.disconnect_button.clicked.connect(self.on_disconnect_clicked)

    def init_log_tab(self):
        """Log 탭: Robot / GUI 로그 출력."""

        layout = QVBoxLayout()
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(10)
        self.log_tab.setLayout(layout)

        layout.addWidget(self.make_section_title("Robot Log"))
        self.log_tabs = QTabWidget()

        layout.addWidget(self.log_tabs, 1)

        # ------------------------
        # ALL LOG
        # ------------------------

        self.all_log_tab = QWidget()
        self.log_tabs.addTab(
            self.all_log_tab,
            "All Log"
        )

        all_layout = QVBoxLayout()
        self.all_log_tab.setLayout(all_layout)

        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(3000)

        all_layout.addWidget(self.log_view)

        # ------------------------
        # BRINGUP
        # ------------------------

        self.bringup_tab = QWidget()
        self.log_tabs.addTab(self.bringup_tab, "Bringup")

        bringup_layout = QVBoxLayout()
        self.bringup_tab.setLayout(bringup_layout)

        self.bringup_log = QPlainTextEdit()
        self.bringup_log.setReadOnly(True)
        self.bringup_log.setMaximumBlockCount(3000)

        bringup_layout.addWidget(self.bringup_log)
        # ------------------------
        # SANDART
        # ------------------------

        self.sandart_tab = QWidget()
        self.log_tabs.addTab(self.sandart_tab, "Sandart")

        sand_layout = QVBoxLayout()
        self.sandart_tab.setLayout(sand_layout)

        self.sandart_log = QPlainTextEdit()
        self.sandart_log.setReadOnly(True)
        self.sandart_log.setMaximumBlockCount(3000)

        sand_layout.addWidget(self.sandart_log)
        # ------------------------
        # DRAW
        # ------------------------

        self.draw_tab = QWidget()
        self.log_tabs.addTab(self.draw_tab, "Drawing")

        draw_layout = QVBoxLayout()
        self.draw_tab.setLayout(draw_layout)

        self.draw_log = QPlainTextEdit()
        self.draw_log.setReadOnly(True)
        self.draw_log.setMaximumBlockCount(3000)

        draw_layout.addWidget(self.draw_log)
        # ------------------------
        # LIFECYCLE LOG
        # ------------------------

        self.lifecycle_tab = QWidget()

        self.log_tabs.addTab(
            self.lifecycle_tab,
            "Lifecycle"
        )

        life_layout = QVBoxLayout()

        self.lifecycle_tab.setLayout(
            life_layout
        )

        self.lifecycle_log = QPlainTextEdit()

        self.lifecycle_log.setReadOnly(True)

        self.lifecycle_log.setMaximumBlockCount(
            3000
        )

        life_layout.addWidget(
            self.lifecycle_log
        )

        # ------------------------
        # ERROR
        # ------------------------

        self.error_tab = QWidget()
        self.log_tabs.addTab(self.error_tab, "Error")

        error_layout = QVBoxLayout()
        self.error_tab.setLayout(error_layout)

        self.error_log = QPlainTextEdit()
        self.error_log.setReadOnly(True)
        self.error_log.setMaximumBlockCount(3000)

        error_layout.addWidget(self.error_log)
    def init_emergency_tab(self):
        """Emergency 탭: 제어 버튼."""

        layout = QVBoxLayout()
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(16)
        self.emergency_tab.setLayout(layout)

        layout.addWidget(self.make_section_title("Robot Control"))

        self.home_button = QPushButton("EXIT")
        self.emergency_button = QPushButton("🔴 EMERGENCY STOP")

        self.home_button.setMinimumHeight(44)
        self.emergency_button.setMinimumHeight(56)

        self.emergency_button.setStyleSheet(
            "font-size: 18px; font-weight: 700; "
            "background-color: #b00020; color: white;"
        )

        layout.addWidget(self.home_button)
        layout.addSpacing(10)
        layout.addWidget(self.emergency_button)
        layout.addStretch()

        self.home_button.clicked.connect(self.on_home_clicked)
        self.emergency_button.clicked.connect(self.on_emergency_clicked)

    def set_draw_parameter(
        self,
        tool=None,
        force=None,
        speed=None,
        robot_speed=None,
        sampling=None,
    ):
        """현재 Tool / Force / Speed 표시 갱신."""

        if tool is not None:
            self.current_tool = tool
        if force is not None:
            self.current_force = force
        if speed is not None:
            self.current_speed = speed
        if robot_speed is not None:
            self.current_robot_speed = robot_speed

        if sampling is not None:
            self.current_sampling = sampling

        self.tool_value_label.setText(str(self.current_tool))
        self.force_value_label.setText(f"{self.current_force:.2f} N")
        self.speed_value_label.setText(f"{self.current_speed} mm/s")
        self.robot_speed_value_label.setText(
            f"{self.current_robot_speed:.0f} mm/s"
        )

        self.sampling_value_label.setText(
            f"{self.current_sampling:.1f} mm"
        )
    def set_robot_status(
        self,
        connected=None,
        servo=None,
        force=None,
        compliance=None,
        drawing=None,
    ):
        """Robot 상태 표시 갱신."""

        if connected is not None:
            self.connected_label.setText(
                f"{'🟢' if connected else '🔴'} Connected : {'ON' if connected else 'OFF'}"
            )

        if servo is not None:
            self.servo_label.setText(
                f"{'🟢' if servo else '🔴'} Servo : {'ON' if servo else 'OFF'}"
            )

        if force is not None:
            self.force_label.setText(
                f"{'🟢' if force else '🔴'} Force : {'ON' if force else 'OFF'}"
            )

        if compliance is not None:
            self.compliance_label.setText(
                f"{'🟢' if compliance else '🔴'} Compliance : {'ON' if compliance else 'OFF'}"
            )

        if drawing is not None:
            self.drawing_label.setText(
                f"{'🟢' if drawing else '⚪'} Drawing : {'RUNNING' if drawing else 'IDLE'}"
            )

    def add_log(self, message):
        """로그창과 터미널에 동시에 로그 출력."""

        now = datetime.now().strftime("%H:%M:%S")
        line = f"{now}  {message}"

        print(line)
        self.log_view.appendPlainText(line)
        self.log_view.verticalScrollBar().setValue(
            self.log_view.verticalScrollBar().maximum()
        )
    def add_bringup_log(self, message):

        now = datetime.now().strftime("%H:%M:%S")
        line = f"{now}  {message}"

        print(line)

        self.bringup_log.appendPlainText(line)

        self.bringup_log.verticalScrollBar().setValue(
            self.bringup_log.verticalScrollBar().maximum()
        )
    def add_sandart_log(self, message):

        now = datetime.now().strftime("%H:%M:%S")
        line = f"{now}  {message}"

        print(line)

        self.sandart_log.appendPlainText(line)

        self.sandart_log.verticalScrollBar().setValue(
            self.sandart_log.verticalScrollBar().maximum()
        )
    def add_draw_log(self, message):

        now = datetime.now().strftime("%H:%M:%S")
        line = f"{now}  {message}"

        print(line)

        self.draw_log.appendPlainText(line)

        self.draw_log.verticalScrollBar().setValue(
            self.draw_log.verticalScrollBar().maximum()
        )
    def add_error_log(self, message):

        now = datetime.now().strftime("%H:%M:%S")
        line = f"{now}  {message}"

        print(line)

        self.error_log.appendPlainText(line)

        self.error_log.verticalScrollBar().setValue(
            self.error_log.verticalScrollBar().maximum()
        )
    def add_lifecycle_log(self, message):

        now = datetime.now().strftime("%H:%M:%S")

        line = f"{now}  {message}"

        print(line)

        self.lifecycle_log.appendPlainText(line)

        self.lifecycle_log.verticalScrollBar().setValue(
            self.lifecycle_log.verticalScrollBar().maximum()
        )
    def clear_log(self):
        self.log_view.clear()
        self.bringup_log.clear()
        self.sandart_log.clear()
        self.draw_log.clear()
        self.lifecycle_log.clear()
        self.error_log.clear()

        self.add_log("[INFO] Log cleared")
    def set_node_state(
        self,
        node,
        state,
    ):
        """
        Node 상태 표시

        state=True  -> 🟢
        state=False -> 🔴
        """

        icon = "🟢" if state else "🔴"

        mapping = {
            "bringup": self.node_bringup,
            "controller": self.node_controller,
            "planner": self.node_planner,
            "skeleton": self.node_skeleton,
            "movesx": self.node_movesx,
            "lifecycle": self.node_lifecycle,
        }

        if node not in mapping:
            return

        mapping[node].setText(
            f"{icon} {node.capitalize()}"
        )
    def set_robot_connection_state(self, state):
        """Robot 연결 상태 텍스트 표시."""

        state = str(state).upper()

        if state == "CONNECTED":
            self.robot_state_label.setText("🟢 Robot : CONNECTED")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
        elif state == "CONNECTING":
            self.robot_state_label.setText("🟡 Robot : CONNECTING")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
        elif state == "DRAWING":
            self.robot_state_label.setText("🟢 Robot : DRAWING")
            self.connect_button.setEnabled(False)
            self.disconnect_button.setEnabled(True)
        elif state == "ERROR":
            self.robot_state_label.setText("🔴 Robot : ERROR")
            self.connect_button.setEnabled(True)
            self.disconnect_button.setEnabled(True)
        else:
            self.robot_state_label.setText("🔴 Robot : OFF")
            self.connect_button.setEnabled(True)
            self.disconnect_button.setEnabled(False)

    def on_connect_clicked(self):
        """CONNECT 버튼."""

        self.add_log("[CTRL] CONNECT requested")
        self.connect_requested.emit()

    def on_disconnect_clicked(self):
        """DISCONNECT 버튼."""

        self.add_log("[CTRL] DISCONNECT requested")
        self.disconnect_requested.emit()

    def on_home_clicked(self):
        """EXIT 버튼."""

        self.add_log("[CTRL] EXIT requested")
        self.home_requested.emit()


    def on_emergency_clicked(self):
        """Emergency Stop 버튼. 확인 팝업 후 Signal 발생."""

        result = QMessageBox.warning(
            self,
            "EMERGENCY STOP",
            "정말로 로봇을 긴급 정지하시겠습니까?\n\n"
            "Force / Compliance / Motion을 모두 정지하는 용도입니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if result == QMessageBox.Yes:
            self.add_log("[EMERGENCY] Emergency stop confirmed")
            self.emergency_requested.emit()
        else:
            self.add_log("[EMERGENCY] Emergency stop canceled")
