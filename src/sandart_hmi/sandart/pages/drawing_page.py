"""
=========================================================
Sand Art Robot HMI

pages/drawing_page.py

Drawing Page
=========================================================
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QProgressBar,
)


class DrawingPage(QWidget):

    home_clicked = Signal()
    pause_clicked = Signal()
    stop_clicked = Signal()

    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):

        # ==================================================
        # Root Layout
        # ==================================================

        self.root_layout = QVBoxLayout()
        self.root_layout.setContentsMargins(20,20,20,20)
        self.root_layout.setSpacing(24)

        self.setLayout(self.root_layout)

        # ==================================================
        # Header
        # ==================================================

        self.header = QFrame()
        self.header.setObjectName("Header")

        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(12,8,12,8)
        header_layout.setSpacing(12)

        self.header.setLayout(header_layout)

        self.title = QLabel("🤖 SAND ART ROBOT HMI")
        self.title.setObjectName("HeaderTitle")

        header_layout.addWidget(self.title)

        header_layout.addStretch()

        self.mode_label = QLabel("MODE  AUTO")
        self.mode_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.mode_label)

        self.status_label = QLabel("STATUS  DRAWING")
        self.status_label.setObjectName("ReadyLabel")
        header_layout.addWidget(self.status_label)

        self.servo_label = QLabel("SERVO  ON")
        self.servo_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.servo_label)

        self.time_label = QLabel("14:25:36")
        self.time_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.time_label)

        self.setting = QLabel("⚙")
        self.setting.setAlignment(Qt.AlignCenter)
        self.setting.setObjectName("ModeLabel")

        header_layout.addWidget(self.setting)

        self.root_layout.addWidget(self.header)

        # ==================================================
        # Body
        # ==================================================

        self.body_layout = QHBoxLayout()
        self.body_layout.setSpacing(24)

        self.root_layout.addLayout(
            self.body_layout,
            1
        )

        # ==================================================
        # Sidebar
        # ==================================================

        self.sidebar = QFrame()
        self.sidebar.setObjectName("SideBar")

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(8,18,8,18)
        sidebar_layout.setSpacing(6)

        self.sidebar.setLayout(sidebar_layout)

        steps = [
            "1. 이미지 선택",
            "2. 흑백 변환",
            "3. 엣지 Preview",
            "4. 샌드 Preview",
            "5. Parameter",
            "6. Drawing",
            "7. Finish",
        ]
        for i, text in enumerate(steps):

            label = QLabel(text)

            if i == 5:

                label.setObjectName("StepActive")

            else:

                label.setObjectName("StepNormal")

            sidebar_layout.addWidget(label)

        sidebar_layout.addStretch()

        self.body_layout.addWidget(self.sidebar)

                # ==================================================
        # Content Frame
        # ==================================================

        self.content = QFrame()
        self.content.setObjectName("ContentFrame")

        self.content_layout = QVBoxLayout()
        self.content_layout.setContentsMargins(24,24,24,24)
        self.content_layout.setSpacing(30)

        self.content.setLayout(self.content_layout)

        self.body_layout.addWidget(self.content, 1)

        # ==================================================
        # Page Title
        # ==================================================

        self.page_title = QLabel("DRAWING")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")

        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "샌드아트를 그리고 있습니다.\n"
            "작업이 완료될 때까지 기다려 주세요."
        )

        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")

        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Progress Frame
        # ==================================================

        self.progress_frame = QFrame()
        self.progress_frame.setObjectName("ResultBox")

        progress_layout = QVBoxLayout()
        progress_layout.setContentsMargins(40,40,40,40)
        progress_layout.setSpacing(30)

        self.progress_frame.setLayout(progress_layout)

        # ==================================================
        # Current Status
        # ==================================================

        self.current_status = QLabel(
            "현재 작업 : Ready"
        )

        self.current_status.setAlignment(Qt.AlignCenter)
        self.current_status.setObjectName("MenuTitle")

        progress_layout.addWidget(self.current_status)

        # ==================================================
        # ProgressBar
        # ==================================================

        self.progress_bar = QProgressBar()

        self.progress_bar.setRange(0,100)
        self.progress_bar.setValue(0)

        progress_layout.addWidget(self.progress_bar)

        # ==================================================
        # Progress Text
        # ==================================================

        self.progress_text = QLabel("0 %")

        self.progress_text.setAlignment(Qt.AlignCenter)
        self.progress_text.setObjectName("PageDescription")

        progress_layout.addWidget(self.progress_text)

        # ==================================================
        # Remaining Time
        # ==================================================

        self.remaining_time = QLabel(
            "남은 시간 : -- : --"
        )

        self.remaining_time.setAlignment(Qt.AlignCenter)
        self.remaining_time.setObjectName("PageDescription")

        self.remaining_time.hide()
        # progress_layout.addWidget(self.remaining_time)

        self.content_layout.addWidget(
            self.progress_frame,
            1
        )

                # ==================================================
        # Control Buttons
        # ==================================================

        self.control_layout = QHBoxLayout()
        self.control_layout.setSpacing(20)

        # --------------------------------------------------
        # Pause Button
        # --------------------------------------------------

        self.pause_button = QPushButton("⏸ Pause")
        self.pause_button.setMinimumHeight(60)
        self.pause_button.setMinimumWidth(180)

        self.control_layout.addWidget(self.pause_button)

        # --------------------------------------------------
        # Stop Button
        # --------------------------------------------------

        self.stop_button = QPushButton("■ Stop")
        self.stop_button.setMinimumHeight(60)
        self.stop_button.setMinimumWidth(180)

        self.control_layout.addWidget(self.stop_button)


        self.content_layout.addLayout(self.control_layout)

        # ==================================================
        # Signal
        # ==================================================

        self.pause_button.clicked.connect(
            self.pause_clicked.emit
        )

        self.stop_button.clicked.connect(
            self.stop_clicked.emit
        )


    # ======================================================
    # Progress 변경
    # ======================================================

    def set_progress(self, value):
        """
        진행률 표시 (0~100)
        """

        value = max(0, min(100, value))

        self.progress_bar.setValue(value)
        self.progress_text.setText(f"{value} %")

    # ======================================================
    # 현재 작업 표시
    # ======================================================

    def set_current_status(self, text):
        """
        현재 작업 상태 표시
        """

        self.current_status.setText(
            f"현재 작업 : {text}"
        )

    # ======================================================
    # 남은 시간 표시
    # ======================================================

    def set_remaining_time(self, text):
        """
        남은 시간 표시
        """

        self.remaining_time.setText(
            f"남은 시간 : {text}"
        )

    # ======================================================
    # Header Status 변경
    # ======================================================

    def set_status(self, text):

        self.status_label.setText(
            f"STATUS  {text}"
        )

    # ======================================================
    # Header Mode 변경
    # ======================================================

    def set_mode(self, text):

        self.mode_label.setText(
            f"MODE  {text}"
        )

    # ======================================================
    # Servo 상태 변경
    # ======================================================

    def set_servo(self, state):

        self.servo_label.setText(
            f"SERVO  {state}"
        )

    # ======================================================
    # Header 시간 변경
    # ======================================================

    def set_time(self, text):

        self.time_label.setText(text)