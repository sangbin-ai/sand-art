"""
=========================================================
Sand Art Robot HMI

pages/finish_page.py

Finish Page
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
)


class FinishPage(QWidget):

    home_clicked = Signal()
    exit_clicked = Signal()

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

        self.status_label = QLabel("STATUS  COMPLETE")
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
        # Body Layout
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

            if i == 6:
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
        self.content_layout.setSpacing(28)

        self.content.setLayout(self.content_layout)

        self.body_layout.addWidget(
            self.content,
            1
        )

        # ==================================================
        # Title
        # ==================================================

        self.page_title = QLabel("DRAWING COMPLETE")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")

        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "샌드아트 작업이 정상적으로 완료되었습니다.\n"
            "아래 메뉴를 선택하세요."
        )

        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")

        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Finish Frame
        # ==================================================

        self.finish_frame = QFrame()
        self.finish_frame.setObjectName("ResultBox")

        finish_layout = QVBoxLayout()
        finish_layout.setContentsMargins(40,40,40,40)
        finish_layout.setSpacing(26)

        self.finish_frame.setLayout(finish_layout)

        # ==================================================
        # Complete Message
        # ==================================================

        self.complete_label = QLabel("✅ DRAWING COMPLETE")

        self.complete_label.setAlignment(Qt.AlignCenter)
        self.complete_label.setObjectName("MenuTitle")

        finish_layout.addWidget(self.complete_label)

        # ==================================================
        # Total Time
        # ==================================================

        self.total_time = QLabel(
            "총 작업 시간 : -- : --"
        )

        self.total_time.setAlignment(Qt.AlignCenter)
        self.total_time.setObjectName("PageDescription")

        finish_layout.addWidget(self.total_time)

        # ==================================================
        # Selected Tool
        # ==================================================

        self.tool_label = QLabel(
            "사용 Tool : -"
        )

        self.tool_label.setAlignment(Qt.AlignCenter)
        self.tool_label.setObjectName("PageDescription")

        finish_layout.addWidget(self.tool_label)

        # ==================================================
        # Drawing Information
        # ==================================================

        self.info_label = QLabel(
            "Sand Art 작업이 정상적으로 종료되었습니다."
        )

        self.info_label.setAlignment(Qt.AlignCenter)
        self.info_label.setObjectName("PageDescription")

        finish_layout.addWidget(self.info_label)

        self.content_layout.addWidget(
            self.finish_frame,
            1
        )

                # ==================================================
        # Bottom Buttons
        # ==================================================

        self.button_layout = QHBoxLayout()
        self.button_layout.setSpacing(20)

        # --------------------------------------------------
        # Home Button
        # --------------------------------------------------

        self.home_button = QPushButton("⌂ HOME")
        self.home_button.setMinimumHeight(60)
        self.home_button.setMinimumWidth(180)

        self.button_layout.addWidget(self.home_button)


        # --------------------------------------------------
        # Exit Button
        # --------------------------------------------------

        self.exit_button = QPushButton("✕ EXIT")
        self.exit_button.setMinimumHeight(60)
        self.exit_button.setMinimumWidth(180)

        self.button_layout.addWidget(self.exit_button)

        self.content_layout.addLayout(self.button_layout)

        # ==================================================
        # Signal
        # ==================================================

        self.home_button.clicked.connect(
            self.home_clicked.emit
        )

        self.exit_button.clicked.connect(
            self.exit_clicked.emit
        )

    # ======================================================
    # 총 작업 시간 표시
    # ======================================================

    def set_total_time(self, text):
        """
        예)
        02 : 35
        """

        self.total_time.setText(
            f"총 작업 시간 : {text}"
        )

    # ======================================================
    # 사용 Tool 표시
    # ======================================================

    def set_tool(self, tool):
        """
        FINE
        MEDIUM
        WIDE
        """

        self.tool_label.setText(
            f"사용 Tool : {tool}"
        )

    # ======================================================
    # 완료 메시지 변경
    # ======================================================

    def set_info(self, text):

        self.info_label.setText(text)

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
    # Servo 변경
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