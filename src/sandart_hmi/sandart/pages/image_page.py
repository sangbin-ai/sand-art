"""
=========================================================
Sand Art Robot HMI

pages/image_page.py

Image Select Page

설계 기준
-------------------------
- 키오스크 단계별 화면
- 사진 시안처럼 한 화면에 한 단계만 표시
- 샘플 이미지 / 썸네일 없음
- 사용자가 Open Image 버튼으로 직접 이미지 선택
- Open / Previous / Next 아이콘은 Qt 기본 아이콘 사용
=========================================================
"""

import os

from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QWidget,
    QLabel,
    QPushButton,
    QFileDialog,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QStyle,
    QSizePolicy,
    )


class ImagePage(QWidget):
    """
    이미지 선택 화면

    필요한 PNG 파일:
    - 없음

    사용하는 아이콘:
    - Qt 기본 아이콘
        - QStyle.SP_DialogOpenButton
        - QStyle.SP_ArrowBack
        - QStyle.SP_ArrowForward
    """

    # ======================================================
    # Signal
    # ======================================================

    prev_clicked = Signal()
    next_clicked = Signal()
    image_selected = Signal(str)

    # ======================================================

    def __init__(self):
        super().__init__()

        # 현재 선택된 이미지 경로
        self.image_path = ""

        self.init_ui()

    # ======================================================
    # UI 생성
    # ======================================================

    def init_ui(self):
        """
        이미지 선택 페이지 UI 구성
        """

        # ==================================================
        # Root Layout
        # ==================================================

        self.root_layout = QVBoxLayout()
        # 전체 화면 여백 축소
        self.root_layout.setContentsMargins(20, 20, 20, 20)

        # Header와 Body 간격 축소
        self.root_layout.setSpacing(16)
        self.setLayout(self.root_layout)

        # ==================================================
        # Header
        # ==================================================

        self.header = QFrame()
        self.header.setObjectName("Header")

        header_layout = QHBoxLayout()
        header_layout.setContentsMargins(12, 8, 12, 8)
        header_layout.setSpacing(10)
        self.header.setLayout(header_layout)

        self.title = QLabel("🤖 SAND ART ROBOT HMI")
        self.title.setObjectName("HeaderTitle")

        header_layout.addWidget(self.title)
        header_layout.addStretch()

        self.mode_label = QLabel("MODE  AUTO")
        self.mode_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.mode_label)

        self.status_label = QLabel("STATUS  READY")
        self.status_label.setObjectName("ReadyLabel")
        header_layout.addWidget(self.status_label)

        self.servo_label = QLabel("SERVO  ON")
        self.servo_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.servo_label)

        self.time_label = QLabel("14:25:36")
        self.time_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.time_label)

        self.setting_label = QLabel("⚙")
        self.setting_label.setAlignment(Qt.AlignCenter)
        self.setting_label.setObjectName("ModeLabel")
        header_layout.addWidget(self.setting_label)

        self.root_layout.addWidget(self.header)

        # ==================================================
        # Body Layout
        # ==================================================

        self.body_layout = QHBoxLayout()
        self.body_layout.setSpacing(24)
        self.root_layout.addLayout(self.body_layout, 1)

        # ==================================================
        # Sidebar
        # ==================================================

        self.sidebar = QFrame()
        self.sidebar.setObjectName("SideBar")
        # Sidebar 폭 고정
        self.sidebar.setMinimumWidth(180)
        self.sidebar.setMaximumWidth(220)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(8, 18, 8, 18)
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

        for index, text in enumerate(steps):
            step_label = QLabel(text)

            if index == 0:
                step_label.setObjectName("StepActive")
            else:
                step_label.setObjectName("StepNormal")

            sidebar_layout.addWidget(step_label)

        sidebar_layout.addStretch()

        self.body_layout.addWidget(self.sidebar)

        # ==================================================
        # Content Frame
        # ==================================================

        self.content = QFrame()
        self.content.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.content.setObjectName("ContentFrame")

        self.content_layout = QVBoxLayout()
        self.content_layout.setContentsMargins(24, 24, 24, 24)
        self.content_layout.setSpacing(18)
        self.content.setLayout(self.content_layout)

        self.body_layout.addWidget(self.content, 1)

        # ==================================================
        # Page Title
        # ==================================================

        self.page_title = QLabel("IMAGE SELECT")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")

        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "샌드아트로 변환할 이미지를 선택하세요.\n"
            "JPG, PNG, JPEG, BMP 파일을 사용할 수 있습니다."
        )
        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")

        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Preview Area
        # ==================================================

        self.preview_frame = QFrame()
        self.preview_frame.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.preview_frame.setObjectName("ResultBox")

        preview_layout = QVBoxLayout()
        preview_layout.setContentsMargins(
            16,
            16,
            16,
            16,
        )
        preview_layout.setSpacing(14)
        self.preview_frame.setLayout(preview_layout)

        self.preview = QLabel()
        # 창 크기에 따라 Preview가 자동으로 커지고 작아짐
        self.preview.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.preview.setObjectName("ImagePreview")
        self.preview.setAlignment(Qt.AlignCenter)
        # Preview가 너무 커서 버튼이 아래로 밀리는 현상 방지
        self.preview.setMinimumSize(200,200)
        self.preview.setText(
            "NO IMAGE SELECTED\n\n"
            "OPEN IMAGE 버튼을 눌러\n"
            "샌드아트로 변환할 이미지를 선택하세요."
        )

        preview_layout.addWidget(self.preview)

        self.selected_label = QLabel("Selected Image : None")
        self.selected_label.setAlignment(Qt.AlignCenter)
        self.selected_label.setObjectName("PageDescription")

        preview_layout.addWidget(self.selected_label)

        self.content_layout.addWidget(self.preview_frame, 1)

        # ==================================================
        # Open Image Button
        # ==================================================

        self.open_button_layout = QHBoxLayout()
        self.open_button_layout.addStretch()

        self.select_button = QPushButton("  OPEN IMAGE")
        self.select_button.setMinimumWidth(240)
        self.select_button.setMinimumHeight(50)

        open_icon = QApplication.style().standardIcon(
            QStyle.SP_DialogOpenButton
        )

        self.select_button.setIcon(open_icon)
        self.select_button.setIconSize(QSize(28, 28))

        self.open_button_layout.addWidget(self.select_button)
        self.open_button_layout.addStretch()

        self.content_layout.addLayout(self.open_button_layout)

        # ==================================================
        # Navigation Buttons
        # ==================================================

        nav_layout = QHBoxLayout()
        nav_layout.setSpacing(12)

        self.prev_button = QPushButton("  이전")
        self.prev_button.setObjectName("PrevButton")
        self.prev_button.setMinimumHeight(48)
        self.prev_button.setMinimumWidth(160)

        prev_icon = QApplication.style().standardIcon(
            QStyle.SP_ArrowBack
        )

        self.prev_button.setIcon(prev_icon)
        self.prev_button.setIconSize(QSize(24, 24))

        self.next_button = QPushButton("다음  ")
        self.next_button.setObjectName("NextButton")
        self.next_button.setMinimumHeight(48)
        self.next_button.setMinimumWidth(160)

        next_icon = QApplication.style().standardIcon(
            QStyle.SP_ArrowForward
        )

        self.next_button.setIcon(next_icon)
        self.next_button.setIconSize(QSize(24, 24))

        nav_layout.addWidget(self.prev_button)
        nav_layout.addStretch()
        nav_layout.addWidget(self.next_button)

        self.content_layout.addLayout(nav_layout)

        # ==================================================
        # Signal
        # ==================================================

        self.select_button.clicked.connect(self.open_image)
        self.prev_button.clicked.connect(self.prev_clicked.emit)
        self.next_button.clicked.connect(self.next_clicked.emit)

    # ======================================================
    # 이미지 선택
    # ======================================================

    def open_image(self):
        """
        파일 선택 창을 열고 이미지 미리보기를 표시합니다.
        """

        file_name, _ = QFileDialog.getOpenFileName(
            self,
            "이미지 선택",
            "",
            "Image Files (*.png *.jpg *.jpeg *.bmp)",
        )

        if not file_name:
            return

        self.image_path = file_name

        self.selected_label.setText(
            f"Selected Image : {os.path.basename(file_name)}"
        )

        self.show_selected_image()

        self.image_selected.emit(file_name)

    # ======================================================
    # 선택 이미지 표시
    # ======================================================

    def show_selected_image(self):
        """
        현재 image_path의 이미지를 Preview 크기에 맞춰 표시합니다.
        """

        if not self.image_path:
            return

        pixmap = QPixmap(self.image_path)

        if pixmap.isNull():
            return

        pixmap = pixmap.scaled(

            self.preview.size(),

            Qt.KeepAspectRatio,

            Qt.SmoothTransformation,

        )

        self.preview.setPixmap(pixmap)

    # ======================================================
    # 창 크기 변경
    # ======================================================

    def resizeEvent(self, event):
        """
        창 크기 변경 시 이미지도 다시 맞춰 표시합니다.
        """

        if self.image_path:
            self.show_selected_image()

        super().resizeEvent(event)

    # ======================================================
    # Preview 초기화
    # ======================================================

    def clear_preview(self):
        """
        새 작업 시작 시 이미지 선택 상태를 초기화합니다.
        """

        self.image_path = ""

        self.preview.clear()

        self.preview.setText(
            "NO IMAGE SELECTED\n\n"
            "OPEN IMAGE 버튼을 눌러\n"
            "샌드아트로 변환할 이미지를 선택하세요."
        )

        self.selected_label.setText("Selected Image : None")

    # ======================================================
    # 현재 이미지 경로 반환
    # ======================================================

    def get_image_path(self):
        """
        현재 선택된 이미지 경로를 반환합니다.
        """

        return self.image_path

    # ======================================================
    # Header Status 변경
    # ======================================================

    def set_status(self, text: str):
        """
        Header Status 변경
        """

        self.status_label.setText(f"STATUS  {text}")

    # ======================================================
    # Header Mode 변경
    # ======================================================

    def set_mode(self, text: str):
        """
        Header Mode 변경
        """

        self.mode_label.setText(f"MODE  {text}")

    # ======================================================
    # Servo 상태 변경
    # ======================================================

    def set_servo(self, state: str):
        """
        Servo 상태 변경
        """

        self.servo_label.setText(f"SERVO  {state}")

    # ======================================================
    # 시간 표시 변경
    # ======================================================

    def set_time(self, text: str):
        """
        Header 시간 표시 변경
        """

        self.time_label.setText(text)
