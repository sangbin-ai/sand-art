"""
=========================================================
Sand Art Robot HMI
pages/gray_page.py
Gray Scale Page
=========================================================
"""

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QWidget,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QFrame,
    QSizePolicy,
)
from dialogs.image_preview_dialog import ImagePreviewDialog


class GrayPage(QWidget):
    """
    흑백 변환 화면

    역할:
    - 원본 이미지 표시
    - 흑백 변환 결과 표시
    - 창 크기 변경 시 이미지 자동 리사이즈
    """

    prev_clicked = Signal()
    next_clicked = Signal()

    def __init__(self):
        super().__init__()

        # resizeEvent에서 다시 그리기 위해 원본 Pixmap 저장
        self.original_pixmap = None
        self.gray_pixmap = None

        self.init_ui()

    def init_ui(self):
        # ==================================================
        # Root Layout
        # ==================================================
        self.root_layout = QVBoxLayout()
        self.root_layout.setContentsMargins(20, 20, 20, 20)
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
        self.body_layout.setSpacing(16)
        self.root_layout.addLayout(self.body_layout, 1)

        # ==================================================
        # Sidebar
        # ==================================================
        self.sidebar = QFrame()
        self.sidebar.setObjectName("SideBar")
        self.sidebar.setMinimumWidth(180)
        self.sidebar.setMaximumWidth(220)

        sidebar_layout = QVBoxLayout()
        sidebar_layout.setContentsMargins(8, 14, 8, 14)
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

            if index == 1:
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
        self.content.setObjectName("ContentFrame")
        self.content.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )

        self.content_layout = QVBoxLayout()
        self.content_layout.setContentsMargins(24, 24, 24, 24)
        self.content_layout.setSpacing(18)
        self.content.setLayout(self.content_layout)

        self.body_layout.addWidget(self.content, 1)

        # ==================================================
        # Page Title
        # ==================================================
        self.page_title = QLabel("GRAY SCALE")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")
        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "컬러 이미지를 흑백 이미지로 변환하는 단계입니다.\n"
            "실제 변환 로직은 추후 전처리 py 파일과 연결합니다."
        )
        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")
        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Compare Area
        # ==================================================
        self.compare_frame = QFrame()
        self.compare_frame.setObjectName("ResultBox")
        self.compare_frame.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )

        compare_frame_layout = QHBoxLayout()
        compare_frame_layout.setContentsMargins(16, 16, 16, 16)
        compare_frame_layout.setSpacing(16)
        self.compare_frame.setLayout(compare_frame_layout)

        # ==================================================
        # Left : Original Image
        # ==================================================
        self.left_panel = QVBoxLayout()
        self.left_panel.setSpacing(10)

        self.original_title = QLabel("ORIGINAL IMAGE")
        self.original_title.setAlignment(Qt.AlignCenter)
        self.original_title.setObjectName("PageDescription")
        self.left_panel.addWidget(self.original_title)

        self.original_preview = QLabel()
        self.original_preview.setObjectName("ImagePreview")
        self.original_preview.setAlignment(Qt.AlignCenter)
        self.original_preview.setMinimumSize(220, 220)
        self.original_preview.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.original_preview.setText(
            "ORIGINAL\n\n"
            "이전 단계에서 선택한 이미지"
        )
        self.left_panel.addWidget(self.original_preview, 1)

        compare_frame_layout.addLayout(self.left_panel, 5)

        # ==================================================
        # Center Arrow
        # ==================================================
        self.arrow_label = QLabel("→")
        self.arrow_label.setAlignment(Qt.AlignCenter)
        self.arrow_label.setObjectName("PageTitle")
        self.arrow_label.setMinimumWidth(40)
        compare_frame_layout.addWidget(self.arrow_label)

        # ==================================================
        # Right : Gray Image
        # ==================================================
        self.right_panel = QVBoxLayout()
        self.right_panel.setSpacing(10)

        self.gray_title = QLabel("GRAY IMAGE")
        self.gray_title.setAlignment(Qt.AlignCenter)
        self.gray_title.setObjectName("PageDescription")
        self.right_panel.addWidget(self.gray_title)

        self.gray_preview = QLabel()
        self.gray_preview.setObjectName("ImagePreview")
        self.gray_preview.setAlignment(Qt.AlignCenter)
        self.gray_preview.setMinimumSize(220, 220)
        self.gray_preview.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.gray_preview.setText(
            "GRAY\n\n"
            "흑백 변환 결과 표시 영역"
        )
        self.right_panel.addWidget(self.gray_preview, 1)

        compare_frame_layout.addLayout(self.right_panel, 5)

        self.content_layout.addWidget(self.compare_frame, 1)

        # ==================================================
        # Bottom Navigation
        # ==================================================
        self.button_layout = QHBoxLayout()
        self.button_layout.setSpacing(12)

        self.prev_button = QPushButton("← 이전")
        self.prev_button.setObjectName("PrevButton")
        self.prev_button.setMinimumHeight(48)
        self.prev_button.setMinimumWidth(160)

        self.button_layout.addWidget(self.prev_button)
        self.button_layout.addStretch()

        self.preview_button = QPushButton("Gray Preview")
        self.preview_button.setMinimumHeight(48)
        self.preview_button.setMinimumWidth(180)
        self.preview_button.clicked.connect(
            self.show_preview
        )
        self.button_layout.addWidget(self.preview_button)
        self.button_layout.addStretch()

        self.next_button = QPushButton("다음 →")
        self.next_button.setObjectName("NextButton")
        self.next_button.setMinimumHeight(48)
        self.next_button.setMinimumWidth(160)

        self.button_layout.addWidget(self.next_button)

        self.content_layout.addLayout(self.button_layout)

        # ==================================================
        # Signal
        # ==================================================
        self.prev_button.clicked.connect(self.prev_clicked.emit)
        self.next_button.clicked.connect(self.next_clicked.emit)
    def show_preview(self):

        if self.gray_pixmap is None:
            return

        ImagePreviewDialog(
            self.gray_pixmap,
            self,
        ).exec()
    # ======================================================
    # Original Image 설정
    # ======================================================
    def set_original_image(self, pixmap):
        print("GrayPage set_original_image")
        print("GrayPage set_original_image")
        """
        ImagePage에서 선택한 원본 이미지를 표시
        """

        if pixmap is None:
            return

        self.original_pixmap = pixmap
        self.update_original_preview()

    # ======================================================
    # Gray Image 설정
    # ======================================================
    def set_gray_image(self, pixmap):
        print("GrayPage set_gray_image")
        """
        Gray 변환 결과 표시
        """
        print("GrayPage set_gray_image")
        if pixmap is None:
            return

        self.gray_pixmap = pixmap
        self.update_gray_preview()

    # ======================================================
    # Original Preview 다시 그리기
    # ======================================================
    def update_original_preview(self):
        """
        현재 QLabel 크기에 맞게 원본 이미지 리사이즈
        """

        if self.original_pixmap is None:
            return

        self.original_preview.setPixmap(
            self.original_pixmap.scaled(
                self.original_preview.size(),
                Qt.KeepAspectRatio,
                Qt.FastTransformation,
            )
        )

    # ======================================================
    # Gray Preview 다시 그리기
    # ======================================================
    def update_gray_preview(self):
        """
        현재 QLabel 크기에 맞게 흑백 이미지 리사이즈
        """

        if self.gray_pixmap is None:
            return

        self.gray_preview.setPixmap(
            self.gray_pixmap.scaled(
                self.gray_preview.size(),
                Qt.KeepAspectRatio,
                Qt.FastTransformation,
            )
        )

    # ======================================================
    # 창 크기 변경 시 이미지 자동 리사이즈
    # ======================================================
    def resizeEvent(self, event):
        """
        창 크기 변경 시 Preview 이미지도 같이 크기 변경
        """

        self.update_original_preview()
        self.update_gray_preview()

        super().resizeEvent(event)

    # ======================================================
    # 화면 초기화
    # ======================================================
    def clear_page(self):
        """
        다음 작업 시작 시 화면 초기화
        """

        self.original_pixmap = None
        self.gray_pixmap = None

        self.original_preview.clear()
        self.gray_preview.clear()

        self.original_preview.setText(
            "ORIGINAL\n\n"
            "이전 단계에서 선택한 이미지"
        )

        self.gray_preview.setText(
            "GRAY\n\n"
            "흑백 변환 결과 표시 영역"
        )

    # ======================================================
    # Header Status 변경
    # ======================================================
    def set_status(self, text: str):
        self.status_label.setText(f"STATUS  {text}")

    # ======================================================
    # Header Mode 변경
    # ======================================================
    def set_mode(self, text: str):
        self.mode_label.setText(f"MODE  {text}")

    # ======================================================
    # Servo 상태 변경
    # ======================================================
    def set_servo(self, state: str):
        self.servo_label.setText(f"SERVO  {state}")

    # ======================================================
    # 시간 표시 변경
    # ======================================================
    def set_time(self, text: str):
        self.time_label.setText(text)

    def clear_preview(self):
        """
        Gray Preview 초기화
        """

        self.original_pixmap = None
        self.gray_pixmap = None

        self.original_preview.clear()
        self.gray_preview.clear()

        self.original_preview.setText("NO IMAGE")
        self.gray_preview.setText("GRAY IMAGE")