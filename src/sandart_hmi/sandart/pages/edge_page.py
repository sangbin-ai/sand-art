"""
=========================================================
Sand Art Robot HMI
pages/edge_page.py
Edge Detection Page - Responsive Version
=========================================================
"""
from dialogs.image_preview_dialog import ImagePreviewDialog
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


class EdgePage(QWidget):

    prev_clicked = Signal()
    next_clicked = Signal()

    def __init__(self):
        super().__init__()

        # 창 크기 변경 시 다시 그리기 위해 원본 Pixmap 저장
        self.previous_pixmap = None
        self.edge_pixmap = None

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

        self.setting = QLabel("⚙")
        self.setting.setAlignment(Qt.AlignCenter)
        self.setting.setObjectName("ModeLabel")
        header_layout.addWidget(self.setting)

        self.root_layout.addWidget(self.header)

        # ==================================================
        # Body
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

        for i, text in enumerate(steps):
            label = QLabel(text)

            if i == 2:
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
        self.page_title = QLabel("EDGE DETECTION")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")
        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "흑백 이미지를 기반으로\n"
            "Edge를 추출하는 단계입니다."
        )
        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")
        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Compare Frame
        # ==================================================
        self.compare_frame = QFrame()
        self.compare_frame.setObjectName("ResultBox")
        self.compare_frame.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )

        compare_layout = QHBoxLayout()
        compare_layout.setContentsMargins(16, 16, 16, 16)
        compare_layout.setSpacing(16)
        self.compare_frame.setLayout(compare_layout)

        # ==================================================
        # Left Image : Histogram Image
        # ==================================================
        self.left_layout = QVBoxLayout()
        self.left_layout.setSpacing(10)

        self.left_title = QLabel("GRAY IMAGE")
        self.left_title.setAlignment(Qt.AlignCenter)
        self.left_title.setObjectName("PageDescription")
        self.left_layout.addWidget(self.left_title)

        self.previous_preview = QLabel()
        self.previous_preview = QLabel()
        self.previous_preview.setObjectName("ImagePreview")
        self.previous_preview.setAlignment(Qt.AlignCenter)
        self.previous_preview.setMinimumSize(220, 220)
        self.previous_preview.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.previous_preview.setText(
            "GRAY IMAGE\n\n"
            "이전 단계 결과"
        )

        self.left_layout.addWidget(self.previous_preview, 1)
        compare_layout.addLayout(self.left_layout, 5)

        # ==================================================
        # Arrow
        # ==================================================
        self.arrow = QLabel("→")
        self.arrow.setAlignment(Qt.AlignCenter)
        self.arrow.setObjectName("PageTitle")
        self.arrow.setMinimumWidth(40)
        compare_layout.addWidget(self.arrow)

        # ==================================================
        # Right Image : Edge Image
        # ==================================================
        self.right_layout = QVBoxLayout()
        self.right_layout.setSpacing(10)

        self.right_title = QLabel("EDGE IMAGE")
        self.right_title.setAlignment(Qt.AlignCenter)
        self.right_title.setObjectName("PageDescription")
        self.right_layout.addWidget(self.right_title)

        self.edge_preview = QLabel()
        self.edge_preview.setObjectName("ImagePreview")
        self.edge_preview.setAlignment(Qt.AlignCenter)
        self.edge_preview.setMinimumSize(220, 220)
        self.edge_preview.setSizePolicy(
            QSizePolicy.Expanding,
            QSizePolicy.Expanding,
        )
        self.edge_preview.setText(
            "EDGE\n\n"
            "Edge 결과 표시"
        )

        self.right_layout.addWidget(self.edge_preview, 1)
        compare_layout.addLayout(self.right_layout, 5)

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

        self.preview_button = QPushButton("Edge Preview")
        self.preview_button.clicked.connect(
            self.show_preview   
        )
        self.preview_button.setMinimumHeight(48)
        self.preview_button.setMinimumWidth(180)

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

        # Edge Preview 버튼은 추후 edge.py 연결 예정
    def show_preview(self):
        """
        Edge 이미지를 확대해서 표시
        """

        if self.edge_pixmap is None:
            return

        ImagePreviewDialog(
            self.edge_pixmap,
            self,
        ).exec()
    # ======================================================
    # Histogram Image 표시
    # ======================================================
    def set_previous_image(self, pixmap):
        print("EdgePage set_previous_image")
        """
        Gray 결과 이미지를 이전 단계 이미지로 표시
        """

        if pixmap is None:
            return

        self.previous_pixmap = pixmap
        self.update_previous_preview()

    # ======================================================
    # Edge Image 표시
    # ======================================================
    def set_edge_image(self, pixmap):
        """
        Edge 결과 이미지를 오른쪽 Preview에 표시합니다.
        """

        if pixmap is None:
            return

        self.edge_pixmap = pixmap
        self.update_edge_preview()

    # ======================================================
    # Histogram Preview 다시 그리기
    # ======================================================
    def update_previous_preview(self):
        """
        현재 QLabel 크기에 맞게 이전 단계 이미지를 다시 그림
        """

        if self.previous_pixmap is None:
            return

        self.previous_preview.setPixmap(
            self.previous_pixmap.scaled(
                self.previous_preview.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )
    # ======================================================
    # Edge Preview 다시 그리기
    # ======================================================
    def update_edge_preview(self):
        """
        현재 QLabel 크기에 맞게 Edge 이미지를 리사이즈합니다.
        """

        if self.edge_pixmap is None:
            return

        self.edge_preview.setPixmap(
            self.edge_pixmap.scaled(
                self.edge_preview.size(),
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    # ======================================================
    # 창 크기 변경 시 이미지 자동 리사이즈
    # ======================================================
    def resizeEvent(self, event):
        """
        창 크기 변경 시 Preview 이미지도 같이 크기 변경
        """

        self.update_previous_preview()
        self.update_edge_preview()

        super().resizeEvent(event)

    # ======================================================
    # 화면 초기화
    # ======================================================
    def clear_page(self):
        """
        다음 작업 시작 시 화면 초기화
        """

        self.previous_pixmap = None
        self.edge_pixmap = None

        self.previous_preview.clear()
        self.edge_preview.clear()

        self.previous_preview.setText(
            "gray_image\n\n"
            "이전 단계 결과"
        )

        self.edge_preview.setText(
            "EDGE\n\n"
            "Edge 결과 표시"
        )

    # ======================================================
    # Header Status 변경
    # ======================================================
    def set_status(self, text):
        self.status_label.setText(f"STATUS  {text}")

    # ======================================================
    # Header Mode 변경
    # ======================================================
    def set_mode(self, text):
        self.mode_label.setText(f"MODE  {text}")

    # ======================================================
    # Servo 변경
    # ======================================================
    def set_servo(self, state):
        self.servo_label.setText(f"SERVO  {state}")

    # ======================================================
    # Header 시간 변경
    # ======================================================
    def set_time(self, text):
        self.time_label.setText(text)
