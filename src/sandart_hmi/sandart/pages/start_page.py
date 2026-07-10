"""
=========================================================
Sand Art Robot HMI

pages/start_page.py

Start Menu Page
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
)


class StartPage(QWidget):
    """
    시작 화면

    아이콘 파일명:
    - assets/icons/sand_reset.png
    - assets/icons/image_select.png
    """

    sand_reset_clicked = Signal()
    image_select_clicked = Signal()

    def __init__(self):
        super().__init__()

        self.init_ui()

    def init_ui(self):
        # ==================================================
        # Root Layout
        # ==================================================
        self.root_layout = QVBoxLayout()
        # 전체 화면 바깥 여백을 줄여서 화면 잘림 방지
        self.root_layout.setContentsMargins(20, 20, 20, 20)

        # Header와 Content 사이 간격 축소
        self.root_layout.setSpacing(16)
        self.setLayout(self.root_layout)

        # ==================================================
        # Header
        # ==================================================
        self.header = QFrame()
        self.header.setObjectName("Header")

        header_layout = QHBoxLayout()
        # Header 내부 여백 축소
        header_layout.setContentsMargins(12, 8, 12, 8)

        # Header 안 버튼 간격 축소
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
        # Main Content Frame
        # ==================================================
        self.content = QFrame()
        self.content.setObjectName("ContentFrame")

        self.content_layout = QVBoxLayout()
        # 중앙 Content 여백 축소
        self.content_layout.setContentsMargins(24, 24, 24, 24)

        # 제목/설명/카드 사이 간격 축소
        self.content_layout.setSpacing(20)
        self.content.setLayout(self.content_layout)

        self.root_layout.addWidget(self.content, 1)

        # ==================================================
        # Main Title
        # ==================================================
        self.main_title = QLabel("작업을 선택하세요")
        self.main_title.setAlignment(Qt.AlignCenter)
        self.main_title.setObjectName("PageTitle")
        self.content_layout.addWidget(self.main_title)

        self.main_desc = QLabel(
            "샌드아트 작업을 시작하기 전에 모래를 정리하거나,\n"
            "새로운 이미지를 선택하여 변환 작업을 진행할 수 있습니다."
        )
        self.main_desc.setAlignment(Qt.AlignCenter)
        self.main_desc.setObjectName("PageDescription")
        self.content_layout.addWidget(self.main_desc)

        # ==================================================
        # Card Area
        # ==================================================
        self.card_area = QHBoxLayout()
        self.card_area.setSpacing(30)

        self.content_layout.addStretch()
        self.content_layout.addLayout(self.card_area)
        self.content_layout.addStretch()

        self.card_area.addStretch()

        # ==================================================
        # Sand Reset Card
        # ==================================================
        self.reset_card = QFrame()
        self.reset_card.setObjectName("MenuCard")
        # 카드가 너무 커서 화면이 잘리므로 크기 축소
        self.reset_card.setMinimumSize(300, 360)
        self.reset_card.setMaximumSize(360, 420)

        reset_layout = QVBoxLayout()
        reset_layout.setContentsMargins(28, 28, 28, 28)
        reset_layout.setSpacing(18)
        reset_layout.setAlignment(Qt.AlignCenter)
        self.reset_card.setLayout(reset_layout)

        self.reset_icon = QLabel()
        self.reset_icon.setAlignment(Qt.AlignCenter)
        self.reset_icon.setMinimumSize(110, 110)

        reset_pixmap = QPixmap("/home/rokey/Downloads/sandart/icon/sand_reset.png")
        if not reset_pixmap.isNull():
            self.reset_icon.setPixmap(
                reset_pixmap.scaled(
                    110,
                    110,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )
        else:
            self.reset_icon.setText("🏖")
            self.reset_icon.setObjectName("MenuIcon")

        reset_layout.addWidget(self.reset_icon)

        self.reset_title = QLabel("SAND RESET")
        self.reset_title.setAlignment(Qt.AlignCenter)
        self.reset_title.setObjectName("MenuTitle")
        reset_layout.addWidget(self.reset_title)

        self.reset_desc = QLabel(
            "새로운 그림을 그리기 전에\n"
            "모래 표면을 평탄하게 정리합니다."
        )
        self.reset_desc.setAlignment(Qt.AlignCenter)
        self.reset_desc.setObjectName("MenuDescription")
        reset_layout.addWidget(self.reset_desc)

        reset_layout.addStretch()

        self.reset_button = QPushButton("샌드 리셋 시작")
        self.reset_button.setMinimumHeight(50)
        reset_layout.addWidget(self.reset_button)

        self.card_area.addWidget(self.reset_card)
                # ==================================================
        # Image Select Card
        # ==================================================
        self.image_card = QFrame()
        self.image_card.setObjectName("MenuCard")
        # 카드가 너무 커서 화면이 잘리므로 크기 축소
        self.image_card.setMinimumSize(300, 360)
        self.image_card.setMaximumSize(360, 420)

        image_layout = QVBoxLayout()
        image_layout.setContentsMargins(28, 28, 28, 28)
        image_layout.setSpacing(18)
        image_layout.setAlignment(Qt.AlignCenter)
        self.image_card.setLayout(image_layout)

        self.image_icon = QLabel()
        self.image_icon.setAlignment(Qt.AlignCenter)
        self.image_icon.setMinimumSize(110, 110)

        image_pixmap = QPixmap("/home/rokey/Downloads/sandart/icon/image_select.png")

        if not image_pixmap.isNull():

            self.image_icon.setPixmap(
                image_pixmap.scaled(
                    110,
                    110,
                    Qt.KeepAspectRatio,
                    Qt.SmoothTransformation,
                )
            )

        else:

            self.image_icon.setText("🖼")
            self.image_icon.setObjectName("MenuIcon")

        image_layout.addWidget(self.image_icon)

        self.image_title = QLabel("IMAGE SELECT")

        self.image_title.setAlignment(Qt.AlignCenter)

        self.image_title.setObjectName("MenuTitle")

        image_layout.addWidget(self.image_title)

        self.image_desc = QLabel(
            "샌드아트로 변환할\n"
            "이미지를 선택합니다."
        )

        self.image_desc.setAlignment(Qt.AlignCenter)

        self.image_desc.setObjectName("MenuDescription")

        image_layout.addWidget(self.image_desc)

        image_layout.addStretch()

        self.image_button = QPushButton(
            "이미지 선택 시작"
        )

        self.image_button.setMinimumHeight(50)

        image_layout.addWidget(
            self.image_button
        )

        self.card_area.addWidget(
            self.image_card
        )

        self.card_area.addStretch()

        # ==================================================
        # Bottom Information
        # ==================================================

        self.version = QLabel(
            "Sand Art Robot HMI   Version 1.0"
        )

        self.version.setAlignment(
            Qt.AlignCenter
        )

        self.version.setObjectName(
            "PageDescription"
        )

        self.content_layout.addWidget(
            self.version
        )

        # ==================================================
        # Signal
        # ==================================================

        self.reset_button.clicked.connect(
            self.sand_reset_clicked.emit
        )

        self.image_button.clicked.connect(
            self.image_select_clicked.emit
        )

        # ==================================================
        # Object Name
        # ==================================================

        self.setObjectName("StartPage")

            # ======================================================
    # Header Status 변경
    # ======================================================

    def set_status(self, text: str):
        """
        Header Status 변경

        예)
        READY
        RUNNING
        COMPLETE
        """

        self.status_label.setText(
            f"STATUS  {text}"
        )

    # ======================================================
    # Header Mode 변경
    # ======================================================

    def set_mode(self, text: str):
        """
        Header Mode 변경

        예)

        AUTO

        MANUAL
        """

        self.mode_label.setText(
            f"MODE  {text}"
        )

    # ======================================================
    # Servo 상태
    # ======================================================

    def set_servo(self, state: str):
        """
        SERVO 표시

        예)

        ON

        OFF
        """

        self.servo_label.setText(
            f"SERVO  {state}"
        )

    # ======================================================
    # Header Time
    # ======================================================

    def set_time(self, text: str):
        """
        Header 시간 표시

        추후

        QTimer

        연결 예정
        """

        self.time_label.setText(text)

    # ======================================================
    # Sand Reset 버튼 활성
    # ======================================================

    def enable_reset_button(
        self,
        enable=True,
    ):

        self.reset_button.setEnabled(
            enable
        )

    # ======================================================
    # Image 버튼 활성
    # ======================================================

    def enable_image_button(
        self,
        enable=True,
    ):

        self.image_button.setEnabled(
            enable
        )

    # ======================================================
    # Sand Reset 아이콘 변경
    # ======================================================

    def set_reset_icon(
        self,
        icon_path,
    ):

        pixmap = QPixmap(icon_path)

        if pixmap.isNull():

            return

        self.reset_icon.setPixmap(

            pixmap.scaled(

                150,

                150,

                Qt.KeepAspectRatio,

                Qt.SmoothTransformation,

            )

        )

    # ======================================================
    # Image 아이콘 변경
    # ======================================================

    def set_image_icon(
        self,
        icon_path,
    ):

        pixmap = QPixmap(icon_path)

        if pixmap.isNull():

            return

        self.image_icon.setPixmap(

            pixmap.scaled(

                150,

                150,

                Qt.KeepAspectRatio,

                Qt.SmoothTransformation,

            )

        )

    # ======================================================
    # Page 초기화
    # ======================================================

    def reset_page(self):
        """
        시작 페이지 초기화

        추후

        작업 완료 후

        Home으로 돌아올 때

        사용 예정
        """

        self.set_status("READY")

        self.set_mode("AUTO")

        self.set_servo("ON")

        self.enable_reset_button(True)

        self.enable_image_button(True)

        self.set_time("14:25:36")