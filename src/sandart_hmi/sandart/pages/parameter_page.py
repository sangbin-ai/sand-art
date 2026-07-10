"""
=========================================================
Sand Art Robot HMI

pages/parameter_page.py

Parameter Setting Page
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
    QSlider,
    QSpinBox,
    QDoubleSpinBox,
)


class ParameterPage(QWidget):

    prev_clicked = Signal()
    next_clicked = Signal()

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

            if i == 4:

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
        self.content_layout.setSpacing(26)

        self.content.setLayout(self.content_layout)

        self.body_layout.addWidget(self.content,1)

        # ==================================================
        # Title
        # ==================================================

        self.page_title = QLabel("DRAWING PARAMETERS")
        self.page_title.setAlignment(Qt.AlignCenter)
        self.page_title.setObjectName("PageTitle")

        self.content_layout.addWidget(self.page_title)

        self.page_desc = QLabel(
            "샌드아트 그리기 파라미터를 설정합니다.\n"
            "Force는 실제 로봇에 적용되는 힘(N)입니다."
        )

        self.page_desc.setAlignment(Qt.AlignCenter)
        self.page_desc.setObjectName("PageDescription")

        self.content_layout.addWidget(self.page_desc)

        # ==================================================
        # Parameter Frame
        # ==================================================

        self.parameter_frame = QFrame()
        self.parameter_frame.setObjectName("ResultBox")

        parameter_layout = QVBoxLayout()
        parameter_layout.setContentsMargins(40,40,40,40)
        parameter_layout.setSpacing(32)

        self.parameter_frame.setLayout(parameter_layout)

        # ==================================================
        # Drawing Speed
        # ==================================================

        speed_layout = QHBoxLayout()

        self.speed_label = QLabel("Drawing Speed")
        self.speed_label.setMinimumWidth(180)

        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(20,120)
        self.speed_slider.setValue(60)

        self.speed_value = QSpinBox()
        self.speed_value.setRange(20,120)
        self.speed_value.setValue(60)
        self.speed_value.setMinimumWidth(90)

        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_layout.addWidget(self.speed_value)

        parameter_layout.addLayout(speed_layout)
        # ==================================================
        # Robot Speed
        # ==================================================

        robot_speed_layout = QHBoxLayout()

        self.robot_speed_label = QLabel("Robot Speed")
        self.robot_speed_label.setMinimumWidth(180)

        self.robot_speed_slider = QSlider(Qt.Horizontal)
        self.robot_speed_slider.setRange(50, 200)
        self.robot_speed_slider.setValue(110)

        self.robot_speed_value = QSpinBox()
        self.robot_speed_value.setRange(50, 200)
        self.robot_speed_value.setValue(110)
        self.robot_speed_value.setMinimumWidth(90)

        robot_speed_layout.addWidget(self.robot_speed_label)
        robot_speed_layout.addWidget(self.robot_speed_slider)
        robot_speed_layout.addWidget(self.robot_speed_value)

        parameter_layout.addLayout(robot_speed_layout)
        # ==================================================
        # Draw Force
        # ==================================================

        pressure_layout = QHBoxLayout()

        self.pressure_label = QLabel("Drawing Force (N)")
        self.pressure_label.setMinimumWidth(180)

        self.pressure_slider = QSlider(Qt.Horizontal)
        self.pressure_slider.setRange(1,100)
        self.pressure_slider.setValue(15)

        self.pressure_value = QDoubleSpinBox()
        self.pressure_value.setRange(0.1, 10.0)
        self.pressure_value.setSingleStep(0.1)
        self.pressure_value.setDecimals(1)
        self.pressure_value.setSuffix(" N")
        self.pressure_value.setValue(1.0)
        self.pressure_value.setMinimumWidth(100)
        pressure_layout.addWidget(self.pressure_label)
        pressure_layout.addWidget(self.pressure_slider)
        pressure_layout.addWidget(self.pressure_value)

        parameter_layout.addLayout(pressure_layout)

        self.content_layout.addWidget(
            self.parameter_frame,
            1
        )

                # ==================================================
        # Sampling
        # ==================================================

        sampling_layout = QHBoxLayout()

        self.sampling_label = QLabel("Sampling")
        self.sampling_label.setMinimumWidth(180)

        self.sampling_slider = QSlider(Qt.Horizontal)
        self.sampling_slider.setRange(5,20)
        self.sampling_slider.setValue(10)

        self.sampling_value = QSpinBox()
        self.sampling_value.setRange(5,20)
        self.sampling_value.setValue(10)
        self.sampling_value.setMinimumWidth(90)

        sampling_layout.addWidget(self.sampling_label)
        sampling_layout.addWidget(self.sampling_slider)
        sampling_layout.addWidget(self.sampling_value)

        parameter_layout.addLayout(sampling_layout)

        # ==================================================
        # Slider <-> SpinBox Sync
        # ==================================================

        self.speed_slider.valueChanged.connect(
            self.speed_value.setValue
        )

        self.speed_value.valueChanged.connect(
            self.speed_slider.setValue
        )

        self.robot_speed_slider.valueChanged.connect(
            self.robot_speed_value.setValue
        )

        self.robot_speed_value.valueChanged.connect(
            self.robot_speed_slider.setValue
        )

        self.pressure_slider.valueChanged.connect(
            lambda v: self.pressure_value.setValue(v / 10.0)
        )

        self.pressure_value.valueChanged.connect(
            lambda v: self.pressure_slider.setValue(int(v * 10))
        )

        self.sampling_slider.valueChanged.connect(
            self.sampling_value.setValue
        )

        self.sampling_value.valueChanged.connect(
            self.sampling_slider.setValue
        )

        # ==================================================
        # Bottom Navigation
        # ==================================================

        self.button_layout = QHBoxLayout()
        self.button_layout.setSpacing(20)

        self.prev_button = QPushButton("← 이전")
        self.prev_button.setObjectName("PrevButton")
        self.prev_button.setMinimumHeight(56)
        self.prev_button.setMinimumWidth(180)

        self.button_layout.addWidget(self.prev_button)

        self.button_layout.addStretch()

        self.next_button = QPushButton("다음 →")
        self.next_button.setObjectName("NextButton")
        self.next_button.setMinimumHeight(56)
        self.next_button.setMinimumWidth(180)

        self.button_layout.addWidget(self.next_button)

        self.content_layout.addLayout(self.button_layout)

        # ==================================================
        # Signal
        # ==================================================

        self.prev_button.clicked.connect(
            self.prev_clicked.emit
        )

        self.next_button.clicked.connect(
            self.next_clicked.emit
        )

    # ======================================================
    # Parameter 반환
    # ======================================================

    def get_parameter(self):

        return {

            "draw_speed": self.speed_slider.value(),

            "robot_speed": self.robot_speed_slider.value(),

            "force": self.pressure_slider.value() / 10.0,

            "sampling": self.sampling_slider.value() / 10.0,

        }

    # ======================================================
    # Parameter 초기화
    # ======================================================

    def reset_parameter(self):

        self.speed_slider.setValue(50)
        self.robot_speed_slider.setValue(50)
        self.pressure_slider.setValue(15)
        self.sampling_slider.setValue(50)

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