"""
=========================================================
Image Preview Dialog
공통 확대 미리보기
=========================================================
"""

from PySide6.QtCore import Qt
from PySide6.QtGui import QPixmap
from PySide6.QtWidgets import (
    QDialog,
    QLabel,
    QVBoxLayout,
    QScrollArea,
)


class ImagePreviewDialog(QDialog):

    def __init__(self, pixmap: QPixmap, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Image Preview")
        self.resize(1200, 900)

        self.original_pixmap = pixmap
        self.scale_factor = 1.0

        layout = QVBoxLayout(self)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)

        self.label = QLabel()
        self.label.setAlignment(Qt.AlignCenter)

        self.scroll.setWidget(self.label)

        layout.addWidget(self.scroll)

        self.update_image()

    def update_image(self):

        if self.original_pixmap is None:
            return

        w = int(self.original_pixmap.width() * self.scale_factor)
        h = int(self.original_pixmap.height() * self.scale_factor)

        self.label.setPixmap(
            self.original_pixmap.scaled(
                w,
                h,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        )

    def wheelEvent(self, event):

        if event.angleDelta().y() > 0:
            self.scale_factor *= 1.15
        else:
            self.scale_factor /= 1.15

        if self.scale_factor < 0.1:
            self.scale_factor = 0.1

        if self.scale_factor > 10:
            self.scale_factor = 10

        self.update_image()

    def mouseDoubleClickEvent(self, event):

        self.scale_factor = 1.0
        self.update_image()

    def keyPressEvent(self, event):

        if event.key() == Qt.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)    