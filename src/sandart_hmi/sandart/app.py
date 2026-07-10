"""
=========================================================
Sand Art Robot HMI

app.py

프로그램 실행 파일
=========================================================
"""

import sys
from PySide6.QtCore import Qt, QProcess, Signal, QTimer
from PySide6.QtWidgets import QApplication

from ui.main_window import MainWindow


def main():
    """
    프로그램 시작 함수
    """

    app = QApplication(sys.argv)

    window = MainWindow()
    window.show()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()