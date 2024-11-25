from PySide6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QProgressBar,
    QLabel,
)
from PySide6.QtCore import Qt


class StyledProgressBar(QProgressBar):
    def setValue(self, value, invert):
        if value == 0:
            self.setStyleSheet("")
        if invert:
            self.setInvertedAppearance(True)
            self.setStyleSheet(
                """
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                color: black;
                background-color: #f0f0f0;
                font: bold;
            }
            QProgressBar::chunk {
                background-color: #FF0000;
            }
            QProgressBar::chunk:horizontal {
                margin: 0px;
                width: 1px;
                background-color: #FF0000;
            }
        """
            )
        else:
            self.setInvertedAppearance(False)
            self.setStyleSheet(
                """
            QProgressBar {
                border: 2px solid grey;
                border-radius: 5px;
                text-align: center;
                color: black;
                background-color: #f0f0f0;
                font: bold;
            }
            QProgressBar::chunk {
                background-color: #0078d7;
            }
            QProgressBar::chunk:horizontal {
                margin: 0px;
                width: 1px;
                background-color: #0078d7;
            }
        """
            )

        super().setValue(value)


class Gauge(QWidget):
    def __init__(self, value_max, value_min, units):
        super().__init__()

        self.value_max = value_max
        self.value_min = value_min
        self.units = units

        self.initUI()

    def initUI(self):
        layout = QHBoxLayout()

        self.progress_bar = StyledProgressBar()
        self.progress_bar.setOrientation(Qt.Horizontal)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        layout.addWidget(self.progress_bar)

        self.value_label = QLabel(f"0.0 {self.units}", self)
        self.value_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.value_label)
        self.setLayout(layout)

    def update_value(self, value):
        rounded_value = round(float(value), 2)
        self.value_label.setText(f"{rounded_value} {self.units}")
        if rounded_value >= 0:
            bar_value = int((rounded_value / self.value_max) * 100)
        else:
            bar_value = int(-(rounded_value / self.value_min) * 100)

        if bar_value >= 0 and bar_value > 100:
            self.progress_bar.setValue(100, invert=False)
        if bar_value >= 0 and bar_value <= 100:
            self.progress_bar.setValue(int(bar_value), invert=False)
        if bar_value < 0 and bar_value > 100:
            self.progress_bar.setValue(int(100), invert=True)
        if bar_value < 0 and bar_value <= 100:
            self.progress_bar.setValue(int(-bar_value), invert=True)
