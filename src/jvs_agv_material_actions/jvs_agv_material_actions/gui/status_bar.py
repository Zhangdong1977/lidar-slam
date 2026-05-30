"""Status bar widget showing connection status, phase and progress."""

from PyQt5 import QtCore, QtWidgets


class StatusBar(QtWidgets.QWidget):
    """Top-level status bar: connection indicator, phase label, progress bar."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        self._conn_label = QtWidgets.QLabel('连接: 等待')
        self._conn_label.setStyleSheet(
            'font-size: 18px; font-weight: bold; color: orange;')
        layout.addWidget(self._conn_label)

        layout.addSpacing(20)

        self._phase_label = QtWidgets.QLabel('阶段: IDLE')
        self._phase_label.setStyleSheet(
            'font-size: 18px; font-weight: bold;')
        layout.addWidget(self._phase_label)

        layout.addSpacing(20)

        self._progress_bar = QtWidgets.QProgressBar()
        self._progress_bar.setRange(0, 100)
        self._progress_bar.setValue(0)
        self._progress_bar.setMinimumWidth(300)
        layout.addWidget(self._progress_bar)

        layout.addStretch()

    def set_connected(self, connected: bool):
        if connected:
            self._conn_label.setText('连接: OK')
            self._conn_label.setStyleSheet(
                'font-size: 18px; font-weight: bold; color: green;')
        else:
            self._conn_label.setText('连接: 断开')
            self._conn_label.setStyleSheet(
                'font-size: 18px; font-weight: bold; color: red;')

    def set_phase(self, phase: str):
        self._phase_label.setText('阶段: %s' % phase)

    def set_progress(self, progress: float):
        self._progress_bar.setValue(int(progress * 100))

    def reset(self):
        self.set_connected(True)
        self._phase_label.setText('阶段: IDLE')
        self._progress_bar.setValue(0)
