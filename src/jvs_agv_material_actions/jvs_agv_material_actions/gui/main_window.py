"""Main window: left-right split layout with collapsible history.

Layout:
  Top:    Status bar (full width)
  Left:   ActionPanel (fixed ~250px) — badge, summary, buttons
  Right:  Material table (stretch) + collapsible history log
"""

from datetime import datetime

from PyQt5 import QtCore, QtWidgets

from .status_bar import StatusBar
from .action_panel import ActionPanel
from .material_table import MaterialTable


class MainWindow(QtWidgets.QMainWindow):
    """JVS-VGA控制台 main window — left-right split layout."""

    def __init__(self, bridge, action_server, parent=None):
        """
        Args:
            bridge: ActionBridge with pyqtSignal
            action_server: MaterialActionServer node
        """
        super().__init__(parent)
        self._bridge = bridge
        self._server = action_server
        self._active = False  # whether an action is currently in progress
        self._history_count = 0

        self.setWindowTitle('JVS-VGA控制台')
        self.setMinimumSize(900, 700)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root_layout = QtWidgets.QVBoxLayout(central)
        root_layout.setSpacing(6)
        root_layout.setContentsMargins(6, 6, 6, 6)

        # ---- Top: Status bar (full width) ----
        self._status_bar = StatusBar()
        root_layout.addWidget(self._status_bar)

        # ---- Middle: Left-right split ----
        body_layout = QtWidgets.QHBoxLayout()
        body_layout.setSpacing(6)

        # Left panel: action summary + buttons (fixed width)
        self._action_panel = ActionPanel()
        self._action_panel.setFixedWidth(250)
        body_layout.addWidget(self._action_panel)

        # Right area: material table + collapsible history
        right_layout = QtWidgets.QVBoxLayout()
        right_layout.setSpacing(6)

        # Material table (stretches to fill available space)
        self._material_table = MaterialTable()
        right_layout.addWidget(self._material_table, stretch=1)

        # Collapsible history log
        self._history_toggle_btn = QtWidgets.QPushButton('▶ 操作历史')
        self._history_toggle_btn.setCheckable(True)
        self._history_toggle_btn.setChecked(False)
        self._history_toggle_btn.setMinimumHeight(36)
        self._history_toggle_btn.setStyleSheet(
            'QPushButton { font-size: 16px; text-align: left; '
            'padding: 4px 12px; background-color: #f5f5f5; '
            'border: 1px solid #ddd; border-radius: 4px; }'
            'QPushButton:checked { background-color: #e8e8e8; }')
        self._history_toggle_btn.clicked.connect(self._toggle_history)
        right_layout.addWidget(self._history_toggle_btn)

        self._history_log = QtWidgets.QTextEdit()
        self._history_log.setReadOnly(True)
        self._history_log.setMaximumHeight(140)
        self._history_log.setVisible(False)
        right_layout.addWidget(self._history_log)

        body_layout.addLayout(right_layout, stretch=1)

        root_layout.addLayout(body_layout, stretch=1)

        # ---- Connect signals ----
        bridge.goal_received.connect(self._on_goal_received)
        bridge.feedback_updated.connect(self._on_feedback_updated)
        bridge.action_cancelled.connect(self._on_action_cancelled)
        bridge.action_finished.connect(self._on_action_finished)

        self._action_panel.submit_clicked.connect(self._on_submit)
        self._action_panel.fail_all_clicked.connect(self._on_fail_all)
        self._action_panel.cancel_clicked.connect(self._on_cancel)

        # Initial state
        self._status_bar.set_connected(True)
        self._log('系统就绪，等待 Action ...')

    # ---- History toggle ----

    def _toggle_history(self, checked):
        """Toggle history log visibility."""
        if checked:
            self._history_toggle_btn.setText('▼ 操作历史')
            self._history_log.setVisible(True)
        else:
            self._history_toggle_btn.setText('▶ 操作历史 (%d)' % self._history_count)
            self._history_log.setVisible(False)

    # ---- Slots for bridge signals (called from Qt main thread) ----

    @QtCore.pyqtSlot(str, dict)
    def _on_goal_received(self, action_type: str, goal_dict: dict):
        self._active = True
        self._action_panel.set_goal_data(goal_dict)
        self._material_table.set_materials(goal_dict.get('materials', []))
        self._log('%s Action 收到 — Job: %s, 物料数: %d'
                  % (action_type, goal_dict.get('job_id', '?'),
                     len(goal_dict.get('materials', []))))

    @QtCore.pyqtSlot(str, float, str)
    def _on_feedback_updated(self, phase: str, progress: float, message: str):
        self._status_bar.set_phase(phase)
        self._status_bar.set_progress(progress)

    @QtCore.pyqtSlot()
    def _on_action_cancelled(self):
        self._log('Action 被取消')

    @QtCore.pyqtSlot(str, str)
    def _on_action_finished(self, status: str, message: str):
        self._active = False
        self._action_panel.reset()
        self._material_table.reset()
        self._status_bar.reset()
        self._log('Action 完成: %s %s' % (status, message or ''))

    # ---- Slots for button clicks ----

    def _on_submit(self):
        """Operator clicked [提交]. Determine status from checkboxes."""
        checked_materials = self._material_table.get_checked_materials()
        if not checked_materials:
            return

        checked_count = sum(1 for c, _ in checked_materials if c)
        total = len(checked_materials)

        if checked_count == total:
            status = 'SUCCEEDED'
        elif checked_count > 0:
            status = 'PARTIAL'
        else:
            status = 'FAILED'

        message = self._action_panel.get_message()
        self._log('提交: %s (%d/%d 物料) %s'
                  % (status, checked_count, total,
                     '— ' + message if message else ''))
        self._server.submit_result(status, message, checked_materials)
        self._active = False

    def _on_fail_all(self):
        """Operator clicked [全部失败]."""
        checked_materials = self._material_table.get_checked_materials()
        message = self._action_panel.get_message() or '全部失败'
        self._log('提交: FAILED — %s' % message)
        self._server.submit_result('FAILED', message, checked_materials)
        self._active = False

    def _on_cancel(self):
        """Operator clicked [取消]."""
        message = self._action_panel.get_message()
        self._log('提交: CANCELLED %s'
                  % ('— ' + message if message else ''))
        self._server.cancel_result()
        self._active = False

    # ---- Helpers ----

    def _log(self, text: str):
        ts = datetime.now().strftime('%H:%M:%S')
        self._history_log.append('[%s] %s' % (ts, text))
        # Auto-scroll to bottom
        sb = self._history_log.verticalScrollBar()
        sb.setValue(sb.maximum())
        # Update toggle button text when collapsed
        self._history_count += 1
        if not self._history_toggle_btn.isChecked():
            self._history_toggle_btn.setText(
                '▶ 操作历史 (%d)' % self._history_count)
