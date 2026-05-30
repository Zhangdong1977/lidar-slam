"""Main window: assembles all panels and connects signals."""

from datetime import datetime

from PyQt5 import QtCore, QtWidgets

from .status_bar import StatusBar
from .action_panel import ActionPanel
from .material_table import MaterialTable


class MainWindow(QtWidgets.QMainWindow):
    """JVS-VGA控制台 main window."""

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

        self.setWindowTitle('JVS-VGA控制台')
        self.setMinimumSize(700, 600)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setSpacing(6)

        # Status bar
        self._status_bar = StatusBar()
        layout.addWidget(self._status_bar)

        # Action info + buttons
        self._action_panel = ActionPanel()
        layout.addWidget(self._action_panel)

        # Material table
        mat_group = QtWidgets.QGroupBox('物料清单')
        mat_layout = QtWidgets.QVBoxLayout()
        mat_group.setLayout(mat_layout)
        self._material_table = MaterialTable()
        mat_layout.addWidget(self._material_table)
        layout.addWidget(mat_group)

        # History log
        history_group = QtWidgets.QGroupBox('操作历史')
        history_layout = QtWidgets.QVBoxLayout()
        history_group.setLayout(history_layout)
        self._history_log = QtWidgets.QTextEdit()
        self._history_log.setReadOnly(True)
        self._history_log.setMaximumHeight(120)
        history_layout.addWidget(self._history_log)
        layout.addWidget(history_group)

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
