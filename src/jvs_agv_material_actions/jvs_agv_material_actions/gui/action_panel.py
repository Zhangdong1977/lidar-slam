"""Left-side action panel: summary badge, key info, detail button, action buttons.

Refactored from the original vertical layout to a compact left sidebar:
  - Action type badge (large colored label)
  - 4 key fields: Job, Order, Point, Location
  - [📋详情] button to open detail dialog
  - 3 action buttons stacked vertically
  - Message input at bottom
"""

from PyQt5 import QtCore, QtWidgets

from .styles import (
    COLOR_PRIMARY, COLOR_PRIMARY_HOVER,
    COLOR_DANGER, COLOR_DANGER_HOVER,
    COLOR_WARNING, COLOR_WARNING_HOVER,
    COLOR_DISABLED, TOUCH_BTN_FONT_PX,
)
from .action_detail_dialog import ActionDetailDialog


class ActionPanel(QtWidgets.QWidget):
    """Left sidebar panel: action summary + control buttons.

    Signals:
        submit_clicked: operator clicked [提交]
        fail_all_clicked: operator clicked [全部失败]
        cancel_clicked: operator clicked [取消]
    """

    submit_clicked = QtCore.pyqtSignal()
    fail_all_clicked = QtCore.pyqtSignal()
    cancel_clicked = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(6, 6, 6, 6)
        layout.setSpacing(8)

        # --- Action Type Badge ---
        self._badge = QtWidgets.QLabel('等待 Action')
        self._badge.setAlignment(QtCore.Qt.AlignCenter)
        self._badge.setStyleSheet(
            'QLabel { font-size: 22px; font-weight: bold; '
            'color: #666; background-color: #e0e0e0; '
            'border-radius: 8px; padding: 12px 8px; }')
        self._badge.setMinimumHeight(52)
        layout.addWidget(self._badge)

        # --- Key Info Summary (4 fields) ---
        info_widget = QtWidgets.QWidget()
        info_layout = QtWidgets.QVBoxLayout(info_widget)
        info_layout.setContentsMargins(0, 4, 0, 4)
        info_layout.setSpacing(2)

        self._job_label = QtWidgets.QLabel('Job: -')
        self._order_label = QtWidgets.QLabel('订单: -')
        self._point_label = QtWidgets.QLabel('点位: -')
        self._location_label = QtWidgets.QLabel('位置: -')

        for lbl in (self._job_label, self._order_label,
                    self._point_label, self._location_label):
            lbl.setWordWrap(True)
            info_layout.addWidget(lbl)

        layout.addWidget(info_widget)

        # --- Detail Button ---
        self._detail_btn = QtWidgets.QPushButton('📋 详情')
        self._detail_btn.setMinimumHeight(40)
        self._detail_btn.setStyleSheet(
            'QPushButton { font-size: 16px; padding: 6px 12px; '
            'background-color: #e3f2fd; color: #1565C0; '
            'border: 1px solid #90CAF9; border-radius: 6px; }'
            'QPushButton:hover { background-color: #bbdefb; }'
            'QPushButton:disabled { background-color: #f5f5f5; color: #bbb; '
            'border-color: #ddd; }')
        self._detail_btn.setEnabled(False)
        self._detail_btn.clicked.connect(self._show_detail)
        layout.addWidget(self._detail_btn)

        layout.addSpacing(8)

        # --- Separator ---
        line = QtWidgets.QFrame()
        line.setFrameShape(QtWidgets.QFrame.HLine)
        line.setFrameShadow(QtWidgets.QFrame.Sunken)
        layout.addWidget(line)

        layout.addSpacing(4)

        # --- Action Buttons (stacked vertically) ---
        self._submit_btn = QtWidgets.QPushButton('✔ 提 交')
        self._submit_btn.setStyleSheet(
            'QPushButton { background-color: %s; color: white; '
            'font-size: %dpx; padding: 14px 16px; }'
            'QPushButton:hover { background-color: %s; }'
            'QPushButton:disabled { background-color: %s; }'
            % (COLOR_PRIMARY, TOUCH_BTN_FONT_PX,
               COLOR_PRIMARY_HOVER, COLOR_DISABLED))
        self._submit_btn.setEnabled(False)
        self._submit_btn.setMinimumHeight(56)
        self._submit_btn.clicked.connect(self.submit_clicked.emit)
        layout.addWidget(self._submit_btn)

        self._fail_btn = QtWidgets.QPushButton('✘ 全部失败')
        self._fail_btn.setStyleSheet(
            'QPushButton { background-color: %s; color: white; '
            'font-size: %dpx; padding: 14px 16px; }'
            'QPushButton:hover { background-color: %s; }'
            'QPushButton:disabled { background-color: %s; }'
            % (COLOR_DANGER, TOUCH_BTN_FONT_PX,
               COLOR_DANGER_HOVER, COLOR_DISABLED))
        self._fail_btn.setEnabled(False)
        self._fail_btn.setMinimumHeight(56)
        self._fail_btn.clicked.connect(self.fail_all_clicked.emit)
        layout.addWidget(self._fail_btn)

        self._cancel_btn = QtWidgets.QPushButton('⏹ 取 消')
        self._cancel_btn.setStyleSheet(
            'QPushButton { background-color: %s; color: white; '
            'font-size: %dpx; padding: 14px 16px; }'
            'QPushButton:hover { background-color: %s; }'
            'QPushButton:disabled { background-color: %s; }'
            % (COLOR_WARNING, TOUCH_BTN_FONT_PX,
               COLOR_WARNING_HOVER, COLOR_DISABLED))
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.setMinimumHeight(56)
        self._cancel_btn.clicked.connect(self.cancel_clicked.emit)
        layout.addWidget(self._cancel_btn)

        # --- Message Input ---
        msg_label = QtWidgets.QLabel('消息:')
        layout.addWidget(msg_label)

        self._message_edit = QtWidgets.QLineEdit()
        self._message_edit.setPlaceholderText('可选备注...')
        self._message_edit.setEnabled(False)
        layout.addWidget(self._message_edit)

        # Push everything to top
        layout.addStretch()

        # --- Detail Dialog ---
        self._detail_dialog = None
        self._current_goal = {}

    def _show_detail(self):
        """Open the detail dialog."""
        if self._detail_dialog is None:
            self._detail_dialog = ActionDetailDialog(self.window())
        self._detail_dialog.set_goal_data(self._current_goal)
        self._detail_dialog.exec_()

    def set_goal_data(self, goal_dict: dict):
        """Update display with new goal data and enable buttons."""
        self._current_goal = dict(goal_dict)

        action_type = goal_dict.get('action_type', '?')
        if action_type == 'LOAD':
            self._badge.setText('装货 LOAD')
            self._badge.setStyleSheet(
                'QLabel { font-size: 22px; font-weight: bold; '
                'color: white; background-color: #4CAF50; '
                'border-radius: 8px; padding: 12px 8px; }')
        else:
            self._badge.setText('卸货 UNLOAD')
            self._badge.setStyleSheet(
                'QLabel { font-size: 22px; font-weight: bold; '
                'color: white; background-color: #2196F3; '
                'border-radius: 8px; padding: 12px 8px; }')

        self._job_label.setText('Job: %s' % goal_dict.get('job_id', '-'))
        self._order_label.setText('订单: %s' % goal_dict.get('order_no',
                                                               goal_dict.get('order_id', '-')))
        self._point_label.setText('点位: %s' % goal_dict.get('point_id', '-'))
        self._location_label.setText('位置: %s' % goal_dict.get('location_id', '-'))

        self._submit_btn.setEnabled(True)
        self._fail_btn.setEnabled(True)
        self._cancel_btn.setEnabled(True)
        self._message_edit.setEnabled(True)
        self._detail_btn.setEnabled(True)

        # Update detail dialog if already open
        if self._detail_dialog is not None:
            self._detail_dialog.set_goal_data(goal_dict)

    def get_message(self) -> str:
        return self._message_edit.text().strip()

    def reset(self):
        """Clear info and disable buttons."""
        self._current_goal = {}

        self._badge.setText('等待 Action')
        self._badge.setStyleSheet(
            'QLabel { font-size: 22px; font-weight: bold; '
            'color: #666; background-color: #e0e0e0; '
            'border-radius: 8px; padding: 12px 8px; }')

        for lbl in (self._job_label, self._order_label,
                    self._point_label, self._location_label):
            prefix = lbl.text().split(':')[0]
            lbl.setText('%s: -' % prefix)

        self._submit_btn.setEnabled(False)
        self._fail_btn.setEnabled(False)
        self._cancel_btn.setEnabled(False)
        self._message_edit.setEnabled(False)
        self._message_edit.clear()
        self._detail_btn.setEnabled(False)

        if self._detail_dialog is not None:
            self._detail_dialog.reset()
