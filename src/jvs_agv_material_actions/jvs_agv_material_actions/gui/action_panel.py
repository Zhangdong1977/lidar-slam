"""Action info panel and result control buttons."""

from PyQt5 import QtCore, QtWidgets


class ActionPanel(QtWidgets.QWidget):
    """Displays current action info (type, job_id, order, etc.) and control buttons.

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

        main_layout = QtWidgets.QVBoxLayout(self)
        main_layout.setContentsMargins(4, 4, 4, 4)

        # --- Action Info Group ---
        info_group = QtWidgets.QGroupBox('Action 信息')
        info_layout = QtWidgets.QFormLayout()
        info_group.setLayout(info_layout)

        self._type_label = QtWidgets.QLabel('-')
        self._job_label = QtWidgets.QLabel('-')
        self._order_label = QtWidgets.QLabel('-')
        self._step_label = QtWidgets.QLabel('-')
        self._vehicle_label = QtWidgets.QLabel('-')
        self._point_label = QtWidgets.QLabel('-')
        self._location_label = QtWidgets.QLabel('-')
        self._trace_label = QtWidgets.QLabel('-')

        info_layout.addRow('类型:', self._type_label)
        info_layout.addRow('Job ID:', self._job_label)
        info_layout.addRow('订单:', self._order_label)
        info_layout.addRow('步骤:', self._step_label)
        info_layout.addRow('车辆:', self._vehicle_label)
        info_layout.addRow('点位:', self._point_label)
        info_layout.addRow('位置:', self._location_label)
        info_layout.addRow('Trace:', self._trace_label)

        main_layout.addWidget(info_group)

        # --- Control Buttons ---
        ctrl_group = QtWidgets.QGroupBox('操作')
        ctrl_layout = QtWidgets.QVBoxLayout()
        ctrl_group.setLayout(ctrl_layout)

        btn_row = QtWidgets.QHBoxLayout()
        self._submit_btn = QtWidgets.QPushButton('✔ 提交')
        self._submit_btn.setStyleSheet(
            'QPushButton { background-color: #4CAF50; color: white; '
            'font-size: 14px; padding: 8px 20px; }'
            'QPushButton:hover { background-color: #45a049; }'
            'QPushButton:disabled { background-color: #aaa; }')
        self._submit_btn.setEnabled(False)
        self._submit_btn.clicked.connect(self.submit_clicked.emit)
        btn_row.addWidget(self._submit_btn)

        self._fail_btn = QtWidgets.QPushButton('✘ 全部失败(FAILED)')
        self._fail_btn.setStyleSheet(
            'QPushButton { background-color: #f44336; color: white; '
            'font-size: 14px; padding: 8px 20px; }'
            'QPushButton:hover { background-color: #da190b; }'
            'QPushButton:disabled { background-color: #aaa; }')
        self._fail_btn.setEnabled(False)
        self._fail_btn.clicked.connect(self.fail_all_clicked.emit)
        btn_row.addWidget(self._fail_btn)

        self._cancel_btn = QtWidgets.QPushButton('⏹ 取消(CANCELLED)')
        self._cancel_btn.setStyleSheet(
            'QPushButton { background-color: #FF9800; color: white; '
            'font-size: 14px; padding: 8px 20px; }'
            'QPushButton:hover { background-color: #e68a00; }'
            'QPushButton:disabled { background-color: #aaa; }')
        self._cancel_btn.setEnabled(False)
        self._cancel_btn.clicked.connect(self.cancel_clicked.emit)
        btn_row.addWidget(self._cancel_btn)

        ctrl_layout.addLayout(btn_row)

        msg_row = QtWidgets.QHBoxLayout()
        msg_label = QtWidgets.QLabel('消息:')
        self._message_edit = QtWidgets.QLineEdit()
        self._message_edit.setPlaceholderText('可选：输入备注消息...')
        self._message_edit.setEnabled(False)
        msg_row.addWidget(msg_label)
        msg_row.addWidget(self._message_edit)
        ctrl_layout.addLayout(msg_row)

        main_layout.addWidget(ctrl_group)

    def set_goal_data(self, goal_dict: dict):
        """Update display with new goal data and enable buttons."""
        action_type = goal_dict.get('action_type', '?')
        type_text = '装货 (LOAD)' if action_type == 'LOAD' else '卸货 (UNLOAD)'
        self._type_label.setText(type_text)
        self._job_label.setText(str(goal_dict.get('job_id', '-')))
        self._order_label.setText(str(goal_dict.get('order_no',
                                                    goal_dict.get('order_id', '-'))))
        self._step_label.setText(str(goal_dict.get('step_no', '-')))
        self._vehicle_label.setText(str(goal_dict.get('vehicle_name', '-')))
        self._point_label.setText(str(goal_dict.get('point_id', '-')))
        self._location_label.setText(str(goal_dict.get('location_id', '-')))
        self._trace_label.setText(str(goal_dict.get('trace_id', '-')))

        self._submit_btn.setEnabled(True)
        self._fail_btn.setEnabled(True)
        self._cancel_btn.setEnabled(True)
        self._message_edit.setEnabled(True)

    def get_message(self) -> str:
        return self._message_edit.text().strip()

    def reset(self):
        """Clear info and disable buttons."""
        for lbl in (self._type_label, self._job_label, self._order_label,
                    self._step_label, self._vehicle_label, self._point_label,
                    self._location_label, self._trace_label):
            lbl.setText('-')
        self._submit_btn.setEnabled(False)
        self._fail_btn.setEnabled(False)
        self._cancel_btn.setEnabled(False)
        self._message_edit.setEnabled(False)
        self._message_edit.clear()
