"""Action detail dialog: shows full action info in a popup.

Touch-friendly dialog with large fonts and a prominent close button.
"""

from PyQt5 import QtCore, QtWidgets


class ActionDetailDialog(QtWidgets.QDialog):
    """Popup dialog showing complete Action information (8 fields).

    Opened from the left panel [📋详情] button.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Action 详细信息')
        self.setMinimumSize(420, 400)
        self.setWindowFlags(self.windowFlags()
                            | QtCore.Qt.WindowCloseButtonHint)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(12)

        # Info form
        form_group = QtWidgets.QGroupBox('完整信息')
        form_layout = QtWidgets.QFormLayout()
        form_group.setLayout(form_layout)

        self._type_label = QtWidgets.QLabel('-')
        self._job_label = QtWidgets.QLabel('-')
        self._order_label = QtWidgets.QLabel('-')
        self._step_label = QtWidgets.QLabel('-')
        self._vehicle_label = QtWidgets.QLabel('-')
        self._point_label = QtWidgets.QLabel('-')
        self._location_label = QtWidgets.QLabel('-')
        self._trace_label = QtWidgets.QLabel('-')

        for label_text, value_label in [
            ('类型:', self._type_label),
            ('Job ID:', self._job_label),
            ('订单:', self._order_label),
            ('步骤:', self._step_label),
            ('车辆:', self._vehicle_label),
            ('点位:', self._point_label),
            ('位置:', self._location_label),
            ('Trace:', self._trace_label),
        ]:
            label = QtWidgets.QLabel(label_text)
            label.setMinimumWidth(80)
            # Make value labels selectable for copy
            value_label.setTextInteractionFlags(
                QtCore.Qt.TextSelectableByMouse)
            form_layout.addRow(label, value_label)

        layout.addWidget(form_group)

        # Close button
        close_btn = QtWidgets.QPushButton('关 闭')
        close_btn.setMinimumHeight(48)
        close_btn.setStyleSheet(
            'QPushButton { background-color: #2196F3; color: white; '
            'font-size: 20px; padding: 12px 32px; border-radius: 8px; }'
            'QPushButton:hover { background-color: #1976D2; }')
        close_btn.clicked.connect(self.accept)
        layout.addWidget(close_btn, alignment=QtCore.Qt.AlignCenter)

    def set_goal_data(self, goal_dict: dict):
        """Update display with goal data."""
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

    def reset(self):
        """Clear all fields."""
        for lbl in (self._type_label, self._job_label, self._order_label,
                    self._step_label, self._vehicle_label, self._point_label,
                    self._location_label, self._trace_label):
            lbl.setText('-')
