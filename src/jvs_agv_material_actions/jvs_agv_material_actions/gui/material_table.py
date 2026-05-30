"""Material table widget with checkboxes for item selection."""

from PyQt5 import QtCore, QtWidgets

from .checkbox_delegate import CheckBoxDelegate
from .styles import TOUCH_ROW_HEIGHT


class MaterialTable(QtWidgets.QWidget):
    """Table showing material list with checkboxes.

    Each row has a checkbox column. The operator checks off items
    that were successfully loaded/unloaded.

    Methods:
        set_materials(materials): populate the table from goal data
        get_checked_materials(): return list of (checked, material_dict)
        select_all() / deselect_all(): convenience for testing
        reset(): clear the table
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Toolbar: select all / deselect all
        toolbar = QtWidgets.QHBoxLayout()
        self._select_all_btn = QtWidgets.QPushButton('全选')
        self._select_all_btn.setMinimumWidth(100)
        self._select_all_btn.clicked.connect(self.select_all)
        toolbar.addWidget(self._select_all_btn)

        self._deselect_all_btn = QtWidgets.QPushButton('全不选')
        self._deselect_all_btn.setMinimumWidth(100)
        self._deselect_all_btn.clicked.connect(self.deselect_all)
        toolbar.addWidget(self._deselect_all_btn)

        toolbar.addStretch()
        layout.addLayout(toolbar)

        # Table
        self._table = QtWidgets.QTableWidget()
        self._table.setColumnCount(7)
        self._table.setHorizontalHeaderLabels(
            ['✓', '编码', '名称', '数量', '单位', '批次', '容器'])
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self._table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._table.verticalHeader().setVisible(False)

        # Touch-friendly checkbox delegate for column 0
        self._check_delegate = CheckBoxDelegate(self._table)
        self._table.setItemDelegateForColumn(0, self._check_delegate)

        # Checkbox column fixed width
        self._table.setColumnWidth(0, 60)

        layout.addWidget(self._table)

        self._materials = []  # original list of material dicts

    def set_materials(self, materials: list):
        """Populate table from goal materials list."""
        self._materials = list(materials)
        self._table.setRowCount(len(materials))

        for row, mat in enumerate(materials):
            # Checkbox item
            check_item = QtWidgets.QTableWidgetItem()
            check_item.setFlags(QtCore.Qt.ItemIsUserCheckable
                                | QtCore.Qt.ItemIsEnabled)
            check_item.setCheckState(QtCore.Qt.Unchecked)
            check_item.setTextAlignment(QtCore.Qt.AlignCenter)
            self._table.setItem(row, 0, check_item)

            # Data columns
            self._table.setItem(row, 1,
                                QtWidgets.QTableWidgetItem(
                                    mat.get('material_code', '')))
            self._table.setItem(row, 2,
                                QtWidgets.QTableWidgetItem(
                                    mat.get('material_name', '')))
            self._table.setItem(row, 3,
                                QtWidgets.QTableWidgetItem(
                                    str(mat.get('quantity', 0))))
            self._table.setItem(row, 4,
                                QtWidgets.QTableWidgetItem(
                                    mat.get('unit', '')))
            self._table.setItem(row, 5,
                                QtWidgets.QTableWidgetItem(
                                    mat.get('batch_no', '')))
            self._table.setItem(row, 6,
                                QtWidgets.QTableWidgetItem(
                                    mat.get('container_code', '')))

            # Touch-friendly row height
            self._table.setRowHeight(row, TOUCH_ROW_HEIGHT)

        self._table.resizeColumnsToContents()
        # Restore checkbox column width after resizeColumnsToContents
        self._table.setColumnWidth(0, 60)

    def get_checked_materials(self) -> list:
        """Return list of (checked: bool, material_dict)."""
        result = []
        for row in range(self._table.rowCount()):
            item = self._table.item(row, 0)
            checked = (item is not None
                       and item.checkState() == QtCore.Qt.Checked)
            result.append((checked, self._materials[row]))
        return result

    def select_all(self):
        for row in range(self._table.rowCount()):
            item = self._table.item(row, 0)
            if item is not None:
                item.setCheckState(QtCore.Qt.Checked)

    def deselect_all(self):
        for row in range(self._table.rowCount()):
            item = self._table.item(row, 0)
            if item is not None:
                item.setCheckState(QtCore.Qt.Unchecked)

    def reset(self):
        self._table.setRowCount(0)
        self._materials.clear()
