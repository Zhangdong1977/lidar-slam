"""Large touch-friendly checkbox delegate for QTableWidget.

The default Qt checkbox indicator is ~13x13px — too small for finger taps.
This delegate draws a 40x40px checkbox centered in each cell and makes
the entire cell area the tap target.
"""

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtGui import QColor, QPen, QBrush, QPainter
from PyQt5.QtCore import Qt

from .styles import TOUCH_CHECK_SIZE, COLOR_CHECK_BG, COLOR_CHECK_BORDER


class CheckBoxDelegate(QtWidgets.QStyledItemDelegate):
    """Touch-friendly checkbox delegate.

    - paint(): draws a large checkbox (40x40px) centered in the cell
    - editorEvent(): entire cell is the tap target — click anywhere toggles
    - sizeHint(): returns the checkbox size for layout
    """

    def __init__(self, parent=None):
        super().__init__(parent)

    def paint(self, painter, option, index):
        """Draw a large checkbox centered in the cell."""
        painter.save()

        # Determine checked state from model data
        checked = index.data(Qt.CheckStateRole) == Qt.Checked

        # Calculate centered position for the checkbox
        size = TOUCH_CHECK_SIZE
        x = option.rect.x() + (option.rect.width() - size) // 2
        y = option.rect.y() + (option.rect.height() - size) // 2
        rect = QtCore.QRect(x, y, size, size)

        # Draw background with selection highlight
        if option.state & QtWidgets.QStyle.State_Selected:
            painter.fillRect(option.rect, option.palette.highlight())

        # Draw checkbox box
        pen = QPen(QColor(COLOR_CHECK_BORDER), 2)
        painter.setPen(pen)
        painter.setBrush(Qt.white)
        painter.drawRoundedRect(rect, 4, 4)

        if checked:
            # Fill with green
            painter.setBrush(QBrush(QColor(COLOR_CHECK_BG)))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(rect, 4, 4)

            # Draw checkmark
            pen = QPen(Qt.white, 3)
            painter.setPen(pen)
            # Checkmark: two lines forming a "✓"
            margin = size * 0.25
            painter.drawLine(
                int(rect.left() + margin),
                int(rect.top() + size * 0.5),
                int(rect.left() + size * 0.45),
                int(rect.bottom() - margin),
            )
            painter.drawLine(
                int(rect.left() + size * 0.45),
                int(rect.bottom() - margin),
                int(rect.right() - margin),
                int(rect.top() + margin),
            )

        painter.restore()

    def editorEvent(self, event, model, option, index):
        """Toggle checkbox on mouse press / touch anywhere in the cell."""
        if event.type() in (QtCore.QEvent.MouseButtonPress,
                            QtCore.QEvent.MouseButtonDblClick,
                            QtCore.QEvent.TouchBegin):
            if event.button() in (Qt.LeftButton, Qt.NoButton):
                # Toggle check state
                current = index.data(Qt.CheckStateRole)
                new_state = Qt.Unchecked if current == Qt.Checked else Qt.Checked
                model.setData(index, new_state, Qt.CheckStateRole)
                return True
        return False

    def sizeHint(self, option, index):
        """Return size hint based on checkbox size."""
        return QtCore.QRect(0, 0, TOUCH_CHECK_SIZE, TOUCH_CHECK_SIZE).size()
