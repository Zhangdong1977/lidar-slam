"""Global stylesheet and size constants for touch-friendly UI."""

# Touch-screen size constants (based on Material Design / Apple HIG)
TOUCH_MIN_HEIGHT = 48       # Button minimum height (dp)
TOUCH_ROW_HEIGHT = 48       # Table row height (px)
TOUCH_CHECK_SIZE = 40       # Checkbox draw size (px)
TOUCH_FONT_PX = 18          # Base font size (px, ~14pt)
TOUCH_BTN_FONT_PX = 22      # Button font size (px, ~16pt)
TOUCH_HEADER_FONT_PX = 20   # GroupBox title font size (px, ~15pt)
LEFT_PANEL_WIDTH = 250      # Left sidebar fixed width (px)

# Colors
COLOR_PRIMARY = '#4CAF50'       # Green (submit)
COLOR_PRIMARY_HOVER = '#45a049'
COLOR_DANGER = '#f44336'        # Red (fail)
COLOR_DANGER_HOVER = '#da190b'
COLOR_WARNING = '#FF9800'       # Orange (cancel)
COLOR_WARNING_HOVER = '#e68a00'
COLOR_DISABLED = '#aaaaaa'      # Gray (disabled)
COLOR_CHECK_BG = '#4CAF50'      # Checked checkbox fill
COLOR_CHECK_BORDER = '#999999'  # Unchecked checkbox border

# Global QSS stylesheet for touch-friendly sizing
GLOBAL_STYLESHEET = """
/* ---------- Base ---------- */
* {
    font-size: 18px;
}

/* ---------- GroupBox ---------- */
QGroupBox {
    font-size: 20px;
    font-weight: bold;
    padding-top: 24px;
    border: 1px solid #cccccc;
    border-radius: 6px;
    margin-top: 8px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
}

/* ---------- Buttons ---------- */
QPushButton {
    min-height: 48px;
    font-size: 22px;
    border-radius: 8px;
    padding: 12px 24px;
}

/* ---------- Line Edit ---------- */
QLineEdit {
    min-height: 40px;
    font-size: 18px;
    padding: 4px 8px;
}

/* ---------- Progress Bar ---------- */
QProgressBar {
    min-height: 28px;
    text-align: center;
    font-size: 14px;
    border-radius: 4px;
}

/* ---------- Table ---------- */
QTableWidget {
    font-size: 18px;
    gridline-color: #dddddd;
}
QTableWidget::item {
    padding: 4px 8px;
}

/* ---------- Header ---------- */
QHeaderView::section {
    font-size: 18px;
    font-weight: bold;
    padding: 8px 4px;
    background-color: #f0f0f0;
    border: 1px solid #dddddd;
}

/* ---------- Text Edit ---------- */
QTextEdit {
    font-size: 16px;
}

/* ---------- Scroll Bar (wider for touch) ---------- */
QScrollBar:vertical {
    width: 20px;
}
QScrollBar::handle:vertical {
    min-height: 40px;
    background: #cccccc;
    border-radius: 4px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
"""
