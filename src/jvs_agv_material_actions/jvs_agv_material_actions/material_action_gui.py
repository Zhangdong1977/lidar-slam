"""Main entry point: integrates rclpy executor with Qt event loop.

Threading model:
  - Main thread: QApplication.exec_() (Qt requirement on Linux/X11)
  - Background thread: rclpy MultiThreadedExecutor.spin()
  - Communication: ActionBridge pyqtSignal (thread-safe, queued to main thread)
"""

import os
import sys
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor

from PyQt5 import QtWidgets

from .material_action_server import MaterialActionServer
from .gui.main_window import MainWindow


def main(args=None):
    # Check display availability
    if not os.environ.get('DISPLAY'):
        print('ERROR: DISPLAY environment variable not set. '
              'Cannot launch GUI.', file=sys.stderr)
        sys.exit(1)

    # 1. Initialize rclpy BEFORE QApplication (for ROS2 arg parsing)
    rclpy.init(args=args)

    # 2. Create the action server node
    node = MaterialActionServer()

    # 3. Spin ROS2 in a background thread (use spin_once loop to avoid busy-wait)
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    def _spin_loop():
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

    spin_thread = threading.Thread(target=_spin_loop, daemon=True)
    spin_thread.start()

    # 4. Create Qt application
    app = QtWidgets.QApplication(sys.argv)
    app.setApplicationName('JVS-VGA控制台')

    # 5. Create main window, pass bridge + server
    window = MainWindow(node.bridge, node)
    window.show()

    # 6. Run Qt event loop (main thread)
    try:
        ret = app.exec_()
    except KeyboardInterrupt:
        ret = 0

    # 7. Cleanup
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    node.destroy_node()
    rclpy.shutdown()

    sys.exit(ret)


if __name__ == '__main__':
    main()
