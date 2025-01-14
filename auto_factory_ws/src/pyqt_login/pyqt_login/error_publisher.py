import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QMessageBox
from std_msgs.msg import String
from PyQt5.QtCore import pyqtSlot, Qt

class ErrorPublisherWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_ros()

    def init_ui(self):
        self.setWindowTitle('Error Publisher')
        self.setGeometry(150, 150, 200, 100)

        self.button_publish = QPushButton('Publish "Battery Low"', self)
        self.button_publish.clicked.connect(self.publish_error)

        layout = QVBoxLayout()
        layout.addWidget(self.button_publish)
        self.setLayout(layout)

    def init_ros(self):
        rclpy.init()
        self.node = Node('error_publisher_node')
        self.publisher = self.node.create_publisher(String, '/error', 10)
        self.timer = self.node.create_timer(0.1, self.spin_ros)  # 100ms마다 spin_once 호출

    @pyqtSlot()
    def publish_error(self):
        msg = String()
        msg.data = 'battery low'
        self.publisher.publish(msg)
        QMessageBox.information(self, 'Publish', 'Published "battery low" to /error topic.')
        self.node.get_logger().info('Published "battery low" to /error topic.')

    def spin_ros(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def closeEvent(self, event):
        self.node.destroy_node()
        rclpy.shutdown()
        event.accept()

def main(args=None):
    app = QApplication(sys.argv)
    window = ErrorPublisherWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
