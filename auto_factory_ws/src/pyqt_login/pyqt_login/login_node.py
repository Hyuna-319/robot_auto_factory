import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QListWidget,
    QListWidgetItem, QVBoxLayout, QHBoxLayout, QGroupBox, QMessageBox, QSizePolicy, QProgressBar
)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer, Qt, QObject, pyqtSlot
from PyQt5.QtGui import QImage, QPixmap, QFont
import cv2
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from dotenv import load_dotenv
import threading
import os

# --- Communicator class for signal passing ---
class Communicator(QObject):
    error_signal = pyqtSignal(str)  # Signal to pass error messages
    state_signal = pyqtSignal(int)  # Signal to pass state updates

# --- VideoThread for video streaming ---
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, source=0):
        super().__init__()
        self.source = source
        self._run_flag = True

    def run(self):
        # Open video source
        cap = cv2.VideoCapture(self.source)
        while self._run_flag:
            ret, cv_img = cap.read()
            if ret:
                # Convert to RGB
                cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
                height, width, channel = cv_img.shape
                bytes_per_line = 3 * width
                qt_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
                # Emit signal
                self.change_pixmap_signal.emit(qt_img)
        # Release video source
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# --- ManipulationPublisherNode publishes to '/manipulation' topic ---
class ManipulationPublisherNode(Node):
    def __init__(self):
        super().__init__('manipulation_publisher')
        self.publisher = self.create_publisher(String, '/manipulation', 10)
        self.get_logger().info('ManipulationPublisherNode has been initialized and publishing to "/manipulation" topic.')

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Published "{command}" to /manipulation topic.')

# --- ManipulationListenerNode subscribes to '/manipulation' topic ---
class ManipulationListenerNode(Node):
    def __init__(self):
        super().__init__('manipulation_listener')
        self.subscription = self.create_subscription(
            String,
            '/manipulation',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info("ManipulationListener node has been initialized and subscribed to /manipulation topic.")

    def listener_callback(self, msg):
        message = msg.data
        self.get_logger().info(f"Received message: {message}")

        # Perform actions based on message
        if message == "play":
            self.get_logger().info("Play command received. Executing 'play' action.")
            # Add actual action code here
        elif message == "stop":
            self.get_logger().info("Stop command received. Executing 'stop' action.")
            # Add actual action code here
        elif message == "pause":
            self.get_logger().info("Pause command received. Executing 'pause' action.")
            # Add actual action code here
        elif message == "resume":
            self.get_logger().info("Resume command received. Executing 'resume' action.")
            # Add actual action code here
        elif message == "reset":
            self.get_logger().info("Reset command received. Executing 'reset' action.")
            # Add actual action code here
        elif message == "conveyor_on":
            self.get_logger().info("Conveyor ON command received. Turning the conveyor ON.")
            # Add actual action code here
        elif message == "conveyor_off":
            self.get_logger().info("Conveyor OFF command received. Turning the conveyor OFF.")
            # Add actual action code here
        else:
            self.get_logger().warning(f"Unknown command received: {message}")

# --- ErrorSubscriberNode subscribes to '/error' topic and sends emails ---
class ErrorSubscriberNode(Node):
    def __init__(self, email_config, communicator: Communicator):
        super().__init__('error_subscriber')
        self.email_config = email_config
        self.communicator = communicator
        self.subscription = self.create_subscription(
            String,
            '/error',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('ErrorSubscriberNode has been initialized and subscribed to "/error" topic.')

    def listener_callback(self, msg):
        error_message = msg.data
        self.get_logger().error(f'Received error message: {error_message}')
        # Send email in a separate thread
        threading.Thread(target=self.send_email, args=(error_message,)).start()
        # Emit error_signal
        self.communicator.error_signal.emit(error_message)

    def send_email(self, error_message):
        try:
            smtp_server = self.email_config['smtp_server']
            smtp_port = self.email_config['smtp_port']
            sender_email = self.email_config['sender_email']
            sender_password = self.email_config['sender_password']
            receiver_email = self.email_config['receiver_email']

            subject = "ROS2 Error Notification"
            body = f"An error has occurred in the ROS2 system:\n\n{error_message}"

            # MIME setup
            msg = MIMEMultipart()
            msg['From'] = sender_email
            msg['To'] = receiver_email
            msg['Subject'] = subject
            msg.attach(MIMEText(body, 'plain'))

            # SMTP connection and send email
            server = smtplib.SMTP(smtp_server, smtp_port)
            server.starttls()
            server.login(sender_email, sender_password)
            text = msg.as_string()
            server.sendmail(sender_email, receiver_email, text)
            server.quit()

            self.get_logger().info(f'Error email sent to {receiver_email}.')
        except Exception as e:
            self.get_logger().error(f'Failed to send email: {e}')

# --- StateSubscriberNode subscribes to 'state' topic and emits state_signal ---
class StateSubscriberNode(Node):
    def __init__(self, communicator: Communicator):
        super().__init__('state_subscriber')
        self.communicator = communicator
        self.subscription = self.create_subscription(
            Int32,
            'state',
            self.listener_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.get_logger().info('StateSubscriberNode has been initialized and subscribed to "state" topic.')

    def listener_callback(self, msg):
        state = msg.data
        self.get_logger().info(f"Received state: {state}")
        self.communicator.state_signal.emit(state)

# --- LoginWindow class ---
class LoginWindow(QWidget):
    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.main_window = None
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle('Login')
        self.setGeometry(100, 100, 300, 150)

        self.label_username = QLabel('Username:')
        self.input_username = QLineEdit()

        self.label_password = QLabel('Password:')
        self.input_password = QLineEdit()
        self.input_password.setEchoMode(QLineEdit.Password)

        self.button_login = QPushButton('Login')
        self.button_login.clicked.connect(self.handle_login)

        layout = QVBoxLayout()
        layout.addWidget(self.label_username)
        layout.addWidget(self.input_username)
        layout.addWidget(self.label_password)
        layout.addWidget(self.input_password)
        layout.addWidget(self.button_login)
        self.setLayout(layout)

    @pyqtSlot()
    def handle_login(self):
        username = self.input_username.text()
        password = self.input_password.text()

        # Simple username and password check
        if username == 'rokey' and password == 'rokey':
            QMessageBox.information(self, 'Login', 'Login Successful!')
            self.manipulation_node.get_logger().info(f'User {username} logged in successfully.')
            self.open_main_window()
        else:
            QMessageBox.warning(self, 'Login', 'Invalid username or password.')

    def open_main_window(self):
        self.main_window = MainWindow(self.manipulation_node, self.communicator)
        self.main_window.show()
        self.close()

# --- MainWindow class ---
class MainWindow(QMainWindow):
    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.init_ui()

    def init_ui(self):
        self.setWindowTitle("터틀봇 조종 GUI")
        self.setGeometry(100, 100, 1200, 800)  # Initial window size

        # Main widget and layout
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        # Top layout (video and YOLO images)
        self.top_layout = QHBoxLayout()
        self.main_layout.addLayout(self.top_layout)

        # Real-time webcam video display
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)  # Fixed size
        self.video_label.setStyleSheet("border: 1px solid black;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.top_layout.addWidget(self.video_label)

        # YOLO image layout
        self.yolo_layout = QVBoxLayout()
        self.top_layout.addLayout(self.yolo_layout)

        # YOLO start image
        self.yolo_start_label = QLabel("YOLO Start Image", self)
        self.yolo_start_label.setFixedSize(320, 240)
        self.yolo_start_label.setStyleSheet("border: 1px solid black;")
        self.yolo_start_label.setAlignment(Qt.AlignCenter)
        self.yolo_layout.addWidget(self.yolo_start_label)

        # YOLO end image
        self.yolo_end_label = QLabel("YOLO End Image", self)
        self.yolo_end_label.setFixedSize(320, 240)
        self.yolo_end_label.setStyleSheet("border: 1px solid black;")
        self.yolo_end_label.setAlignment(Qt.AlignCenter)
        self.yolo_layout.addWidget(self.yolo_end_label)

        # Loading bar layout (below video)
        self.loading_layout = QVBoxLayout()
        self.main_layout.addLayout(self.loading_layout)

        # State loading bar
        self.loading_bar = QProgressBar(self)
        self.loading_bar.setMaximum(10)
        self.loading_bar.setValue(0)
        self.loading_bar.setTextVisible(False)
        self.loading_bar.setFixedHeight(30)
        self.loading_layout.addWidget(self.loading_bar)

        # Current state label
        self.state_label = QLabel("현재 상태: 명령 받았음 (0)", self)
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 14px;")
        self.loading_layout.addWidget(self.state_label)

        # Middle layout (job list and time)
        self.middle_layout = QHBoxLayout()
        self.main_layout.addLayout(self.middle_layout)

        # Job list and time
        self.job_group = QGroupBox("작업 목록")
        self.job_layout = QVBoxLayout()
        self.job_group.setLayout(self.job_layout)
        self.middle_layout.addWidget(self.job_group, 2)

        self.job_list = QListWidget(self)
        self.job_list.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Font size setting
        font = QFont()
        font.setPointSize(14)  # Set desired font size (e.g., 14)
        self.job_list.setFont(font)

        self.job_layout.addWidget(self.job_list)

        self.job_time_label = QLabel("소요 시간: 0초", self)
        self.job_time_label.setAlignment(Qt.AlignCenter)
        self.job_time_label.setStyleSheet("font-size: 16px;")
        self.job_layout.addWidget(self.job_time_label)

        self.populate_jobs()

        # Control layout (manual control buttons and conveyor control)
        self.control_group = QGroupBox("수동 조종")
        self.control_layout = QVBoxLayout()
        self.control_group.setLayout(self.control_layout)
        self.middle_layout.addWidget(self.control_group, 1)

        # Manual control buttons
        self.manual_controls_layout = QHBoxLayout()
        self.control_layout.addLayout(self.manual_controls_layout)

        self.play_button = QPushButton("Play", self)
        self.play_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.play_button)

        self.stop_button = QPushButton("Stop", self)
        self.stop_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.stop_button)

        self.pause_button = QPushButton("Pause", self)
        self.pause_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.pause_button)

        self.resume_button = QPushButton("Resume", self)
        self.resume_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.resume_button)

        self.reset_button = QPushButton("Reset", self)
        self.reset_button.setFixedSize(100, 50)
        self.manual_controls_layout.addWidget(self.reset_button)

        # Conveyor control buttons
        self.conveyor_group = QGroupBox("컨베이어 조작")
        self.conveyor_layout = QHBoxLayout()
        self.conveyor_group.setLayout(self.conveyor_layout)
        self.control_layout.addWidget(self.conveyor_group)

        self.conveyor_on_button = QPushButton("On", self)
        self.conveyor_on_button.setFixedSize(100, 50)
        self.conveyor_layout.addWidget(self.conveyor_on_button)

        self.conveyor_off_button = QPushButton("Off", self)
        self.conveyor_off_button.setFixedSize(100, 50)
        self.conveyor_layout.addWidget(self.conveyor_off_button)

        # Timer setup (update job time)
        self.start_time = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_time)

        # Connect buttons to functions
        self.play_button.clicked.connect(self.play_job)
        self.stop_button.clicked.connect(self.stop_job)
        self.pause_button.clicked.connect(self.pause_job)
        self.resume_button.clicked.connect(self.resume_job)
        self.reset_button.clicked.connect(self.reset_job)
        self.conveyor_on_button.clicked.connect(self.conveyor_on)
        self.conveyor_off_button.clicked.connect(self.conveyor_off)
        self.job_list.currentItemChanged.connect(self.select_job)

        # Connect communicator signals
        self.communicator.state_signal.connect(self.update_loading_bar)
        self.communicator.error_signal.connect(self.show_error_popup)

        # Start video thread
        self.video_thread = VideoThread()
        self.video_thread.change_pixmap_signal.connect(self.update_image)
        self.video_thread.start()

    def populate_jobs(self):
        jobs = [
            ("Job1", "red*2, blue*1, goto goal 1"),
            ("Job2", "red*1, blue*2, goto goal 2"),
            ("Job3", "red*1, goto goal 3"),
        ]
        for name, details in jobs:
            item = QListWidgetItem(f"{name}: {details}")
            self.job_list.addItem(item)

    def select_job(self, current, previous):
        if current:
            QMessageBox.information(self, "작업 선택", f"선택된 작업: {current.text()}")
            # Start timer
            self.start_time = time.time()
            self.timer.start(1000)  # Update every second

    def update_time(self):
        if self.start_time:
            elapsed = int(time.time() - self.start_time)
            self.job_time_label.setText(f"소요 시간: {elapsed}초")

    def play_job(self):
        QMessageBox.information(self, "Play", "작업을 시작합니다.")
        self.publish_manipulation_command("play")  # Publish "play" command

    def stop_job(self):
        QMessageBox.information(self, "Stop", "작업을 중지합니다.")
        self.publish_manipulation_command("stop")  # Publish "stop" command

    def pause_job(self):
        QMessageBox.information(self, "Pause", "작업을 일시 중지합니다.")
        self.publish_manipulation_command("pause")  # Publish "pause" command

    def resume_job(self):
        QMessageBox.information(self, "Resume", "작업을 재개합니다.")
        self.publish_manipulation_command("resume")  # Publish "resume" command

    def reset_job(self):
        QMessageBox.information(self, "Reset", "작업을 초기화합니다.")
        self.job_time_label.setText("소요 시간: 0초")
        self.timer.stop()
        self.start_time = None
        self.publish_manipulation_command("reset")  # Publish "reset" command

    def conveyor_on(self):
        QMessageBox.information(self, "Conveyor", "컨베이어를 켭니다.")
        self.publish_manipulation_command("conveyor_on")  # Publish "conveyor_on" command

    def conveyor_off(self):
        QMessageBox.information(self, "Conveyor", "컨베이어를 끕니다.")
        self.publish_manipulation_command("conveyor_off")  # Publish "conveyor_off" command

    def publish_manipulation_command(self, command: str):
        self.manipulation_node.publish_command(command)

    def closeEvent(self, event):
        """Close the window and stop threads"""
        self.video_thread.stop()
        event.accept()

    def update_image(self, qt_img):
        self.video_label.setPixmap(QPixmap.fromImage(qt_img).scaled(
            self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def update_loading_bar(self, state):
        """Update the loading bar and state label based on the state"""
        state_descriptions = {
            0: "명령 받았음",
            1: "이동 시작",
            2: "피킹 장소 도착",
            3: "박스 잡기 완료",
            4: "컨베이어 벨트 올리기 중",
            5: "이동 중",
            6: "바구니 장소 도착",
            7: "바구니 잡는 중",
            8: "이동 중",
            9: "하역장 도착 완료",
            10: "하차 완료"
        }

        if state in state_descriptions:
            description = state_descriptions[state]
            self.loading_bar.setValue(state)
            self.state_label.setText(f"현재 상태: {description} ({state})")
        else:
            self.loading_bar.setValue(0)
            self.state_label.setText("현재 상태: 알 수 없음")

    def show_error_popup(self, message):
        QMessageBox.critical(self, 'Error', message)

# --- Main function ---
def main():
    load_dotenv()  # Load .env file if present

    # Initialize ROS2
    rclpy.init(args=None)

    # Email configuration
    email_config = {
        'smtp_server': 'smtp.gmail.com',         # SMTP server address
        'smtp_port': 587,                        # SMTP port
        'sender_email': 'kwilee0426@gmail.com',  # Sender email address
        'sender_password': 'dgnbwiqaizoekfvd',   # Sender email password or app password
        'receiver_email': 'ssm06081@gmail.com',  # Receiver email address
    }

    # Create Communicator object
    communicator = Communicator()

    # Create ROS2 nodes
    manipulation_node = ManipulationPublisherNode()
    manipulation_listener_node = ManipulationListenerNode()
    error_subscriber_node = ErrorSubscriberNode(email_config, communicator)
    state_subscriber_node = StateSubscriberNode(communicator)

    # Create an executor and add all nodes
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(manipulation_node)
    executor.add_node(manipulation_listener_node)
    executor.add_node(error_subscriber_node)
    executor.add_node(state_subscriber_node)

    # Create and start ROS spinning in a separate thread
    def spin_executor():
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass

    spin_thread = threading.Thread(target=spin_executor, daemon=True)
    spin_thread.start()

    # Create PyQt5 application
    app = QApplication(sys.argv)

    # Create LoginWindow
    login_window = LoginWindow(manipulation_node, communicator)
    login_window.show()

    # Execute the application
    exit_code = app.exec_()

    # Shutdown ROS
    executor.shutdown()
    manipulation_node.destroy_node()
    manipulation_listener_node.destroy_node()
    error_subscriber_node.destroy_node()
    state_subscriber_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
