# combined_gui.py
import sys
import threading
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QLabel, QLineEdit, QPushButton, QVBoxLayout, QHBoxLayout,
    QSpinBox, QGroupBox, QMessageBox, QSizePolicy, QProgressBar
)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QObject, pyqtSlot, QThread
from PyQt5.QtGui import QImage, QPixmap, QFont, QIntValidator

from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
import os
from dotenv import load_dotenv  # Python용 dotenv 가져오기
import re  # Added for email validation

# --- Communicator class for signal passing ---
class Communicator(QObject):
    error_signal = pyqtSignal(str)  # Signal to pass error messages
    state_signal = pyqtSignal(int)  # Signal to pass state updates

# --- VideoThread for video streaming ---
class VideoThread(QThread):
    change_pixmap_signal = pyqtSignal(QImage)

    def __init__(self, source=0, parent=None):
        super().__init__(parent)
        self.source = source
        self._run_flag = True

    def run(self):
        # Open video source
        cap = cv2.VideoCapture(self.source)
        if not cap.isOpened():
            self.change_pixmap_signal.emit(QImage())  # Emit empty image on failure
            self.parent().get_logger().error(f"Cannot open camera with index {self.source}")
            return
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
            else:
                self.parent().get_logger().warning(f"Failed to read frame from camera {self.source}")
        # Release video source
        cap.release()

    def stop(self):
        self._run_flag = False
        self.wait()

# --- ROS2 Nodes ---
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

        # Check if receiver_email is valid
        if not self.validate_email(self.email_config['receiver_email']):
            # Set to default email
            default_email = 'kwilee0426@gmail.com'
            self.email_config['receiver_email'] = default_email
            # Emit a signal to inform GUI
            self.communicator.error_signal.emit(f"Invalid or no receiver email. Email will be sent to {default_email}.")

        # Send email in a separate thread
        threading.Thread(target=self.send_email, args=(error_message,), daemon=True).start()
        # Emit error_signal with the error message
        self.communicator.error_signal.emit(f"Error: {error_message}")

    def validate_email(self, email):
        """Validate email using regex"""
        regex = r'^[\w\.-]+@[\w\.-]+\.\w+$'
        return re.match(regex, email) is not None

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
    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator, email_config: dict):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.email_config = email_config  # Store reference to email_config
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

        self.input_username.returnPressed.connect(self.handle_login)
        self.input_password.returnPressed.connect(self.handle_login)
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
        self.main_window = MainWindow(self.manipulation_node, self.communicator, self.email_config)
        self.main_window.show()
        self.close()

# --- MainWindow class ---
class MainWindow(QMainWindow):
    image_received_signal = pyqtSignal(QImage)  # Signal to update the image in the GUI

    def __init__(self, manipulation_node: ManipulationPublisherNode, communicator: Communicator, email_config: dict):
        super().__init__()
        self.manipulation_node = manipulation_node
        self.communicator = communicator
        self.email_config = email_config  # Reference to email_config
        self.bridge = CvBridge()  # CV Bridge for image conversion
        self.init_ui()
        
        # 기존 publisher 선언 아래에 추가
        self.conveyor_distance_publisher = self.manipulation_node.create_publisher(Int32, '/conveyor_distance', 10)

        # Initialize operation time variables
        self.current_state = "대기 중"
        self.operation_start_time = None
        self.operation_timer = QTimer()
        self.operation_timer.timeout.connect(self.update_operation_time)
        
        # /order_end 토픽 구독 추가
        self.order_end_subscription = self.manipulation_node.create_subscription(
            String,
            '/order_end',
            self.order_end_callback,
            10
        )

        self.start_time = None
        self.elapsed_time = 0  # in seconds
        
        self.aruco_subscription = self.manipulation_node.create_subscription(
            CompressedImage,
            'aruco_image',
            self.aruco_image_callback,
            10)
            
        # Publishers
        qos_order = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.order_publisher = self.manipulation_node.create_publisher(String, '/order', qos_profile=qos_order)

        qos_belt = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        self.belt_publisher = self.manipulation_node.create_publisher(Bool, '/belt', qos_profile=qos_belt)

        # Subscriber for /image_topic
        self.compressed_image_subscription = self.manipulation_node.create_subscription(
            CompressedImage,
            '/image_raw/compressed',
            self.compressed_image_callback,
            10
        )
        self.image_received_signal.connect(self.update_image_label)

        # Connect the signal to the slot
        self.image_received_signal.connect(self.update_image_label)

        # Connect communicator signals
        self.communicator.error_signal.connect(self.show_error_popup)

    def init_ui(self):
        self.setWindowTitle("자동화 시스템")
        self.setGeometry(100, 100, 1200, 800)  # Initial window size

        # Main widget and layout
        self.main_widget = QWidget(self)
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout()
        self.main_widget.setLayout(self.main_layout)

        # Bottom layout (video and other controls)
        self.bottom_layout = QHBoxLayout()
        self.main_layout.addLayout(self.bottom_layout)

        self.left_layout = QVBoxLayout()
        self.bottom_layout.addLayout(self.left_layout)

        # Real-time webcam video display
        self.video_label = QLabel(self)
        self.video_label.setFixedSize(640, 480)  # Fixed size
        self.video_label.setStyleSheet("border: 1px solid black;")
        self.video_label.setAlignment(Qt.AlignCenter)
        self.left_layout.addWidget(self.video_label)

        self.loading_layout = QVBoxLayout()
        self.left_layout.addLayout(self.loading_layout)

        # Current state label
        # 상태 표시 레이아웃 수정
        self.state_label = QLabel("현재 상태: 대기 중", self)
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setStyleSheet("font-size: 14px;")
        self.loading_layout.addWidget(self.state_label)

        # 확인 버튼 추가
        self.confirm_button = QPushButton("확인", self)
        self.confirm_button.clicked.connect(self.confirm_completion)
        self.confirm_button.hide()  # 초기에는 숨김
        self.confirm_button.setFixedSize(80, 30)  # 가로 80px, 세로 30px로 크기 고정
        self.loading_layout.addWidget(self.confirm_button)

        # 버튼을 담을 수평 레이아웃 생성
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        button_layout.addWidget(self.confirm_button)
        button_layout.addStretch()

        # 기존 loading_layout에 button_layout 추가
        self.loading_layout.addLayout(button_layout)

        # 작동 시간 레이블
        self.operation_time_label = QLabel("작동 시간: 0초", self)
        self.operation_time_label.setAlignment(Qt.AlignCenter)
        self.operation_time_label.setStyleSheet("font-size: 14px;")
        self.loading_layout.addWidget(self.operation_time_label)

        # Right side: Job list and controls
        self.right_layout = QVBoxLayout()
        self.bottom_layout.addLayout(self.right_layout)

        # Image display label (replacing Yolo Start Image placeholder)
        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setText("No image received")
        self.image_label.setFixedSize(600, 450)
        self.right_layout.addWidget(self.image_label)

        # Job list and time (now containing spin boxes)
        self.job_group = QGroupBox("작업 목록")
        self.job_layout = QVBoxLayout()
        self.job_group.setLayout(self.job_layout)
        self.right_layout.addWidget(self.job_group)

        # Clear existing job_layout contents
        # (If there were any existing widgets, remove them)
        for i in reversed(range(self.job_layout.count())):
            item = self.job_layout.itemAt(i)
            if item.widget():
                item.widget().deleteLater()
            elif item.layout():
                self.clear_layout(item.layout())

        # Red box layout
        red_layout = QVBoxLayout()
        red_label = QLabel('red box')
        red_label.setAlignment(Qt.AlignCenter)
        self.red_spinbox = QSpinBox()
        self.red_spinbox.setRange(0, 2)
        self.red_spinbox.setValue(0)
        red_layout.addWidget(red_label)
        red_layout.addWidget(self.red_spinbox)
        self.job_layout.addLayout(red_layout)

        # Blue box layout
        blue_layout = QVBoxLayout()
        blue_label = QLabel('blue box')
        blue_label.setAlignment(Qt.AlignCenter)
        self.blue_spinbox = QSpinBox()
        self.blue_spinbox.setRange(0, 2)
        self.blue_spinbox.setValue(0)
        blue_layout.addWidget(blue_label)
        blue_layout.addWidget(self.blue_spinbox)
        self.job_layout.addLayout(blue_layout)

  

        # Send button
        self.send_button = QPushButton('send')
        self.send_button.clicked.connect(self.send_order)
        self.send_button.setFixedSize(100, 40)
        self.job_layout.addWidget(self.send_button, alignment=Qt.AlignCenter)

        # Control layout (manual control buttons and conveyor control)
        self.control_group = QGroupBox("수동 조종")
        self.control_layout = QVBoxLayout()
        self.control_group.setLayout(self.control_layout)
        self.right_layout.addWidget(self.control_group)

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

        # Conveyor control buttons (replacing with On/Off buttons from original user_gui)
        self.conveyor_group = QGroupBox("컨베이어 조작")
        self.conveyor_layout = QVBoxLayout()
        self.conveyor_group.setLayout(self.conveyor_layout)
        self.control_layout.addWidget(self.conveyor_group)

        # 컨베이어 거리 입력 필드와 확인 버튼
        self.conveyor_distance_layout = QHBoxLayout()
        self.conveyor_distance_label = QLabel("거리 (cm):")
        self.conveyor_distance_input = QLineEdit()
        self.conveyor_distance_input.setValidator(QIntValidator(0, 1000))  # 0-1000 cm
        self.conveyor_distance_confirm = QPushButton("확인")
        self.conveyor_distance_confirm.clicked.connect(self.confirm_distance)
        self.conveyor_distance_layout.addWidget(self.conveyor_distance_label)
        self.conveyor_distance_layout.addWidget(self.conveyor_distance_input)
        self.conveyor_distance_layout.addWidget(self.conveyor_distance_confirm)
        self.conveyor_layout.addLayout(self.conveyor_distance_layout)

        # On/Off 버튼 레이아웃
        self.conveyor_buttons_layout = QHBoxLayout()
        self.conveyor_layout.addLayout(self.conveyor_buttons_layout)

        # On button
        self.on_button = QPushButton('On')
        self.on_button.clicked.connect(self.belt_on)
        self.on_button.setFixedSize(80, 40)
        self.on_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                border-radius: 10px;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
            QPushButton:pressed {
                background-color: #3e8e41;
            }
        """)
        self.conveyor_buttons_layout.addWidget(self.on_button)

        # Off button
        self.off_button = QPushButton('Off')
        self.off_button.clicked.connect(self.belt_off)
        self.off_button.setFixedSize(80, 40)
        self.off_button.setStyleSheet("""
            QPushButton {
                background-color: #f44336;
                color: white;
                border-radius: 10px;
                font-size: 16px;
            }
            QPushButton:hover {
                background-color: #da190b;
            }
            QPushButton:pressed {
                background-color: #b71c1c;
            }
        """)
        self.conveyor_buttons_layout.addWidget(self.off_button)

        # --- Email Input Section ---
        self.email_group = QGroupBox("Email 설정")
        self.email_layout = QHBoxLayout()
        self.email_group.setLayout(self.email_layout)

        self.email_label = QLabel("Email:")
        self.email_input = QLineEdit()
        self.email_input.setPlaceholderText("Enter receiver email")
        self.email_input.textChanged.connect(self.on_email_changed)  # Connect to re-enable check button

        self.check_button = QPushButton("Check")
        self.check_button.clicked.connect(self.check_email)

        self.email_layout.addWidget(self.email_label)
        self.email_layout.addWidget(self.email_input)
        self.email_layout.addWidget(self.check_button)

        self.control_layout.addWidget(self.email_group)  # Add email group below conveyor control

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

    def order_end_callback(self, msg):
        if msg.data == "end":
            self.set_state("동작 완료")
            self.operation_timer.stop()
        elif msg.data == "error":
            QMessageBox.critical(self, "오류", "컨베이어벨트 연결이 끊겼습니다")

    def set_state(self, state):
        self.current_state = state
        self.update_state_label()
        
        if state == "작동 중":
            self.operation_start_time = time.time()
            self.operation_timer.start(1000)  # 1초마다 업데이트
        elif state == "동작 완료":
            self.operation_timer.stop()
        elif state == "대기 중":
            self.operation_start_time = None
            self.operation_time_label.setText("작동 시간: 0초")

    def update_state_label(self):
        self.state_label.setText(f"현재 상태: {self.current_state}")
        
        if self.current_state == "동작 완료":
            self.confirm_button.show()
        else:
            self.confirm_button.hide()

    def update_operation_time(self):
        if self.operation_start_time:
            elapsed_time = int(time.time() - self.operation_start_time)
            self.operation_time_label.setText(f"작동 시간: {elapsed_time}초")

    def confirm_completion(self):
        self.set_state("대기 중")

    def confirm_distance(self):
        distance = self.conveyor_distance_input.text()
        if distance:
            self.publish_conveyor_distance(distance)
            QMessageBox.information(self, "확인", f"컨베이어 거리가 {distance}cm로 설정되었습니다.")
            self.conveyor_distance_input.clear()  # 입력 필드 초기화
        else:
            QMessageBox.warning(self, "경고", "거리를 입력해주세요.")

    def publish_conveyor_distance(self, distance):
        msg = Int32()
        msg.data = int(distance)
        self.conveyor_distance_publisher.publish(msg)
        self.manipulation_node.get_logger().info(f'Published conveyor distance: {distance} cm')


    def belt_on(self):
        msg = Bool()
        msg.data = True
        self.belt_publisher.publish(msg)
        self.manipulation_node.get_logger().info('Published to /belt: True')

    def clear_layout(self, layout):
        if layout is not None:
            while layout.count():
                child = layout.takeAt(0)
                if child.widget():
                    child.widget().deleteLater()
                elif child.layout():
                    self.clear_layout(child.layout())

    def aruco_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qt_image)
            self.video_label.setPixmap(pixmap.scaled(
                self.video_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        except Exception as e:
            self.manipulation_node.get_logger().error(f"Could not convert compressed image: {e}")

    
    def compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            cv_image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = cv_image_rgb.shape
            bytes_per_line = ch * w
            qt_image = QImage(cv_image_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
            self.image_received_signal.emit(qt_image)
        except Exception as e:
            self.manipulation_node.get_logger().error(f"Error processing compressed image: {e}")

    @pyqtSlot(QImage)
    def update_image_label(self, qt_image):
        pixmap = QPixmap.fromImage(qt_image)
        scaled_pixmap = pixmap.scaled(self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        self.image_label.setPixmap(scaled_pixmap)


    def send_order(self):
        red_value = self.red_spinbox.value()
        blue_value = self.blue_spinbox.value()
        order_str = f'red_box:{red_value},blue_box:{blue_value}'
        msg = String()
        msg.data = order_str
        self.order_publisher.publish(msg)
        self.manipulation_node.get_logger().info(f'Published to /order: {order_str}')
        self.set_state("작동 중")

    def belt_on(self):
        msg = Bool()
        msg.data = True
        self.belt_publisher.publish(msg)
        self.manipulation_node.get_logger().info('Published to /belt: True')

    def belt_off(self):
        msg = Bool()
        msg.data = False
        self.belt_publisher.publish(msg)
        self.manipulation_node.get_logger().info('Published to /belt: False')

    def update_time(self):
        if self.start_time:
            elapsed = int(time.time() - self.start_time)
            # Update operation time label
            self.operation_time_label.setText(f"작동 시간: {elapsed}초")

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
        # Reset any job-related state
        self.publish_manipulation_command("reset")  # Publish "reset" command

    def publish_manipulation_command(self, command: str):
        self.manipulation_node.publish_command(command)

    def closeEvent(self, event):
        """Close the window and stop threads"""
        event.accept()

    def show_error_popup(self, message):
        QMessageBox.critical(self, 'Error', message)

    # --- Email Check Functionality ---
    def check_email(self):
        email = self.email_input.text().strip()
        if self.validate_email(email):
            # Email is valid
            self.email_config['receiver_email'] = email
            QMessageBox.information(self, 'Email Check', 'Email is valid and set successfully!')
            self.check_button.setEnabled(False)  # Disable the check button
        else:
            # Email is invalid
            QMessageBox.warning(self, 'Invalid Email', 'Please enter a valid email address.')

    def validate_email(self, email):
        """Validate email using regex"""
        regex = r'^[\w\.-]+@[\w\.-]+\.\w+$'
        return re.match(regex, email) is not None

    def on_email_changed(self, text):
        """Re-enable the check button if email input is modified"""
        self.check_button.setEnabled(True)

    def update_operation_time(self):
        self.elapsed_time += 1
        self.operation_time_label.setText(f"작동 시간: {self.elapsed_time}초")
def main():
    load_dotenv()  # Load .env file if present

    # Initialize ROS2
    rclpy.init(args=None)

    # Email configuration from environment variables
    email_config = {
        'smtp_server': os.getenv('SMTP_SERVER', 'smtp.gmail.com'),         # SMTP server address
        'smtp_port': int(os.getenv('SMTP_PORT', 587)),                     # SMTP port
        'sender_email': os.getenv('SENDER_EMAIL', 'kwilee0426@gmail.com'), # Sender email address
        'sender_password': os.getenv('SENDER_PASSWORD', 'dgnbwiqaizoekfvd'),               # Sender email password or app password
        'receiver_email': '',                                              # Initially empty, to be set via GUI
    }

    # Create Communicator object
    communicator = Communicator()

    # Create ROS2 nodes
    manipulation_node = ManipulationPublisherNode()
    manipulation_listener_node = ManipulationListenerNode()
    error_subscriber_node = ErrorSubscriberNode(email_config, communicator)
    state_subscriber_node = StateSubscriberNode(communicator)

    # Create a generic ROS2 node for MainWindow
    ros_node = Node('simple_order_gui_node')

    # Create an executor and add all nodes
    executor = MultiThreadedExecutor()
    executor.add_node(manipulation_node)
    executor.add_node(manipulation_listener_node)
    executor.add_node(error_subscriber_node)
    executor.add_node(state_subscriber_node)
    executor.add_node(ros_node)  # Add the node used in MainWindow

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
    login_window = LoginWindow(manipulation_node, communicator, email_config)
    login_window.show()

    # Execute the application
    exit_code = app.exec_()

    # Shutdown ROS
    executor.shutdown()
    manipulation_node.destroy_node()
    manipulation_listener_node.destroy_node()
    error_subscriber_node.destroy_node()
    state_subscriber_node.destroy_node()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)

if __name__ == '__main__':
    main()
