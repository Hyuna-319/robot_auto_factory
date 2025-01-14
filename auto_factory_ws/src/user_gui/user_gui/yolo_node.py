import cv2
import math
import os
from datetime import datetime
from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
import time

def yolo_to_real(x, y):
    """
    YOLO 좌표를 실제 좌표로 변환
    :param x: YOLO X 좌표
    :param y: YOLO Y 좌표
    :return: 실제 X, Y 좌표
    """
    # X 변환 계수
    A_x = -5.574

    # Y 변환 계수
    A_y = 6.077
    B_y = 195.907

    # 변환된 좌표 계산
    real_x = A_x * x
    real_y = A_y * y + B_y

    return real_x, real_y
class YoloZoneNode(Node):
    def __init__(self):
        super().__init__('yolo_zone_node')

        
        self.model = YOLO('/home/seokhwan/ROS/ff_ws/src/best.pt')

        
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            exit()

        # 최소 박스 크기 설정
        self.min_box_width = 70  # 최소 너비 
        self.min_box_height = 70  # 최소 높이 

        # ROS2 퍼블리셔 설정
        self.position_publisher = self.create_publisher(PointStamped, 'object_positions', 10)
        self.image_publisher = self.create_publisher(CompressedImage, '/image_raw/compressed', 10)

        # 화면 업데이트 
        self.timer = self.create_timer(0.03, self.update_frame)

        # 메시지 중복 전송 방지 
        self.message_sent = False

        
        self.bridge = CvBridge()

        self.get_logger().info("YOLOv8 노드가 시작되었습니다.")

    def pixel_to_real(self, cx, cy, depth=30.0):
        """픽셀 좌표를 실세계 좌표로 변환"""
        camera_fov_x = 60  # 카메라 수평 FOV (degrees)
        camera_fov_y = 45  # 카메라 수직 FOV (degrees)
        camera_width = 640
        camera_height = 480

        # 픽셀 → 정규화 좌표
        nx = (cx - camera_width / 2) / (camera_width / 2)
        ny = (cy - camera_height / 2) / (camera_height / 2)

        # 정규화 좌표 → 실세계 좌표
        x = depth * nx * math.tan(math.radians(camera_fov_x / 2))
        y = depth * ny * math.tan(math.radians(camera_fov_y / 2))
        z = depth
        y,x = yolo_to_real(x, y)
        return x, y, z

    def publish_image(self, frame):
        """캡처한 이미지를 CompressedImage로 ROS2에 퍼블리시"""
        try:
            
            compressed_image = cv2.imencode('.jpg', frame)[1].tobytes()

            
            ros_image = CompressedImage()
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.format = "jpeg"
            ros_image.data = compressed_image

           
            self.image_publisher.publish(ros_image)
            self.get_logger().info("이미지가 ROS2로 CompressedImage 타입으로 퍼블리시되었습니다.")
        except Exception as e:
            self.get_logger().error(f"이미지 퍼블리시 중 오류 발생: {e}") 

    def update_frame(self):
        """YOLO 탐지 수행"""
        if self.message_sent:
          
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("프레임을 읽을 수 없습니다.")
            return

        results = self.model(frame)
        red_boxes = []  # Red 박스 
        blue_boxes = []  # Blue 박스 

        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # 박스 좌표
                cls = int(box.cls[0])  # 클래스 ID
                class_name = "red_box" if cls == 0 else "blue_box"

                # 박스 크기 계산
                box_width = x2 - x1
                box_height = y2 - y1

                # 박스 크기 조건 확인
                if box_width >= self.min_box_width and box_height >= self.min_box_height:
                    # 중심 좌표 계산
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2

                    # 픽셀 → 실세계 좌표 변환
                    x, y, z = self.pixel_to_real(cx, cy)

                    # 이미지에 박스와 정보 표시
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"{class_name}: ({cx}, {cy})"
                    cv2.putText(frame, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

                    # 클래스별로 분리
                    if class_name == "red_box":
                        red_boxes.append((class_name, x, y, z))
                    else:
                        blue_boxes.append((class_name, x, y, z))

        # 조건: Red_box 2개, Blue_box 2개, 총 4개일 때만 실행
        if len(red_boxes) == 2 and len(blue_boxes) == 2 and len(red_boxes) + len(blue_boxes) == 4:
            self.message_sent = True  # 메시지 중복 전송 방지
            self.get_logger().info("탐지 조건 만족! 좌표 전송 및 이미지 퍼블리시 시작.")

            # 좌표와 박스 종류 메시지 생성
            for class_name, x, y, z in red_boxes + blue_boxes:
                # ROS2 좌표 퍼블리시
                position_msg = PointStamped()
                position_msg.header.stamp = self.get_clock().now().to_msg()
                position_msg.header.frame_id = class_name  # 클래스 이름을 frame_id에 저장
                position_msg.point.x = x
                position_msg.point.y = y
                position_msg.point.z = z
                self.position_publisher.publish(position_msg)
                self.get_logger().info(f"전송: {position_msg}")

            # 이미지 퍼블리시
            self.publish_image(frame)

        # 화면 표시
        cv2.imshow("YOLO Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()

    def destroy_node(self):
        """자원 해제"""
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloZoneNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
