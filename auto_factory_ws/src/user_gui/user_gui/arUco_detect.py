import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import os

class ArUcoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.publisher_ = self.create_publisher(CompressedImage, 'aruco_image', 10)
        self.position_publisher = self.create_publisher(String, 'marker_positions', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()

        # 카메라 캘리브레이션 데이터 로드
        camera_matrix_path = os.path.expanduser('~/ff_ws/src/camera_matrix_arm.npy')
        dist_coeffs_path = os.path.expanduser('~/ff_ws/src/dist_coeffs_arm.npy')
        try:
            self.camera_matrix = np.load(camera_matrix_path)
            self.dist_coeffs = np.load(dist_coeffs_path)
        except Exception as e:
            self.get_logger().error(f"파일 로드 오류: {e}")
            exit()

        # ArUco 딕셔너리와 탐지 파라미터 설정
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
        self.parameters = aruco.DetectorParameters_create()

        # ArUco 마커 한 변의 실제 길이 (단위: 미터)
        self.marker_length = 0.1 # 10cm

        # USB 카메라 연결
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다.")
            exit()

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("영상을 가져올 수 없습니다.")
            return

        # Grayscale 변환
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ArUco 마커 검출
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # 마커를 그립니다
            aruco.drawDetectedMarkers(frame, corners, ids)

            # 각 마커의 3D 위치와 방향 추정
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

            marker_positions = {}
            for i, id_ in enumerate(ids):
                # 마커 축 그리기
                aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                # 카메라와 마커 간의 거리 및 좌표 저장
                distance_to_camera = tvecs[i][0][2] # Z축 값이 카메라와 마커 간의 거리
                marker_positions[id_[0]] = tvecs[i][0] # 마커 ID와 좌표 저장

                # 마커 정보 출력
                self.get_logger().info(f"\n[마커 ID {id_[0]}]")
                self.get_logger().info(f"카메라와의 거리: {distance_to_camera * 100:.2f} cm")
                self.get_logger().info(f"화면 상 위치 (x, y, z): {tvecs[i][0]} (m)")
                self.get_logger().info(f"라디안 회전각 : {rvecs[i].flatten()}")

            # 마커 ID가 A, B, C(왼쪽부터 오른쪽)로 정렬되었는지 확인
            if len(marker_positions) >= 3:
                # x 좌표를 기준으로 정렬 (왼쪽에서 오른쪽 순서)
                sorted_markers = sorted(marker_positions.items(), key=lambda x: x[1][0])
                A_id, A_pos = sorted_markers[0]
                C_id, C_pos = sorted_markers[1]
                B_id, B_pos = sorted_markers[2]

                # A, C, B 간 거리 계산
                distance_AC = np.linalg.norm(C_pos - A_pos)
                distance_CB = np.linalg.norm(B_pos - C_pos)

                # ACB 삼각형의 각도 계산 (벡터 방향 고려)
                vector_AC = C_pos - A_pos
                vector_CB = B_pos - C_pos
                cos_angle = np.dot(vector_AC, vector_CB) / (np.linalg.norm(vector_AC) * np.linalg.norm(vector_CB))
                angle_ACB = np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))) # 안전한 arccos 계산

                # 교차곱으로 방향 확인 후 각도 교정
                cross_product = np.cross(vector_AC, vector_CB)
                if cross_product[2] < 0: # Z축 기준으로 음수인 경우
                    angle_ACB = 180 - angle_ACB

                # 출력
                self.get_logger().info(f"\n[ACB 관계]")
                self.get_logger().info(f"A(ID {A_id})에서 C(ID {C_id})까지 거리: {distance_AC * 100:.2f} cm")
                self.get_logger().info(f"C(ID {C_id})에서 B(ID {B_id})까지 거리: {distance_CB * 100:.2f} cm")
                self.get_logger().info(f"A-C-B 각도: {angle_ACB:.2f}°")

                # 마커 위치를 문자열로 출력하여 토픽으로 발행
                marker_info = f"A: {A_id}({A_pos}), C: {C_id}({C_pos}), B: {B_id}({B_pos}), ACB 각도: {angle_ACB:.2f}°"
                self.position_publisher.publish(String(data=marker_info))

        # Publish the compressed frame
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.publisher_.publish(msg)

    def __del__(self):
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    aruco_detector = ArUcoDetector()
    rclpy.spin(aruco_detector)
    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
