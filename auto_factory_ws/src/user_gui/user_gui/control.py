import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped


class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_ALL,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL
        )

        # /order 구독
        self.subscription_order = self.create_subscription(
            String,
            '/order',
            self.order_callback,
            qos_profile
        )
        self.subscription_order  # Prevent unused variable warning

        # /sorted_positions 구독
        self.subscription_sorted_positions = self.create_subscription(
            PointStamped,
            'sorted_positions',
            self.sorted_positions_callback,
            10
        )
        self.subscription_sorted_positions  # Prevent unused variable warning

        # 가공된 데이터를 퍼블리시
        self.position_publisher = self.create_publisher(PointStamped, '/processed_positions', qos_profile)

        self.red_box_count = 0
        self.blue_box_count = 0
        self.sorted_positions = []  # 수신된 데이터 리스트
        self.total_signals = 4  # 총 수신할 신호 개수

        self.get_logger().info('Control Node가 시작되었습니다.')

    def order_callback(self, msg):
        """
        /order 토픽에서 데이터를 수신했을 때 호출되는 콜백 함수
        """
        self.get_logger().info(f'수신된 오더 데이터: {msg.data}')
        try:
            # 문자열을 쉼표로 분리
            parts = msg.data.split(',')
            order_dict = {}
            for part in parts:
                key, value = part.split(':')
                order_dict[key.strip()] = int(value.strip())

            # 각 값 추출
            self.red_box_count = order_dict.get('red_box', 0)
            self.blue_box_count = order_dict.get('blue_box', 0)

            # 개별 값 출력
            self.get_logger().info(f'Red Box Count: {self.red_box_count}')
            self.get_logger().info(f'Blue Box Count: {self.blue_box_count}')
        except Exception as e:
            self.get_logger().error(f'오더 메시지 파싱 실패: {e}')

    def sorted_positions_callback(self, msg):
        """
        /sorted_positions 토픽에서 데이터를 수신했을 때 호출되는 콜백 함수
        """
        try:
            position = (msg.header.frame_id, msg.point.x, msg.point.y, msg.point.z)
            self.get_logger().info(f'수신된 정렬된 위치 데이터: {position}')

            # 수신된 데이터를 리스트에 추가
            self.sorted_positions.append(position)

            # 총 신호가 모두 수신되었는지 확인
            if len(self.sorted_positions) == self.total_signals:
                self.process_positions()
        except Exception as e:
            self.get_logger().error(f'정렬된 위치 메시지 처리 실패: {e}')

    def process_positions(self):
        """
        수신된 데이터와 오더를 기반으로 위치 데이터를 가공하고 퍼블리시
        """
        if not self.sorted_positions:
            self.get_logger().warn('정렬된 위치 데이터가 아직 수신되지 않았습니다.')
            return

        # 필요한 데이터만 자르기
        red_count = 0
        blue_count = 0
        selected_positions = []

        for position in self.sorted_positions:
            frame_id, x, y, z = position
            if frame_id == 'red_box' and red_count < self.red_box_count:
                selected_positions.append(position)
                red_count += 1
            elif frame_id == 'blue_box' and blue_count < self.blue_box_count:
                selected_positions.append(position)
                blue_count += 1

            # 필요한 데이터가 모두 수집되면 종료
            if red_count == self.red_box_count and blue_count == self.blue_box_count:
                break

        self.get_logger().info(f'퍼블리시할 위치 데이터: {selected_positions}')

        # 선택된 위치 데이터를 퍼블리시
        for position in selected_positions:
            frame_id, x, y, z = position
            point_msg = PointStamped()
            point_msg.header.frame_id = frame_id
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.point.x = x
            point_msg.point.y = y
            point_msg.point.z = z

           
            self.position_publisher.publish(point_msg)
            self.get_logger().info(f'퍼블리시된 위치: frame_id={frame_id}, x={x}, y={y}, z={z}')

     
        self.sorted_positions = []


def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    try:
        rclpy.spin(control_node)
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
