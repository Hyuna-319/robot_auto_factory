import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


class CubeSorterNode(Node):
    def __init__(self):
        super().__init__('cube_sorter_node')

        # 큐브 위치 정보를 구독
        self.positions_subscription = self.create_subscription(
            PointStamped,  
            'object_positions', 
            self.positions_callback,  
            10  # 큐 사이즈
        )
        self.positions_subscription  # Prevent unused variable warning

        # 정렬된 데이터를 퍼블리시
        self.sorted_publisher = self.create_publisher(PointStamped, 'sorted_positions', 10)

        # 데이터를 저장할 리스트
        self.positions = []
        self.expected_count = 4  # 수신할 데이터의 개수를 설정 
        self.received_count = 0  # 현재까지 수신한 데이터 개수

    def positions_callback(self, msg):
        """
        object_positions 토픽에서 데이터를 수신했을 때 호출되는 콜백 함수.
        """
        # PointStamped 메시지에서 데이터 추출
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        frame_id = msg.header.frame_id  # 클래스 이름 

        
        self.positions.append((frame_id, x, y, z))
        self.received_count += 1
        self.get_logger().info(f"수신된 데이터: {frame_id}, x={x}, y={y}, z={z}")

        # 모든 데이터를 수신한 경우 퍼블리시
        if self.received_count == self.expected_count:
            self.publish_sorted_positions()

    def publish_sorted_positions(self):
        """
        저장된 데이터를 정렬하고 한 번만 퍼블리시
        """
        # 데이터 정렬 (x 값을 기준으로 낮은 순서대로)
        sorted_positions = sorted(self.positions, key=lambda item: item[1])  # item[1]은 x 값

       
        for position in sorted_positions:
            frame_id, x, y, z = position

            
            sorted_msg = PointStamped()
            sorted_msg.header.stamp = self.get_clock().now().to_msg()
            sorted_msg.header.frame_id = frame_id
            sorted_msg.point.x = x
            sorted_msg.point.y = y
            sorted_msg.point.z = z

            self.sorted_publisher.publish(sorted_msg)
            self.get_logger().info(f"퍼블리시된 데이터: {frame_id}, x={x}, y={y}, z={z}")

        
        self.positions = []
        self.received_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CubeSorterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
