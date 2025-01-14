import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ManipulationListener(Node):
    def __init__(self):
        super().__init__('manipulation_listener')
        self.subscription = self.create_subscription(
            String,
            '/manipulation',
            self.listener_callback,
            10  # QoS 프로파일 (큐 사이즈)
        )
        self.get_logger().info("ManipulationListener node has been initialized and subscribed to /manipulation topic.")

    def listener_callback(self, msg):
        message = msg.data
        self.get_logger().info(f"Received message: {message}")

        # 메시지 내용에 따라 행동 수행 (여기서는 프린트)
        if message == "play":
            self.get_logger().info("Play command received. Executing 'play' action.")
        elif message == "stop":
            self.get_logger().info("Stop command received. Executing 'stop' action.")
        elif message == "pause":
            self.get_logger().info("Pause command received. Executing 'pause' action.")
        elif message == "resume":
            self.get_logger().info("Resume command received. Executing 'resume' action.")
        elif message == "reset":
            self.get_logger().info("Reset command received. Executing 'reset' action.")
        elif message == "conveyor_on":
            self.get_logger().info("Conveyor ON command received. Turning the conveyor ON.")
        elif message == "conveyor_off":
            self.get_logger().info("Conveyor OFF command received. Turning the conveyor OFF.")
        else:
            self.get_logger().warning(f"Unknown command received: {message}")

def main(args=None):
    rclpy.init(args=args)
    node = ManipulationListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
