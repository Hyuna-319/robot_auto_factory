# order_belt_listener_with_serial.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial
import time
import threading


class OrderBeltListener(Node):
    def __init__(self):
        super().__init__('order_belt_listener')
        
        
        qos_belt = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE
        )
        
        
        self.belt_subscription = self.create_subscription(
            Bool,
            '/belt',
            self.belt_callback,
            qos_belt
        )
        self.belt_subscription  # Prevent unused variable warning

        # 아두이노 시리얼 통신 설정
        try:
            self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=1)
            time.sleep(2)  
            self.get_logger().info('Serial connection to Arduino established.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            self.arduino = None
        
        
        self.conveyor_on = False
        self.thread = threading.Thread(target=self.send_periodic_data)
        self.thread.daemon = True  
        self.thread.start()

        self.get_logger().info('OrderBeltListener Node has been started.')

    def belt_callback(self, msg):
        # 컨베이어 상태 업데이트
        if msg.data:
            self.get_logger().info('컨베이어가 ON되었습니다.')
            self.conveyor_on = True
        else:
            self.get_logger().info('컨베이어가 OFF되었습니다.')
            self.conveyor_on = False

    def send_periodic_data(self):
       
        while rclpy.ok():
            if self.conveyor_on and self.arduino and self.arduino.is_open:
                try:
                    self.arduino.write("1000\n".encode())  # 아두이노로 1000 전송
                    self.get_logger().info('Sent to Arduino: 1000')
                except Exception as e:
                    self.get_logger().error(f'Error sending data to Arduino: {e}')
            time.sleep(1)  # 1초마다 전송

    def destroy_node(self):
      
        super().destroy_node()
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info('Serial connection closed.')


def main(args=None):
    rclpy.init(args=args)
    order_belt_listener = OrderBeltListener()
    try:
        rclpy.spin(order_belt_listener)
    except KeyboardInterrupt:
        pass
    finally:
        order_belt_listener.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
