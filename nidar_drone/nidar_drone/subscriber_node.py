import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 #std_msgs = message

class AltitudeSubscriber(Node):
    def __init__(self):
        super().__init__('altitude_subscriber')
        
        self.subscription = self.create_subscription(
            Float32,
            '/drone_altitude',
            self.altitude_callback,
            10
        )
        self.get_logger().info('Altitude subscriber started, waiting for messages...')

    def altitude_callback(self, msg):
        self.get_logger().info(f'Received target altitude: {msg.data} m')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudeSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()