import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class AltitudePublisher(Node):
    def __init__(self):
        super().__init__('altitude_publisher')
        
        self.publisher_ = self.create_publisher(Float32, '/drone_altitude', 10)
        
        self.timer = self.create_timer(1.0, self.publish_altitude)
        
        self.altitude = 0.0
        self.get_logger().info('Altitude publisher started!')

    def publish_altitude(self):
        msg = Float32()
        self.altitude += 0.5          # simulate drone climbing
        msg.data = self.altitude
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing altitude: {self.altitude} m')

def main(args=None):
    rclpy.init(args=args)
    node = AltitudePublisher()
    try:
        rclpy.spin(node)    
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()