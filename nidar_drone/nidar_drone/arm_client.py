import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class ArmDroneClient(Node):
    def __init__(self):
        super().__init__('arm_drone_client')
        self.client = self.create_client(SetBool, '/arm_drone')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for arm_drone service...')

        self.send_request(True)   # True = arm, False = disarm

    def send_request(self, arm):
        req = SetBool.Request()
        req.data = arm
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        response = future.result()
        self.get_logger().info(f'Response: {response.message}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmDroneClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()