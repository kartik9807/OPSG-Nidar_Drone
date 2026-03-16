import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool #std_srvs = standard services package that comes

class ArmDroneService(Node):
    def __init__(self):
        super().__init__('arm_drone_service')
        self.srv = self.create_service(
            SetBool,
            '/arm_drone',
            self.arm_callback)
        self.get_logger().info('Arm drone service ready...')

    def arm_callback(self, request, response):
        if request.data:
            self.get_logger().info('Drone ARMED')
            response.success = True
            response.message = 'Drone armed successfully'
        else:
            self.get_logger().info('Drone DISARMED')
            response.success = True
            response.message = 'Drone disarmed successfully'
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArmDroneService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()