from __future__ import annotations #Enable postponed evaluation of type annotations, allowing for forward references and improved readability.
# tells Python to treat type hints as strings and evaluate them later. so that no error occurs
import rclpy #ROS Client Library for Python
from rclpy.node import Node #Node is the base class for all ROS 2 nodes


class HelloNode(Node):

    def __init__(self) -> None:
        """Initialize the node, parameters, and timer."""
        super().__init__('hello_node')
        self.declare_parameter('name', 'Nidar Drone')
        self._count = 0
        self._timer = self.create_timer(1.0, self._on_timer)

    def _on_timer(self) -> None:
        """Timer callback that logs a greeting."""
        name = self.get_parameter('name').get_parameter_value().string_value
        self._count += 1
        self.get_logger().info(f'Hello, {name}! count={self._count}')


def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = HelloNode()
    try:
        rclpy.spin(node) #Keep the node running until it is shut down
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node() #Clean up the node before shutting down
        rclpy.shutdown() #Shut down the ROS 2 client library
