import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
import math

class WaypointNavNode(Node):
    def __init__(self):
        super().__init__('waypoint_nav_node')

        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_pos_callback, sub_qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, sub_qos)

        self.declare_parameter('shape', 'square')
        shape = self.get_parameter('shape').get_parameter_value().string_value
        self.declare_parameter('size', 5.0)
        size = self.get_parameter('size').get_parameter_value().double_value
        self.declare_parameter('height', 5.0)
        height = abs(self.get_parameter('height').get_parameter_value().double_value)

        self.waypoints = self.generate_waypoints(shape, size, height)
        self.current_waypoint_idx = 0
        self.waypoint_threshold = 0.5

        self.offboard_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.state = 'INIT'

        self.get_logger().info(f'Shape: {shape} | Size: {size}m | Height: {height}m')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')

        self.timer = self.create_timer(0.1, self.timer_callback)

    def generate_waypoints(self, shape, size, height):
        h = -abs(height)
        s = size

        if shape == 'square':
            return [
                [0.0,   0.0,        h],
                [s,     0.0,        h],
                [s,     s,          h],
                [0.0,   s,          h],
                [0.0,   0.0,        h],
            ]
        elif shape == 'triangle':
            return [
                [0.0,   0.0,        h],
                [s,     0.0,        h],
                [s/2,   s*0.866,    h],
                [0.0,   0.0,        h],
            ]
        elif shape == 'figure8':
            waypoints = []
            for i in range(9):
                angle = i * (2 * math.pi / 8)
                x = s/2 + (s/2) * math.cos(angle)
                y = (s/2) * math.sin(angle)
                waypoints.append([x, y, h])
            for i in range(9):
                angle = i * (2 * math.pi / 8)
                x = -(s/2) * math.cos(angle)
                y = (s/2) * math.sin(angle)
                waypoints.append([x, y, h])
            return waypoints
        else:
            return self.generate_waypoints('square', size, height)

    def local_pos_callback(self, msg):
        self.vehicle_local_position = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('Offboard mode engaged')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Landing...')
        self.state = 'LANDING'

    def publish_offboard_control_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def publish_trajectory_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = -3.14159
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def distance_to_waypoint(self, waypoint):
        dx = self.vehicle_local_position.x - waypoint[0]
        dy = self.vehicle_local_position.y - waypoint[1]
        dz = self.vehicle_local_position.z - waypoint[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def timer_callback(self):
        current_wp = self.waypoints[min(
            self.current_waypoint_idx, len(self.waypoints)-1)]

        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(
            current_wp[0], current_wp[1], current_wp[2])
        self.offboard_counter += 1

        if self.offboard_counter == 10:
            self.engage_offboard_mode()

        if self.offboard_counter == 15:
            self.arm()
            self.state = 'NAVIGATING'
            self.get_logger().info('Armed! Starting waypoint navigation...')

        if self.state == 'NAVIGATING':
            dist = self.distance_to_waypoint(current_wp)
            self.get_logger().info(
                f'WP {self.current_waypoint_idx+1}/{len(self.waypoints)} '
                f'-> [{current_wp[0]:.1f}, {current_wp[1]:.1f}, {abs(current_wp[2]):.1f}m] '
                f'dist: {dist:.2f}m',
                throttle_duration_sec=0.5)

            if dist < self.waypoint_threshold:
                self.get_logger().info(f'Waypoint {self.current_waypoint_idx+1} reached!')
                self.current_waypoint_idx += 1

                if self.current_waypoint_idx >= len(self.waypoints):
                    self.get_logger().info('All waypoints complete! Landing...')
                    self.land()

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()