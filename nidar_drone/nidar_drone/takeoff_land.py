import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class TakeoffLandNode(Node):
    def __init__(self):
        super().__init__('takeoff_land_node')

        # Target takeoff height in meters (ENU). We convert to PX4/NED (negative z).
        self.declare_parameter('height', 2.0)
        height = float(self.get_parameter('height').get_parameter_value().double_value)
        self.target_height = -abs(height)
        self.get_logger().info(f'Target height: {abs(self.target_height)}m')
        height = float(self.get_parameter('height').value)

        # Qos quality of service
        # QoS for publishing TO PX4
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # QoS for subscribing FROM PX4
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )


        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', pub_qos)
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', pub_qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', pub_qos)

        # Subscribers
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self.local_pos_callback, sub_qos)
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self.status_callback, sub_qos)
        
        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.target_height = -abs(height)  # always negative for NED frame
        self.state = 'INIT'

        # Timer — runs every 100ms
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Takeoff land node started...')

    def local_pos_callback(self, msg):
        self.vehicle_local_position = msg

    def status_callback(self, msg):
        self.vehicle_status = msg

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info('Offboard mode command sent')

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')
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

    def publish_trajectory_setpoint(self, x=0.0, y=0.0, z=-2.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]   # NED frame — z negative = up
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

    def timer_callback(self):
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint(z=self.target_height)
        self.offboard_setpoint_counter += 1

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()

        if self.offboard_setpoint_counter == 15:
            self.arm()
            self.state = 'TAKING_OFF'
            self.get_logger().info(f'Armed! Flying to {abs(self.target_height)}m')

        if self.state == 'TAKING_OFF':
            current_z = self.vehicle_local_position.z
            self.get_logger().info(f'Altitude: {abs(current_z):.2f}m / target: {abs(self.target_height):.2f}m')
            if abs(current_z - self.target_height) < 0.15:
                self.get_logger().info('Target reached — landing now')
                self.land()
    
def main(args=None):
    rclpy.init(args=args)
    node = TakeoffLandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()