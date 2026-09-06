#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from task_012_px4_flight_params.px4_shim import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
    HAS_PX4_MSGS,
)

if HAS_PX4_MSGS:
    from px4_msgs.msg import (
        OffboardControlMode as OffboardControlModeMsg,
        TrajectorySetpoint as TrajectorySetpointMsg,
        VehicleCommand as VehicleCommandMsg,
        VehicleLocalPosition as VehicleLocalPositionMsg,
        VehicleStatus as VehicleStatusMsg,
    )
else:
    # Use std_msgs/String as transport with JSON encoding
    from std_msgs.msg import String as _StringMsg
    import json

    OffboardControlModeMsg = _StringMsg
    TrajectorySetpointMsg = _StringMsg
    VehicleCommandMsg = _StringMsg
    VehicleLocalPositionMsg = _StringMsg
    VehicleStatusMsg = _StringMsg


def _encode_msg(obj):
    """Encode a mock message object to a std_msgs/String."""
    if HAS_PX4_MSGS:
        return obj
    msg = _StringMsg()
    msg.data = json.dumps(obj.__dict__, default=_json_default)
    return msg


def _json_default(o):
    if isinstance(o, (list, tuple)):
        return list(o)
    return str(o)


def _decode_trajectory_setpoint(string_msg):
    """Decode a std_msgs/String back into a TrajectorySetpoint-like object."""
    if HAS_PX4_MSGS:
        return string_msg
    d = json.loads(string_msg.data)
    ts = TrajectorySetpoint()
    ts.position = d.get('position', [0.0, 0.0, 0.0])
    ts.yaw = d.get('yaw', 0.0)
    ts.timestamp = d.get('timestamp', 0)
    return ts


class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard_control_takeoff_and_land')

        # Declare ROS 2 parameters with default values
        self.declare_parameter('takeoff_height', -5.0)
        self.declare_parameter('target_yaw', 1.57079)

        # Retrieve parameter values and store as class attributes
        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.target_yaw = self.get_parameter('target_yaw').value

        # Log the loaded parameters for operator verification
        self.get_logger().info(f"Loaded takeoff_height: {self.takeoff_height}")
        self.get_logger().info(f"Loaded target_yaw: {self.target_yaw}")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlModeMsg, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpointMsg, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommandMsg, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPositionMsg, '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatusMsg, '/fmu/out/vehicle_status',
            self.vehicle_status_callback, qos_profile)

        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        """Callback function for vehicle_local_position topic subscriber."""
        if HAS_PX4_MSGS:
            self.vehicle_local_position = vehicle_local_position
        else:
            d = json.loads(vehicle_local_position.data)
            self.vehicle_local_position.x = d.get('x', 0.0)
            self.vehicle_local_position.y = d.get('y', 0.0)
            self.vehicle_local_position.z = d.get('z', 0.0)

    def vehicle_status_callback(self, vehicle_status):
        """Callback function for vehicle_status topic subscriber."""
        if HAS_PX4_MSGS:
            self.vehicle_status = vehicle_status
        else:
            d = json.loads(vehicle_status.data)
            self.vehicle_status.nav_state = d.get('nav_state', 0)

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(_encode_msg(msg))

    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, self.takeoff_height]
        msg.yaw = self.target_yaw
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(_encode_msg(msg))
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(_encode_msg(msg))

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        self.publish_offboard_control_heartbeat_signal()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_local_position.z > self.takeoff_height and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)

        elif self.vehicle_local_position.z <= self.takeoff_height:
            self.land()
            exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)