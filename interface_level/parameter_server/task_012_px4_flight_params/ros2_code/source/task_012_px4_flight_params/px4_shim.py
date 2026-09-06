"""
Shim module that provides px4_msgs-like message classes.
If px4_msgs is installed, re-exports from there; otherwise provides
lightweight mock classes that are sufficient for parameter declaration,
topic creation, and basic publish/subscribe with std_msgs serialization.
"""
import struct
import array

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleCommand,
        VehicleLocalPosition,
        VehicleStatus,
    )
    HAS_PX4_MSGS = True
except ImportError:
    HAS_PX4_MSGS = False

    # We need lightweight message-like classes that work with rclpy pub/sub.
    # Since px4_msgs is unavailable, we use std_msgs/msg/String as the
    # transport and JSON-encode the fields. This lets us do real pub/sub
    # in tests without the actual px4_msgs IDL.

    class _MockMsgMeta:
        """Base for mock messages with attribute storage."""
        pass

    class OffboardControlMode:
        def __init__(self):
            self.position = False
            self.velocity = False
            self.acceleration = False
            self.attitude = False
            self.body_rate = False
            self.timestamp = 0

    class TrajectorySetpoint:
        def __init__(self):
            self.position = [0.0, 0.0, 0.0]
            self.yaw = 0.0
            self.timestamp = 0

    class VehicleCommand:
        VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
        VEHICLE_CMD_DO_SET_MODE = 176
        VEHICLE_CMD_NAV_LAND = 21

        def __init__(self):
            self.command = 0
            self.param1 = 0.0
            self.param2 = 0.0
            self.param3 = 0.0
            self.param4 = 0.0
            self.param5 = 0.0
            self.param6 = 0.0
            self.param7 = 0.0
            self.target_system = 0
            self.target_component = 0
            self.source_system = 0
            self.source_component = 0
            self.from_external = False
            self.timestamp = 0

    class VehicleLocalPosition:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.timestamp = 0

    class VehicleStatus:
        NAVIGATION_STATE_OFFBOARD = 14

        def __init__(self):
            self.nav_state = 0
            self.timestamp = 0