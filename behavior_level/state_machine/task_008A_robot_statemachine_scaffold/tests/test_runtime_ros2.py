"""
Runtime test for the RobotControlMux ROS2 node.
Exercises the actual compiled node via subprocess, then interacts
with it using rclpy to verify service and topic behavior.
"""
import subprocess
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


PKG = "task_008a_robot_statemachine_scaffold"


def wait_for_node_ready(test_node, target_node_name, timeout=10.0):
    """Wait until the target node appears in the ROS graph."""
    start = time.time()
    while time.time() - start < timeout:
        node_names = [n for n, ns in test_node.get_node_names_and_namespaces()]
        if target_node_name in node_names:
            return True
        rclpy.spin_once(test_node, timeout_sec=0.2)
    return False


@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(scope="module")
def mux_process(ros_context):
    """Launch the robot_control_mux_node as a subprocess."""
    proc = subprocess.Popen(
        ["ros2", "run", PKG, "robot_control_mux_node"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    time.sleep(3)  # Give it time to start
    yield proc
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)


def test_node_starts(mux_process, ros_context):
    """Test that the node starts and is discoverable."""
    test_node = Node("test_node_starts")
    try:
        found = wait_for_node_ready(test_node, "robot_control_mux", timeout=8.0)
        assert found, "robot_control_mux node not found in the ROS graph"
    finally:
        test_node.destroy_node()


def test_operation_mode_topic(mux_process, ros_context):
    """Test that the node publishes on the operationMode topic."""
    from task_008a_robot_statemachine_scaffold.msg import OperationMode

    test_node = Node("test_op_mode_topic")
    received_msgs = []

    def callback(msg):
        received_msgs.append(msg)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    sub = test_node.create_subscription(
        OperationMode, "operationMode", callback, qos
    )

    try:
        start = time.time()
        while time.time() - start < 5.0 and len(received_msgs) == 0:
            rclpy.spin_once(test_node, timeout_sec=0.2)

        assert len(received_msgs) > 0, "No OperationMode messages received"
        msg = received_msgs[-1]
        # Initially should be STOPPED (0) with no emergency stop
        assert msg.mode == 0, f"Expected mode STOPPED(0), got {msg.mode}"
        assert msg.emergency_stop is False, f"Expected no emergency stop, got {msg.emergency_stop}"
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_node()


def test_set_operation_mode_service(mux_process, ros_context):
    """Test the setOperationMode service sets mode and returns success."""
    from task_008a_robot_statemachine_scaffold.srv import SetOperationMode
    from task_008a_robot_statemachine_scaffold.msg import OperationMode

    test_node = Node("test_set_op_mode")
    client = test_node.create_client(SetOperationMode, "setOperationMode")

    try:
        # Wait for service
        assert client.wait_for_service(timeout_sec=8.0), "setOperationMode service not available"

        # Set to AUTONOMOUS mode
        request = SetOperationMode.Request()
        request.operation_mode.mode = OperationMode.AUTONOMOUS  # 1
        request.operation_mode.emergency_stop = False

        future = client.call_async(request)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        assert future.done(), "Service call timed out"
        result = future.result()
        assert result.success is True, f"Expected success=True, got {result.success}"
        assert result.message == "Operation mode set", f"Unexpected message: {result.message}"
    finally:
        test_node.destroy_node()


def test_cmd_vel_reflects_autonomy_mode(mux_process, ros_context):
    """
    Test that after setting AUTONOMOUS mode and publishing autonomy cmd_vel,
    the mux forwards it on cmd_vel topic.
    """
    from task_008a_robot_statemachine_scaffold.srv import SetOperationMode
    from task_008a_robot_statemachine_scaffold.msg import OperationMode
    from geometry_msgs.msg import Twist

    test_node = Node("test_cmd_vel_autonomy")
    received_twists = []

    def twist_cb(msg):
        received_twists.append(msg)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

    # Subscribe to cmd_vel output
    sub = test_node.create_subscription(Twist, "cmd_vel", twist_cb, qos)

    # Publish autonomy cmd_vel
    pub = test_node.create_publisher(Twist, "autonomy/cmd_vel", qos)

    # Service client to set mode
    client = test_node.create_client(SetOperationMode, "setOperationMode")

    try:
        assert client.wait_for_service(timeout_sec=8.0), "Service not available"

        # Set AUTONOMOUS mode
        req = SetOperationMode.Request()
        req.operation_mode.mode = OperationMode.AUTONOMOUS
        req.operation_mode.emergency_stop = False
        future = client.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            rclpy.spin_once(test_node, timeout_sec=0.1)
        assert future.done(), "Service call timed out"

        # Publish a twist on autonomy topic
        twist = Twist()
        twist.linear.x = 1.5
        twist.angular.z = 0.3

        # Give time for mode to take effect, then publish repeatedly
        received_twists.clear()
        for _ in range(20):
            pub.publish(twist)
            rclpy.spin_once(test_node, timeout_sec=0.1)
            time.sleep(0.1)

        # Check that we received a cmd_vel with our values
        matching = [t for t in received_twists
                    if abs(t.linear.x - 1.5) < 0.01 and abs(t.angular.z - 0.3) < 0.01]
        assert len(matching) > 0, (
            f"Expected cmd_vel with linear.x=1.5, angular.z=0.3 but got "
            f"{[(t.linear.x, t.angular.z) for t in received_twists[-5:]]}"
        )
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_node()


def test_emergency_stop_zeroes_cmd_vel(mux_process, ros_context):
    """
    Test that when emergency stop is active, cmd_vel is zero regardless of input.
    """
    from task_008a_robot_statemachine_scaffold.srv import SetOperationMode
    from task_008a_robot_statemachine_scaffold.msg import OperationMode
    from geometry_msgs.msg import Twist

    test_node = Node("test_estop")
    received_twists = []

    def twist_cb(msg):
        received_twists.append(msg)

    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    sub = test_node.create_subscription(Twist, "cmd_vel", twist_cb, qos)
    pub = test_node.create_publisher(Twist, "autonomy/cmd_vel", qos)
    client = test_node.create_client(SetOperationMode, "setOperationMode")

    try:
        assert client.wait_for_service(timeout_sec=8.0)

        # Set AUTONOMOUS + emergency stop
        req = SetOperationMode.Request()
        req.operation_mode.mode = OperationMode.AUTONOMOUS
        req.operation_mode.emergency_stop = True
        future = client.call_async(req)
        start = time.time()
        while not future.done() and time.time() - start < 5.0:
            rclpy.spin_once(test_node, timeout_sec=0.1)

        # Publish non-zero twist
        twist = Twist()
        twist.linear.x = 5.0

        received_twists.clear()
        for _ in range(15):
            pub.publish(twist)
            rclpy.spin_once(test_node, timeout_sec=0.1)
            time.sleep(0.1)

        # All received cmd_vel should be zero
        assert len(received_twists) > 0, "No cmd_vel messages received"
        for t in received_twists[-5:]:
            assert abs(t.linear.x) < 0.01, (
                f"Expected zero cmd_vel during e-stop, got linear.x={t.linear.x}"
            )
    finally:
        test_node.destroy_subscription(sub)
        test_node.destroy_node()