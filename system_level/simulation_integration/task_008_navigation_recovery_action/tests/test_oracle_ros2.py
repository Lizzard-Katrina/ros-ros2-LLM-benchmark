import re
import pytest
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "gazebo_model_states.cpp"

def get_content():
    with open(CPP_FILE, "r", encoding="utf-8") as f:
        return f.read()

def test_no_ros1_dependencies():
    """Ensure no ROS1 headers or namespace calls remain."""
    content = get_content()
    # Check for ROS1 headers
    assert not re.search(r"#include <ros/ros\.h>", content), "Legacy ROS1 header found!"
    # Check for ROS1 namespace usage
    assert not re.search(r"\bros::", content), "Legacy ros:: namespace usage found!"
    # Check for legacy message headers (e.g., .h instead of .hpp)
    assert not re.search(r"gazebo_msgs/ModelStates\.h", content), "Legacy message header format found!"

## --- Category 2: Subscriber Creation ---

def test_subscriber_migration():
    """Verify subscriptions to /gazebo/model_states and /gazebo/link_states exist."""
    content = get_content()
    # Check for model_states subscription
    assert re.search(r"create_subscription<gazebo_msgs::msg::ModelStates>\s*\(\s*\"/gazebo/model_states\"", content), \
        "Missing or incorrect ROS2 subscription to /gazebo/model_states"
    # Check for link_states subscription
    assert re.search(r"create_subscription<gazebo_msgs::msg::LinkStates>\s*\(\s*\"/gazebo/link_states\"", content), \
        "Missing or incorrect ROS2 subscription to /gazebo/link_states"

## --- Category 3: Service Client Usage ---

def test_service_client_and_call():
    """Verify service client for /gazebo/set_model_state is created and invoked."""
    content = get_content()
    # Check client creation
    assert re.search(r"create_client<gazebo_msgs::srv::SetModelState>\s*\(\s*\"/gazebo/set_model_state\"", content), \
        "Service client for /gazebo/set_model_state was not created using ROS2 API."
    # Check for the service call within the loop
    assert re.search(r"set_model_state\s*\(.*?\)", content), \
        "The set_model_state helper function is not called in the main loop."

## --- Category 4: Pose and Twist Initialization ---

def test_pose_and_twist_initialization():
    """Ensure Pose and Twist objects are initialized and passed to the service."""
    content = get_content()
    # Check for ROS2 geometry_msgs types (msg::Pose / msg::Twist)
    assert re.search(r"geometry_msgs::msg::Pose", content), "geometry_msgs::msg::Pose not found."
    assert re.search(r"geometry_msgs::msg::Twist", content), "geometry_msgs::msg::Twist not found."
    # Verify they are passed as arguments (looking for 'ball_model_pose' and 'zero_twist' or equivalent)
    assert re.search(r"set_model_state\s*\(\s*\"ball\"\s*,\s*\"world\"\s*,\s*.*?\s*,\s*.*?\)", content), \
        "Service call does not pass required Pose and Twist arguments."
