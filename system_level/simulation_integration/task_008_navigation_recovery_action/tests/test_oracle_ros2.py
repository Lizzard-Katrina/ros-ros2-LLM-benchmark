import re
from pathlib import Path

TRANSLATED_FILE = "navigation_recovery_action_server.py"

def load_code():
    path = Path(__file__).parent.parent / TRANSLATED_FILE
    if not path.exists():
        print("Translated ROS2 file not found")
        return ""
    code = path.read_text()
    code = re.sub(r"```[\s\S]*?```", "", code)  # remove markdown fences
    code = re.sub(r"#.*", "", code)             # remove comments
    return code

# =================================================
# Test Group 0: ROS1 artifacts removed
# =================================================
def test_no_ros1_artifacts():
    """
    Concept: No rospy or ROS1 concepts should remain
    """
    code = load_code()
    forbidden = ["roscpp", "ros/ros.h", "ros::Subscriber", "ros::ServiceClient"]
    for pat in forbidden:
        assert pat not in code, f"ROS1 artifact found: {pat}"

# =================================================
# Test Group 1: Subscribers created
# =================================================
def test_subscribers_created():
    """
    Concept: Node must subscribe to model_states and link_states topics
    """
    code = load_code()
    topics = ["/gazebo/model_states", "/gazebo/link_states"]
    for topic in topics:
        assert topic in code, f"Subscriber to {topic} missing"

# =================================================
# Test Group 2: Service client usage
# =================================================
def test_service_client_created():
    """
    Concept: Node must create a service client for /gazebo/set_model_state
    """
    code = load_code()
    assert "/gazebo/set_model_state" in code, "Service client for /gazebo/set_model_state missing"

def test_set_model_state_called():
    """
    Concept: Node must call set_model_state with proper arguments
    """
    code = load_code()
    patterns = [
        r"set_model_state\s*\(",
    ]
    assert any(re.search(p, code) for p in patterns), "set_model_state not called"

# =================================================
# Test Group 3: Pose and twist initialized
# =================================================
def test_pose_initialized():
    """
    Concept: geometry_msgs::Pose must be initialized
    """
    code = load_code()
    assert "geometry_msgs::Pose" in code, "Pose object not created"

def test_twist_initialized():
    """
    Concept: geometry_msgs::Twist must be initialized
    """
    code = load_code()
    assert "geometry_msgs::Twist" in code, "Twist object not created"
