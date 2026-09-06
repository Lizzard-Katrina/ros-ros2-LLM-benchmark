import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[0] / "scan_to_scan_filter_chain.cpp"

def get_content():
    with open(CPP_FILE, 'r') as f:
        content = f.read()
    return re.sub(r'//.*|/\*[\s\S]*?\*/', '', content)

def test_tf_listener_persistence():
    """Behavior: Ensure TF Listener is not a local variable (it must persist)."""
    content = get_content()
    assert not re.search(r"auto\s+tf_listener\s*=", content), \
        "Failure: TF Listener must be a class member. Local listeners are destroyed after constructor exit."

def test_filter_execution_flexible():
    """Behavior: Support both in-place and 2-arg update signatures."""
    content = get_content()
    update_pattern = r"filter_chain_\.update\s*\([^)]+\)"
    assert re.search(update_pattern, content), \
        "Failure: filter_chain_.update() was never called."

def test_no_lifecycle_hallucination():
    """Behavior: Ensure standard Nodes don't use Lifecycle methods."""
    content = get_content()
    lifecycle_methods = ["on_activate", "on_configure", "set_on_new_subscription_callback"]
    for method in lifecycle_methods:
        assert method not in content, \
            f"Failure: Detected LifecycleNode method '{method}' in a standard rclcpp::Node."

def test_member_variable_consistency():
    """Behavior: Ensure correct member names from the header are used (pub/sub)."""
    content = get_content()
    assert "output_pub_" in content or "output_pub_->publish" in content, \
        "Failure: Use the correct class member 'output_pub_' for publishing."