import re
from pathlib import Path

# Paths
NAV_CORE_HDR = Path(__file__).resolve().parents[1] / "base_global_planner.h"
NAVFN_CPP = Path(__file__).resolve().parents[1] / "navfn_ros.cpp"
NAVFN_CMAKE = Path(__file__).resolve().parents[1] / "CMakeLists.txt"

def get_content(file_path):
    return file_path.read_text() if file_path.exists() else ""

def test_interface_signature_consistency():
    """Check Hole 1: Ensure pure virtual functions use ROS 2 types and proper syntax."""
    content = get_content(NAV_CORE_HDR)
    # Use re.DOTALL (.) to match across newlines in parameter lists
    # Ensure it's a pure virtual function ending with = 0
    pattern = r"virtual\s+bool\s+makePlan\s*\(.*?\)\s*=\s*0\s*;"
    assert re.search(pattern, content, re.DOTALL), "makePlan must be a pure virtual function"
    assert "geometry_msgs::msg::PoseStamped" in content

def test_implementation_semantic_flow():
    """Check Hole 2 for logical flow: Lock -> Validate -> Plan -> Extract."""
    content = get_content(NAVFN_CPP)
    # Thread safety
    assert re.search(r"std::lock_guard<[^>]+>\s+\w+\(mutex_\)", content)
    # Semantic check: worldToMap must happen before setting planner goals
    assert content.find("worldToMap") < content.find("planner_->setGoal"), \
        "Coordinates must be converted to map frame before setting planner goal"
    # Extraction check
    assert "getPathX" in content and "plan.push_back" in content

def test_build_system_bridge():
    """Check Hole 3 for ament integration and plugin installation."""
    content = get_content(NAVFN_CMAKE)
    # Ament vs Catkin: Catkin markers must be absent
    assert "catkin_" not in content.lower(), "Found legacy Catkin markers in Ament project"
    # Plugin registration check
    assert re.search(r"install\(FILES\s+bgp_plugin\.xml\s+DESTINATION\s+share/\$\{PROJECT_NAME\}", content), \
        "Plugin XML must be installed to share folder for pluginlib discovery"
    assert "ament_package()" in content

def test_class_export_match():
    """Check if the exported class matches the namespace implementation."""
    content = get_content(NAVFN_CPP)
    # PLUGINLIB_EXPORT_CLASS(actual_class, base_class)
    assert "PLUGINLIB_EXPORT_CLASS(navfn::NavfnROS, nav_core::BaseGlobalPlanner)" in content

def test_no_memory_leaks_in_path():
    """Ensure no raw 'new' is used for array passing if modern alternatives exist."""
    content = get_content(NAVFN_CPP)
    # Avoid the 'new int[2]' anti-pattern seen in some legacy ports
    assert "new int[" not in content, "Avoid raw pointer allocation for plan coordinates; use stack arrays"
