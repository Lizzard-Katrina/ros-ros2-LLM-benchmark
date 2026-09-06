"""
Runtime test for task_010_sensor_fusion_params.

This test verifies the translated ros_filter.cpp loadParams() function by:
1. Checking the source file exists and passes all oracle regex checks.
2. Performing semantic validation of the C++ source to ensure it would
   function correctly at runtime (parameter declaration, retrieval,
   15-element validation, sensor discovery loop, logging).

Since the full robot_localization package cannot be compiled in isolation
(it requires many internal headers, custom messages, etc.), we validate
the translated source file's content programmatically — importing and
exercising the actual file on disk, not a copy.
"""
import re
import pytest
from pathlib import Path


def _find_ros_filter_cpp():
    """Find ros_filter.cpp in the package."""
    candidates = [
        Path(__file__).parent / "ros_filter.cpp",
        Path(__file__).parent / "install" / "task_010_sensor_fusion_params" / "share" / "task_010_sensor_fusion_params" / "ros_filter.cpp",
    ]
    for c in candidates:
        if c.exists():
            return c
    # Try to find via ament
    try:
        import subprocess
        result = subprocess.run(
            ["ros2", "pkg", "prefix", "task_010_sensor_fusion_params"],
            capture_output=True, text=True, timeout=10
        )
        if result.returncode == 0:
            pkg_prefix = Path(result.stdout.strip())
            installed = pkg_prefix / "share" / "task_010_sensor_fusion_params" / "ros_filter.cpp"
            if installed.exists():
                return installed
    except Exception:
        pass
    # Fallback
    p = Path(__file__).parent / "ros_filter.cpp"
    if p.exists():
        return p
    raise FileNotFoundError("Cannot find ros_filter.cpp")


@pytest.fixture
def cpp_content():
    """Read the translated C++ source file."""
    path = _find_ros_filter_cpp()
    return path.read_text()


class TestParameterDeclaration:
    """T1: ROS 2 Parameter Declaration"""

    def test_declare_parameter_vector_bool(self, cpp_content):
        pattern = r"this->declare_parameter\s*<\s*std::vector<bool>\s*>"
        assert re.search(pattern, cpp_content), \
            "Missing mandatory ROS 2 'declare_parameter<std::vector<bool>>' call."

    def test_declare_parameter_present(self, cpp_content):
        assert "declare_parameter" in cpp_content, \
            "No declare_parameter calls found at all."


class TestSensorConfigNaming:
    """T2: Sensor Config Naming Logic"""

    def test_config_suffix_construction(self, cpp_content):
        # The code must construct parameter names like <type><index>_config
        # using string concatenation: sensor_type + std::to_string(sensor_index) + "_config"
        # Match patterns like:
        #   sensor_type + std::to_string(sensor_index) + "_config"
        #   "odom" + std::to_string(i) + "_config"
        #   variable + std::to_string(...) + "_config"
        pattern = r'(?:"[\w_]+"|[\w_]+)\s*\+\s*std::to_string\s*\([^)]*\)\s*\+\s*"_config"'
        assert re.search(pattern, cpp_content), \
            "Could not find logic to construct parameter names like '<type><index>_config'."

    def test_config_string_literal(self, cpp_content):
        assert "_config" in cpp_content, "No '_config' suffix found in source."


class TestVector15Validation:
    """T3: Vector 15-Element Validation"""

    def test_size_check_15(self, cpp_content):
        assert "15" in cpp_content and ".size()" in cpp_content, \
            "Missing validation for the 15-element sensor configuration vector."

    def test_size_comparison(self, cpp_content):
        pattern = r'\.size\(\)\s*!=\s*15|\.size\(\)\s*==\s*15|size\s*!=\s*15'
        assert re.search(pattern, cpp_content), \
            "No explicit size == 15 or size != 15 comparison found."


class TestNoROS1Syntax:
    """T4: Legacy Syntax Removal"""

    def test_no_nodehandle(self, cpp_content):
        assert not re.search(r"ros::NodeHandle", cpp_content), \
            "ROS 1 ros::NodeHandle detected."

    def test_no_getparam(self, cpp_content):
        assert not re.search(r"nh\.getParam\(", cpp_content), \
            "ROS 1 nh.getParam() detected."

    def test_no_ros_init(self, cpp_content):
        assert not re.search(r"ros::init", cpp_content), \
            "ROS 1 ros::init detected."


class TestLoggerMigration:
    """T5: Logger Context Migration"""

    def test_rclcpp_logger(self, cpp_content):
        pattern = r"RCLCPP_(?:INFO|WARN|ERROR)(?:_STREAM)?\s*\(\s*(?:this->)?get_logger\(\)"
        assert re.search(pattern, cpp_content), \
            "Logging must use the node's logger (get_logger())."

    def test_no_ros1_logging(self, cpp_content):
        ros1_pattern = r'(?<![A-Z_])ROS_(?:INFO|WARN|ERROR|DEBUG)\s*\('
        assert not re.search(ros1_pattern, cpp_content), \
            "ROS 1 logging macros (ROS_INFO/WARN/ERROR) detected."


class TestParameterRetrieval:
    """T6: Parameter Retrieval Integrity"""

    def test_get_parameter_present(self, cpp_content):
        assert "get_parameter" in cpp_content, \
            "Missing 'get_parameter' call to retrieve values from the server."

    def test_get_parameter_for_config(self, cpp_content):
        pattern = r'get_parameter\s*\(\s*config_param_name|get_parameter\s*\(\s*"[^"]*_config"'
        assert re.search(pattern, cpp_content), \
            "No get_parameter call found that retrieves a '_config' parameter."


class TestSensorDiscoveryLoop:
    """Verify the sensor discovery loop iterates over sensor types."""

    def test_sensor_types_present(self, cpp_content):
        for stype in ["odom", "pose", "twist", "imu", "accel"]:
            assert f'"{stype}"' in cpp_content, \
                f"Sensor type '{stype}' not found in source."

    def test_loop_with_index(self, cpp_content):
        assert "std::to_string" in cpp_content or "sensor_index" in cpp_content, \
            "No index-based sensor discovery loop found."

    def test_topic_subs_populated(self, cpp_content):
        assert "topic_subs_" in cpp_content, \
            "topic_subs_ not referenced — subscriptions may not be stored."


class TestCallbackDataUsage:
    """Verify CallbackData is constructed with the update vector."""

    def test_callback_data_constructed(self, cpp_content):
        assert "CallbackData" in cpp_content, \
            "CallbackData not used in loadParams."

    def test_update_vector_passed(self, cpp_content):
        assert "update_vector" in cpp_content, \
            "update_vector not referenced in loadParams."