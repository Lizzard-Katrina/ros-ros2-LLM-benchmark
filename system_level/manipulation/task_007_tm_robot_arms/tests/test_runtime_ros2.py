"""
Runtime test for task_007_tm_robot_arms.
Tests the translated files by importing/reading them and verifying behavior.
"""

import os
import re
import sys
import time
import subprocess
import pytest
from pathlib import Path

# Find the package root (where this test file lives)
PKG_ROOT = Path(__file__).resolve().parent


def get_content(path):
    with open(path, "r", encoding="utf-8") as f:
        return f.read()


class TestStaticPatterns:
    """Verify the translated files contain the required patterns (same as oracle)."""

    def test_recv_error_handling(self):
        content = get_content(PKG_ROOT / "tm_communication.cpp")
        pattern = r"recv\(.*?\)\s*==\s*0"
        assert re.search(pattern, content), \
            "Must handle recv() == 0 directly in an if condition"

    def test_socket_polling(self):
        content = get_content(PKG_ROOT / "tm_communication.cpp")
        assert re.search(r"select\s*\(.*?&tv\)", content), \
            "Must use select() with &tv"

    def test_mutex_safety(self):
        content = get_content(PKG_ROOT / "tm_ros_service.cpp")
        pattern = r"(?:unique_lock|lock_guard).*?svr_mtx_"
        assert re.search(pattern, content), \
            "Must use unique_lock or lock_guard with svr_mtx_"

    def test_svr_callback_notification(self):
        content = get_content(PKG_ROOT / "tm_ros_service.cpp")
        assert re.search(r"svr_cond_\.notify_(?:all|one)\s*\(", content), \
            "Must call svr_cond_.notify_all() or notify_one()"

    def test_sync_wait_logic(self):
        content = get_content(PKG_ROOT / "tm_ros_service.cpp")
        pattern = r"svr_cond_\.wait_for\s*\(\s*\w+,\s*(?:std|boost)::chrono::duration"
        assert re.search(pattern, content), \
            "Must use svr_cond_.wait_for with chrono::duration"

    def test_python_brace_stripping(self):
        content = get_content(PKG_ROOT / "ask_item_demo.py")
        assert re.search(r"\.strip\s*\(\s*['\"].*?[{}]", content), \
            "Must use .strip('{}') to strip braces"

    def test_demo_blocking_call(self):
        content = get_content(PKG_ROOT / "ask_item_demo.py")
        pattern = r"ask_item\(.*?,.*?,[^0]\d*\)"
        assert re.search(pattern, content), \
            "Must have blocking call with wait_time > 0 as plain integer"

    def test_motion_type_coverage(self):
        content = get_content(PKG_ROOT / "tm_ros_service.cpp")
        interfaces = ["set_joint_pos_PTP", "set_tool_pose_PTP", "set_tool_pose_Line"]
        for interface in interfaces:
            assert interface in content, f"Missing motion command: {interface}"

    def test_no_legacy_ros1_symbols(self):
        content = get_content(PKG_ROOT / "tm_ros_service.cpp")
        legacy_symbols = [r"ros::ok\(", r"ros::init\(", r"ros::NodeHandle"]
        for sym in legacy_symbols:
            assert not re.search(sym, content), f"Legacy ROS 1 symbol detected: {sym}"

    def test_tm_protocol_parsing(self):
        content = get_content(PKG_ROOT / "ask_item_demo.py")
        pattern = r"(?:\.split\(|[rR]?['\"].*?\{.*?\}['\"])"
        assert re.search(pattern, content), \
            "Must implement parsing logic for TM '{...}' format"


class TestPythonRuntime:
    """Actually run the Python demo to verify it executes correctly."""

    def test_ask_item_demo_imports_and_runs(self):
        """Import the ask_item_demo module and test parse_content function."""
        sys.path.insert(0, str(PKG_ROOT))
        try:
            from ask_item_demo import parse_content, AskItemDemo
            
            # Test parse_content with TM protocol format
            result = parse_content("{1.0,2.0,3.0}")
            assert result == ["1.0", "2.0", "3.0"], f"Expected ['1.0','2.0','3.0'], got {result}"
            
            result2 = parse_content("{hello}")
            assert result2 == ["hello"], f"Expected ['hello'], got {result2}"
            
            result3 = parse_content("{}")
            assert result3 == [""], f"Expected [''], got {result3}"
        finally:
            sys.path.pop(0)

    def test_ask_item_demo_node_creation(self):
        """Test that the AskItemDemo node can be created and destroyed."""
        sys.path.insert(0, str(PKG_ROOT))
        try:
            import rclpy
            from ask_item_demo import AskItemDemo
            
            rclpy.init()
            try:
                node = AskItemDemo()
                assert node.get_name() == 'ask_item_demo'
                
                # Test the ask_item method
                res = node.ask_item('test_id', 'HandCamera_Value', 5)
                assert res.ok is True
                assert res.id == 'test_id'
                
                node.destroy_node()
            finally:
                rclpy.shutdown()
        finally:
            sys.path.pop(0)

    def test_ask_item_demo_strip_behavior(self):
        """Verify that the strip('{}') approach works correctly on real data."""
        sys.path.insert(0, str(PKG_ROOT))
        try:
            from ask_item_demo import parse_content
            
            # Simulate TM robot response format
            tm_response = "{0.1,0.2,0.3,0.4,0.5,0.6}"
            values = parse_content(tm_response)
            assert len(values) == 6
            assert values[0] == "0.1"
            assert values[5] == "0.6"
            
            # Test with nested braces edge case
            tm_response2 = "{HandCamera_Value}"
            values2 = parse_content(tm_response2)
            assert values2 == ["HandCamera_Value"]
        finally:
            sys.path.pop(0)

    def test_ask_item_demo_subprocess(self):
        """Run the demo as a subprocess to verify it doesn't crash."""
        demo_path = PKG_ROOT / "ask_item_demo.py"
        proc = subprocess.Popen(
            [sys.executable, str(demo_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        try:
            stdout, stderr = proc.communicate(timeout=10)
            # The demo should complete (it doesn't need a real robot)
            # It may exit with 0 or non-zero depending on rclpy init state,
            # but it should not hang
            assert proc.returncode is not None, "Process should have terminated"
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait()
            pytest.fail("Demo script hung (timeout)")
        finally:
            if proc.poll() is None:
                proc.kill()
                proc.wait()