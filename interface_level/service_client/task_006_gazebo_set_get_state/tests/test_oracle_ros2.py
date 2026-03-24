import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "gazebo_model_states.cpp"


def _code() -> str:
    return CPP_FILE.read_text(encoding="utf-8", errors="ignore")

def _assert_has(pat: str, code: str, msg: str, flags=0):
    if not re.search(pat, code, flags):
        raise AssertionError(msg + f"\nMissing pattern: {pat}")

def _assert_not_has(pat: str, code: str, msg: str, flags=0):
    if re.search(pat, code, flags):
        raise AssertionError(msg + f"\nForbidden pattern found: {pat}")


class TestOracleGazeboSetGetStateServiceClient:
    def test_01_ros2_only_no_ros1_residue(self):
        code = _code()
        _assert_has(r"\brclcpp\b", code, "Must use ROS2 rclcpp (include/namespace).")
        for pat, why in [
            (r"\bros::NodeHandle\b", "ROS1 NodeHandle should not appear."),
            (r"\bros::ServiceClient\b", "ROS1 ServiceClient should not appear."),
            (r"\bros::Subscriber\b", "ROS1 Subscriber should not appear."),
            (r"\bros::init\b", "ROS1 ros::init should not appear."),
            (r"\bROS_(INFO|ERROR|WARN|DEBUG)\b", "ROS1 logging macros should not appear."),
            (r"\.call\s*\(", "ROS1 synchronous service .call(...) should not appear."),
        ]:
            _assert_not_has(pat, code, why)

    def test_02_service_client_type_and_name(self):
        code = _code()
        _assert_has(
            r"create_client\s*<\s*gazebo_msgs::srv::SetModelState\s*>",
            code,
            "Must create ROS2 client for gazebo_msgs::srv::SetModelState (create_client<...>).",
        )
        _assert_has(
            r'"/gazebo/set_model_state"\s*',
            code,
            'Must target Gazebo service "/gazebo/set_model_state".',
        )

    def test_03_retry_wait_for_service_with_shutdown_guard(self):
        code = _code()
        _assert_has(r"wait_for_service\s*\(", code, "Must call wait_for_service(...).")
        _assert_has(r"rclcpp::ok\s*\(\s*\)", code, "Must check rclcpp::ok() while waiting/retrying.")
        _assert_has(
            r"\b(while|for|do)\b[\s\S]{0,500}\bwait_for_service\s*\(",
            code,
            "Service availability should be retried in a loop (while/for/do) using wait_for_service(...).",
            flags=re.DOTALL,
        )

    def test_04_request_assigns_all_model_state_fields(self):
        code = _code()
        for pat, msg in [
            (r"(request|req)\s*->\s*model_state\s*\.\s*model_name\s*=", "Must assign request->model_state.model_name."),
            (r"(request|req)\s*->\s*model_state\s*\.\s*reference_frame\s*=", "Must assign request->model_state.reference_frame."),
            (r"(request|req)\s*->\s*model_state\s*\.\s*pose\s*=", "Must assign request->model_state.pose."),
            (r"(request|req)\s*->\s*model_state\s*\.\s*twist\s*=", "Must assign request->model_state.twist."),
        ]:
            _assert_has(pat, code, msg)

    def test_05_full_service_call_chain_success_and_failure_handling(self):
        code = _code()

        _assert_has(r"async_send_request\s*\(", code, "Must call async_send_request(request).")
        _assert_has(r"spin_until_future_complete\s*\(", code, "Must wait via spin_until_future_complete(...).")

        # Require explicit SUCCESS check
        _assert_has(
            r"spin_until_future_complete\s*\([^;]*\)\s*==\s*rclcpp::FutureReturnCode::SUCCESS",
            code,
            "Must check that spin_until_future_complete(...) returns SUCCESS.",
            flags=re.DOTALL,
        )

        # High-tier strictness: require a non-SUCCESS handling path (timeout/failure)
        # e.g., '!= SUCCESS' or an 'else' branch with RCLCPP_ERROR/WARN and return
        _assert_has(
            r"(!=\s*rclcpp::FutureReturnCode::SUCCESS)|"
            r"(else[\s\S]{0,300}RCLCPP_(ERROR|WARN)[\s\S]{0,200}(return|throw))",
            code,
            "Must handle the non-SUCCESS case (timeout/failure) after spin_until_future_complete.",
            flags=re.DOTALL,
        )

        _assert_has(
            r"\b(auto|const\s+auto)\s+\w+\s*=\s*\w+\.get\s*\(\s*\)\s*;",
            code,
            "Must retrieve service response from future via future.get() and store it.",
        )
        _assert_has(
            r"\w+\s*->\s*success\b",
            code,
            "Must inspect response->success to decide success/failure handling.",
        )
        _assert_has(
            r"\bRCLCPP_(INFO|ERROR|WARN|DEBUG)\b",
            code,
            "Must use ROS2 logging (RCLCPP_*).",
        )

    def test_06_task_specific_subscriptions_combined(self):
        code = _code()
        for pat, msg in [
            (r'"/gazebo/model_states"\s*', 'Must subscribe to "/gazebo/model_states".'),
            (r'"/gazebo/link_states"\s*', 'Must subscribe to "/gazebo/link_states".'),
            (r"\bgazebo_msgs::msg::ModelStates\b", "Must use gazebo_msgs::msg::ModelStates."),
            (r"\bgazebo_msgs::msg::LinkStates\b", "Must use gazebo_msgs::msg::LinkStates."),
            (r'"ball"\s*', 'Must reference model name "ball".'),
            (r'"ball::body"\s*', 'Must reference link name "ball::body".'),
        ]:
            _assert_has(pat, code, msg)

        _assert_has(
            r"\bname\b[\s\S]{0,600}\bpose\b|\bpose\b[\s\S]{0,600}\bname\b",
            code,
            "Callbacks should connect msg.name to msg.pose (indexing concept).",
            flags=re.DOTALL,
        )

    # High-tier strictness: callbacks should use SharedPtr (ROS2 canonical, avoids pass-by-value)
    def test_07_callbacks_use_sharedptr_message_types(self):
        code = _code()
        _assert_has(
            r"gazebo_msgs::msg::ModelStates::SharedPtr",
            code,
            "ModelStates callback should accept a SharedPtr message (ROS2 canonical signature).",
        )
        _assert_has(
            r"gazebo_msgs::msg::LinkStates::SharedPtr",
            code,
            "LinkStates callback should accept a SharedPtr message (ROS2 canonical signature).",
        )

    # High-tier strictness: node pointer passed into spin_until_future_complete should be a shared_ptr (or shared_from_this)
    def test_08_waits_for_future_via_spin_or_executor(self):
        code = _code()

    # Accept either rclcpp::spin_until_future_complete(node, future, ...)
    # OR executor.spin_until_future_complete(future, ...)
        _assert_has(
            r"(rclcpp::spin_until_future_complete\s*\()|"
            r"(spin_until_future_complete\s*\(\s*\w+\s*\))",
            code,
            "Must wait for the service future using rclcpp::spin_until_future_complete(...) or an executor's spin_until_future_complete(...).",
            flags=re.DOTALL,
        )

