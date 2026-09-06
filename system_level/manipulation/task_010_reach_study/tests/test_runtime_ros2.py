"""
Runtime test for task_010_reach_study.

Since this package contains C++ source files that depend on external reach_ros headers
(which are not available as standard ros-humble packages), we perform source-level
verification that the translated files exist, are syntactically consistent with ROS2,
and contain the correct filled-in logic. We also verify the package builds successfully
and that the files are installed to the expected location.
"""
import os
import re
import subprocess
import pytest
from pathlib import Path


# Locate the source files at the package root
PKG_ROOT = Path(__file__).resolve().parent

IK_SOLVER_FILE = PKG_ROOT / "moveit_ik_solver.cpp"
EVAL_FILE = PKG_ROOT / "manipulability_moveit.cpp"
TARGET_GEN_FILE = PKG_ROOT / "transformed_point_cloud_target_pose_generator.cpp"


def get_content(file_path):
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()


def has_ros1_namespace(content):
    """Check for ROS1 'ros::' namespace usage, excluding valid ROS2 patterns like tf2_ros::."""
    # Find all occurrences of 'ros::' and check they are not preceded by valid ROS2 prefixes
    for match in re.finditer(r'ros::', content):
        start = match.start()
        # Check if this 'ros::' is part of a valid ROS2 namespace like tf2_ros::
        prefix = content[max(0, start - 10):start]
        if re.search(r'(tf2_|nav2_|move_base_|_)$', prefix):
            continue
        # It's a standalone 'ros::' which is ROS1
        return True
    return False


class TestFilesExist:
    """Verify all translated source files exist."""

    def test_ik_solver_exists(self):
        assert IK_SOLVER_FILE.exists(), f"Missing: {IK_SOLVER_FILE}"

    def test_manipulability_exists(self):
        assert EVAL_FILE.exists(), f"Missing: {EVAL_FILE}"

    def test_target_gen_exists(self):
        assert TARGET_GEN_FILE.exists(), f"Missing: {TARGET_GEN_FILE}"


class TestNoROS1Remnants:
    """Ensure no ROS1 artifacts remain in any file."""

    @pytest.fixture(autouse=True)
    def load_contents(self):
        self.files = {
            "moveit_ik_solver.cpp": get_content(IK_SOLVER_FILE),
            "manipulability_moveit.cpp": get_content(EVAL_FILE),
            "transformed_point_cloud_target_pose_generator.cpp": get_content(TARGET_GEN_FILE),
        }

    def test_no_ros_namespace(self):
        for fname, content in self.files.items():
            assert not has_ros1_namespace(content), f"Found ROS1 'ros::' in {fname}"

    def test_no_ros_header(self):
        for fname, content in self.files.items():
            assert "ros/ros.h" not in content, f"Found 'ros/ros.h' in {fname}"

    def test_no_ros1_logging(self):
        for fname, content in self.files.items():
            assert "ROS_INFO" not in content, f"Found ROS_INFO in {fname}"
            assert "ROS_ERROR" not in content, f"Found ROS_ERROR in {fname}"

    def test_no_ros1_time_patterns(self):
        for fname, content in self.files.items():
            # Check for standalone ros:: patterns (ROS1), not tf2_ros:: etc.
            for pat_name, pat in [
                ('ros::Time', r'(?<!tf2_)ros::Time'),
                ('ros::Duration', r'(?<!tf2_)ros::Duration'),
                ('ros::Rate', r'(?<!tf2_)ros::Rate'),
                ('ros::NodeHandle', r'(?<!tf2_)ros::NodeHandle'),
            ]:
                assert not re.search(pat, content), f"Found '{pat_name}' in {fname}"


class TestIKSolverLogic:
    """Verify the IK solver TODO was filled correctly."""

    @pytest.fixture(autouse=True)
    def load_content(self):
        self.content = get_content(IK_SOLVER_FILE)

    def test_state_update_called(self):
        assert re.search(r'state\.update\(\);', self.content), \
            "RobotState::update() must be called in solveIK"

    def test_setFromIK_with_callback(self):
        ik_logic = r'(setFromIK|searchPositionIK).*?isIKSolutionValid'
        assert re.search(ik_logic, self.content, re.DOTALL), \
            "IK logic must use setFromIK/searchPositionIK with isIKSolutionValid callback"

    def test_boost_bind_used(self):
        assert "boost::bind" in self.content, \
            "Must use boost::bind for the IK callback"

    def test_seed_subset_extraction(self):
        assert "extractSubset" in self.content, \
            "Must extract seed subset from the input map"

    def test_copy_joint_positions(self):
        assert "copyJointGroupPositions" in self.content, \
            "Must copy joint group positions after successful IK"

    def test_returns_solution_vector(self):
        assert re.search(r'return\s*\{\s*solution\s*\}', self.content), \
            "Must return {solution} on success"

    def test_returns_empty_on_failure(self):
        assert re.search(r'return\s*\{\s*\}', self.content), \
            "Must return {} on failure"


class TestManipulabilityLogic:
    """Verify the manipulability TODO was filled correctly."""

    @pytest.fixture(autouse=True)
    def load_content(self):
        self.content = get_content(EVAL_FILE)

    def test_getJacobian_called(self):
        assert "getJacobian" in self.content, \
            "Must call getJacobian to retrieve the Jacobian matrix"

    def test_svd_used(self):
        assert "Eigen::JacobiSVD" in self.content, \
            "Must use Eigen::JacobiSVD for SVD computation"

    def test_singular_values(self):
        assert "singularValues" in self.content, \
            "Must extract singular values from SVD"

    def test_state_update(self):
        assert re.search(r'state\.update\(\)', self.content), \
            "Must call state.update() after setting joint positions"

    def test_characteristic_length_implementation(self):
        assert "characteristic_length" in self.content, \
            "Must compute characteristic_length"
        assert "setToDefaultValues" in self.content, \
            "Must set state to default values in calculateCharacteristicLength"
        assert "getTipFrame" in self.content, \
            "Must get the TCP tip frame"
        assert "getActiveJointModels" in self.content, \
            "Must iterate over active joint models"

    def test_partial_jacobian_extraction(self):
        assert "partial_jacobian" in self.content, \
            "Must extract partial Jacobian when row subset is active"

    def test_no_ros1_model_loading(self):
        assert "getSharedRobotModel" not in self.content, \
            "Must not use ROS1 getSharedRobotModel"


class TestTF2Migration:
    """Verify the TF2 target pose generator was migrated correctly."""

    @pytest.fixture(autouse=True)
    def load_content(self):
        self.content = get_content(TARGET_GEN_FILE)

    def test_tf2_time_point_zero(self):
        assert "tf2::TimePointZero" in self.content, \
            "Must use tf2::TimePointZero"

    def test_transform_to_eigen(self):
        assert "tf2::transformToEigen" in self.content, \
            "Must use tf2::transformToEigen"

    def test_lookup_transform(self):
        assert "lookupTransform" in self.content, \
            "Must use lookupTransform"

    def test_chrono_duration(self):
        duration_pattern = r'(tf2::durationFromSec|std::chrono::seconds|rclcpp::Duration)'
        assert re.search(duration_pattern, self.content), \
            "Must use ROS2-compatible duration"

    def test_transform_applied(self):
        apply_pattern = r'(pose\s*=\s*transform\s*\*\s*pose|push_back\(transform\s*\*\s*pose\))'
        assert re.search(apply_pattern, self.content), \
            "Must apply transform to poses"

    def test_generate_function_present(self):
        assert "TransformedPointCloudTargetPoseGenerator::generate" in self.content, \
            "Must implement the generate() method"

    def test_rclcpp_clock(self):
        assert "rclcpp::Clock" in self.content, \
            "Must use rclcpp::Clock for the TF2 buffer"


class TestPackageBuild:
    """Verify the package can be found and built by colcon."""

    def test_package_xml_exists(self):
        assert (PKG_ROOT / "package.xml").exists()

    def test_cmake_exists(self):
        assert (PKG_ROOT / "CMakeLists.txt").exists()

    def test_package_xml_has_correct_name(self):
        content = get_content(PKG_ROOT / "package.xml")
        assert "task_010_reach_study" in content

    def test_colcon_build(self):
        """Actually attempt to build the package with colcon."""
        result = subprocess.run(
            ["colcon", "build", "--packages-select", "task_010_reach_study",
             "--event-handlers", "console_cohesion+"],
            cwd=str(PKG_ROOT.parent),
            capture_output=True,
            text=True,
            timeout=120,
        )
        assert result.returncode == 0, \
            f"colcon build failed:\nstdout: {result.stdout}\nstderr: {result.stderr}"


class TestROS2APIUsage:
    """Verify ROS2-specific API patterns are used correctly across all files."""

    @pytest.fixture(autouse=True)
    def load_contents(self):
        self.ik_content = get_content(IK_SOLVER_FILE)
        self.manip_content = get_content(EVAL_FILE)
        self.tpg_content = get_content(TARGET_GEN_FILE)

    def test_rclcpp_node_in_ik_solver(self):
        assert "rclcpp::Node::make_shared" in self.ik_content, \
            "IK solver must use rclcpp::Node::make_shared"

    def test_rclcpp_node_in_manipulability(self):
        assert "rclcpp::Node::make_shared" in self.manip_content, \
            "Manipulability must use rclcpp::Node::make_shared"

    def test_moveit_msgs_msg_namespace(self):
        assert "moveit_msgs::msg::" in self.ik_content, \
            "IK solver must use moveit_msgs::msg:: namespace (ROS2)"

    def test_planning_scene_msg_type(self):
        assert "moveit_msgs/msg/planning_scene.hpp" in self.ik_content, \
            "Must include ROS2 planning_scene message header"

    def test_qos_in_publisher(self):
        assert "rclcpp::QoS" in self.ik_content, \
            "Must use rclcpp::QoS for publisher configuration"

    def test_tf2_eigen_hpp(self):
        assert "tf2_eigen/tf2_eigen.hpp" in self.tpg_content, \
            "Must include ROS2 tf2_eigen header (.hpp)"