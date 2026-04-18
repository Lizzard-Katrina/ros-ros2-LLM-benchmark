import re
from pathlib import Path

# --- File Path Definitions ---
# Using the structure you specified: Path(__file__).resolve().parents[1]
IK_SOLVER_FILE = Path(__file__).resolve().parents[1] / "moveit_ik_solver.cpp"
EVAL_FILE = Path(__file__).resolve().parents[1] / "manipulability_moveit.cpp"
TARGET_GEN_FILE = Path(__file__).resolve().parents[1] / "transformed_point_cloud_target_pose_generator.cpp"

def get_content(file_path):
    # Fixed: Changed "current" to "r" (read mode)
    with open(file_path, "r", encoding="utf-8") as f:
        return f.read()

# --- 1. Global ROS 1 Remnants Check ---
def test_ros1_remnants_check():
    """
    Check if the files are free from ROS 1 namespaces, headers, and macros.
    """
    file_map = {
        "moveit_ik_solver.cpp": IK_SOLVER_FILE,
        "manipulability_moveit.cpp": EVAL_FILE,
        "transformed_point_cloud_target_pose_generator.cpp": TARGET_GEN_FILE
    }
    
    for f_name, f_path in file_map.items():
        content = get_content(f_path)
        
        # Check for ROS 1 namespaces and headers
        assert "ros::" not in content, f"FAILED in {f_name}: Detected ROS 1 'ros::' namespace."
        assert "ros/ros.h" not in content, f"FAILED in {f_name}: Detected ROS 1 header."
        
        # Check for ROS 1 logging macros
        assert "ROS_INFO" not in content, f"FAILED in {f_name}: Found ROS 1 logging macro."
        assert "ROS_ERROR" not in content, f"FAILED in {f_name}: Found ROS 1 logging macro."
        
        # Check for specific ROS 1 classes
        ros1_patterns = [r'ros::Time', r'ros::Duration', r'ros::Rate', r'ros::NodeHandle']
        for pattern in ros1_patterns:
            assert not re.search(pattern, content), f"FAILED in {f_name}: Found ROS 1 pattern '{pattern}'."

# --- 2. IK Solver Logic Check ---
def test_ros2_compliance_and_ik():
    """
    Verify MoveIt 2 IK solver logic and RobotState management.
    """
    content = get_content(IK_SOLVER_FILE)
    
    # Ensure RobotState is updated before solving
    assert re.search(r'state\.update\(\);', content), "RobotState::update() must be called."

    # Flexible IK call check: 
    # Accepts setFromIK or searchPositionIK
    # Accepts boost::bind or C++ Lambdas for callbacks
    ik_logic = r'(setFromIK|searchPositionIK).*?isIKSolutionValid'
    assert re.search(ik_logic, content, re.DOTALL), "IK logic must use MoveIt 2 APIs with validity callback."

# --- 3. TF2 and Target Generation Check ---
def test_tf2_ros2_migration():
    """
    Verify TF2 migration, including lookup and Eigen integration.
    """
    content = get_content(TARGET_GEN_FILE)
    
    # Flexible duration: Accepts tf2::durationFromSec, std::chrono, or rclcpp::Duration
    duration_pattern = r'(tf2::durationFromSec|std::chrono::seconds|rclcpp::Duration)'
    assert re.search(duration_pattern, content), "Must use ROS 2 compatible duration handling."

    # Core TF2 components
    assert "lookupTransform" in content, "Must use lookupTransform."
    assert "tf2::TimePointZero" in content, "Must use tf2::TimePointZero for lookup."
    assert "tf2::transformToEigen" in content, "Must use tf2::transformToEigen."

    # Transformation application check
    # Accepts in-place modification or push_back into new vector
    apply_pattern = r'(pose\s*=\s*transform\s*\*\s*pose|push_back\(transform\s*\*\s*pose\))'
    assert re.search(apply_pattern, content), "The transform must be applied to the target poses."

# --- 4. Manipulability Logic Check ---
def test_manipulability_eval_logic():
    """
    Verify Jacobian matrix processing and SVD score calculation.
    """
    content = get_content(EVAL_FILE)
    
    assert "getJacobian" in content, "Must extract the Jacobian matrix."
    assert "Eigen::JacobiSVD" in content, "Must use SVD for manipulability analysis."
    assert "singularValues" in content, "Must compute singular values."
