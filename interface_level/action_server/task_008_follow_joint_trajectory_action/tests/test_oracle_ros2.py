import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] / "trajectory_planner_ros.cpp"

def read_code():
    code= CPP_FILE.read_text()
    code = re.sub(r'/\*[\s\S]*?\*/', '', code)
    code = re.sub(r'//.*', '', code)
    return code

# ------------------------
# Concept 1: class exists
# ------------------------
def test_class_exists():
    code = read_code()
    assert re.search(r"class\s+TrajectoryPlannerROS", code), "TrajectoryPlannerROS class not found"

# ------------------------
# Concept 2: checkTrajectory method exists
# ------------------------
def test_check_trajectory_method():
    code = read_code()
    assert re.search(r"bool\s+TrajectoryPlannerROS::checkTrajectory\s*\(", code), "checkTrajectory method missing"

# ------------------------
# Concept 3: uses costmap_ros_->getRobotPose
# ------------------------
def test_get_robot_pose_used():
    code = read_code()
    assert re.search(r"costmap_ros_->getRobotPose", code), "Robot pose acquisition via costmap_ros_ missing"

# ------------------------
# Concept 4: handles update_map logic
# ------------------------
def test_update_map_logic():
    code = read_code()
    assert re.search(r"if\s*\(\s*update_map\s*\)", code), "update_map conditional not implemented"
    assert re.search(r"tc_->updatePlan", code), "update_map branch must call tc_->updatePlan"

# ------------------------
# Concept 5: uses odometry with lock
# ------------------------
def test_odom_lock_usage():
    code = read_code()
    assert re.search(r"boost::recursive_mutex::scoped_lock", code), "odom_lock usage missing"
    assert re.search(r"base_odom\s*=\s*base_odom_", code), "Odometry copy inside lock missing"

# ------------------------
# Concept 6: calls tc_->checkTrajectory with correct args
# ------------------------
def test_tc_check_trajectory_args():
    code = read_code()
    pattern = (
        r"tc_->checkTrajectory\s*\(\s*global_pose\.pose\.position\.x\s*,"
        r"\s*global_pose\.pose\.position\.y\s*,"
        r"\s*tf2::getYaw\(global_pose\.pose\.orientation\)\s*,"
        r"\s*base_odom\.twist\.twist\.linear\.x\s*,"
        r"\s*base_odom\.twist\.twist\.linear\.y\s*,"
        r"\s*base_odom\.twist\.twist\.angular\.z\s*,"
        r"\s*vx_samp\s*,\s*vy_samp\s*,\s*vtheta_samp"
    )
    assert re.search(pattern, code, re.DOTALL), "tc_->checkTrajectory call with correct args missing"

# ------------------------
# Concept 7: returns boolean
# ------------------------
def test_returns_bool():
    code = read_code()
    assert re.search(r"return\s+tc_->checkTrajectory", code), "checkTrajectory must return boolean result"

# ------------------------
# Concept 8: warns if getRobotPose fails
# ------------------------
def test_warns_on_pose_fail():
    code = read_code()
    assert re.search(r"ROS_WARN.*Failed to get the pose", code), "Missing ROS_WARN for failed getRobotPose"

# ------------------------
# Concept 9: no ROS1 remnants
# ------------------------
def test_no_ros1_api_leftovers():
    code = read_code()
    forbidden = [
        r"ros::", r"tf::", r"nav_msgs::Odometry", r"costmap_2d::Costmap2DROS"
    ]
    for f in forbidden:
        assert not re.search(f, code), f"ROS1 API leftover detected: {f}"
