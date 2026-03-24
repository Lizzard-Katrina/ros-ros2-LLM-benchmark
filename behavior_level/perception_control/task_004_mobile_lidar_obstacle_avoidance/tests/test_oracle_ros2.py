import re
from pathlib import Path

PY_FILE = Path(__file__).resolve().parents[1] / "laser_obstacle_avoid_360_node.py"


def _code() -> str:
    assert PY_FILE.exists(), f"Expected Python file at {PY_FILE}, but it does not exist."
    return PY_FILE.read_text(encoding="utf-8", errors="ignore")


def _assert_has(pattern: str, msg: str):
    code = _code()
    if re.search(pattern, code, flags=re.MULTILINE | re.DOTALL) is None:
        raise AssertionError(msg + f"\nMissing pattern:\n{pattern}")


def _assert_not_has(pattern: str, msg: str):
    code = _code()
    if re.search(pattern, code, flags=re.MULTILINE | re.DOTALL) is not None:
        raise AssertionError(msg + f"\nForbidden pattern found:\n{pattern}")



# 1) ROS2 scaffold + forbid ROS1
def test_ros2_scaffold_and_no_ros1():
    _assert_has(r'\bimport\s+rclpy\b', "Must import rclpy (ROS2 Python).")
    _assert_has(r'from\s+rclpy\.node\s+import\s+Node\b', "Must use rclpy Node (from rclpy.node import Node).")
    _assert_not_has(r'\bimport\s+rospy\b|rospy\.', "Must not use ROS1 rospy APIs in the ROS2 translation.")


# 2) ROS2 lifecycle + node creation + spin/shutdown
def test_ros2_lifecycle_and_node_creation():
    _assert_has(r'\brclpy\.init\s*\(', "Must call rclpy.init(...) in main().")
    _assert_has(r'\brclpy\.spin\s*\(', "Must spin the node with rclpy.spin(node).")
    _assert_has(r'\brclpy\.shutdown\s*\(', "Must call rclpy.shutdown() for clean exit.")
    _assert_has(
        r'(class\s+\w+\s*\(\s*Node\s*\)\s*:)|'
        r'(rclpy\.create_node\s*\(\s*[\'"][^\'"]+[\'"]\s*\))|'
        r'(\bNode\s*\(\s*[\'"][^\'"]+[\'"]\s*\))',
        "Must create a ROS2 node (Node subclass or Node('name') / rclpy.create_node('name')).",
    )


# 3) Interfaces: LaserScan sub /scan + Twist pub /cmd_vel
def test_scan_subscription_and_cmd_vel_publisher():
    _assert_has(r'from\s+sensor_msgs\.msg\s+import\s+LaserScan\b', "Must import LaserScan from sensor_msgs.msg.")
    _assert_has(r'from\s+geometry_msgs\.msg\s+import\s+Twist\b', "Must import Twist from geometry_msgs.msg.")
    _assert_has(
        r'create_subscription\s*\(\s*LaserScan\s*,\s*[\'"]/scan[\'"]\s*,',
        "Must create LaserScan subscription on '/scan'.",
    )
    _assert_has(
        r'create_publisher\s*\(\s*Twist\s*,\s*[\'"]/cmd_vel[\'"]',
        "Must create Twist publisher on '/cmd_vel'.",
    )


# 4) 10Hz control loop (timer) + publish
def test_ros2_timer_rate_and_publish():
    _assert_has(
        r'create_timer\s*\(\s*(0\.1|1\s*/\s*10|1\.0\s*/\s*10)',
        "Should use a ROS2 timer at ~10Hz (create_timer(0.1, ...) or 1/10).",
    )
    _assert_has(r'\.publish\s*\(\s*', "Must publish commands via publisher.publish(...).")


# 5) Keep 12-region structure + obstacle threshold constants exist
def test_keeps_reference_region_names_and_thresholds():
    _assert_has(
        r'\bREGIONS\b\s*=\s*\[.*front_C.*front_L.*left_R.*left_C.*left_L.*back_R.*back_C.*back_L.*right_R.*right_C.*right_L.*front_R.*\]',
        "Must keep the 12 named regions list REGIONS (front_C ... front_R) like the ROS1 reference.",
    )
    _assert_has(r'\bOBSTACLE_DIST\b|\bobstacle_dist\b', "Must keep obstacle threshold concept (OBSTACLE_DIST / obstacle_dist).")
    _assert_has(r'\bREGIONAL_ANGLE\b|\bregional_angle\b', "Must keep regional angle concept (REGIONAL_ANGLE / regional_angle).")
    _assert_has(r'\bPI\b|\bpi\b', "Must keep PI/pi constant for turn-time computation (reference uses PI).")


# 6) HARD fidelity gate: partition ranges by slicing into contiguous chunks
def test_partitions_ranges_by_slicing():
    _assert_has(r'\b(msg|scan)\.ranges\b', "Callback must read LaserScan ranges (msg.ranges / scan.ranges).")
    _assert_has(
        r'(ranges|msg\.ranges|scan\.ranges)\s*\[\s*[^:\]]+\s*:\s*[^:\]]+\s*\]',
        "Must chunk ranges via slicing (ranges[a:b]) to form contiguous regions (fidelity to ROS1 partitioning).",
    )
    _assert_has(r'enumerate\s*\(\s*REGIONS\s*\)', "Should iterate regions with enumerate(REGIONS) (or equivalent).")


# 7) IdentifyRegions strategy fidelity: filter obstacles by threshold and ignore inf
def test_identifyregions_filtering_strategy():
    # Must show filtering against threshold
    _assert_has(
        r'(<=\s*(OBSTACLE_DIST|obstacle_dist))|(<\s*(OBSTACLE_DIST|obstacle_dist))',
        "Region obstacle extraction must compare ranges to obstacle threshold (<= OBSTACLE_DIST).",
    )
    # Must show ignoring infinite readings (any common way)
    _assert_has(
        r"(float\('inf'\))|(\bmath\.isfinite\b)|(\bisfinite\b)|(\binf\b)|([\"']inf[\"'])",
        "Must ignore/handle infinite LaserScan readings (ROS1 reference excludes 'inf').",
    )
    # Must store per-region obstacle lists (Regions_Report concept)
    _assert_has(
        r'\bRegions_Report\b\s*=\s*\{|\bregions_report\b\s*=\s*\{',
        "Must keep Regions_Report dict (region -> list of obstacle readings) like reference.",
    )


# 8) Decision strategy fidelity: must have Regions_Distances signed cost table AND use abs(...) cost notion
def test_decision_uses_signed_cost_table_and_abs_cost():
    _assert_has(
        r'\bRegions_Distances\b\s*=\s*\{',
        "Must keep Regions_Distances mapping (reference uses signed costs around front_C).",
    )
    # evidence of negative and positive costs (right side negative, left side positive)
    _assert_has(
        r'Regions_Distances\s*=\s*\{[^}]*:\s*-\d+[^}]*:\s*\d+',
        "Regions_Distances should include both negative and positive integer costs (fidelity to reference).",
    )
    # decision cost uses abs(...) somewhere (reference uses abs(distance difference))
    _assert_has(
        r'\babs\s*\(',
        "Decision logic should use abs(...) for deviation cost like reference (abs cost to goal region).",
    )


# 9) Control law fidelity: act + angular_vel sign normalization + sleep turn-time formula + TRANS/NORMAL modes
def test_control_laws_and_modes_match_reference_semantics():
    # Must maintain Urgency_Report with act/angular_vel/sleep keys
    _assert_has(
        r'\bUrgency_Report\b\s*=\s*\{[^}]*[\'"]act[\'"][^}]*[\'"]angular_vel[\'"][^}]*[\'"]sleep[\'"][^}]*\}',
        "Must keep Urgency_Report dict with keys act/angular_vel/sleep (reference state).",
    )

    # Must compute angular_vel from TRANS_ANG_VEL and a sign/normalization of regional_dist
    _assert_has(r'\bTRANS_ANG_VEL\b|\btrans_ang_vel\b', "Must keep TRANS_ANG_VEL concept (reference constant turn rate).")
    _assert_has(
        r'(regional_dist|max\s*\(\s*1\s*,\s*abs\s*\(\s*regional_dist\s*\)\s*\))|'
        r'(\bregional_dist\b.*\babs\s*\(\s*regional_dist\s*\))',
        "angular_vel should depend on a signed regional_dist with normalization/sign logic (reference: regional_dist/max(1,abs(regional_dist))).",
    )
    _assert_has(
        r'angular_vel|Urgency_Report\s*\[\s*[\'"]angular_vel[\'"]\s*\]',
        "Must compute/store an angular_vel decision (Urgency_Report['angular_vel']).",
    )

    # Must compute sleep from abs(regional_dist)*REGIONAL_ANGLE*PI/(180*TRANS_ANG_VEL) in some recognizable way
    _assert_has(
        r'[\'"]sleep[\'"]',
        "Must compute/store a sleep/turn-duration field (reference keeps Urgency_Report['sleep']).",
    )
    _assert_has(
        r'(REGIONAL_ANGLE|regional_angle).*(180)|(180).*(REGIONAL_ANGLE|regional_angle)',
        "Turn duration computation should relate REGIONAL_ANGLE and 180 (degrees→radians/time) like reference.",
    )
    _assert_has(
        r'(PI|pi).*(TRANS_ANG_VEL|trans_ang_vel)|(TRANS_ANG_VEL|trans_ang_vel).*(PI|pi)',
        "Turn duration computation should relate PI and TRANS_ANG_VEL like reference.",
    )

    # Must implement the two motion modes: TRANS (avoidance) vs NORMAL (clear path)
    _assert_has(r'\bTRANS_LIN_VEL\b|\btrans_lin_vel\b', "Must keep TRANS_LIN_VEL (reverse/transition velocity) concept.")
    _assert_has(r'\bNORMAL_LIN_VEL\b|\bnormal_lin_vel\b', "Must keep NORMAL_LIN_VEL (forward cruise velocity) concept.")
    _assert_has(r'\.linear\.x\s*=', "Must assign Twist.linear.x somewhere.")
    _assert_has(r'\.angular\.z\s*=', "Must assign Twist.angular.z somewhere.")
    _assert_has(
        r'if\s+.*Urgency_Report\s*\[\s*[\'"]act[\'"]\s*\].*:|if\s*\(\s*.*Urgency_Report\s*\[\s*[\'"]act[\'"]\s*\].*\)\s*:',
        "Must branch on Urgency_Report['act'] to choose avoidance vs normal motion.",
    )
    _assert_has(
        r'angular\.z\s*=\s*.*Urgency_Report\s*\[\s*[\'"]angular_vel[\'"]\s*\]',
        "Avoidance motion must set Twist.angular.z from Urgency_Report['angular_vel'] (reference linkage).",
    )
