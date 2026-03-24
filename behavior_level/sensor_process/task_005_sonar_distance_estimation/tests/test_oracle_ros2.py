import re
from pathlib import Path

CPP_FILE = Path(__file__).resolve().parents[1] /"turtle.cpp"
def get_content():
    with open(CPP_FILE, 'r') as f:
        # Keep it pure: No comments, just the logic
        return re.sub(r'//.*?\n|/\*.*?\*/', '', f.read(), flags=re.DOTALL)

def test_holonomic_kinematics():
    """Concept: 2D Physics. Checks for X/Y velocity coupling."""
    content = get_content()
    x_pattern = r"pos_\.rx\(\)\s*\+?=\s*.*?(?:cos|sin).*?lin_vel_x_"
    y_pattern = r"pos_\.ry\(\)\s*[-+]=\s*.*?(?:cos|sin).*?lin_vel_y_"
    assert re.search(x_pattern, content) and re.search(y_pattern, content), \
        "Failure: Incomplete kinematics. Missing X or Y velocity projection."

def test_sonar_geometry_robust_v4():
    """Robustly checks for the intersection formula: (delta / direction)."""
    content = get_content()
    # Support both direct division and intermediate variables (dx, dy)
    # Matches: / dx, / dy, / cos(, / sin(
    intersection_math = r"/\s*(?:dx|dy|(?:std::)?(?:cos|sin)\()"
    assert re.search(intersection_math, content) and "canvas_width" in content, \
        "Failure: Geometric Error. Could not find analytical distance calculation (delta / projection)."

def test_numerical_stability_epsilon():
    """
    Concept: Numerical Robustness.
    FAIL if: Only checks for direction (dx < 0).
    PASS if: Checks for magnitude (abs(dx) > epsilon) to prevent division by zero.
    """
    content = get_content()
    # The AI's previous code only did 'if (dx < 0.0)'. This is unstable.
    # We look for a check against a small value (1e-6, 0.0001) or a non-zero check.
    stability_check = r"(?:std::)?(?:abs|fabs)\(d[xy]\)\s*>\s*(?:0|1e-|0\.)"
    assert re.search(stability_check, content), \
        "Failure: Stability Error. Missing epsilon guard. division-by-zero will occur when rays are axial."

def test_sonar_max_range_limit():
    """
    Concept: Sensor Realism.
    FAIL if: Returns numeric_limits::max() when no wall is hit.
    PASS if: Implements a logical max sensing distance (e.g., 5.0).
    """
    content = get_content()
    # If the sonar doesn't hit a wall, it shouldn't return infinity.
    # We check if there's a constant or a min() cap applied to the final sonar_distance.
    range_limit = r"(?:sonar_distance_|sonar_dist).*?=\s*.*?(?:\d+\.\d+|range_max|max_range)"
    assert re.search(range_limit, content) and "max()" not in content.split("sonar_distance_")[-1], \
        "Failure: Sensor Range Error. Sonar must have a finite maximum range limit."

def test_sonar_y_mirroring_fix():
    """
    Concept: Coordinate Alignment.
    Checks if dy uses -sin(theta) to account for Qt's inverted Y-axis.
    """
    content = get_content()
    # If dy = sin(theta), the sonar is upside down in turtlesim's world.
    assert re.search(r"dy\s*=\s*-\s*(?:std::)?sin", content), \
        "Failure: Mapping Error. Sonar Y-direction must be inverted for Qt coordinates."

def test_frame_transformation_accuracy():
    """Standard turtlesim Y-axis flip check for pose publication."""
    content = get_content()
    y_flip = r"p->y\s*=\s*canvas_height\s*-\s*pos_\.y\(\)"
    assert re.search(y_flip, content), \
        "Failure: Frame Error. Failed to map internal Y to Pose message convention."
