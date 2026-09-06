"""
Mock action types for Door and MoveBase since the original
pr2_doors and move_base_msgs action types don't exist as ROS2 packages.
"""


class _Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Header:
    def __init__(self):
        self.frame_id = ""


class Door:
    ROT_DIR_COUNTERCLOCKWISE = 1
    HINGE_P2 = 2

    def __init__(self):
        self.frame_p1 = _Point()
        self.frame_p2 = _Point()
        self.door_p1 = _Point()
        self.door_p2 = _Point()
        self.travel_dir = _Point()
        self.rot_dir = 0
        self.hinge = 0
        self.header = _Header()


class DoorGoal:
    def __init__(self):
        self.door = Door()


class _Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0


class _Pose:
    def __init__(self):
        self.position = _Point()
        self.orientation = _Quaternion()


class _PoseStamped:
    def __init__(self):
        self.header = _Header()
        self.pose = _Pose()


class MoveBaseGoal:
    def __init__(self):
        self.target_pose = _PoseStamped()


class MoveBase:
    """Action type stand-in for MoveBase action."""
    pass