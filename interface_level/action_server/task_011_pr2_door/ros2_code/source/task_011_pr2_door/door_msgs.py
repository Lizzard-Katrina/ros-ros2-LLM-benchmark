"""
Minimal stand-in message/action types for door_msgs which do not exist in ROS2.
These are pure Python data classes used to preserve the original code's semantics.
"""

class Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Header:
    def __init__(self):
        self.frame_id = ""

class Door:
    ROT_DIR_COUNTERCLOCKWISE = 1
    HINGE_P2 = 2

    def __init__(self):
        self.frame_p1 = Point()
        self.frame_p2 = Point()
        self.door_p1 = Point()
        self.door_p2 = Point()
        self.travel_dir = Vector3()
        self.rot_dir = 0
        self.hinge = 0
        self.header = Header()

class DoorGoal:
    def __init__(self):
        self.door = Door()

class DoorResult:
    def __init__(self):
        self.door = Door()

class DoorFeedback:
    def __init__(self):
        pass

class DoorAction:
    """Placeholder action type for door_msgs/DoorAction."""
    Goal = DoorGoal
    Result = DoorResult
    Feedback = DoorFeedback