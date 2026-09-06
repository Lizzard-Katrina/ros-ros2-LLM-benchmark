"""
Minimal stand-in message/action types for move_base_msgs which mirrors
the real move_base_msgs but works without the actual action definition.
"""

class Point:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class Quaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0

class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()

class Header:
    def __init__(self):
        self.frame_id = ""

class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()

class MoveBaseGoal:
    def __init__(self):
        self.target_pose = PoseStamped()

class MoveBaseResult:
    def __init__(self):
        pass

class MoveBaseFeedback:
    def __init__(self):
        pass

class MoveBaseAction:
    """Placeholder action type for move_base_msgs/MoveBaseAction."""
    Goal = MoveBaseGoal
    Result = MoveBaseResult
    Feedback = MoveBaseFeedback