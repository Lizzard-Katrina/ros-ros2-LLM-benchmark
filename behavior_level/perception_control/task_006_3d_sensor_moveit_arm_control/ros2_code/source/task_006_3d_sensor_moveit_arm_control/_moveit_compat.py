"""
Compatibility shim: provides moveit_msgs types either from the real
ros-humble-moveit-msgs package or from lightweight stubs so the module
can be imported (and structurally tested) in environments where
moveit_msgs is not installed.
"""

_HAVE_MOVEIT = False

try:
    from moveit_msgs.msg import Grasp, MoveItErrorCodes, PlaceLocation, CollisionObject  # noqa: F401
    from moveit_msgs.action import Pickup, Place  # noqa: F401
    from moveit_msgs.srv import GetPlanningScene  # noqa: F401
    _HAVE_MOVEIT = True
except ImportError:
    pass

if not _HAVE_MOVEIT:
    # ---- Stub message / action / service types ----

    class _StubMsg:
        """Base for stub message classes that accept arbitrary kwargs."""
        def __init__(self, **kwargs):
            for k, v in kwargs.items():
                setattr(self, k, v)

    # --- MoveItErrorCodes ---
    class MoveItErrorCodes(_StubMsg):
        SUCCESS = 1
        FAILURE = 99999
        PLANNING_FAILED = -1
        INVALID_MOTION_PLAN = -2
        MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE = -3
        CONTROL_FAILED = -4
        UNABLE_TO_AQUIRE_SENSOR_DATA = -5
        TIMED_OUT = -6
        PREEMPTED = -7
        START_STATE_IN_COLLISION = -10
        START_STATE_VIOLATES_PATH_CONSTRAINTS = -11
        GOAL_IN_COLLISION = -12
        GOAL_VIOLATES_PATH_CONSTRAINTS = -13
        GOAL_CONSTRAINTS_VIOLATED = -14
        INVALID_GROUP_NAME = -15
        INVALID_GOAL_CONSTRAINTS = -16
        INVALID_ROBOT_STATE = -17
        INVALID_LINK_NAME = -18
        INVALID_OBJECT_NAME = -19
        FRAME_TRANSFORM_FAILURE = -21
        COLLISION_CHECKING_UNAVAILABLE = -22
        ROBOT_STATE_STALE = -23
        SENSOR_INFO_STALE = -24
        COMMUNICATION_FAILURE = -25
        NO_IK_SOLUTION = -31

        def __init__(self, **kw):
            self.val = kw.get('val', 0)

    # --- Grasp ---
    class Grasp(_StubMsg):
        def __init__(self, **kw):
            from geometry_msgs.msg import PoseStamped
            self.id = kw.get('id', '')
            self.grasp_pose = kw.get('grasp_pose', PoseStamped())
            self.grasp_quality = kw.get('grasp_quality', 0.0)
            self.pre_grasp_approach = kw.get('pre_grasp_approach', _StubMsg())
            self.post_grasp_retreat = kw.get('post_grasp_retreat', _StubMsg())
            self.post_place_retreat = kw.get('post_place_retreat', _StubMsg())
            self.max_contact_force = kw.get('max_contact_force', 0.0)
            self.allowed_touch_objects = kw.get('allowed_touch_objects', [])

    # --- PlaceLocation ---
    class PlaceLocation(_StubMsg):
        def __init__(self, **kw):
            from geometry_msgs.msg import PoseStamped
            self.place_pose = kw.get('place_pose', PoseStamped())
            self.pre_place_approach = kw.get('pre_place_approach', _StubMsg())
            self.post_place_retreat = kw.get('post_place_retreat', _StubMsg())
            self.allowed_touch_objects = kw.get('allowed_touch_objects', [])

    # --- CollisionObject ---
    class CollisionObject(_StubMsg):
        ADD = 0
        REMOVE = 1
        APPEND = 2
        MOVE = 3
        def __init__(self, **kw):
            from std_msgs.msg import Header
            self.header = kw.get('header', Header())
            self.id = kw.get('id', '')
            self.operation = kw.get('operation', 0)
            self.primitives = kw.get('primitives', [])
            self.primitive_poses = kw.get('primitive_poses', [])

    # --- PlanningOptions stub ---
    class _RobotState(_StubMsg):
        def __init__(self):
            self.is_diff = False

    class _PlanningSceneDiff(_StubMsg):
        def __init__(self):
            self.is_diff = False
            self.robot_state = _RobotState()

    class _PlanningOptions(_StubMsg):
        def __init__(self):
            self.planning_scene_diff = _PlanningSceneDiff()
            self.plan_only = False
            self.replan = False
            self.replan_attempts = 0

    # --- Pickup action ---
    class _PickupGoal(_StubMsg):
        def __init__(self, **kw):
            self.target_name = ''
            self.group_name = ''
            self.possible_grasps = []
            self.allowed_planning_time = 0.0
            self.planning_options = _PlanningOptions()
            self.plan_only = False
            self.allowed_touch_objects = []
            self.attached_object_touch_links = []

    class _PickupResult(_StubMsg):
        def __init__(self):
            self.error_code = MoveItErrorCodes()

    class Pickup:
        Goal = _PickupGoal
        Result = _PickupResult

    # --- Place action ---
    class _PlaceGoal(_StubMsg):
        def __init__(self, **kw):
            self.group_name = ''
            self.attached_object_name = ''
            self.place_locations = []
            self.allowed_planning_time = 0.0
            self.planning_options = _PlanningOptions()
            self.allowed_touch_objects = []

    class _PlaceResult(_StubMsg):
        def __init__(self):
            self.error_code = MoveItErrorCodes()

    class Place:
        Goal = _PlaceGoal
        Result = _PlaceResult

    # --- GetPlanningScene service ---
    class _PlanningSceneComponents(_StubMsg):
        WORLD_OBJECT_NAMES = 1
        def __init__(self):
            self.components = 0

    class _GetPlanningSceneRequest(_StubMsg):
        def __init__(self):
            self.components = _PlanningSceneComponents()

    class _WorldStub(_StubMsg):
        def __init__(self):
            self.collision_objects = []

    class _SceneStub(_StubMsg):
        def __init__(self):
            self.world = _WorldStub()

    class _GetPlanningSceneResponse(_StubMsg):
        def __init__(self):
            self.scene = _SceneStub()

    class GetPlanningScene:
        Request = _GetPlanningSceneRequest
        Response = _GetPlanningSceneResponse