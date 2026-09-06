"""
Stub service definitions mimicking gazebo_msgs.srv.SetModelState for ROS2.
"""


class _ModelState:
    """Mimics gazebo_msgs/msg/ModelState."""
    def __init__(self):
        self.model_name = ''
        self.pose = None
        self.twist = None
        self.reference_frame = ''


class _SetModelStateRequest:
    """Mimics SetModelState.Request."""
    def __init__(self):
        self.model_state = _ModelState()


class _SetModelStateResponse:
    """Mimics SetModelState.Response."""
    def __init__(self):
        self.success = False
        self.status_message = ''


class SetModelState:
    """Mimics the gazebo_msgs/srv/SetModelState service type."""
    Request = _SetModelStateRequest
    Response = _SetModelStateResponse