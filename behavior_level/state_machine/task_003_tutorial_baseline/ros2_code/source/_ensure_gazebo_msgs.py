"""
Helper module that ensures gazebo_msgs is importable.
If the real gazebo_msgs ROS2 package is installed, it is used.
Otherwise, a local stub is injected into sys.modules.
"""
import importlib
import sys
import os


def ensure():
    """Make sure 'gazebo_msgs' and 'gazebo_msgs.srv' are importable."""
    try:
        importlib.import_module('gazebo_msgs.srv')
        return  # Real package available
    except (ImportError, ModuleNotFoundError):
        pass

    # Use our stub
    pkg_root = os.path.dirname(os.path.abspath(__file__))
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    import gazebo_msgs_stub
    import gazebo_msgs_stub.srv

    # Inject into sys.modules so `from gazebo_msgs.srv import ...` works
    sys.modules['gazebo_msgs'] = gazebo_msgs_stub
    sys.modules['gazebo_msgs.srv'] = gazebo_msgs_stub.srv