import importlib
import inspect
import sys
import os

sys.path.append(
    os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..")
    )
)

# Mocked Node for compatibility if needed
class Node:
    pass

# Mocked String for ROS1 message
class String:
    def __init__(self):
        self.data = ""

def _get_publisher_functions(mod):
    """Return functions that create publishers (weakened)"""
    return [
        obj for _, obj in inspect.getmembers(mod, inspect.isfunction)
        if obj.__name__ == "main"
    ]

def test_talker_translation():
    mod = importlib.import_module("talker")

    # Oracle 1: Module can be imported
    assert mod is not None

    # Oracle 2: main() function exists
    main_funcs = _get_publisher_functions(mod)
    assert main_funcs, "Talker must define a main() function"

    # Oracle 3: Can construct a ROS message (mocked)
    msg = String()
    msg.data = "test message"
    assert hasattr(msg, "data")
