import os
import json
import builtins

import sys
import types
# -------------------------------
# 1. mock os.path.exists
# -------------------------------
_real_exists = os.path.exists

def fake_exists(path):
    if "models/lego_" in path and path.endswith("model.json"):
        return True
    return _real_exists(path)

os.path.exists = fake_exists


# -------------------------------
# 2. mock open()
# -------------------------------
_real_open = builtins.open

def fake_open(path, *args, **kwargs):
    path_str = str(path)   # 🔑 关键：统一转成字符串
    if path_str.endswith("model.json"):
        return FakeFile()
    return _real_open(path, *args, **kwargs)

builtins.open = fake_open

class FakeFile:
    def __enter__(self):
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        return False
    def read(self):
        return json.dumps({
            "corners": [
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0]
            ]
        })
builtins.open = fake_open



# --- mock kinematics ---
kinematics = types.ModuleType("kinematics")
def fake_fk(*args, **kwargs):
    return None
kinematics.forward_kinematics = fake_fk
sys.modules["kinematics"] = kinematics

# --- mock control_msgs ---
control_msgs = types.ModuleType("control_msgs")
control_msgs.action = types.ModuleType("control_msgs.action")
control_msgs.action.GripperCommand = object
control_msgs.msg = types.ModuleType("control_msgs.msg")  # <--- 新增
sys.modules["control_msgs"] = control_msgs
sys.modules["control_msgs.action"] = control_msgs.action
sys.modules["control_msgs.msg"] = control_msgs.msg

# --- mock gazebo_msgs ---
gazebo_msgs = types.ModuleType("gazebo_msgs")
gazebo_msgs.msg = types.ModuleType("gazebo_msgs.msg")
gazebo_msgs.msg.ModelStates = object
sys.modules["gazebo_msgs"] = gazebo_msgs
sys.modules["gazebo_msgs.msg"] = gazebo_msgs.msg

# --- mock gazebo_ros_link_attacher ---
gazebo_ros_link_attacher = types.ModuleType("gazebo_ros_link_attacher")
gazebo_ros_link_attacher.srv = types.ModuleType("gazebo_ros_link_attacher.srv")
gazebo_ros_link_attacher.srv.SetStatic = object
gazebo_ros_link_attacher.srv.Attach = object
sys.modules["gazebo_ros_link_attacher"] = gazebo_ros_link_attacher
sys.modules["gazebo_ros_link_attacher.srv"] = gazebo_ros_link_attacher.srv



import pytest
import rclpy
from rclpy.node import Node
import motion_planning

THIS_DIR = os.path.dirname(__file__)
PKG_ROOT = os.path.abspath(os.path.join(THIS_DIR, ".."))
sys.path.insert(0, PKG_ROOT)

@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_motion_planner_is_ros2_node(ros_context):
    """
    Oracle test: Ensure the MotionPlanner class is a ROS2 Node
    """
    planner = MotionPlanner()
    assert isinstance(planner, Node), "MotionPlanner should inherit from rclpy.node.Node"

def test_motion_planner_has_model_info(ros_context):
    """
    Oracle test: Check that MotionPlanner retains MODEL_INFO dictionary
    """
    planner = MotionPlanner()
    assert hasattr(planner, "MODELS_INFO"), "Planner must contain MODELS_INFO"
    assert isinstance(planner.MODELS_INFO, dict), "MODELS_INFO should be a dictionary"

def test_motion_planner_has_straighten_method(ros_context):
    """
    Oracle test: MotionPlanner should have the straighten method
    """
    planner = MotionPlanner()
    assert hasattr(planner, "straighten"), "MotionPlanner should have straighten method"
    assert callable(planner.straighten)
	

