import sys
import types

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



import pytest
import rclpy
from rclpy.node import Node
from pyquaternion import Quaternion
from controller import ArmController

import os

THIS_DIR = os.path.dirname(__file__)
PKG_ROOT = os.path.abspath(os.path.join(THIS_DIR, ".."))

sys.path.insert(0, PKG_ROOT)



@pytest.fixture(scope="module")
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()

def test_arm_controller_is_ros2_compatible(ros_context):
    """
    Oracle test: ArmController should work in ROS2 context
    """
    controller = ArmController()
    # 这里只检查 ROS2 环境兼容性（publisher 可以创建）
    assert hasattr(controller, "joints_pub"), "Controller should have a ROS2 publisher"

def test_arm_controller_move_to_signature(ros_context):
    """
    Oracle test: move_to method exists and accepts correct arguments
    """
    controller = ArmController()
    assert hasattr(controller, "move_to")
    assert callable(controller.move_to)

    # Check that move_to accepts position and quaternion
    try:
        controller.move_to(x=0.1, y=0.1, z=0.1, target_quat=Quaternion())
    except Exception as e:
        pytest.fail(f"move_to method raised exception: {e}")

def test_arm_controller_gripper_state(ros_context):
    """
    Oracle test: Controller has gripper_state attribute and default is 0
    """
    controller = ArmController()
    assert hasattr(controller, "gripper_state")
    assert controller.gripper_state == 0
