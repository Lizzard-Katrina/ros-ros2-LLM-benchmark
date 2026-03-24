import sys
from tests.mocks import rclpy as mock_rclpy
from tests.mocks import cv2 as mock_cv2
from tests.mocks import cv_bridge as mock_cv_bridge

# 先挂载 rclpy 包
sys.modules['rclpy'] = mock_rclpy

# 再挂载 rclpy.node 子模块
import types
mock_rclpy.node = types.ModuleType("node")  # 创建 node 模块
mock_rclpy.node.Node = mock_rclpy.node.Node if hasattr(mock_rclpy.node, "Node") else type("Node", (), {})  
sys.modules['rclpy.node'] = mock_rclpy.node

# 挂载 OpenCV mocks
sys.modules['cv2'] = mock_cv2
sys.modules['cv_bridge'] = mock_cv_bridge

