import re
import pytest
from pathlib import Path

CODE_FILE = Path(__file__).resolve().parents[1] / "turtle_patrol_server.py"

import re
import pytest

class TestPatrolOracleHardcore:
    @classmethod
    def setup_class(cls):
        try:
            with open(CODE_FILE, 'r') as f:
                cls.source = f.read()
        except FileNotFoundError:
            cls.source = ""

    def get_function_block(self, func_name):
        """提取函数体，处理可能的 async 异步定义"""
        pattern = rf"(?:async\s+)?def\s+{func_name}\s*\(.*?\):([\s\S]*?)(?=async\s+def|def|\Z)"
        match = re.search(pattern, self.source)
        return match.group(1) if match else ""


    def test_go_front_strict_closed_loop(self):
        """
        [HARDCORE] 判定是否使用了真·位移计算
        要求：必须同时出现起点记录 (initial/start) 和 欧氏距离公式 (sqrt/pow)
        """
        blk = self.get_function_block("go_front")
        
        # 必须记录起点（防止模型直接用实时 odom 坐标做简单阈值判定）
        has_initial_record = re.search(r"(initial|start|origin)_(x|y|pos)", blk)
        # 必须使用欧氏距离公式
        has_distance_formula = re.search(r"(math\.sqrt|math\.hypot|\*\*2\s*\+\s*.*?\*\*2)", blk)
        
        msg = "❌ go_front is too simple. Use initial pose and Euclidean distance formula."
        assert has_initial_record and has_distance_formula, msg

    def test_turn_shortest_path_logic(self):
        """
        [HARDCORE] 判定是否解决了角速度环回问题 (The classic -pi/pi issue)
        要求：必须使用三角函数进行差分平滑，避免反向打方向盘
        """
        blk = self.get_function_block("turn")
        
        # 极严苛匹配：必须出现 atan2(sin(diff), cos(diff)) 结构
        # 这种结构是判断模型是否具备“机器人专家级”控制逻辑的唯一标准
        shortest_path_pattern = r"atan2\(.*math\.sin\(.*?\).*?math\.cos\(.*?\).*?\)"
        
        msg = "❌ turn logic is fragile. Must use atan2(sin(error), cos(error)) for shortest path."
        assert re.search(shortest_path_pattern, blk), msg

    def test_proportional_control_law(self):
        """
        [HARDCORE] 判定是否使用了比例控制 (P-Control)
        要求：速度指令不能是硬编码的常量 (如 0.2)，必须乘以 error
        """
        blk = self.get_function_block("turn")
        
        # 检查是否定义了增益常数 (Kp/gain) 且速度指令与之相关
        # 或者检查角速度赋值语句是否包含乘法运算（error * gain）
        p_control_pattern = r"angular\.z\s*=\s*.*?\*.*?(diff|error)"
        
        msg = "❌ Steady-state velocity detected. Use Proportional Control (Kp * error) for smoothness."
        assert re.search(p_control_pattern, blk), msg

    def test_emergency_safety_check(self):
        """
        [HARDCORE] 判定是否包含死循环保护
        要求：在 while 循环中必须有 rclpy.ok() 或超时机制，防止传感器掉线导致机器人失控
        """
        blk = self.get_function_block("turn")
        assert "rclpy.ok()" in blk or "timeout" in blk.lower(), \
            "❌ Safety risk: loop lacks rclpy.ok() check or timeout protection."
