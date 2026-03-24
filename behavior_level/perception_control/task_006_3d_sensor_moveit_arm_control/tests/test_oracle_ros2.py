import re
from pathlib import Path

CODE_FILE = Path(__file__).resolve().parents[1] / "pick_and_place_server.py"

class TestPickAndPlaceOracleTight:
    @classmethod
    def setup_class(cls):
        try:
            with open(CODE_FILE, 'r') as f:
                cls.source = f.read()
        except FileNotFoundError:
            cls.source = ""

    def get_function_block(self, func_name):
        """准确提取函数体，处理 ROS2 中可能存在的 async def"""
        pattern = rf"(?:async\s+)?def\s+{func_name}\s*\(.*?\):([\s\S]*?)(?=async\s+def|def|\Z)"
        match = re.search(pattern, self.source)
        return match.group(1) if match else ""


    def test_1_pure_ros2_standard(self):
        """核心：绝对纯净的 ROS2 环境 (拉低得分项)"""
        # 只要代码里还留着 rospy 的影子，直接 0 分
        assert "rospy" not in self.source.lower(), "❌ ROS1 leftovers detected! Remove all 'rospy' references."
        assert "import rclpy" in self.source, "❌ Missing rclpy import."
        assert re.search(r"class\s+\w+\s*\(.*?Node.*?\):", self.source), "❌ Must use ROS2 Node class inheritance."

    def test_2_wait_logic_atomic(self):
        """核心：等待逻辑的完整性 (合并 Loop + Service + ID Check)"""
        blk = self.get_function_block("wait_for_planning_scene_object")
        # 必须同时满足：1.有while循环 2.调用了服务 3.显式检查了 id
        has_loop = re.search(r"while\s+", blk)
        has_service = re.search(r"(call|get_planning_scene)", blk)
        has_id_check = re.search(r"\.id\s*==\s*object_name", blk)
        
        assert has_loop and has_service and has_id_check, \
            "❌ Wait logic is incomplete. Must have a loop, service call, AND explicit ID comparison."

    def test_3_grasp_sequence_integrity(self):
        """核心：抓取流水线的严苛顺序 (合并 Cleanup + Double Add + Sync)"""
        blk = self.get_function_block("grasp_object")
        
        # 检查是否包含：移除动作 -> 两次添加动作 -> 同步调用
        # 这里用顺序正则：remove ... add ... add ... wait_for_planning_scene
        integrity_pattern = (
            r"remove[\s\S]*?"                 # 先清理
            r"add_box[\s\S]*?add_box[\s\S]*?" # 必须添加两次物体
            r"wait_for_planning_scene"        # 必须调用同步函数
        )
        assert re.search(integrity_pattern, blk), \
            "❌ Grasp sequence violated. Must follow: Cleanup -> Add Part & Table -> Sync."

    def test_4_async_fallback_mastery(self):
        """核心：高级异步回退能力 (最高难度项)"""
        blk = self.get_function_block("place_object")
        
        # 强制要求：第一次尝试(await) -> 失败判定(if) -> 回退组(arm_torso) -> 第二次尝试(await) -> 状态返回
        # 如果模型漏掉任何一个 await 或者没写 if 判定，直接判定失败
        strict_fallback = (
            r"await[\s\S]*?"                 # 1st await
            r"if\s+[\s\S]*?['\"]arm_torso['\"][\s\S]*?" # if block with fallback group
            r"await[\s\S]*?"                 # 2nd await
            r"return\s+.*?(?:code|val|result)" # Final return
        )
        assert re.search(strict_fallback, blk), \
            "❌ Failed to implement a perfect async fallback structure with result return."
