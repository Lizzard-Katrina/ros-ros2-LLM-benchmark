import os
import openai
import time
import importlib.util
import sys
from std_msgs.msg import String
import rclpy
from rclpy.node import Node

# ------------------------------
# 配置 OpenAI
# ------------------------------
openai.api_key = os.getenv("OPENAI_API_KEY")
if not openai.api_key:
    raise ValueError("OPENAI_API_KEY not set!")

# ------------------------------
# 配置文件路径
# ------------------------------
parts = ["edge_detection_node_part1.py", "edge_detection_node_part2.py"]
ros1_dir = "ros1_code"
ros2_ws_src = os.path.expanduser("~/ros2_ws/src/camera_edge_detection/")
os.makedirs(ros2_ws_src, exist_ok=True)

# ------------------------------
# LLM 转换 ROS1 -> ROS2
# ------------------------------
for part_file in parts:
    ros1_path = os.path.join(ros1_dir, part_file)
    with open(ros1_path, "r") as f:
        ros1_code = f.read()

    prompt = f"""
Convert this ROS1 Python node to ROS2 (rclpy):
{ros1_code}

Requirements:
- Keep functionality: subscribe /camera/image_raw, apply Canny, publish /edges
- Insert print statements to show image shape and min/max pixel values
- Output runnable Python code
"""

    response = openai.chat.completions.create(
        model="gpt-4.1-mini",
        messages=[{"role": "user", "content": prompt}]
    )

    ros2_code = response.choices[0].message["content"]

    out_file = os.path.join(ros2_ws_src, part_file.replace(".py", "_ros2.py"))
    with open(out_file, "w") as f:
        f.write(ros2_code)
    print(f"[INFO] Generated ROS2 node: {out_file}")

# ------------------------------
# 轻量验证（Pub/Sub 测试）
# ------------------------------
TEST_MESSAGES = ["test1", "test2", "test3"]
RECEIVED_MESSAGES = []

class TestListener(Node):
    def __init__(self):
        super().__init__('test_listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        print(f"[listener] Received: {msg.data}")
        RECEIVED_MESSAGES.append(msg.data)

class TestTalker(Node):
    def __init__(self):
        super().__init__('test_talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)

    def publish_messages(self):
        for msg_text in TEST_MESSAGES:
            msg = String()
            msg.data = msg_text
            self.publisher.publish(msg)
            print(f"[talker] Published: {msg_text}")
            time.sleep(0.5)

def main():
    rclpy.init()
    listener = TestListener()
    talker = TestTalker()

    # 发布消息 + spin_once 循环接收
    for _ in range(5):
        talker.publish_messages()
        rclpy.spin_once(listener, timeout_sec=0.5)

    listener.destroy_node()
    talker.destroy_node()
    rclpy.shutdown()

    # 验证消息
    if all(msg in RECEIVED_MESSAGES for msg in TEST_MESSAGES):
        print("\n[RESULT] PASS: All messages received!")
    else:
        print("\n[RESULT] FAIL: Missing messages!")
        print("Expected:", TEST_MESSAGES)
        print("Received:", RECEIVED_MESSAGES)

if __name__ == "__main__":
    main()
