import os
import openai

# Use OpenRouter instead of OpenAI
openai.api_key = os.getenv("OPENAI_API_KEY")
openai.api_base = "https://openrouter.ai/api/v1"  # OpenRouter base URL

print("OpenRouter Key:", os.getenv("OPENAI_API_KEY"))

parts = ["edge_detection_node_part1.py", "edge_detection_node_part2.py"]
ros2_ws_src = os.path.expanduser("~/ros2_ws/src/camera_edge_detection/")

for part_file in parts:
    with open(f"ros1_code/{part_file}", "r") as f:
        ros1_code = f.read()

    prompt = f"""
Convert this ROS1 Python node to ROS2 (rclpy):
{ros1_code}

Requirements:
- Keep functionality: subscribe /camera/image_raw, apply Canny, publish /edges
- Insert print statements to show image shape and min/max pixel values
- Output runnable Python code
"""

    # === OpenRouter-compliant model call ===
    response = openai.chat.completions.create(
    	model="openai/gpt-4.1-mini",
    	messages=[{"role": "user", "content": prompt}]
	)
    ros2_code = response.choices[0].message.content
    out_file = os.path.join(ros2_ws_src, part_file.replace(".py", "_ros2.py"))
    with open(out_file, "w") as f:
        f.write(ros2_code)

    print(f"[INFO] Generated ROS2 node: {out_file}")
