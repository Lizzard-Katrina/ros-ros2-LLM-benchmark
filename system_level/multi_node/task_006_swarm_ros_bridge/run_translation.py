import os
import re
from openai import OpenAI

# 1. 读取 API key
api_key = os.environ.get("OPENROUTER_API_KEY")
if not api_key:
    raise RuntimeError("OPENROUTER_API_KEY not set")

# 2. 初始化 client（注意 base_url）
client = OpenAI(
    api_key=api_key,
    base_url="https://openrouter.ai/api/v1"
)

MODEL = "gpt-4.1-mini"

ROS1_DIR = "ros1_code"
ROS2_DIR = "ros2_code"

os.makedirs(ROS2_DIR, exist_ok=True)

import os

# 1. Collect all ROS1 content into a single string
all_ros1_content = ""
file_list = os.listdir(ROS1_DIR)

for fname in file_list:
    ros1_path = os.path.join(ROS1_DIR, fname)
    with open(ros1_path, "r") as f:
        content = f.read()
    # Wrap each file with a header so the LLM knows which is which
    all_ros1_content += f"\nFILE_PATH: {fname}\n"
    all_ros1_content += "----------------------------\n"
    all_ros1_content += f"{content}\n"
    all_ros1_content += "----------------------------\n"

# 2. Update the prompt to handle multiple files
# Note: I kept your specific rules but adjusted the "Your task" and "Rules" 
# slightly to handle the multi-file format without losing your constraints.
prompt = f"""
You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following files are originally from a real ROS1 Husky robot example. 
These files are INTERDEPENDENT parts of the same package.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert these files to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Output the completed source code for EVERY file provided.
- Use the marker [FILENAME: filename] before each completed file's content.
- Do not write ''' at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):
{all_ros1_content}
"""

print("Starting batch translation of all files...")
response = client.chat.completions.create(
    model=MODEL,
    messages=[
        {"role": "system", "content": "You are a ROS2 expert."},
        {"role": "user", "content": prompt}
    ],
    temperature=0.2, timeout=120 # Increased timeout for multiple files
)

full_response = response.choices[0].message.content

# 3. Split the response and save back to separate files
# This assumes the LLM follows the [FILENAME: filename] rule
parts = re.split(r'\[FILENAME:\s*(.*?)\]', full_response)

# parts[1] will be filename, parts[2] will be code, and so on
for i in range(1, len(parts), 2):
    fname = parts[i].strip()
    ros2_code = parts[i+1].strip()
    ros2_path = os.path.join(ROS2_DIR, fname.replace("_todo", ""))
    
    with open(ros2_path, "w") as f:
        f.write(ros2_code)
    print(f"✓ Translated {fname} → {ros2_path}")
