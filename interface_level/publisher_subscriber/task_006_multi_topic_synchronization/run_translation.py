import os
import shutil
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

# in case there are other directories, like 'msg' or 'script'
ROS1_SRC_DIR = os.path.join(ROS1_DIR, "src")

ROS2_SRC_DIR = os.path.join(ROS2_DIR)

os.makedirs(ROS2_SRC_DIR, exist_ok=True)

#os.makedirs(ROS2_DIR, exist_ok=True)

for fname in os.listdir(ROS1_DIR):
    ros1_path = os.path.join(ROS1_DIR, fname)


    # Case 1: copy custom message definitions directly
    if fname == "msg" and os.path.isdir(ros1_path):
        for msg_file in os.listdir(ros1_path):
            if msg_file.endswith(".msg"):
                src_msg = os.path.join(ros1_path, msg_file)
                dst_msg = os.path.join(ROS2_MSG_DIR, msg_file)
                shutil.copy(src_msg, dst_msg)
                print(f"Copied ROS1 msg file: {msg_file}")
        continue

    if fname.endswith(".py") or fname.endswith(".cpp") and os.path.isfile(ros1_path):
        ros2_path = os.path.join(
        ROS2_SRC_DIR, fname.replace("_todo", "")
        )

        with open(ros1_path, "r") as f:
            ros1_code = f.read()

        print(f"Reading ROS1 source file: {fname}")

        prompt = f"""
You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following file is originally from a real ROS1 example.

Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert this file to ROS2 using corresponding language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Do NOT output anything except the completed source code of THIS FILE.
- Only output the full translated function(s),(but if it is contained in a class, include the whole class) nothing else.
- Do not write ''' at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.
ROS1 code:
----------------------------
{ros1_code}
----------------------------
"""
    print("start to translate") 
    response = client.chat.completions.create(
        model=MODEL,
        messages=[
            {"role": "system", "content": "You are a ROS2 expert."},
            {"role": "user", "content": prompt}
        ],
        temperature=0.2,timeout=60
    )
    print("response received")
    ros2_code = response.choices[0].message.content

    with open(ros2_path, "w") as f:
        f.write(ros2_code)

    print(f"✓ Translated {fname} → {ros2_path}")
