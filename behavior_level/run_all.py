import os
import subprocess
import xml.etree.ElementTree as ET
import json
import shutil
from openai import OpenAI

# ================= CONFIGURATION =================
CATEGORIES = [ "perception_control", "sensor_process",  "state_machine"]
BASE_IMAGE = "ros2-benchmark-base"
# Add your desired models from OpenRouter here
MODELS_TO_TEST = [
#    "openai/gpt-5.4",
#   "openai/gpt-4o",
#    "anthropic/claude-opus-4.6",
#   "anthropic/claude-3.7-sonnet",
#    "google/gemini-3.1-pro-preview",
#    "google/gemini-2.5-flash",
#    "meta-llama/llama-3.3-70b-instruct",
#    "deepseek/deepseek-v3.2"
"z-ai/glm-5-turbo",
"moonshotai/kimi-k2.5",
"qwen/qwen3.5-plus-02-15",
"minimax/minimax-m2.7",
"openai/gpt-5.3-codex",
]
# =================================================

api_key = os.environ.get("OPENROUTER_API_KEY")
client = OpenAI(
    api_key=api_key,
    base_url="https://openrouter.ai/api/v1"
)

def clean_llm_code(code):
    """Strips Markdown code blocks from LLM response."""
    lines = code.split('\n')
    if lines and lines[0].strip().startswith('```'):
        lines = lines[1:]
    if lines and lines[-1].strip().startswith('```'):
        lines = lines[:-1]
    return '\n'.join(lines)

def translate_task(task_path, cat_name, task_name, model_name, results_root):
    """Handles LLM translation and saves dialogue per model."""
    ros1_dir = os.path.join(task_path, "ros1_code")
    ros2_dir = os.path.join(task_path, "ros2_code")
    
    if os.path.exists(ros2_dir):
        shutil.rmtree(ros2_dir)
    os.makedirs(ros2_dir, exist_ok=True)

    if not os.path.exists(ros1_dir):
        return False, 0, "ros1_code not found"

    total_tokens = 0
    raw_response_text = ""

    for fname in os.listdir(ros1_dir):
        ros1_path = os.path.join(ros1_dir, fname)
        if os.path.isdir(ros1_path) or not (fname.endswith('.py') or fname.endswith('.cpp') or fname.endswith('.hpp')):
            continue

        ros2_path = os.path.join(ros2_dir, fname.replace("_todo", ""))
        with open(ros1_path, "r", encoding="utf-8") as f:
            ros1_code = f.read()

        prompt = f"""
You are an expert ROS2 migration engineer.

IMPORTANT:
- This is NOT a documentation task.
- This is NOT a code explanation task.
- This is a CODE COMPLETION task.

Context:
The following file is originally from a real ROS1 MoveIt Task Constructor example.
Some code blocks were intentionally REMOVED and replaced with TODO markers.

Your task:
- Convert this file to ROS2 using appropriate language.
- Fill in the missing code at TODO locations.
- Keep all existing function names, signatures, and file structure.
- Do NOT create new files.
- Do NOT split the code.
- Do NOT output anything except the completed C++ source code of THIS FILE.

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
        print(f"   [{model_name}] Requesting translation for: {fname}...")
        try:
            response = client.chat.completions.create(
                model=model_name,
                messages=[
                    {"role": "system", "content": "You are a ROS2 expert."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.2,
            )
            total_tokens += response.usage.total_tokens
            raw_response_text = response.choices[0].message.content
            ros2_code = clean_llm_code(raw_response_text)

            # Save dialogue in model-specific result folder
            dialogue_dir = os.path.join(results_root, cat_name, task_name)
            os.makedirs(dialogue_dir, exist_ok=True)
            with open(os.path.join(dialogue_dir, "dialogue.md"), "w", encoding="utf-8") as f:
                f.write(f"# Prompt\n\n{prompt}\n\n# LLM Response\n\n{raw_response_text}")

            with open(ros2_path, "w", encoding="utf-8") as f:
                f.write(ros2_code)
            print(f"   ✓ Translated: {fname}")
        except Exception as e:
            print(f"   × Failed {fname}: {e}")
            return False, 0, str(e)
            
    return True, total_tokens, raw_response_text

def run_docker_test(task_path, task_name):
    """Executes ROS2 oracle tests inside Docker."""
    container_name = f"test_container_{task_name}"
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)
    subprocess.run(["docker", "create", "--name", container_name, BASE_IMAGE, "tail", "-f", "/dev/null"], check=True)

    try:
        subprocess.run(["docker", "start", container_name], check=True)
        pkg_dir = f"/ros2_ws/src/{task_name}"
        subprocess.run(["docker", "exec", container_name, "mkdir", "-p", pkg_dir], check=True)
        subprocess.run(["docker", "cp", f"{task_path}/ros2_code/.", f"{container_name}:{pkg_dir}/"], check=True)
        subprocess.run(["docker", "exec", container_name, "mkdir", "-p", f"{pkg_dir}/tests"], check=True)
        subprocess.run(["docker", "cp", f"{task_path}/tests/.", f"{container_name}:{pkg_dir}/tests/"], check=True)

        test_command = f"""
        source /opt/ros/humble/setup.bash && \
        touch {pkg_dir}/__init__.py && \
        export PYTHONPATH=$PYTHONPATH:/ros2_ws/src && \
        cd /ros2_ws && \
        python3 -m pytest {pkg_dir}/tests/test_oracle_ros2.py -q --tb=no --junitxml=/tmp/report.xml
        """
        print(f"   Running Oracle Tests for {task_name}...")
        subprocess.run(["docker", "exec", container_name, "bash", "-c", test_command])
        subprocess.run(["docker", "cp", f"{container_name}:/tmp/report.xml", f"{task_path}/test_report.xml"], capture_output=True)
    finally:
        subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)

def parse_junit_xml(xml_path):
    """Parses Junit XML for test counts and failure details."""
    if not os.path.exists(xml_path):
        return {"status": "TEST_CRASHED", "passed": 0, "total": 0, "failures_details": []}
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        suite = root.find("testsuite") if root.tag != "testsuite" else root
        total = int(suite.get("tests", 0))
        failures_count = int(suite.get("failures", 0))
        errors_count = int(suite.get("errors", 0))
        passed = total - failures_count - errors_count

        failures_details = []
        for testcase in suite.findall("testcase"):
            failure = testcase.find("failure")
            if failure is not None:
                failures_details.append({
                    "test_function": testcase.get("name"),
                    "error_message": failure.get("message")
                })
        return {
            "status": "SUCCESS" if (failures_count + errors_count) == 0 else "FAILED",
            "passed": passed, "total": total, "failures_details": failures_details
        }
    except:
        return {"status": "XML_PARSE_ERROR", "passed": 0, "total": 0, "failures_details": []}

def generate_markdown_report(cat_name, data_list, results_root):
    """Generates a summary Markdown table for the category."""
    report_path = os.path.join(results_root, cat_name, "report.md")
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(f"# Benchmark Report: {cat_name.upper()}\n\n")
        f.write("| Task ID | Status | Score | Failed Functions | Tokens |\n")
        f.write("| :--- | :--- | :--- | :--- | :--- |\n")
        for d in data_list:
            failed_funcs = ", ".join([f['test_function'] for f in d['failed_cases']]) if d['failed_cases'] else "None"
            f.write(f"| {d['task_id']} | {d['status']} | {d['score']} | {failed_funcs} | {d['tokens']} |\n")

def main():
    if not api_key:
        print("CRITICAL ERROR: 'OPENROUTER_API_KEY' not set.")
        return

    for model_name in MODELS_TO_TEST:
        print(f"\n\n" + f" BENCHMARKING MODEL: {model_name} ".center(80, "#"))
        
        # Create model-specific subdirectory
        model_safe_name = model_name.replace("/", "_").replace(":", "_")
        model_results_root = os.path.abspath(os.path.join("results", model_safe_name))
        os.makedirs(model_results_root, exist_ok=True)

        benchmark_results = {}

        for cat in CATEGORIES:
            if not os.path.exists(cat): continue
            
            category_summary_list = []
            cat_passed, cat_total, cat_tokens, cat_count = 0, 0, 0, 0
            tasks = sorted([d for d in os.listdir(cat) if d.startswith("task_")])

            for task in tasks:
                task_path = os.path.abspath(os.path.join(cat, task))
                print(f"\n{'='*70}\nModel: {model_name} | Task: {cat}/{task}\n{'='*70}")

                # Step 1: LLM Translation
                success, tokens, raw_dialogue = translate_task(task_path, cat, task, model_name, model_results_root)

                if success:
                    # Step 2: Testing
                    run_docker_test(task_path, task)
                    # Step 3: Parsing
                    res = parse_junit_xml(os.path.join(task_path, "test_report.xml"))
                    res["tokens"] = tokens
                else:
                    res = {"status": "TRANSLATION_FAILED", "passed": 0, "total": 0, "tokens": tokens, "failures_details": []}

                benchmark_results[f"{cat}/{task}"] = res
                
                # Logging details
                task_detail = {
                    "task_id": f"{cat}/{task}",
                    "status": res["status"],
                    "score": f"{res['passed']}/{res['total']}",
                    "pass_rate": f"{(res['passed']/res['total']*100 if res['total']>0 else 0):.2f}%",
                    "tokens": tokens,
                    "failed_cases": res.get("failures_details", [])
                }
                category_summary_list.append(task_detail)

                # Incremental JSON save
                with open(os.path.join(model_results_root, cat, "summary.json"), "w", encoding="utf-8") as f:
                    json.dump(category_summary_list, f, indent=4)

                cat_passed += res['passed']
                cat_total += res['total']
                cat_tokens += res['tokens']
                cat_count += 1
                print(f"   Task Summary: {res['status']} ({res['passed']}/{res['total']})")

            generate_markdown_report(cat, category_summary_list, model_results_root)
            print(f"\n {cat.upper()} SUMMARY: {cat_passed}/{cat_total} cases passed. Tokens: {cat_tokens}")

    print("\n" + " ALL MODELS COMPLETED ".center(80, "="))

if __name__ == "__main__":
    main()
