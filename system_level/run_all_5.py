import os
import subprocess
import xml.etree.ElementTree as ET
import json
import shutil
import re
import argparse
from openai import OpenAI

CATEGORIES = ["navigation","simulation_integration","manipulation","multi_node"] 
BASE_IMAGE = "ros2-benchmark-base"

MODELS_TO_TEST = [
#    "openai/gpt-5.5",
#    "deepseek/deepseek-v4-pro",
    "google/gemini-3.1-pro-preview",
#    "anthropic/claude-opus-4.6",
#    "qwen/qwen3.5-plus-02-15",
#    "z-ai/glm-5-turbo",
]

api_key = os.environ.get("OPENROUTER_API_KEY")
client = OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1")

def translate_system_task(task_path, cat_name, task_name, model_name, results_root, reasoning_effort):
    ros1_dir = os.path.join(task_path, "ros1_code")
    ros2_dir = os.path.join(task_path, "ros2_code")

    if os.path.exists(ros2_dir): shutil.rmtree(ros2_dir)
    os.makedirs(ros2_dir, exist_ok=True)

    all_ros1_content = ""
    file_list = sorted(os.listdir(ros1_dir))
    for fname in file_list:
        if not (fname.endswith('.py') or fname.endswith('.cpp') or fname.endswith('.hpp')):
            continue
        ros1_path = os.path.join(ros1_dir, fname)
        with open(ros1_path, "r", encoding="utf-8") as f:
            content = f.read()
        all_ros1_content += f"\nFILE_PATH: {fname}\n"
        all_ros1_content += "----------------------------\n"
        all_ros1_content += f"{content}\n"
        all_ros1_content += "----------------------------\n"

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
- Do not write quoting marks at the beginning or at the end of the file!

Rules:
- Replace ROS1 APIs with ROS2 equivalents.
- Implement meaningful logic at TODO sections (do not leave TODO empty).
- Do not explain.
- Do not add comments unrelated to the original code.

ROS1 code (Multiple Files):
{all_ros1_content}"""

    print(f"   [{model_name}] Requesting batch translation: {task_name}...")
    try:
        kwargs = {
            "model": model_name,
            "messages": [
                {"role": "system", "content": "You are a ROS2 expert."},
                {"role": "user", "content": prompt}
            ],
            "temperature": 0.0, 
            "timeout": 180
        }
        
        if reasoning_effort and reasoning_effort.lower() != "default":
            kwargs["extra_body"] = {"reasoning_effort": reasoning_effort}

        response = client.chat.completions.create(**kwargs)
        full_response = response.choices[0].message.content
        total_tokens = response.usage.total_tokens

        dialogue_dir = os.path.join(results_root, cat_name, task_name)
        os.makedirs(dialogue_dir, exist_ok=True)
        with open(os.path.join(dialogue_dir, "dialogue.md"), "w", encoding="utf-8") as f:
            f.write(f"# Prompt\n\n{prompt}\n\n# LLM Response\n\n{full_response}")

        parts = re.split(r'\[FILENAME:\s*(.*?)\]', full_response)
        for i in range(1, len(parts), 2):
            fname = parts[i].strip()
            ros2_code = parts[i+1].strip()
            ros2_code = re.sub(r'^```[a-z]*\n', '', ros2_code)
            ros2_code = re.sub(r'\n```$', '', ros2_code)
            
            ros2_path = os.path.join(ros2_dir, fname.replace("_todo", ""))
            with open(ros2_path, "w", encoding="utf-8") as f:
                f.write(ros2_code)
            print(f"     ✓ Saved: {fname}")

        return True, total_tokens, full_response
    except Exception as e:
        print(f"   × Failed {task_name}: {e}")
        return False, 0, str(e)

def run_docker_test(task_path, task_name):
    container_name = f"test_container_{task_name}"
    subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)
    subprocess.run(["docker", "run", "-d", "--name", container_name, BASE_IMAGE, "tail", "-f", "/dev/null"], check=True)
    try:
        pkg_dir = f"/ros2_ws/src/{task_name}"
        subprocess.run(["docker", "exec", container_name, "mkdir", "-p", pkg_dir], check=True)
        subprocess.run(["docker", "cp", f"{task_path}/ros2_code/.", f"{container_name}:{pkg_dir}/"], check=True)
        subprocess.run(["docker", "exec", container_name, "mkdir", "-p", f"{pkg_dir}/tests"], check=True)
        subprocess.run(["docker", "cp", f"{task_path}/tests/.", f"{container_name}:{pkg_dir}/tests/"], check=True)

        test_command = f"source /opt/ros/humble/setup.bash && touch {pkg_dir}/__init__.py && export PYTHONPATH=$PYTHONPATH:/ros2_ws/src && cd /ros2_ws && python3 -m pytest {pkg_dir}/tests/ -q --tb=no --junitxml=/tmp/report.xml"
        subprocess.run(["docker", "exec", container_name, "bash", "-c", test_command])
        subprocess.run(["docker", "cp", f"{container_name}:/tmp/report.xml", f"{task_path}/test_report.xml"], capture_output=True)
    finally:
        subprocess.run(["docker", "rm", "-f", container_name], capture_output=True)

def parse_junit_xml(xml_path):
    if not os.path.exists(xml_path):
        return {"status": "TEST_CRASHED", "passed": 0, "total": 0, "failures_details": []}
    try:
        tree = ET.parse(xml_path)
        root = tree.getroot()
        suite = root.find("testsuite") if root.tag != "testsuite" else root
        total = int(suite.get("tests", 0))
        failures = int(suite.get("failures", 0))
        errors = int(suite.get("errors", 0))
        passed = total - failures - errors
        details = []
        for tc in suite.findall("testcase"):
            fail_node = tc.find("failure")
            if fail_node is not None:
                details.append({"test_function": tc.get("name"), "error_message": fail_node.get("message")})
        return {"status": "SUCCESS" if (failures + errors) == 0 else "FAILED", "passed": passed, "total": total, "failures_details": details}
    except:
        return {"status": "XML_PARSE_ERROR", "passed": 0, "total": 0, "failures_details": []}

def generate_markdown_report(cat_name, data_list, results_root):
    report_path = os.path.join(results_root, cat_name, "report.md")
    os.makedirs(os.path.dirname(report_path), exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as f:
        f.write(f"# System Level Benchmark Report: {cat_name.upper()}\n\n")
        f.write("| Task ID | Status | Score | Failed Functions | Tokens |\n")
        f.write("| :--- | :--- | :--- | :--- | :--- |\n")
        for d in data_list:
            failed_funcs = ", ".join([f['test_function'] for f in d['failed_cases']]) if d['failed_cases'] else "None"
            f.write(f"| {d['task_id']} | {d['status']} | {d['score']} | {failed_funcs} | {d['tokens']} |\n")


def main():
    parser = argparse.ArgumentParser(description="Run ROS level benchmark")
    parser.add_argument("--reasoning_effort", type=str, default="none", help="Reasoning effort override")
    parser.add_argument("--runs", type=int, default=4, help="Total loops to execute")
    parser.add_argument("--start_run", type=int, default=2, help="Start index for results directory")
    args = parser.parse_args()

    if not api_key: return print("CRITICAL ERROR: 'OPENROUTER_API_KEY' not set.")

    for run_id in range(args.start_run, args.start_run + args.runs):
        results_dir_name = f"results{run_id}"
        print(f"\n{'='*80}")
        print(f" STARTING RUN {run_id} -> Output: {results_dir_name} ".center(80, "="))
        print(f"{'='*80}")

        for model_name in MODELS_TO_TEST:
            print(f"\n\n" + f" SYSTEM LEVEL RUN: {model_name} (Run {run_id}) ".center(80, "#"))
            model_safe_name = model_name.replace("/", "_").replace(":", "_")
            model_results_root = os.path.abspath(os.path.join(results_dir_name, model_safe_name))

            for cat in CATEGORIES:
                if not os.path.exists(cat): continue
                cat_passed, cat_total, cat_tokens, cat_count = 0, 0, 0, 0
                category_summary_list = []
                tasks = sorted([d for d in os.listdir(cat) if d.startswith("task_")])

                for task in tasks:
                    task_path = os.path.abspath(os.path.join(cat, task))
                    print(f"\n[Processing] {cat}/{task}")

                    success, tokens, _ = translate_system_task(task_path, cat, task, model_name, model_results_root, args.reasoning_effort)

                    if success:
                        run_docker_test(task_path, task)
                        res = parse_junit_xml(os.path.join(task_path, "test_report.xml"))
                        res["tokens"] = tokens
                    else:
                        res = {"status": "TRANSLATION_FAILED", "passed": 0, "total": 0, "tokens": tokens, "failures_details": []}
                    
                    pass_rate = f"{(res['passed']/res['total']*100 if res['total']>0 else 0):.2f}%"
                    category_summary_list.append({
                        "task_id": f"{cat}/{task}",
                        "status": res["status"],
                        "score": f"{res['passed']}/{res['total']}",
                        "pass_rate": pass_rate,
                        "tokens": tokens,
                        "failed_cases": res.get("failures_details", [])
                    })

                    json_dir = os.path.join(model_results_root, cat)
                    os.makedirs(json_dir, exist_ok=True)
                    with open(os.path.join(json_dir, "summary.json"), "w") as f:
                        json.dump(category_summary_list, f, indent=4)

                    cat_passed += res['passed']
                    cat_total += res['total']
                    cat_tokens += res['tokens']
                    cat_count += 1
                    print(f"   Task Summary: {res['status']} ({res['passed']}/{res['total']})")

                generate_markdown_report(cat, category_summary_list, model_results_root)
                print(f"\n {cat.upper()} SUMMARY: {cat_passed}/{cat_total} cases passed. Tokens: {cat_tokens}")

    print("\n" + " ALL MODELS AND RUNS COMPLETED ".center(80, "="))

if __name__ == "__main__":
    main()
