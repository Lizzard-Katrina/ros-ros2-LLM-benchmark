import os
import re
import json
import pandas as pd
from openai import OpenAI

# ==========================================
# 0. Configuration
# ==========================================
api_key = os.environ.get("OPENROUTER_API_KEY")
client = OpenAI(api_key=api_key, base_url="https://openrouter.ai/api/v1")

LEVEL_DIRS = ["behavior_level", "interface_level", "system_level"]

# ==========================================
# 1. System Prompt (Error Classification Tree)
# ==========================================
JUDGE_SYSTEM_PROMPT = """
You are a senior ROS 2 software engineer and code review expert. Your task is to analyze the LLM-generated ROS 1 to ROS 2 migration code against a SPECIFIC test case.

You will be provided with:
1. Oracle Status (SUCCESS or FAILED)
2. Target Test Case Name (if FAILED)
3. Generated Code (ROS 2)
4. Oracle Test Code
5. Specific Error Log for this test case

Strictly select the most accurate core reason for THIS SPECIFIC test outcome from the following categories:

If the Oracle Status is FAILED:
[A1] Basic Syntax / Compilation Error
[A2] Missing Dependencies / Headers
[B1] Legacy ROS 1 API not replaced
[B2] Hallucinated API / Non-existent ROS 2 API
[C1] Node/Lifecycle Logic Issue
[C2] QoS Profile Mismatch
[C3] Other Semantic/Behavioral Error
[E1] False Positive (Style/Format Mismatch) - The code is functionally correct for ROS 2, but the oracle test failed it due to rigid formatting, style, or naming expectations.
[E2] False Negative (Missed Hard Error) - The code contains a critical hard error (e.g., will fail compilation or crash>

If the Oracle Status is SUCCESS:
[P1] True Success - The code is correct and safely passed the tests.
[E2] False Negative (Missed Hard Error) - The code contains a critical hard error (e.g., will fail compilation or crash at runtime) but the oracle test failed to catch it.

You must output STRICTLY in JSON format without any markdown wrappers (e.g., do not use ```json). Output the JSON object directly:
{"error_category": "E1", "reasoning": "A brief, one-sentence explanation of why this category was chosen."}
"""

# ==========================================
# 2. Helper Functions
# ==========================================
def extract_code_from_dialogue(dialogue_path):
    if not os.path.exists(dialogue_path):
        return ""
    with open(dialogue_path, "r", encoding="utf-8") as f:
        text = f.read()

    pattern = r"```[a-zA-Z\+\-]*\n(.*?)```"
    matches = re.findall(pattern, text, re.DOTALL)
    return matches[-1].strip() if matches else ""

def call_llm_judge(status, test_name, generated_code, oracle_test, error_log):
    user_prompt = f"""
    [Oracle Status]: {status}
    [Target Test Case]: {test_name}

    [Generated Code]:
    {generated_code}

    [Oracle Test]:
    {oracle_test}

    [Error Log]:
    {error_log}
    """

    try:
        response = client.chat.completions.create(
            model="anthropic/claude-opus-4.7",
            response_format={ "type": "json_object" },
            messages=[
                {"role": "system", "content": JUDGE_SYSTEM_PROMPT},
                {"role": "user", "content": user_prompt}
            ],
            temperature=0.1
        )
        content = response.choices[0].message.content
        result_json = json.loads(content)
        return result_json.get("error_category", "UNKNOWN"), result_json.get("reasoning", "No reasoning provided.")
    except Exception as e:
        print(f"  [!] API Call Failed: {e}")
        return "ERROR", str(e)

# ==========================================
# 3. Main Pipeline
# ==========================================
def main():
    analysis_records = []

    for base_dir in LEVEL_DIRS:
        results_dir = os.path.join(base_dir, "results")
        
        if not os.path.exists(results_dir):
            print(f"[!] Warning: Could not find results directory at {results_dir}. Skipping...")
            continue
            
        print(f"\n{'='*60}")
        print(f"🚀 Processing Level Directory: {base_dir}")
        print(f"{'='*60}")

        model_dirs = [d for d in os.listdir(results_dir) if os.path.isdir(os.path.join(results_dir, d))]

        for model_name in model_dirs:
            model_path = os.path.join(results_dir, model_name)
            print(f"\n--- Model: {model_name} ---")

            categories = [d for d in os.listdir(model_path) if os.path.isdir(os.path.join(model_path, d))]

            for category in categories:
                summary_path = os.path.join(model_path, category, "summary.json")

                if not os.path.exists(summary_path):
                    continue

                with open(summary_path, "r", encoding="utf-8") as f:
                    try:
                        summary_data = json.load(f)
                    except json.JSONDecodeError:
                        print(f"  [!] Failed to parse JSON: {summary_path}")
                        continue

                for task_data in summary_data:
                    status = task_data.get("status", "UNKNOWN")
                    task_id = task_data.get("task_id", "")

                    if "/" not in task_id:
                        task_category, task_name = category, task_id
                    else:
                        task_category, task_name = task_id.split("/", 1)

                    print(f"\n  -> Task [{status}]: {task_id}")

                    dialogue_path = os.path.join(model_path, task_category, task_name, "dialogue.md")
                    
                    oracle_test_path = os.path.join(base_dir, task_category, task_name, "tests", "test_oracle_ros2.py")

                    generated_code = extract_code_from_dialogue(dialogue_path)
                    oracle_test_code = ""
                    if os.path.exists(oracle_test_path):
                        with open(oracle_test_path, "r", encoding="utf-8") as f:
                            oracle_test_code = f.read()

                    if not generated_code:
                        generated_code = "// NO CODE GENERATED OR PARSE FAILED."
                    if not oracle_test_code:
                        oracle_test_code = "# ORACLE TEST NOT FOUND."


                    if status == "SUCCESS":
                        test_name = "OVERALL_CODE_CHECK"
                        error_log = "No specific errors. Passed all oracle tests. Please verify if this is a True Success"
                        print(f"     * Analyzing full success pass for E2/P1...", end="", flush=True)

                        category_label, reasoning = call_llm_judge("SUCCESS", test_name, generated_code, oracle_test_code, error_log)
                        print(f" [{category_label}]")

                        analysis_records.append({
                            "Model": model_name,
                            "Category": task_category,
                            "Task_ID": task_name,
                            "Test_Case": test_name,
                            "Original_Status": status,
                            "Error_Category": category_label,
                            "Reasoning": reasoning
                        })
                    elif status == "FAILED":
                        failed_cases = task_data.get("failed_cases", [])

                        if failed_cases:
                            for fc in failed_cases:
                                test_name = fc.get('test_function', 'Unknown Test')
                                test_error_log = fc.get('error_message', '')

                                print(f"     * Analyzing test case: {test_name}...", end="", flush=True)
                                category_label, reasoning = call_llm_judge("FAILED", test_name, generated_code, oracle_test_code, test_error_log)
                                print(f" [{category_label}]")

                                analysis_records.append({
                                    "Model": model_name,
                                    "Category": task_category,
                                    "Task_ID": task_name,
                                    "Test_Case": test_name,
                                    "Original_Status": status,
                                    "Error_Category": category_label,
                                    "Reasoning": reasoning
                                })
                        else:
                            print("     * [!] No failed cases found in JSON but status is FAILED.")

                        test_name = "OVERALL_HIDDEN_ERROR_CHECK"
                        hidden_error_log = "Ignore the specific failed tests for a moment. Look at the entire generated code..."
                        print(f"     * Analyzing OVERALL code for hidden E2...", end="", flush=True)

                        category_label, reasoning = call_llm_judge("SUCCESS", test_name, generated_code, oracle_test_code, hidden_error_log)
                        print(f" [{category_label}]")

                        analysis_records.append({
                            "Model": model_name,
                            "Category": task_category,
                            "Task_ID": task_name,
                            "Test_Case": test_name,
                            "Original_Status": status,
                            "Error_Category": category_label,
                            "Reasoning": reasoning
                        })

    if analysis_records:
        df = pd.DataFrame(analysis_records)
        output_csv = "error_analysis_results_by_testcase.csv"
        df.to_csv(output_csv, index=False, encoding="utf-8-sig")
        print(f"\n🎉 All done! Processed all 3 levels. Error analysis saved to {output_csv}")
    else:
        print("\nNo tasks found to analyze.")

if __name__ == "__main__":
    main()
if __name__ == "__main__":
    main()
