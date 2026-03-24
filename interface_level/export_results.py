import os
import json
import csv

def export_to_csv():
    results_dir = "results"
    output_file = "benchmark_final_report.csv"
    
    # Define the headers for your table
    headers = ["Model", "Category", "Task ID", "Status", "Score", "Pass Rate", "Tokens", "Failed Functions"]
    
    rows = []

    # Walk through the results directory
    if not os.path.exists(results_dir):
        print("No results directory found!")
        return

    # Iterate through model folders (e.g., openai_gpt-4o)
    for model_folder in os.listdir(results_dir):
        model_path = os.path.join(results_dir, model_folder)
        if not os.path.isdir(model_path): continue
        
        # Iterate through category folders (e.g., action_server)
        for cat_folder in os.listdir(model_path):
            summary_path = os.path.join(model_path, cat_folder, "summary.json")
            
            if os.path.exists(summary_path):
                with open(summary_path, "r", encoding="utf-8") as f:
                    tasks = json.load(f)
                    for task in tasks:
                        # Extract failed function names into a single string
                        failed_funcs = ", ".join([fc['test_function'] for fc in task.get('failed_cases', [])])
                        
                        rows.append([
                            model_folder,
                            cat_folder,
                            task['task_id'],
                            task['status'],
                            task['score'],
                            task['pass_rate'],
                            task['tokens'],
                            failed_funcs if failed_funcs else "None"
                        ])

    # Write to CSV
    with open(output_file, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(headers)
        writer.writerows(rows)
    
    print(f"✅ Success! All results exported to: {output_file}")

if __name__ == "__main__":
    export_to_csv()
